#!/bin/bash
set -e

# Определяем путь к корню нашего проекта
PROJECT_ROOT=$(pwd)

cd $HOME

echo "=== 1. Добавление репозитория ROS 2 Jazzy ==="
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl gnupg2 lsb-release -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo "=== 2. Обновление списков и установка всех зависимостей ==="
sudo apt update
sudo apt install -y \
  build-essential cmake g++ gdb git meson ninja-build \
  python3-colcon-common-extensions python3-pip python3.12-venv \
  tcpdump lm-sensors v4l-utils ffmpeg \
  ros-jazzy-ros-base ros-dev-tools \
  ros-jazzy-cv-bridge ros-jazzy-sensor-msgs ros-jazzy-foxglove-bridge \
  ros-jazzy-slam-toolbox \
  libcamera-dev libcamera-tools \
  gstreamer1.0-libcamera gstreamer1.0-tools \
  gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
  libboost-dev libgnutls28-dev zlib1g-dev libtiff-dev libpng-dev \
  libjpeg-dev libyaml-dev liblttng-ust-dev python3-yaml python3-ply \
  python3-jinja2 libevent-dev libsdl2-dev libunwind-dev libyuv-dev \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev

echo "=== 3. Настройка прав пользователя и системы ==="
sudo usermod -a -G dialout $USER
sudo usermod -a -G video $USER
sudo timedatectl set-ntp true
echo "bcm2835-v4l2" | sudo tee -a /etc/modules

echo "=== 4. Настройка config.txt для камеры ==="
CONFIG_FILE="/boot/firmware/config.txt"

# 1. Убеждаемся, что camera_auto_detect включен (меняем 0 на 1 или добавляем, если нет)
if grep -q "^camera_auto_detect=" $CONFIG_FILE; then
    sudo sed -i 's/^camera_auto_detect=.*/camera_auto_detect=1/' $CONFIG_FILE
else
    echo "camera_auto_detect=1" | sudo tee -a $CONFIG_FILE
fi

# 2. Добавляем оверлеи, если их еще нет (проверка по уникальной строке imx708)
if ! grep -q "dtoverlay=imx708" $CONFIG_FILE; then
    echo -e "\n# Jeweler Robot Hardware Setup\ndtoverlay=vc4-kms-v3d\ndtoverlay=imx708" | sudo tee -a $CONFIG_FILE
fi

echo "=== 5. Настройка Swap 4GB ==="
# Отключаем всё, что связано с этим файлом, если оно активно
sudo swapoff /swapfile 2>/dev/null || true

# Создаем файл заново нужного размера
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile

# Добавляем в fstab, только если этой строки там еще нет
if ! grep -q "/swapfile none swap sw 0 0" /etc/fstab; then
  echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
fi

echo "Swap настроен:"
free -h

echo "=== 6. Сборка и установка libpisp (обязательно для RPi 5) ==="
rm -rf libpisp
git clone --depth 1 https://github.com/raspberrypi/libpisp.git
cd libpisp
meson setup build --buildtype=release
ninja -C build
sudo ninja -C build install
sudo ldconfig
cd ..

echo "=== 7. Сборка и установка libcamera ==="
rm -rf libcamera
git clone --depth 1 https://github.com/raspberrypi/libcamera.git
cd libcamera
meson setup build --buildtype=release -Dgstreamer=enabled -Dpycamera=enabled
ninja -C build
sudo ninja -C build install
sudo ldconfig
cd ..

echo "=== 8. Сборка и установка rpicam-apps ==="
rm -rf rpicam-apps
git clone --depth 1 https://github.com/raspberrypi/rpicam-apps.git
cd rpicam-apps
meson setup build --buildtype=release
ninja -C build
sudo ninja -C build install
sudo ldconfig
cd ..

echo "=== 9. Фикс путей библиотек для Ubuntu ==="
echo "/usr/local/lib/aarch64-linux-gnu" | sudo tee /etc/ld.so.conf.d/rpicam.conf
sudo ldconfig

echo "=== 10. Настройка ROS 2 Workspace и Лидара ==="
# 1. Создаем воркспейс, если его еще нет
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 2. Клонируем драйвер лидара (если папки еще нет)
if [ ! -d "sllidar_ros2" ]; then
    git clone https://github.com/Slamtec/sllidar_ros2.git
fi

# 3. Установка udev-правил (чтобы лидар всегда был на /dev/rplidar и с правами)
echo "Настройка udev правил для лидара..."
cd sllidar_ros2
# Скрипт Slamtec требует sudo внутри
chmod +x scripts/create_udev_rules.sh
sudo ./scripts/create_udev_rules.sh
cd ~/ros2_ws

# 4. Сборка лидара
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select sllidar_ros2

# 5. Добавляем сорсинг воркспейса в .bashrc, чтобы команды ros2 работали сразу
if ! grep -q "ros2_ws/install/setup.bash" ~/.bashrc; then
  echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
  echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
fi

echo "=== 11. Настройка Python окружения ==="
cd $HOME
python3 -m pip install --upgrade pip --break-system-packages
python3 -m pip install ultralytics==8.4.21 openvino==2026.0.0 numpy==2.4.3 opencv-python --break-system-packages

echo "=== 12. Сборка Jeweler_Nav ==="
# 1. Делаем линк пакета из репозитория в воркспейс ROS2
# Это позволит править код в папке репозитория, и он сразу будет готов к сборке
ln -sfn ~/Jeweler_Software/ros2_ws/src/jeweler_nav ~/ros2_ws/src/jeweler_nav

# 2. Делаем Python скрипты исполняемыми
chmod +x ~/Jeweler_Software/ros2_ws/src/jeweler_nav/scripts/*.py

# 3. Сборка пакета
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select jeweler_nav

# 4. Сорсинг
if ! grep -q "ros2_ws/install/setup.bash" ~/.bashrc; then
  echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
fi

echo "=== Установка завершена! Перезагрузите систему: sudo reboot ==="
