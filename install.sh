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

# Определяем рабочую директорию, чтобы загрузки не мусорили где попало
# (например, создадим временную папку внутри домашней директории)
BUILD_DIR="$HOME/jeweler_build_deps"
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

echo "=== 6. Сборка и установка libpisp (обязательно для RPi 5) ==="
# Проверяем, установлена ли библиотека в систему
if ldconfig -p | grep -q libpisp; then
    echo "libpisp уже установлена в системе. Пропускаем."
else
    echo "libpisp не найдена. Начинаем сборку..."
    if [ ! -d "libpisp" ]; then
        git clone --depth 1 https://github.com/raspberrypi/libpisp.git
    fi
    cd libpisp
    # Проверяем, была ли уже создана папка сборки, чтобы не делать meson setup дважды
    [ ! -d "build" ] && meson setup build --buildtype=release
    ninja -C build
    sudo ninja -C build install
    sudo ldconfig
    cd "$BUILD_DIR"
fi

echo "=== 7. Сборка и установка libcamera ==="
# Проверяем наличие libcamera.so
if ldconfig -p | grep -q libcamera\.so; then
    echo "libcamera уже установлена в системе. Пропускаем."
else
    echo "libcamera не найдена. Начинаем сборку..."
    if [ ! -d "libcamera" ]; then
        git clone --depth 1 https://github.com/raspberrypi/libcamera.git
    fi
    cd libcamera
    [ ! -d "build" ] && meson setup build --buildtype=release -Dgstreamer=enabled -Dpycamera=enabled
    ninja -C build
    sudo ninja -C build install
    sudo ldconfig
    cd "$BUILD_DIR"
fi

echo "=== 8. Сборка и установка rpicam-apps ==="
# У rpicam-apps есть бинарник rpicam-still. Проверим его наличие в системе
if which rpicam-still > /dev/null 2>&1; then
    echo "rpicam-apps уже установлены в системе. Пропускаем."
else
    echo "rpicam-apps не найдены. Начинаем сборку..."
    if [ ! -d "rpicam-apps" ]; then
        git clone --depth 1 https://github.com/raspberrypi/rpicam-apps.git
    fi
    cd rpicam-apps
    [ ! -d "build" ] && meson setup build --buildtype=release
    ninja -C build
    sudo ninja -C build install
    sudo ldconfig
    cd "$BUILD_DIR"
fi

echo "=== 9. Фикс путей библиотек для Ubuntu ==="
# Запись файла конфигурации тоже сделаем умной, чтобы не дублировать строки
if [ ! -f /etc/ld.so.conf.d/rpicam.conf ]; then
    echo "/usr/local/lib/aarch64-linux-gnu" | sudo tee /etc/ld.so.conf.d/rpicam.conf
    sudo ldconfig
fi

echo "=== 10. Настройка ROS 2 Workspace и Лидара ==="
mkdir -p "$HOME/ros2_ws/src"
cd "$HOME/ros2_ws/src"

if [ ! -d "sllidar_ros2" ]; then
    echo "Клонирование драйвера лидара..."
    git clone https://github.com/Slamtec/sllidar_ros2.git
fi

echo "Настройка udev правил для лидара..."
cd sllidar_ros2
chmod +x scripts/create_udev_rules.sh
sudo ./scripts/create_udev_rules.sh

# Возвращаемся в корень воркспейса для сборки
cd "$HOME/ros2_ws"
source /opt/ros/jazzy/setup.bash

# Проверяем, собран ли уже пакет sllidar_ros2. Если папка install/sllidar_ros2 существует, colcon можно пропустить
if [ ! -d "install/sllidar_ros2" ]; then
    echo "Сборка пакета лидара..."
    colcon build --symlink-install --packages-select sllidar_ros2
else
    echo "Пакет sllidar_ros2 уже собран. Пропускаем."
fi

# 5. Добавляем сорсинг воркспейса в .bashrc (проверяем, чтобы не дублировать строки)
if ! grep -q "opt/ros/jazzy/setup.bash" ~/.bashrc; then
  echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
fi

if ! grep -q "ros2_ws/install/setup.bash" ~/.bashrc; then
  echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
fi

echo "=== 11. Настройка Python окружения ==="
cd $HOME
python3 -m pip install --upgrade pip --break-system-packages
python3 -m pip install ultralytics==8.4.21 openvino==2026.0.0 numpy==2.4.3 opencv-python --break-system-packages

echo "=== 12. Сборка Jeweler_Nav ==="
# 1. Делаем линк пакета из репозитория в воркспейс ROS2
# Это позволит править код в папке репозитория, и он сразу будет готов к сборке

# Автоматически определяем абсолютный путь к папке, где лежит этот инсталлер
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Вычисляем путь к папке пакета ROS2 внутри репозитория
JEWELER_NAV_SRC="${SCRIPT_DIR}/ros2_ws/src/jeweler_nav"

# 1. Делаем линк пакета из репозитория в воркспейс ROS2
# Создаем папку назначения, если ее еще нет, чтобы ln не выдал ошибку
mkdir -p "$HOME/ros2_ws/src"
ln -sfn "$JEWELER_NAV_SRC" "$HOME/ros2_ws/src/jeweler_nav"

# 2. Делаем Python скрипты исполняемыми
chmod +x "${JEWELER_NAV_SRC}/scripts/"*.py

# 3. Сборка пакета
cd "$HOME/ros2_ws"
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select jeweler_nav

# 4. Сорсинг
if ! grep -q "ros2_ws/install/setup.bash" ~/.bashrc; then
  echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
fi

echo "=== Установка завершена! Перезагрузите систему: sudo reboot ==="
