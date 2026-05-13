cmake_minimum_required(VERSION 3.8)
project(jeweler_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Поиск зависимостей
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Указываем общую папку заголовков для всех таргетов
include_directories(include)

# --- 1. Старая нода (Simple Commander) ---
add_executable(simple_commander src/simple_commander.cpp)
ament_target_dependencies(simple_commander
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  sensor_msgs
)

# --- 2. Новая нода (Jeweler Navigator) ---
# Обязательно указываем ВСЕ .cpp файлы, которые относятся к этой ноде
add_executable(jeweler_navigator
  src/JewelerNavigator.cpp
  src/GlobalPlanner.cpp
  src/ObstacleManager.cpp
)
ament_target_dependencies(jeweler_navigator
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  sensor_msgs
  visualization_msgs
)

# --- 3. Установка ---
# Добавляем оба исполняемых файла, чтобы ros2 run их видел
install(TARGETS
  simple_commander
  jeweler_navigator
  DESTINATION lib/${PROJECT_NAME}
)

# --- 4. Установка ресурсов (Launch, Scripts, URDF) ---
install(DIRECTORY
  launch
  scripts
  urdf
  config
  DESTINATION share/${PROJECT_NAME}
)

# Экспорт интерфейсов (важно для работы инклюдов в ROS2)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
