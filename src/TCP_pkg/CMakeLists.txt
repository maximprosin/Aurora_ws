cmake_minimum_required(VERSION 3.8)
project(TCP_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Находим зависимости ROS 2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros_msgs REQUIRED)

# ============================================
# ЦЕЛЬ 1: TCP-сервер
# ============================================
add_executable(tcp_server_node src/TCP_pkg_server_node.cpp)

# Теперь добавляем зависимости после объявления цели
ament_target_dependencies(tcp_server_node
    rclcpp
    mavros_msgs
)

# ============================================
# ЦЕЛЬ 2: TCP-клиент
# ============================================
add_executable(tcp_client_node src/TCP_pkg_client_node.cpp)

# И для клиента тоже добавляем зависимости после add_executable
ament_target_dependencies(tcp_client_node
    rclcpp
)

# ============================================
# Установка и сборка
# ============================================
install(TARGETS
    tcp_server_node
    tcp_client_node
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
