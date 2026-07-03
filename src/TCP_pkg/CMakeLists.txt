cmake_minimum_required(VERSION 3.8)
project(TCP_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================================
# ПОИСК ЗАВИСИМОСТЕЙ
# ============================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(std_msgs REQUIRED)  # <-- ДОБАВИТЬ!

# ============================================
# TCP-КЛИЕНТ
# ============================================
add_executable(tcp_client_node src/TCP_pkg_client_pkg_node.cpp)
ament_target_dependencies(tcp_client_node
    rclcpp
    std_msgs  # <-- ДОБАВИТЬ!
)

# ============================================
# TCP-СЕРВЕР
# ============================================
add_executable(tcp_server_node src/TCP_pkg_server_pkg_node.cpp)
ament_target_dependencies(tcp_server_node
    rclcpp
    mavros_msgs
    std_msgs  # <-- ДОБАВИТЬ (если тоже использует std_msgs)
)

# ============================================
# УСТАНОВКА
# ============================================
install(TARGETS
    tcp_client_node
    tcp_server_node
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
