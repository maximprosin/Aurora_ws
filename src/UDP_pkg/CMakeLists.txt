cmake_minimum_required(VERSION 3.8)
project(udp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

# ★ GStreamer ★
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-app-1.0)

include_directories(${GST_INCLUDE_DIRS})
link_directories(${GST_LIBRARY_DIRS})
add_definitions(${GST_DEFINITIONS})

include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(udp_pkg_server_node
  src/udp_pkg_server_node.cpp
)

ament_target_dependencies(udp_pkg_server_node
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  Boost
)

target_link_libraries(udp_pkg_server_node ${GST_LIBRARIES})
target_include_directories(udp_pkg_server_node PRIVATE ${GST_INCLUDE_DIRS})

add_executable(udp_pkg_client_node
  src/udp_pkg_client_node.cpp
)

ament_target_dependencies(udp_pkg_client_node
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  Boost
)

target_link_libraries(udp_pkg_client_node ${GST_LIBRARIES})
target_include_directories(udp_pkg_client_node PRIVATE ${GST_INCLUDE_DIRS})

install(TARGETS
  udp_pkg_server_node
  udp_pkg_client_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()