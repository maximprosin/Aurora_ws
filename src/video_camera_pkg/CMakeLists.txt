cmake_minimum_required(VERSION 3.8)
project(video_camera_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# --- Поиск GStreamer ---
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-app-1.0)

add_executable(video_camera_pkg_node src/video_camera_pkg_node.cpp)

target_include_directories(video_camera_pkg_node PRIVATE
  ${GST_INCLUDE_DIRS}
)

target_link_libraries(video_camera_pkg_node
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${cv_bridge_LIBRARIES}
  ${OpenCV_LIBS}
  ${GST_LIBRARIES}
)

ament_target_dependencies(video_camera_pkg_node
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

install(TARGETS video_camera_pkg_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()