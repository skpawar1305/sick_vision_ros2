cmake_minimum_required(VERSION 3.8)

project ("sick_vision_ros2")

find_package(ament_cmake REQUIRED)
find_package (rclcpp REQUIRED)
find_package (OpenCV REQUIRED)
find_package (sick_vision_api REQUIRED)
find_package (sick_libimg REQUIRED)

add_executable (node
  src/main.cpp
)
ament_target_dependencies(node rclcpp OpenCV)

target_include_directories(node PUBLIC
  ${SICK_VISION_API_INCLUDE_DIR}
  ${SICK_LIBIMG_INCLUDE_DIR}
)

target_link_libraries(node
  ${SICK_VISION_API_LIBRARIES}
  ${SICK_LIBIMG_LIBRARIES}
)

install(TARGETS
  node
  DESTINATION lib
)

ament_package()