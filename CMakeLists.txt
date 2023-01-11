cmake_minimum_required(VERSION 3.8)

project ("sick_vision_ros2")

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sick_vision_api REQUIRED)
find_package(sick_libimg REQUIRED)

add_library(node SHARED src/main.cpp)
ament_target_dependencies(node ament_index_cpp rclcpp rclcpp_components cv_bridge image_transport OpenCV)
rclcpp_components_register_node(node
PLUGIN "sick_vision_ros2::SickVision"
EXECUTABLE vision_node)

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
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY cti
  DESTINATION share/${PROJECT_NAME}
)

ament_package()