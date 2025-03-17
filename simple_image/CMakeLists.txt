cmake_minimum_required(VERSION 3.16)
project(simple_image)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ianvs REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

# SHARED is required for composable nodes
add_library(
  ${PROJECT_NAME} SHARED
  src/simple_image_node.cpp
)
target_include_directories(
  ${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                         "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC rclcpp::rclcpp
  PRIVATE cv_bridge::cv_bridge ianvs::ianvs message_filters::message_filters
          rclcpp_components::component
)

rclcpp_components_register_node(
	${PROJECT_NAME} PLUGIN simple_image::SimpleImageNode EXECUTABLE
  simple_image_node
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}-targets
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
