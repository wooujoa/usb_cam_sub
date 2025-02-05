cmake_minimum_required(VERSION 3.8)
project(usb_cam_sub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)


add_executable(sub src/usb_cam_sub.cpp)
ament_target_dependencies(sub rclcpp sensor_msgs cv_bridge image_transport OpenCV)

install(TARGETS
  sub
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
