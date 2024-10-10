cmake_minimum_required(VERSION 3.5)
project(temperature_monitoring)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(temp_pub src/temp_pub.cpp)
ament_target_dependencies(temp_pub rclcpp std_msgs)

add_executable(temp_sub src/temp_sub.cpp)
ament_target_dependencies(temp_sub rclcpp std_msgs)

install(TARGETS
  temp_pub
  temp_sub
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
