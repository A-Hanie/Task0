cmake_minimum_required(VERSION 3.8)
project(simple_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)  
find_package(simple_system_interfaces REQUIRED)  

add_executable(plant_node src/plant_node.cpp)
add_executable(controller_node src/controller_node.cpp)  

ament_target_dependencies(plant_node
  rclcpp
  std_msgs
  simple_system_interfaces
)

ament_target_dependencies(controller_node  
  rclcpp
  std_msgs
  simple_system_interfaces
)

install(TARGETS
  plant_node
  controller_node
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)


ament_package()
