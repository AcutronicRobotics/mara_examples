cmake_minimum_required(VERSION 3.5)
project(gripper_minimal_subscriber)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hrim_actuator_gripper_msgs REQUIRED)

add_executable(${PROJECT_NAME} src/gripper_minimal_subscriber.cpp)
ament_target_dependencies(${PROJECT_NAME} rclcpp hrim_actuator_gripper_msgs)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# install(
#   PROGRAMS
#   scripts/gripper_minimal_client.py
#   DESTINATION lib/${PROJECT_NAME})

ament_package()
