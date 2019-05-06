cmake_minimum_required(VERSION 3.5)
project(gripper_minimal_service)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hrim_actuator_gripper_srvs REQUIRED)

add_executable(gripper_minimal_client_v1 src/gripper_minimal_client_v1.cpp)
ament_target_dependencies(gripper_minimal_client_v1 rclcpp hrim_actuator_gripper_srvs)

add_executable(gripper_minimal_client_v2 src/gripper_minimal_client_v2.cpp)
ament_target_dependencies(gripper_minimal_client_v2 rclcpp hrim_actuator_gripper_srvs)

install(TARGETS
  gripper_minimal_client_v1
  gripper_minimal_client_v2
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
  scripts/gripper_minimal_client_v1.py
  scripts/gripper_minimal_client_v2.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
