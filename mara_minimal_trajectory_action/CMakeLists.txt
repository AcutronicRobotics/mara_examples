cmake_minimum_required(VERSION 3.5)
project(mara_minimal_trajectory_action)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(hrim_actuator_rotaryservo_actions REQUIRED)

add_executable(${PROJECT_NAME} src/mara_minimal_trajectory_action.cpp)
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    rclcpp_action
    hrim_actuator_rotaryservo_actions
)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
