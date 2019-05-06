cmake_minimum_required(VERSION 3.5)
project(mara_minimal_subscriber)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hrim_actuator_rotaryservo_msgs REQUIRED)

add_executable(mara_minimal_subscriber_v1 src/mara_minimal_subscriber_v1.cpp)
ament_target_dependencies(mara_minimal_subscriber_v1 rclcpp hrim_actuator_rotaryservo_msgs)

add_executable(mara_minimal_subscriber_v2 src/mara_minimal_subscriber_v2.cpp)
ament_target_dependencies(mara_minimal_subscriber_v2 rclcpp hrim_actuator_rotaryservo_msgs)

install(TARGETS
  mara_minimal_subscriber_v1
  mara_minimal_subscriber_v2
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/mara_minimal_subscriber_v1.py
  scripts/mara_minimal_subscriber_v2.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
