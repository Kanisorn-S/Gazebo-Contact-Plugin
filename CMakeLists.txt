cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(contact_plugin)

find_package(gazebo REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(contact SHARED ContactPlugin.cc)

# Just link Gazebo and rclcpp — std_msgs is header-only
target_link_libraries(contact
  ${GAZEBO_LIBRARIES}
  rclcpp::rclcpp
)

install(TARGETS contact DESTINATION lib)
