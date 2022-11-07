cmake_minimum_required(VERSION 3.0.2)
project(virtual_force_sensor_rtc)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)
cmake_policy(SET CMP0046 OLD)

find_package(catkin REQUIRED COMPONENTS
  rtmbuild
  )

rtmbuild_init()

catkin_package(
)

add_subdirectory(rtc)
