cmake_minimum_required(VERSION 3.16)
project(hardware_interface_bldc)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  trajectory_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  #${PROJECT_SOURCE_DIRECTORY}/include
)


add_executable(${PROJECT_NAME} src/hw_interface_bldc.cpp)

add_subdirectory(include/MathOpsLib)       #these directories must have CMakeLists.txt
add_subdirectory(include/CANCommLib)

target_link_libraries(${PROJECT_NAME} MathOps CANComm)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
