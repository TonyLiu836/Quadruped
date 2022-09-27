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


add_executable(${PROJECT_NAME} src/hardware_interface_bldc)

add_subdirectory(MathOpsLib)
add_subdirectory(CANCommLib)

target_link_libraries(${PROJECT_NAME} MathOpsLib CANCommLib)
target_link_libraries(${PROJECT_NAME} ${catkin_INCLUDE_DIRS})