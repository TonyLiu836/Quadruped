cmake_minimum_required(VERSION 3.10.0)
project(hardware_interface)

set(CMAKE_BULD_TYPE Debug)
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp trajectory_msgs sensor_msgs std_msgs)

catkin_package()
include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(${PROJECT_NAME} src/hw_interface.cpp)

add_subdirectory(PCA9685Lib)

target_link_libraries(${PROJECT_NAME} pca9685lib)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
