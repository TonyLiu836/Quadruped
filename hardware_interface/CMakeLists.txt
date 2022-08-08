cmake_minimum_required(VERSION 3.0.2)
project(hardware_interface)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package()

#add_library(pca9685lib STATIC src/PCA9685.cpp include/hw_interface.h)
#target_link_libraries()

include_directories(include ${catkin_INCLUDE_DIRS})
add_subdirectory(PCA9685Lib)

#add_executable(parrot src/hw_interface_demo.cpp)
#target_link_libraries(parrot ${catkin_LIBRARIES})

add_executable(hwInterface src/hw_interface.cpp)
target_link_libraries(hwInterface ${catkin_LIBRARIES})

target_link_libraries(hwInterface pca9685lib)
#target_link_libraries(${PROJECT_NAME} pca9685lib) #library name (pca9685lib) should match with the name in the CMakeLists.txt file in the library's directory
