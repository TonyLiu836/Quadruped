cmake_minimum_required(VERSION 3.0.2)
project(joint_traj_example)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  trajectory_msgs
)

catkin_package()

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/traj_subscriber.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

