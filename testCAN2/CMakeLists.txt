cmake_minimum_required(VERSION 3.16)
project(CAN_comm_lib)

include_directories(MathOpsLib)

add_executable(CAN_comm_lib CANCommLib/CAN_comm_libV2.cpp MathOpsLib/math_ops.cpp)

