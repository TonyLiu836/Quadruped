#ifndef CAN_COMM_H
#define CAN_COMM_H

/*
#ifndef PF_CAN
#define PF_CAN 29
#endif
*/

#include "../MathOpsLib/math_ops.h"
#include <cstring>
#include <linux/can.h>
#include <sys/time.h>
#include <iostream>
#include <linux/can/raw.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <vector>
#include <typeinfo>

void open_port(std::string can_id, int bitrate);
void close_port(std::string can_id);
void pack_msg(can_frame& frame, float pos, float vel, float KP, float KD, float T_FF);
void unpack_msg(std::vector<int> data, MotorStatusStruct& motor);
void enter_motor_mode(const char* interface_name, int id_can, MotorStatusStruct& motor);
void exit_motor_mode(const char* interface_name, int id_can, MotorStatusStruct& motor);
void zero_position_sensor(const char* interface_name, int id_can, MotorStatusStruct& motor);
void write_can_frame(const char* interface_name, int id_can, int pos, int vel, int KP, int KD, int FF, MotorStatusStruct& motor);


