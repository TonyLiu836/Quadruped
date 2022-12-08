#ifndef CAN_COMM_LIB_H
#define CAN_COMM_LIB_H
#endif

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

class MotorController
{
    public: 
        const char* interf_name;
        int motor_id, p_des, v_des, kp, kd, t_ff;

        MotorController(const char* interface_name, int motorID);
        void pack_msg(can_frame& frame, float pos, float vel, float KP, float KD, float T_FF);
        void unpack_msg(std::vector<int> data);
        void enter_motor_mode();
        void exit_motor_mode();
        void zero_position_sensor();
        void write_can_frame(int pos, int vel, int KP, int KD, int FF);

};