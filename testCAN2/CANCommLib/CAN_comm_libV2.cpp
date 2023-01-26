
#ifndef PF_CAN
#define PF_CAN 29
#endif

// #include "../MathOpsLib/math_ops.h"
#include "math_ops.h"
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

#include <unistd.h>
unsigned int second = 1000000;

//limits 
#define P_MIN -12.5f  // -4*pi              #in radians
#define P_MAX 12.5f   // 4*pi
#define V_MIN -65.0f
#define V_MAX 65.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -10.0f        //originally -18.0
#define T_MAX 10.0f         //originall 18.0

int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);

float radToDeg(float rad){
    float pi = 3.1415926535;
    return float(rad * (180/pi));
}

float degToRad(float deg){
    float pi = 3.1415926535;
    return float(deg * (pi/180));
}

void open_port(std::string interf_name, int bitrate){
    std::string set_bitrate = "sudo ip link set " + interf_name + " type can bitrate " + std::to_string(bitrate);
    std::string can_up = "sudo ip link set up " + interf_name;
    system(set_bitrate.c_str());
    system(can_up.c_str());
    std::cout << "port opened @ " << interf_name << " with bitrate: " << bitrate << std::endl;
}

void close_port(std::string can_id){
    std::string close_port = "sudo ifconfig " + can_id + " down";
    system(close_port.c_str());
    std::cout << "port " << can_id << " closed" << std::endl;
}


class MotorController
{
public:
    const char* interf_name;
    int motor_id;
    float p_des, v_des, kp, kd, t_ff;

    //vars used to bind socket to CAN interfaces
    struct sockaddr_can addr;
    struct ifreq ifr;
    socklen_t len = sizeof(addr);

    //constructor to initialize attributes of each motor
    MotorController(const char* interface_name, int motorID){
        interf_name = interface_name;
        motor_id = motorID; 
        p_des = 0.0;
        v_des = 0.0;
        kp = 0.0;
        kd = 0.0;
        t_ff = 0.0;
        
        //bind socket to CAN interfaces
        strcpy(ifr.ifr_name, interface_name);
        ioctl(s, SIOCGIFINDEX, &ifr);
        addr.can_family=AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        bind(s, (struct sockaddr *)&addr, sizeof(addr));
    }
    
    void unpack_msg(std::vector<int> data){
        int pos_int = (data[1] << 8) | data[2];
        int vel_int = (data[3] << 4) | (data[4] >> 4);
        int FF_int = ((data[4] & 0xF) << 8) | data[5];

        p_des = uint_to_float(pos_int, P_MIN, P_MAX, 16);
        v_des = uint_to_float(vel_int, V_MIN, V_MAX, 12);
        t_ff = uint_to_float(FF_int, T_MIN, T_MAX, 12);

        std::cout << "pos= " << p_des << " | vel= " << v_des << " | FF= " << t_ff << std::endl;
    }

    void enter_motor_mode(){
        int nbytes;
        
        //write CAN frame
        struct can_frame frame;
        struct can_frame recv_frame;

        frame.can_id  = motor_id;          //motor id
        frame.can_dlc = 8;              //len of data 
        frame.data[0] = 0xFF;
        frame.data[1] = 0xFF;
        frame.data[2] = 0xFF;
        frame.data[3] = 0xFF;
        frame.data[4] = 0xFF;
        frame.data[5] = 0xFF;
        frame.data[6] = 0xFF;
        frame.data[7] = 0xFC;
        
        nbytes = write(s, &frame, sizeof(struct can_frame));
        

        std::cout << "Motor Mode Enabled!" << std::endl; 
        
        // int nbytes2 = recvfrom(s, &frame, sizeof(struct can_frame), addr.can_ifindex, (struct sockaddr*)&addr, &len);
        int nbytes2 = read(s, &frame, sizeof(struct can_frame));
        if (nbytes2 < 0){
            perror("read");
        }

        std::vector<int> recv_data;
        for (int i = 0; i < sizeof(frame.data)/sizeof(frame.data[0]); i++){
            recv_data.push_back((int)frame.data[i]);
        }

        for (int i: recv_data){
            std::cout << i << " "<< std::endl;
        }

        unpack_msg(recv_data);
    }

    void exit_motor_mode(){          
        int nbytes;
        
        //write CAN frame
        struct can_frame frame;
        frame.can_id  = motor_id;          //motor id
        frame.can_dlc = 8;              //len of data 
        frame.data[0] = 0xFF;
        frame.data[1] = 0xFF;
        frame.data[2] = 0xFF;
        frame.data[3] = 0xFF;
        frame.data[4] = 0xFF;
        frame.data[5] = 0xFF;
        frame.data[6] = 0xFF;
        frame.data[7] = 0xFD;
        
        //send 0 command to eliminate the initial "kick" from motor controller
        write_can_frame(0, 0, 0, 0, 0);
        
        nbytes = write(s, &frame, sizeof(struct can_frame));
        
        // int nbytes2 = recvfrom(s, &frame, sizeof(struct can_frame), addr.can_ifindex, (struct sockaddr*)&addr, &len);
        int nbytes2 = read(s, &frame, sizeof(struct can_frame));

        std::vector<int> recv_data;
        for (int i = 0; i < sizeof(frame.data)/sizeof(frame.data[0]); i++){
            recv_data.push_back((int)frame.data[i]);
        }
        
        unpack_msg(recv_data);
        std::cout << "Motor Mode Disabled!" << std::endl;
    }

    void zero_position_sensor(){

        int nbytes;

        //write CAN frame
        struct can_frame frame;
        frame.can_id  = motor_id;          //motor id
        frame.can_dlc = 8;              //len of data 
        frame.data[0] = 0xFF;
        frame.data[1] = 0xFF;
        frame.data[2] = 0xFF;
        frame.data[3] = 0xFF;
        frame.data[4] = 0xFF;
        frame.data[5] = 0xFF;
        frame.data[6] = 0xFF;
        frame.data[7] = 0xFE;
        
        nbytes = write(s, &frame, sizeof(struct can_frame));

        std::cout << "Current position set to zero!" << std::endl;

        // int nbytes2 = recvfrom(s, &frame, sizeof(struct can_frame), addr.can_ifindex, (struct sockaddr*)&addr, &len); 
        int nbytes2 = read(s, &frame, sizeof(struct can_frame));
        
        std::vector<int> recv_data;
        for (int i = 0; i < sizeof(frame.data)/sizeof(frame.data[0]); i++){
            recv_data.push_back((int)frame.data[i]);
        }

        unpack_msg(recv_data);
    }

    void pack_msg(can_frame& frame, float pos, float vel, float KP, float KD, float T_FF){
    
        //std::cout << frame << std::endl;
    
        int pos_int = float_to_uint(pos, P_MIN,P_MAX, 16);
        int vel_int = float_to_uint(vel, V_MIN, V_MAX, 12);
        int KP_int = float_to_uint(KP, KP_MIN, KP_MAX, 12);
        int KD_int = float_to_uint(KD, KD_MIN, KD_MAX, 12);
        int T_FF_int = float_to_uint(T_FF, T_MIN, T_MAX, 12);

        frame.data[0] = pos_int >> 8;                                       //pos[15~8]
        frame.data[1] = pos_int & 0xFF;                                     //pos[7~0]
        frame.data[2] = vel_int >> 4;                                       //vel[11~4]
        frame.data[3] = ((vel_int & 0xF) << 4) |(KP_int >> 8);              //vel [3~0] | KP [11~8]
        frame.data[4] = KP_int & 0xFF;                                      //KP[7~0]
        frame.data[5] = KD_int >> 4;                                        //KD[11~8]
        frame.data[6] = ((KD_int & 0xF) << 4) | (T_FF_int >> 8);            //KD[3~0] | FF[11~8]
        frame.data[7] = T_FF_int & 0xff;                                    //FF[7~0]
    }  

    void write_can_frame(float pos, float vel, float KP, float KD, float FF){
        
        int nbytes;
        
        //write CAN frame
        struct can_frame frame;
        
        frame.can_id  = motor_id;        //motor id
        frame.can_dlc = 8;              //len of data 

        pack_msg(frame, pos, vel, KP, KD, FF);

        nbytes = write(s, &frame, sizeof(struct can_frame));
        // int nbytes2 = recvfrom(s, &frame, sizeof(struct can_frame), addr.can_ifindex, (struct sockaddr*)&addr, &len);
        int nbytes2 = read(s, &frame, sizeof(struct can_frame));
        
        std::vector<int> recv_data;
        for (int i = 0; i < sizeof(frame.data)/sizeof(frame.data[0]); i++){
            recv_data.push_back((int)frame.data[i]);
        }

        unpack_msg(recv_data);
    }
};


//purple actuator = M2
//yellow actuator = M1 
//used to test functions above
int main(){
    //setup CAN
    const char *interface_name = "can0";           
    open_port(interface_name, 1000000);    
    
    MotorController M1 {interface_name, 1};
    // MotorController M2 {interface_name, 2};

    // MotorController M2 {}
    M1.enter_motor_mode();
    // M2.enter_motor_mode();

    M1.zero_position_sensor();
    // M2.zero_position_sensor();

    // write_can_frame(POS, VEL, KP, KD, FF);
    // M1.write_can_frame(3.1415,0,3,0.2,0);
    M1.write_can_frame(2,2,2,2,2);
    usleep(4*second);
    // M2.write_can_frame(0.5233,0,3,0.2,0);          //currently purple actuator
    // M1.write_can_frame(0,0,0,0,0);
    // M1.write_can_frame(1,1,1,1,1);
    // usleep(2*second);

    M1.exit_motor_mode();
    // M2.exit_motor_mode();

    close_port(interface_name);

    std::cout << "Finished running";
    return 0;
    
}

