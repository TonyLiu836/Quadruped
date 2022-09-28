
#ifndef PF_CAN
#define PF_CAN 29
#endif

//#define M1 0x01

#include "../MathOpsLib/math_ops.h"
#include "CAN_comm.h"
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


//limits 
#define P_MIN -12.5f  // -4*pi
#define P_MAX 12.5f   // 4*pi
#define V_MIN -45.0f
#define V_MAX 45.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -18.0f
#define T_MAX 18.0f

//can_frame frame;

int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);

/*
struct MotorStatusStruct
{
    std::string name;
    int motor_id;
    int p_des, v_des, kp,kd,t_ff;
};
*/



/*
float fmaxf(float x, float y){
    /// Returns maximum of x, y ///
    return (((x)>(y))?(x):(y));
    }
 
float fminf(float x, float y){
    /// Returns minimum of x, y ///
    return (((x)<(y))?(x):(y));
    }
 
float fmaxf3(float x, float y, float z){
    /// Returns maximum of x, y, z ///
    return (x > y ? (x > z ? x : z) : (y > z ? y : z));
    }
 
float fminf3(float x, float y, float z){
    /// Returns minimum of x, y, z ///
    return (x < y ? (x < z ? x : z) : (y < z ? y : z));
    }
    
void limit_norm(float *x, float *y, float limit){
    /// Scales the lenght of vector (x, y) to be <= limit ///
    float norm = sqrt(*x * *x + *y * *y);
    if(norm > limit){
        *x = *x * limit/norm;
        *y = *y * limit/norm;
        }
    }
 
 
int float_to_uint(float x, float x_min, float x_max, int bits){
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
    }
    
    
float uint_to_float(int x_int, float x_min, float x_max, int bits){
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
    }
*/

void open_port(std::string can_id, int bitrate){
    std::string set_bitrate = "sudo ip link set " + can_id + " type can bitrate " + std::to_string(bitrate);
    std::string can_up = "sudo ip link set up " + can_id;
    system(set_bitrate.c_str());
    system(can_up.c_str());
    std::cout << "port opened @ " << can_id << " with bitrate: " << bitrate << std::endl;
}

void close_port(std::string can_id){
    std::string close_port = "sudo ifconfig " + can_id + " down";
    system(close_port.c_str());
    std::cout << "port " << can_id << " closed" << std::endl;
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


void unpack_msg(std::vector<int> data, MotorStatusStruct& motor){

    int pos_int = data[0] << 8 | data[1];
    int vel_int = data[2] << 4 | data[3] >> 4;
    int kp_int = (data[3] & 0x0F) << 8 | data[4];
    int kd_int = data[5] << 4 | data[6] >> 4;
    int FF_int = (data[6] & 0x0F) << 8 | data[7];
    
    motor.p_des = uint_to_float(pos_int, P_MIN, P_MAX, 16);
    motor.v_des = uint_to_float(vel_int, V_MIN, V_MAX, 12);
    motor.kp = uint_to_float(kp_int, KP_MIN, KP_MAX, 12);
    motor.kd = uint_to_float(kd_int, KD_MIN, KD_MAX, 12);
    motor.t_ff = uint_to_float(FF_int, T_MIN, T_MAX, 12);
}


void enter_motor_mode(const char* interface_name, int id_can, MotorStatusStruct& motor){
    int nbytes;
    const char *ifname = interface_name;
    struct sockaddr_can addr;
    socklen_t len = sizeof(addr);
    struct ifreq ifr;
    strcpy(ifr.ifr_name, ifname);
    ioctl(s, SIOCGIFINDEX, &ifr);
    addr.can_family=AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    bind(s, (struct sockaddr *)&addr, sizeof(addr));
    
    //write CAN frame
    struct can_frame frame;

    frame.can_id  = id_can;          //motor id
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
    
    //get current motor pos, vel, current
    //nbytes = read(s, &frame, sizeof(struct can_frame)); 
    int nbytes2 = recvfrom(s, &frame, sizeof(struct can_frame), addr.can_ifindex, (struct sockaddr*)&addr, &len);
    
    std::cout << "nbytes2= " <<  nbytes2 << std::endl;

    std::vector<int> recv_data;
    for (int i = 0; i < sizeof(frame.data)/sizeof(frame.data[0]); i++){
        recv_data.push_back((int)frame.data[i]);
    }
    
    unpack_msg(recv_data, motor);
    
    std::cout << "pos= " << motor.p_des << " vel= " << motor.v_des << " kp= " << motor.kp << " kd= " << motor.kd << " FF= " << motor.t_ff << std::endl;
}


void exit_motor_mode(const char* interface_name, int id_can, MotorStatusStruct& motor){
        
    int nbytes;
    const char *ifname = interface_name;
    struct sockaddr_can addr;
    socklen_t len = sizeof(addr);
    struct ifreq ifr;
    strcpy(ifr.ifr_name, ifname);
    ioctl(s, SIOCGIFINDEX, &ifr);
    addr.can_family=AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    bind(s, (struct sockaddr *)&addr, sizeof(addr));
    
    //write CAN frame
    struct can_frame frame;
    frame.data[0] = 0xFF;
    frame.data[1] = 0xFF;
    frame.data[2] = 0xFF;
    frame.data[3] = 0xFF;
    frame.data[4] = 0xFF;
    frame.data[5] = 0xFF;
    frame.data[6] = 0xFF;
    frame.data[7] = 0xFD;
    
    nbytes = write(s, &frame, sizeof(struct can_frame));

    std::cout << "Motor Mode Disabled!" << std::endl;
    
    //get current motor pos, vel, current
    //nbytes = read(s, &frame, sizeof(struct can_frame)); 
    int nbytes2 = recvfrom(s, &frame, sizeof(struct can_frame), addr.can_ifindex, (struct sockaddr*)&addr, &len);
    //unpack_msg(frame, motor);
    std::vector<int> recv_data;
    for (int i = 0; i < sizeof(frame.data)/sizeof(frame.data[0]); i++){
        recv_data.push_back((int)frame.data[i]);
    }
    unpack_msg(recv_data, motor);
    std::cout << "pos= " << motor.p_des << " vel= " << motor.v_des << " kp= " << motor.kp << " kd= " << motor.kd << " FF= " << motor.t_ff << std::endl;
    
}


void zero_position_sensor(const char* interface_name, int id_can, MotorStatusStruct& motor){

    int nbytes;
    const char *ifname = interface_name;
    struct sockaddr_can addr;
    socklen_t len = sizeof(addr);
    struct ifreq ifr;
    strcpy(ifr.ifr_name, ifname);
    ioctl(s, SIOCGIFINDEX, &ifr);
    addr.can_family=AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    bind(s, (struct sockaddr *)&addr, sizeof(addr));
    
    //write CAN frame
    struct can_frame frame;

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
    
    //get current motor pos, vel, current
    //nbytes = read(s, &frame, sizeof(struct can_frame));
    int nbytes2 = recvfrom(s, &frame, sizeof(struct can_frame), addr.can_ifindex, (struct sockaddr*)&addr, &len); 
    //std::cout << "here1" << std::endl;
    //unpack_msg(frame, motor);
    
    std::vector<int> recv_data;
    for (int i = 0; i < sizeof(frame.data)/sizeof(frame.data[0]); i++){
        recv_data.push_back((int)frame.data[i]);
    }
    //std::cout << "here2" << std::endl;
    
    unpack_msg(recv_data, motor);
    std::cout << "pos= " << motor.p_des << " vel= " << motor.v_des << " kp= " << motor.kp << " kd= " << motor.kd << " FF= " << motor.t_ff << std::endl;
}



void write_can_frame(const char* interface_name, int id_can, int pos, int vel, int KP, int KD, int FF, MotorStatusStruct& motor){
       
    int nbytes;
    const char *ifname = interface_name;
    struct sockaddr_can addr;
    socklen_t len = sizeof(addr);
    struct ifreq ifr;
    strcpy(ifr.ifr_name, ifname);
    ioctl(s, SIOCGIFINDEX, &ifr);
    addr.can_family=AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    bind(s, (struct sockaddr *)&addr, sizeof(addr));
    
    //write CAN frame
    struct can_frame frame;

    frame.can_id  = id_can;          //motor id
    frame.can_dlc = 8;              //len of data 

    pack_msg(frame, pos, vel, KP, KD, FF);
    
    nbytes = write(s, &frame, sizeof(struct can_frame));

    
    std::cout << "data written: " << std::endl;
    
    //get current motor pos, vel, current
    //nbytes = read(s, &frame, sizeof(struct can_frame)); 
    int nbytes2 = recvfrom(s, &frame, sizeof(struct can_frame), addr.can_ifindex, (struct sockaddr*)&addr, &len);
    
    std::vector<int> recv_data;
    for (int i = 0; i < sizeof(frame.data)/sizeof(frame.data[0]); i++){
        recv_data.push_back((int)frame.data[i]);
    }
    unpack_msg(recv_data, motor);
    std::cout << "pos= " << motor.p_des << " vel= " << motor.v_des << " kp= " << motor.kp << " kd= " << motor.kd << " FF= " << motor.t_ff << std::endl;
}


//used to test functions above
/*
int main(){
    //setup CAN
    
    MotorStatusStruct motor1;
    
    const char *interf_name = "can0";           
    open_port(interf_name, 1000000);
    
    enter_motor_mode(interf_name, M1, motor1);
    zero_position_sensor(interf_name, M1, motor1);
    
    //writing CAN frame
    write_can_frame(interf_name, M1, 1, 1, 1, 1, 1, motor1);
    
    exit_motor_mode(interf_name, M1, motor1);
    
    close_port(interf_name);
    

    return 0;
}
*/
