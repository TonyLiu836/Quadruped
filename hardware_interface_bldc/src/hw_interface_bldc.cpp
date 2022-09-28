#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>

#include <iostream>
#include <stdio.h>
#include <thread>         
#include <vector>         
#include <unistd.h>
#include <string>
#include <string.h>

#include <std_msgs/String.h>
#include "../include/CANCommLib/CAN_comm.h"
#include "../include/MathOpsLib/math_ops.h"

/*
    Subscribe to trajectory_msgs/JointTrajectory . This ROS message contains the joint angles (12DOF) that the actuators can use to move the robot.

    Publish all the actuators' current angle using sensor_msgs/JointState to 'joint_states' topic.

    Control the actuators and read its angle (optional) programmatically.
*/


/*
struct MotorStatusStruct
{
    std::string name;
    int motor_id;
    int p_des, v_des, kp,kd,t_ff;
};
*/

std::vector<MotorStatusStruct> motorStatus;

/* may need it later 
float radToDeg(float rad){
    float pi = 3.1415926535;

    return float(rad * (180/pi));
}
*/


//callback from subscriber
void receivedMsg(const trajectory_msgs::JointTrajectory::ConstPtr& receivedMsg){
    //assume position is in radians for now (9.27.2022)
    for (int i = 0; i<12; i++){
        motorStatus[i].p_des = receivedMsg->points[0].positions[i];
        motorStatus[i].v_des = receivedMsg->points[0].velocities[i];
        motorStatus[i].t_ff = receivedMsg->points[0].effort[i];
    }
    
    
}

//initialize all motors - assigns CAN interface, motor id, and enters motor mode
void setupMotors(){
    for (int i = 0; i < 12; ++i){
        motorStatus.push_back(MotorStatusStruct());
        motorStatus[i].name = "M"+ std::to_string(i);          //set name of motor to be M1, M2,etc
        
        const char *interf_name;
        switch(i)                                 //each leg uses a different CAN interface 
        {
            case 0 ... 2:
            {
                interf_name = "can0";
            }
            case 3 ... 5:
            {
                interf_name = "can1";
            }  
            case 6 ... 8:
            {
                interf_name = "can2";
            }
            case 9 ... 11:
            { 
                interf_name = "can3";
            }
        }
        
        motorStatus[i].motor_id = (int)((i%3) + 1);
        
        open_port(interf_name, 1000000);
        
        //the pos, vel, torque initial values should be found here
        enter_motor_mode(interf_name, motorStatus[i].motor_id, motorStatus[i]);
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "hw_interface_bldc");
    ros::NodeHandle nh;
    
    std::vector<std::string> CAN_interfs = {"can0", "can1", "can2", "can3"};
    
    sensor_msgs::JointState joint_state;
    joint_state.position.resize(12);
    joint_state.velocity.resize(12);
    joint_state.effort.resize(12);              //same as t_ff? 
    joint_state.name = {"motor_front_left_hip", "motor_front_left_upper_leg", "motor_front_left_lower_leg", "motor_front_right_hip", "motor_front_right_upper_leg", "motor_front_right_lower_leg", "motor_back_left_hip", "motor_back_left_upper_leg", "motor_back_left_lower_leg", "motor_back_right_hip", "motor_back_right_upper_leg", "motor_back_right_lower_leg"};
    
    ros::Publisher pub = nh.advertise<sensor_msgs::JointState>("joint_states", 100);
    ros::Subscriber sub = nh.subscribe<trajectory_msgs::JointTrajectory>("joint_group_position_controller/command", 100, &receivedMsg);
    
    setupMotors();

    while(ros::ok()){

        joint_state.header.stamp = ros::Time::now();
        for (int i=0; i<12; ++i){                       //update joint_state with current state of robot (pos, vel, torque)
            joint_state.position[i] = motorStatus[i].p_des;
            joint_state.velocity[i] = motorStatus[i].v_des;
            joint_state.effort[i] = motorStatus[i].t_ff;
        }
        pub.publish(joint_state);

        //ros::spin();
    }
}

