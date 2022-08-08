//hardware interface between RPi and PCA9685 to control servo motors

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
//#include <PiPCA9685/PCA9685.h>

// Includes for PCA9685
#include "../PCA9685Lib/PCA9685.h"
#include <iostream>
#include <stdio.h>
#include <thread>         
#include <vector>         
#include <unistd.h>
#include <string>
#include <string.h>

#include <ros/ros.h>
#include <hw_interface.h>
#include <std_msgs/String.h>


#define MIN_PULSE_WIDTH 900
#define MAX_PULSE_WIDTH 2100
#define FREQUENCY 50

using namespace std;

int offset = 0;

//motor channels
int ch0 = 0;
int ch1 = 1;
int ch2 = 2;
int ch3 = 3;
int ch4 = 4;
int ch5 = 5;
int ch6 = 6;
int ch7 = 7;
int ch8 = 8;
int ch9 = 9;
int ch10 = 10;
int ch11 = 11;

//robot joint state
sensor_msgs::JointState robot_state;

int actuator;
const char* joint_name[12] = {"back_left_hip", "back_left_lower_leg", "back_left_upper_leg", "back_right_hip", "back_right_lower_leg", "back_right_upper_leg", "front_left_hip", "front_left_lower_leg", "front_left_upper_leg", "front_right_hip","front_right_lower_leg","front_right_upper_leg"};
float pos[12];
double currPos[12];
float vel[12];
float eff[12];



//Declaration of Functions used ==================================
void pwmwrite(int& angle, PCA9685 pwm, int& channel);


//def map(self, x, in_min, in_max, out_min, out_max):
int mapVals(int x, int in_min, int in_max, int out_min, int out_max) {
        return ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

//def _angle_to_analog(self, angle):
int angleToAnalog(int angle) {
      float pulse_wide;
      int analog_value;
      
      pulse_wide = mapVals(angle,0,180,MIN_PULSE_WIDTH,MAX_PULSE_WIDTH);
      analog_value = int(float(pulse_wide) /  1000000 * FREQUENCY * 4096);
      return (analog_value);
}

void pwmwrite(int& angle, PCA9685 pwm, int& channel) {
    int val = 0;

    if (angle > 180) {
       angle = 179;
    }
    if (angle < 0) {
       angle = 1;
    }
    
    val = angleToAnalog(angle);
    //not sure what offset does
    val += offset;

    //setPWM(self, channel, on, off
    //channel: The channel that should be updated with the new values (0..15)
    //on: The tick (between 0..4095) when the signal should transition from low to high
    //off:the tick (between 0..4095) when the signal should transition from high to low
    
    pwm.setPWM(channel,0,val);
    //usleep(30);
    cout << "Channel: " << channel << "\tSet to angle: " << angle << "\tVal: " << val << endl;
    //return(0);
}


//void anglesReceived(const trajectory_msgs::JointTrajectory& msg){
//}

/*
int main(int argc, char** argv){
    ros::init(argc, argc, "hw_interface");
    ros::NodeHandle nh;
    
    //publisher that sends current joint angles
    ros::publisher pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1000);

    //subscriber, receives joint angles used to move robot 
    ros::Subscriber sub = nh.subscribe("", 1000)
    
    
    //ros::spin();
    while(ros::ok()){
        
        
        ros::spinOnce()
        pub.publish()
    }
}
*/


template<>
void Hw_interface<sensor_msgs::JointState, trajectory_msgs::JointTrajectory>::subscriberCallback(const trajectory_msgs::JointTrajectory::ConstPtr& receivedMsg){
    ROS_INFO("I received the following: %s", receivedMsg->joint_names);
    ROS_INFO("Sending the received message on 'joint state' topic");
    //std_msgs::String echo_msg;    
    //echo_msg.data = receivedMsg->data;
    //publisherObj.publish(echo_msg);
    
    PCA9685 pwm;
    pwm.init(1,0x40);
    pwm.setPWMFreq(FREQUENCY); 
    //trajectory_msgs::JointTrajectory robot_traj = receivedMsg->data;
    
    //set the current position to the newest incoming trajectory 
    //currPos = receivedMsg->joint_trajectory->points->positions;
    for(int i=0; i < receivedMsg->points.size(); i++){
        currPos[i] = receivedMsg->points[i].positions[0];
    }
    //currPos = receivedMsg->points;
    
    //command the 12 joints to move to desired positoins
    for (int ind=0; ind<12; ind++){
        pwmwrite(currPos[ind], pwm, ind);
        pos[ind] = currPos[ind];
    }
    
    //robot_state.name_length = 12;
    //robot_state.velocity_length = 12;
    //robot_state.effort_length = 12;

    //robot_state.header.stamp = nH.now();
    //robot_state.header.frame_id = "dog";
    
    for(int ind=0; ind <12; ind++){
        robot_state.name[ind] = joint_name[ind];
        robot_state.position[ind] = pos[ind];
        robot_state.velocity[ind] = vel[ind];
        robot_state.effort[ind] = eff[ind];
    }
    
    publisherObj.publish(robot_state);
    
}


int main(int argc, char** argv){
    ros::init(argc, argv, "publisher_subscriber_demo");

    Hw_interface<std_msgs::String, std_msgs::String> hwInterface("joint_states", "joint_group_position_controller/command", 1000);
    ros::spin();
}

