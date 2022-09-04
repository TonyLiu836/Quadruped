#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
//#include <PiPCA9685/PCA9685.h>

// Includes for PCA9685
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

#define MIN_PULSE_WIDTH 900                 //original is 900
#define MAX_PULSE_WIDTH 2100                //original is 2100

using namespace std;

int offset = 0;

float pos[12];
float currPos[12];

//Declaration of Functions used ==================================
void pwmwrite(int& angle, PCA9685 pwm, size_t& channel);


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

void pwmwrite(float& angle, PCA9685 pwm, size_t& channel){
//void pwmwrite(int angle, PCA9685 pwm, int& channel){
//void pwmwrite(int& angle, PCA9685 pwm, int& channel) {        original line that uses pass-by-reference for the angle
    int val = 0;

    if (angle > 180) {
       angle = 179;
    }
    if (angle < 0) {
       angle = 1;
    }
    
    val = angleToAnalog(angle);
    val += offset;

    //setPWM(self, channel, on, off
    //channel: The channel that should be updated with the new values (0..15)
    //on: The tick (between 0..4095) when the signal should transition from low to high
    //off:the tick (between 0..4095) when the signal should transition from high to low
    pwm.setPWM(channel,0,val);
}

float radToDeg(float rad){
    float pi = 3.1415926535;
    return (rad * (180/pi));
}

template<>
void Hw_interface<sensor_msgs::JointState, trajectory_msgs::JointTrajectory>::subscriberCallback(const trajectory_msgs::JointTrajectory::ConstPtr& receivedMsg){
//void Hw_interface<trajectory_msgs::JointTrajectory>::subscriberCallback(const trajectory_msgs::JointTrajectory::ConstPtr& receivedMsg){
    //set the current position to the newest incoming trajectory   
    joint_state.header.stamp = ros::Time::now();  
    for(size_t i=0; i < 12; ++i){
        currPos[i] = receivedMsg->points[0].positions[i];    
    }
   
    //command the 12 joints to move to desired positoins
    for (size_t ind=0; ind<12; ++ind){
        float angleDeg = radToDeg(currPos[ind]);
        pwmwrite(angleDeg, pwm, ind);
        pos[ind] = currPos[ind];
        joint_state.position[ind] = pos[ind];
    }

    publisherObj.publish(joint_state);      
}

int main(int argc, char** argv){
    ros::init(argc, argv, "hw_interface");
    Hw_interface<sensor_msgs::JointState, trajectory_msgs::JointTrajectory> hwInterface("joint_states", "joint_group_position_controller/command", 100);
    ros::spin();
}

