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

#define MIN_PULSE_WIDTH 500 //900                 //original is 900
#define MAX_PULSE_WIDTH 2500 //2100                //original is 2100

using namespace std;

int offset = 0;

//const char* joint_name[16] = {"back_left_foot", "back_left_hip", "back_left_lower_leg", "back_left_upper_leg", "back_right_foot","back_right_hip", "back_right_lower_leg", "back_right_upper_leg", "front_left_foot","front_left_hip", "front_left_lower_leg", "front_left_upper_leg", "front_right", "front_right_hip","front_right_lower_leg","front_right_upper_leg"};

//const char* joint_names[12] = {"motor_front_left_hip", "motor_front_left_upper_leg", "motor_front_left_lower_leg", "motor_front_right_hip", "motor_front_right_upper_leg", "motor_front_right_lower_leg", "motor_back_left_hip", "motor_back_left_upper_leg", "motor_back_left_lower_leg", "motor_back_right_hip", "motor_back_right_upper_leg", "motor_back_right_lower_leg"};

float pos[12];
float currPos[12];
//float vel[12];
//float eff[12];

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
      
      //pulse_wide = mapVals(angle,0,180,MIN_PULSE_WIDTH,MAX_PULSE_WIDTH);
      pulse_wide = mapVals(angle,0,180,MIN_PULSE_WIDTH,MAX_PULSE_WIDTH);
      analog_value = int(float(pulse_wide) /  1000000 * FREQUENCY * 4096);
      return (analog_value);
}

int convChamp2Servo(int angleChamp){
    int angleServo= int(angleChamp * 180 / 270);       //here angleChamp is the angle sent by champ (0~180), need to conv this to be in range (0~270)
    return angleServo;                                //convert angleChamp to the actual angle the servo (angleServo) will rotate to.
}

void pwmwrite(float& angle, PCA9685 pwm, size_t& channel){
//void pwmwrite(int angle, PCA9685 pwm, int& channel){
//void pwmwrite(int& angle, PCA9685 pwm, int& channel) {        original line that uses pass-by-reference for the angle
    int val = 0;
    
    if (angle > 180) {
       angle = 179;
    }
    /*
    if(angle > 270){
        angle = 269;    
    }
    */
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
    //cout << "Channel: " << channel << "\tSet to angle: " << angle << "\tVal: " << val << endl;
    //return(0);
}

float radToDeg(float rad){
    float pi = 3.1415926535;
    if (rad < 0){
        return float(-1 * rad * (180/pi));
    }

}

template<>
void Hw_interface<sensor_msgs::JointState, trajectory_msgs::JointTrajectory>::subscriberCallback(const trajectory_msgs::JointTrajectory::ConstPtr& receivedMsg){
//void Hw_interface<trajectory_msgs::JointTrajectory>::subscriberCallback(const trajectory_msgs::JointTrajectory::ConstPtr& receivedMsg){
    //set the current position to the newest incoming trajectory   
    joint_state.header.stamp = ros::Time::now();  

    //ROS_INFO_STREAM("joint name= " << receivedMsg->joint_names[i] << "angle= " << angleDeg);
    ROS_INFO_STREAM("received joint positions:" << receivedMsg->points[0]);
    for(size_t i=0; i < 12; ++i){
        currPos[i] = receivedMsg->points[0].positions[i];   
        float angleDeg = radToDeg(currPos[i]); 
        //ROS_INFO_STREAM("joint name= " << receivedMsg->joint_names[i] << "angle= " << angleDeg);

    }
    
    /*
    //command the 12 joints to move to desired positoins
    for (size_t ind=0; ind<12; ++ind){
        float angleServo;
        int angleDeg = radToDeg(currPos[ind]);


        if (ind == 1 || ind == 2|| ind == 4 || ind ==5){
            angleServo = convChamp2Servo(-1*angleDeg);        
        }
        else{
            angleServo = convChamp2Servo(angleDeg);
        }

       
        pwmwrite(angleServo, pwm, ind);
        pos[ind] = currPos[ind];
        //robot_state.name[ind] = receivedMsg->joint_names[ind];
        joint_state.position[ind] = pos[ind];
    }
    */
    
    for (size_t ind=0; ind<12; ++ind){
        float servoAngle; 
        int angleDeg = radToDeg(currPos[ind]);
        if ((ind<3) || (5<ind && ind<9)){                          //left legs
            //ROS_INFO_STREAM("****reaches here LEFT LEG****");
            if (ind==1 || ind==2 || ind==7 || ind==8){
                angleDeg = -1*angleDeg;
            }
            servoAngle = convChamp2Servo(angleDeg);
        }
        else{                                           //right legs
            //ROS_INFO_STREAM("****reaches here RIGHT LEG ****");
            
            /*            
            if (ind==5 || ind==11){                   //might need to include the upper leg joints as well (don't need to multiply leg angles by -1)
                //angleDeg = -1*angleDeg;
                angleDeg = angleDeg;
            }    
            */    
            servoAngle = convChamp2Servo(int(180 - angleDeg));   
        }

        pwmwrite(servoAngle, pwm, ind);
        pos[ind] = currPos[ind];
        joint_state.position[ind] = pos[ind];
    
    }
    //ROS_INFO_STREAM("*****************");    
    publisherObj.publish(joint_state);      
}



int main(int argc, char** argv){
    ros::init(argc, argv, "hw_interface");
    
    Hw_interface<sensor_msgs::JointState, trajectory_msgs::JointTrajectory> hwInterface("joint_states", "joint_group_position_controller/command", 100);
    //Hw_interface<trajectory_msgs::JointTrajectory> hwInterface("joint_state", "joint_group_position_controller/command", 1000);
    ros::spin();
}

