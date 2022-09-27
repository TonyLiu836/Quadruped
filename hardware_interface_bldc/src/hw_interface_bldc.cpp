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

#include <ros/ros.h>
#include <hw_interface.h>
#include <std_msgs/String.h>


/*
    Subscribe to trajectory_msgs/JointTrajectory . This ROS message contains the joint angles (12DOF) that the actuators can use to move the robot.

    Publish all the actuators' current angle using sensor_msgs/JointState to 'joint_states' topic.

    Control the actuators and read its angle (optional) programmatically.
*/

sensor_msgs::JointState joint_state;
joint_state.position.resize(12);
joint_state.velocity.resize(12);
joint_state.effort.resize(12);              //same as t_ff? 

struct MotorStatusStruct
{
    std::string name;
    int p_des, v_des, kp,kd,t_ff;
};

/*
float radToDeg(float rad){
    float pi = 3.1415926535;

    return float(rad * (180/pi));
}
*/


void receivedMsg(const trajectory_msgs::JointTrajectory::ConstPtr& receivedMsg){
    //assume position is in radians for now (9.27.2022)
    for (int i = 0; i<12; i++){
        motorStatus[i].p_des = receivedMsg->points[0].positions[i];
        motorStatus[i].v_des = receivedMsg->velocities[i];
        motorStatus[i].t_ff = receivedMsg->effort[i];
    }
    
    
}


int main(int argc, char** argv){
    ros::init(argc, argv, "hw_interface_bldc");
    ros::NodeHandle nh;
    
    ros::Publisher pub = nh.advertise<sensor_msgs::JointState>("joint_states", 100);
    ros::Subscriber sub = nh.subscribe<trajectory_msgs::JointTrajectory>("joint_group_position_controller/command", 100);

    std::vector<MotorStatusStruct> motorStatus;
    for (int i = 0; i < 12; ++i){
        motorStatus.push_back(MotorStatusStruct());
        motorStats[i].name = "M"+str(i);                     //set name of motor immediately
    }

    while(ros::ok()){
        

        joint_state.header.stamp = ros::Time::now();
        for (int i=0, i<12; ++i){                       //update joint_state with current state of robot (pos, vel, torque)
            joint_state.position[i] = motorStatus[i].p_des;
            joint_state.velocity[i] = motorStatus[i].v_des;
            joint_state.effort[i] = motorStatus[i].t_ff;
        }
        pub.publish(joint_state);

        //ros::spin();
    }
}

