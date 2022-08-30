#ifndef HW_INTERFACE_H
#define HW_INTERFACE_H
#define FREQUENCY 50

#include<ros/ros.h>
#include<string>
#include<trajectory_msgs/JointTrajectory.h>
#include<sensor_msgs/JointState.h>
#include "../PCA9685Lib/PCA9685.h"
//extern PCA9685 pwm;

template<typename PublishT, typename SubscribeT>  //declare types for the messages of the publisher and subscriber
//template<typename SubscribeT>

class Hw_interface{
public:
    Hw_interface(){
        
    }
    //Hw_interface(sensor_msgs::JointState publishTopicName, trajectory_msgs::JointTrajectory subscribeTopicName, int queueSize){
    Hw_interface(std::string publishTopicName, std::string subscribeTopicName, int queueSize)
    //HW_interface(std::string subscribeTopicName, int queueSize)
    {
        publisherObj = nH.advertise<PublishT>(publishTopicName, queueSize);
        subscriberObj = nH.subscribe<SubscribeT>(subscribeTopicName, queueSize, &Hw_interface::subscriberCallback, this);   
        //jointStatePublisher = nH.advertise<sensor_msgs::JointState>("Joint",100);

        joint_state.position.resize(12);
        joint_state.velocity.resize(12);
        joint_state.effort.resize(12);
        //joint_state.name.resize(12);
        //joint_state.name = {"motor_front_left_hip", "motor_front_left_upper_leg", "motor_front_left_lower_leg", "motor_front_right_hip", "motor_front_right_upper_leg", "motor_front_right_lower_leg", "motor_back_left_hip", "motor_back_left_upper_leg", "motor_back_left_lower_leg", "motor_back_right_hip", "motor_back_right_upper_leg", "motor_back_right_lower_leg"};

        //joint_state.name = {"motor_front_left_hip","motor_front_left_upper_leg", "motor_front_left_lower_leg","motor_back_left_hip", "motor_back_left_upper_leg", "motor_back_left_lower_leg", "motor_front_right_hip", "motor_front_right_upper_leg", "motor_front_right_lower_leg", "motor_back_right_hip", "motor_back_right_upper_leg", "motor_back_right_lower_leg"};
        
        //this is the order listed in Open_quadruped repository
        joint_state.name = {"motor_front_left_hip", "motor_front_left_upper_leg", "motor_front_left_lower_leg", "motor_front_right_hip", "motor_front_right_upper_leg", "motor_front_right_lower_leg", "motor_back_left_hip", "motor_back_left_upper_leg", "motor_back_left_lower_leg", "motor_back_right_hip", "motor_back_right_upper_leg", "motor_back_right_lower_leg"};
        pwm.init(1, 0x40);
        pwm.setPWMFreq(FREQUENCY);
    }
    void subscriberCallback(const typename SubscribeT::ConstPtr& receivedMsg);

protected:
    ros::Subscriber subscriberObj;
    ros::Publisher publisherObj;
    //ros::Publisher jointStatePublisher;    
    ros::NodeHandle nH;
    sensor_msgs::JointState joint_state;
    PCA9685 pwm;
};

#endif
