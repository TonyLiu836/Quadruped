#ifndef HW_INTERFACE_H
#define HW_INTERFACE_H

#include<ros/ros.h>
#include<string>
#include<trajectory_msgs/JointTrajectory.h>
#include<sensor_msgs/JointState.h>

//extern PCA9685 pwm;

template<typename PublishT, typename SubscribeT>  //declare types for the messages of the publisher and subscriber

class Hw_interface{
public:
    Hw_interface(){}
    Hw_interface(sensor_msgs::JointState publishTopicName, trajectory_msgs::JointTrajectory subscribeTopicName, int queueSize){
        publisherObj = nH.advertise<PublishT>(publishTopicName, queueSize);
        subscriberObj = nH.subscribe<SubscribeT>(subscribeTopicName, queueSize, &Hw_interface::subscriberCallback, this);   
    }    
    void subscriberCallback(const typename SubscribeT::ConstPtr& receivedMsg);

protected:
    ros::Subscriber subscriberObj;
    ros::Publisher publisherObj;
    ros::NodeHandle nH;

};

#endif
