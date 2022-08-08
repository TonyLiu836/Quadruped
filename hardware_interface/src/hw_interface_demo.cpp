#include <ros/ros.h>
#include <hw_interface.h>
#include <std_msgs/String.h>

template<>
void Hw_interface<std_msgs::String, std_msgs::String>::subscriberCallback(const std_msgs::String::ConstPtr& receivedMsg){
    ROS_INFO("I received the following: %s", receivedMsg->data.c_str());
    ROS_INFO("Sending the received message on 'echo' topic");
    std_msgs::String echo_msg;
    echo_msg.data = receivedMsg->data;
    publisherObj.publish(echo_msg);
}


int main(int argc, char** argv){
    ros::init(argc, argv, "publisher_subscriber_demo");

    Hw_interface<std_msgs::String, std_msgs::String> parrot("echo", "chatter", 1);
    ros::spin();

}
