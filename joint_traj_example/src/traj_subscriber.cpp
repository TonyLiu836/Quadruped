#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>

void msgReceived(const trajectory_msgs::JointTrajectory::ConstPtr& receivedMsg){
    //ROS_INFO_STREAM("received message(dereferenced): " << *receivedMsg);

    //ROS_INFO_STREAM("received joint positions:" << receivedMsg->points[0]);
    //ROS_INFO_STREAM("joint 1 pos:" << receivedMsg->points[0].positions[3]);
    //ROS_INFO_STREAM("joint 2 pos:" << receivedMsg->points[0][1]);
    //ROS_INFO_STREAM("joint 3 pos:" << receivedMsg->points[0][2]);
    //ROS_INFO_STREAM("received joint positions:" << receivedMsg->points[0][]);
    
     
    for(size_t i=0; i<12; ++i){
    //for(const auto& num :nums){     
       ROS_INFO_STREAM("joint= " << i << "  position= " << receivedMsg->points[0].positions[i]);
    
    } 
       
    
    ROS_INFO_STREAM("*****************");
    //ROS_INFO_STREAM("size of points array: " << receivedMsg->points);
}


int main(int argc, char** argv){
    ros::init(argc, argv, "subscriber_to_joint_traj");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("joint_group_position_controller/command",1000, &msgReceived);

    ros::spin();
}


