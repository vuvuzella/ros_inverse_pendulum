#include "ros/init.h"
#include "ros/ros.h"
#include "ros/subscriber.h"
// #include "std_msgs/Int32.h"
#include "inverse_pendulum_control/demo_msg.h"
#include <iostream>

// void number_callback(const std_msgs::Int32::ConstPtr& msg) {
void number_callback(const inverse_pendulum_control::demo_msg::ConstPtr& msg) {
    ROS_INFO("Received [%d], [%s]", msg->data, msg->greeting.c_str());
}

int main (int argc, char** argv) {
    ros::init(argc, argv, "demo_msg_subscriber");
    ros::NodeHandle node_obj;
    ros::Subscriber number_subscriber = node_obj.subscribe("/demo_msg", 10, number_callback);
    ros::spin();
    return 0;
}

