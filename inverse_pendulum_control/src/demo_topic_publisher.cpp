#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/rate.h"
#include "ros/ros.h"
// #include "std_msgs/Int32.h"
#include "inverse_pendulum_control/demo_msg.h"
#include <iostream>
#include <sstream>

int main(int argc, char** argv) {

    ros::init(argc, argv, "demo_msg_publisher");
    ros::NodeHandle node_obj;
    // ros::Publisher number_publisher = node_obj.advertise<std_msgs::Int32>("/numbers", 10);
    ros::Publisher number_publisher = node_obj.advertise<inverse_pendulum_control::demo_msg>("/demo_msg", 10);
    ros::Rate loop_rate(10);
    int number_count = 0;
    while (ros::ok()) {
        // std_msgs::Int32 msg;
        inverse_pendulum_control::demo_msg msg;
        // msg.data = number_count;
        std::stringstream ss;
        ss << "Hello World";
        msg.greeting = ss.str();
        msg.data = number_count;

        ROS_INFO("%s", msg.greeting.c_str());
        ROS_INFO("%d", msg.data);

        number_publisher.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
        ++number_count;
    }
    return 0;
}
