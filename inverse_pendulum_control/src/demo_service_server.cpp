#include "ros/node_handle.h"
#include "ros/ros.h"
#include "inverse_pendulum_control/demo_srv.h"
#include "ros/service_server.h"
#include <iostream>
#include <sstream>

using namespace std;

bool demo_service_callback(inverse_pendulum_control::demo_srv::Request &req,
        inverse_pendulum_control::demo_srv::Response &res) {
    std::stringstream ss;
    ss << "Received Here";
    res.out = ss.str();

    ROS_INFO("From Client [%s], server saya[%s]", req.in.c_str(), res.out.c_str());
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "demo_service_server");
    ros::NodeHandle n;
    
    ros::ServiceServer service = n.advertiseService("demo_service", demo_service_callback);
    ROS_INFO("Ready to receive from client");
    ros::spin();

    return 0;
}
