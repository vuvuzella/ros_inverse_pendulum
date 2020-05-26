#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/rate.h"
#include "ros/ros.h"
#include "inverse_pendulum_control/demo_srv.h"
#include "ros/service_client.h"
#include "ros/service_server.h"
#include <iostream>
#include <sstream>

using namespace std;

int main(int argc, char** argv) {

    ros::init(argc, argv, "demo_service_client");

    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    ros::ServiceClient client = n.serviceClient<inverse_pendulum_control::demo_srv>("demo_service");

    while (ros::ok()) {
        inverse_pendulum_control::demo_srv srv;
        std::stringstream ss;
        ss << "Sending Here";
        srv.request.in = ss.str();

        if (client.call(srv)) {
            ROS_INFO("From Client [%s], Server says [%s]", srv.request.in.c_str(), srv.response.out.c_str());
        } else {
            ROS_ERROR("Failed to call Service");
            return 1;
        }

        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;

}
