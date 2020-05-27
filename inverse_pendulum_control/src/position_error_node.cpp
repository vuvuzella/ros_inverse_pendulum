#include "ros/init.h"
#include "ros/node_handle.h"
#include <ros/ros.h>
#include "ros/subscriber.h"
#include "ros/rate.h"
#include "sensor_msgs/JointState.h"

class PositionErrorNode {

    public:
        double armPos;
        double armVel;
        double armEffort;

        double conPos;
        double conVel;
        double conEffort;

        const static double ARM_CENTER = 0;

        PositionErrorNode() {
        }
        ~PositionErrorNode() {
        }

        void jointStateCallback(const sensor_msgs::JointState& msg) {
            ROS_INFO("name: [%s]", msg.name.back().c_str());
            for (int i = 0; i < (int)msg.name.size(); i++) {
                if (msg.name[i] == "connector_to_arm") {
                    armPos = msg.position[i];
                    armVel = msg.velocity[i];
                    armEffort = msg.effort[i];
                } else if (msg.name[i] == "foundation_to_connector") {
                    conPos = msg.position[i];
                    conVel = msg.velocity[i];
                    conEffort = msg.effort[i];
                } else {
                    // not concerned with this JointState
                }
            }
        }

        double getArmError() {
            return ARM_CENTER - armPos;
        }
};

int main(int argc, char** argv) {

    ros::init(argc, argv, "position_error_publisher");
    ros::NodeHandle node;

    PositionErrorNode n;

    // Subscribe to /joint_state
    ros::Subscriber jointStateSubscription = node.subscribe("/inverse_pendulum_control/joint_states", 10, &PositionErrorNode::jointStateCallback, &n);

    // Publish the errors
    // ros::Publisher errorPublisher = node.advertise<sensor_msgs::JointState>("/position_error", 10);

    ros::Rate loopRate(10);

    while (ros::ok()) {
        ROS_INFO("arm difference: [%f]", n.getArmError());
        ros::spinOnce();
        loopRate.sleep();
    }
    return 0;
}
