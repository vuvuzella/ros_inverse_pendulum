#include "inverse_pendulum_control.h"
#include "hardware_interface/joint_command_interface.h"
#include "ros/node_handle.h"

namespace inverse_pendulum_control_ns {

    // Controller Initialization
    bool InversePendulumControlClass::init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle& nh) {

        // Retrieve the joint object to control from joint_param key
        std::string jointName;
        if (!nh.getParam("joint_name", jointName)) {
            ROS_ERROR("No joint_name specified");
            return false;
        }
        joint_ = hw->getHandle(jointName);
        return true;
    }

    // Controller Startup
    void InversePendulumControlClass::starting(const ros::Time& time) {
        // Get initial position to use in the control procedure
        init_pos = joint_.getPosition();
    }

    // Controller running
    void InversePendulumControlClass::update(const ros::Time& time, const ros::Duration& period) {
        // TODO: Confirm this action!
        // Perform sinusoidal motion for joind shoulder_pan_joint ?????
        double dpos = init_pos + 10 * sin(ros::Time::now().toSec());
        double cpos = joint_.getPosition();
        // Apply command to the selected joint
        joint_.setCommand(-10 * (cpos - dpos));
        ROS_INFO("Current position[%f]", cpos);
    }

    // Controller exitin
    void InversePendulumControlClass::stopping(const ros::Time& time) {
        // currently empty
    }
};

// Register the plugin:
// PLUGINLIG_EXPORT_CLASS(InversePendulumControlClass_ns::MyPlugin, base_class_namespace::PluginBaseClass)
PLUGINLIB_EXPORT_CLASS(inverse_pendulum_control_ns::InversePendulumControlClass, controller_interface::ControllerBase);



// #include "ros/init.h"
// #include "ros/node_handle.h"
// #include <ros/ros.h>
// #include "ros/subscriber.h"
// #include "ros/rate.h"
// #include "sensor_msgs/JointState.h"

// class InversePendulumControlClass {
// 
//     public:
//         double armPos;
//         double armVel;
//         double armEffort;
// 
//         double conPos;
//         double conVel;
//         double conEffort;
// 
//         const static double ARM_CENTER = 0;
// 
//         InversePendulumControlClass() {
//         }
//         ~InversePendulumControlClass() {
//         }
// 
//         void jointStateCallback(const sensor_msgs::JointState& msg) {
//             ROS_INFO("name: [%s]", msg.name.back().c_str());
//             for (int i = 0; i < (int)msg.name.size(); i++) {
//                 if (msg.name[i] == "connector_to_arm") {
//                     armPos = msg.position[i];
//                     armVel = msg.velocity[i];
//                     armEffort = msg.effort[i];
//                 } else if (msg.name[i] == "foundation_to_connector") {
//                     conPos = msg.position[i];
//                     conVel = msg.velocity[i];
//                     conEffort = msg.effort[i];
//                 } else {
//                     // not concerned with this JointState
//                 }
//             }
//         }
// 
//         double getArmError() {
//             return ARM_CENTER - armPos;
//         }
// };
// 
// int main(int argc, char** argv) {
// 
//     ros::init(argc, argv, "inverse_pendulum_controller");
//     ros::NodeHandle node;
// 
//     InversePendulumControlClass n;
// 
//     // Subscribe to /joint_state
//     ros::Subscriber jointStateSubscription = node.subscribe("/inverse_pendulum_control/joint_states", 10, &InversePendulumControlClass::jointStateCallback, &n);
// 
//     // Publish the errors
//     // ros::Publisher errorPublisher = node.advertise<sensor_msgs::JointState>("/position_error", 10);
// 
//     ros::Rate loopRate(10);
// 
//     while (ros::ok()) {
//         ROS_INFO("arm difference: [%f]", n.getArmError());
//         ros::spinOnce();
//         loopRate.sleep();
//     }
//     return 0;
// }
