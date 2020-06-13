#include "pid_inverse_pendulum.h"
#include "ros/forwards.h"
#include "ros/time.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"

PIDInversePendulum::PIDInversePendulum()
    : imuX(0)
    , imuY(0)
    , imuZ(0)
    , imuW(0)
    , conPos(0)
    , conVel(0)
    , conEffort(0)
    , jointName("")
    , pGain(3.5)  // Proportional-only controller
    , iGain(0)
    , dGain(0)
    , error(0) {
}
PIDInversePendulum::~PIDInversePendulum() {
}

// Read data from joint controllers/sensors
void PIDInversePendulum::jointStateCallback(const sensor_msgs::JointState& msg) {
    // link index:
    // 0 - connector_to_arm link
    // 1 - foundation_to_connector
    // int ctaIndex = 0;
    int ftcIndex = 1;
    conPos = msg.position[ftcIndex];
    conVel = msg.velocity[ftcIndex];
    conEffort = msg.effort[ftcIndex];
    jointName = msg.name[ftcIndex];

    // armPos = msg.position[ctaIndex];
    // armVel = msg.velocity[ctaIndex];
    // armEffort = msg.effort[ctaIndex];
}

// Read data from the IMU
void PIDInversePendulum::imuCallback(const sensor_msgs::Imu& msg) {
    // geometry_msgs::Quaternion q_msg = msg.orientation;
    // ROS_INFO("Received x:[%f] y:[%f] z:[%f]", msg.orientation.x, msg.orientation.y, msg.orientation.z);
    imuX = msg.orientation.x;
    imuY = msg.orientation.y;
    imuZ = msg.orientation.z;
    imuW = msg.orientation.w;
}

// update the PID
// Control the imuX axis
double PIDInversePendulum::updatePID(double goal) {
    double newOffset = 0;
    double pTerm = 0;

    // calculate new Error
    error += goal - imuX;

    //Proportional term
    pTerm = pGain * error;

    newOffset = pTerm;
    return newOffset;
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "pid_inverse_pendulum");
    ros::NodeHandle node;

    PIDInversePendulum control;

    // TODO: integrate these inside PIDInversePendulum Class
    // Subscribe to IMU
    ros::Subscriber imu = node.subscribe("/imu", 10, &PIDInversePendulum::imuCallback, &control);

    // Subscribe to /inverse_pendulum_control/joint_states
    ros::Subscriber jointState = node.subscribe("/inverse_pendulum_control/joint_states", 10, &PIDInversePendulum::jointStateCallback, &control);

    // Publish to /inverse_pendulum_control/joint1_position_controller/command
    ros::Publisher joint1Pub = node.advertise<std_msgs::Float64>("/inverse_pendulum_control/joint1_position_controller/command", 10);

    ros::Rate loopRate(10);

    double goalPosition = 0;

    ROS_INFO("Starting...");
    while (ros::ok()) {

        if (!control.jointName.empty()) {
            std_msgs::Float64 msg;
            double driveValue = control.updatePID(goalPosition);
            msg.data = driveValue;

            joint1Pub.publish(msg);

            ROS_INFO("Error: [%f], imuX: [%f], imuY: [%f], imuZ[%f]", driveValue, control.imuX, control.imuY, control.imuZ);
        }

        ros::spinOnce();
        loopRate.sleep();
    }

}
