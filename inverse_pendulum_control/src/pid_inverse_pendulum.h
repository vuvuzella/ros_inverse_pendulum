#include "ros/init.h"
#include "ros/node_handle.h"
#include <ros/ros.h>
#include "ros/subscriber.h"
#include "ros/rate.h"
#include "sensor_msgs/JointState.h"
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <string.h>
#include <std_msgs/Float64.h>

class PIDInversePendulum {

    public:

        double imuX;
        double imuY;
        double imuZ;
        double imuW;

        // double armPos;
        // double armVel;
        // double armEffort;
    
        double conPos;
        double conVel;
        double conEffort;
        std::string jointName;
    
        double pGain;
        double iGain;
        double dGain;

        double error;

        PIDInversePendulum();
        ~PIDInversePendulum();

        // Callback functions for subscribers
        void jointStateCallback(const sensor_msgs::JointState& msg);
        void imuCallback(const sensor_msgs::Imu& msg);

        // TODO: functions to calculate desired arm tilt
        // TODO: the PID functions to maintain the arm tilt
    
        double updatePID(double goal);
};
