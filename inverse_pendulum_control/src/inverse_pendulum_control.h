#include "ros/node_handle.h"
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

namespace inverse_pendulum_control_ns {

    class InversePendulumControlClass : public controller_interface::Controller<hardware_interface::PositionJointInterface> {
        private:
            hardware_interface::JointHandle joint_;
            double init_pos;
        public:
            bool init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle& n);
            void update(const ros::Time& time, const ros::Duration& period);
            void starting(const ros::Time& time);
            void stopping(const ros::Time& time);

    };

};
