#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include "actionlib/client/simple_client_goal_state.h"
#include <actionlib/client/terminal_state.h>
#include "inverse_pendulum_control/Demo_actionAction.h" // Generated at compile-time
#include <iostream>
#include <sstream>

int main(int argc, char** argv) {
    ros::init(argc, argv, "demo_action_client");

    if (argc != 3) {
        ROS_INFO("%d", argc);
        ROS_WARN("Usage: demo_action_client <goal> <time_to_preempt_in_sec");
        return 1;
    }

    // create action client
    // true causes the client to spin its own thread
    actionlib::SimpleActionClient<inverse_pendulum_control::Demo_actionAction> ac("demo_action", true);

    ROS_INFO("Waiting for action server to start."); 

    ac.waitForServer();

    ROS_INFO("Action server started, sending goal.");

    inverse_pendulum_control::Demo_actionGoal goal;
    goal.count = atoi(argv[1]);

    ROS_INFO("Sending Goal [%d] and Preempt time of [%d]", goal.count, atoi(argv[2]));
    ac.sendGoal(goal);

    // wait for action to return
    bool finished_before_timeout = ac.waitForResult(ros::Duration(atoi(argv[2])));
    // preempt task
    ac.cancelGoal();

    if (finished_before_timeout) {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
        // preempting the process
        ac.cancelGoal();
    } else {
        ROS_INFO("Action did not finish before timeout.");
    }
    return 0;
}
