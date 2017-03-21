#include <stdio.h>

#include <ros/ros.h>
#include "react_controller/ctrlThread.h"

using namespace std;

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "baxter_react_controller");
    ros::NodeHandle _n("baxter_react_controller");

    bool use_robot;
    _n.param<bool>("use_robot", use_robot, true);
    printf("\n");
    ROS_INFO("use_robot flag set to %s", use_robot==true?"true":"false");

    printf("\n");
    CtrlThread arm("baxter_react_controller", "right", !use_robot, "base", "right_gripper");
    printf("\n");

    if (use_robot == false)
    {
        ROS_INFO("Debug mode enabled! Closing.");
        return 0;
    }

    ROS_INFO("READY! Waiting for control messages..\n");
    ros::spin();

    return 0;
}

