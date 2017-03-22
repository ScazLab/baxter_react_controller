#include <stdio.h>

#include <ros/ros.h>
#include "react_controller/ctrlThread.h"

using namespace std;

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "baxter_react_controller");
    ros::NodeHandle _n("baxter_react_controller");

    printf("\n");
    bool use_robot;
    _n.param<bool>("use_robot", use_robot, true);
    ROS_INFO("use_robot flag set to %s", use_robot==true?"true":"false");
    bool is_debug;
    _n.param<bool>("is_debug", is_debug, false);
    ROS_INFO("is_debug flag set to %s", is_debug==true?"true":"false");

    printf("\n");
    CtrlThread arm("baxter_react_controller", "right", !use_robot, "base", "right_gripper", is_debug);
    printf("\n");

    if (is_debug == true)
    {
        ROS_INFO("Debug mode enabled! Closing.");
        return 0;
    }

    ROS_INFO("READY! Waiting for control messages..\n");
    ros::spin();

    return 0;
}

