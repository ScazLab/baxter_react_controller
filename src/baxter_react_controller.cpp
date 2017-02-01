#include <stdio.h>

#include <ros/ros.h>

using namespace std;

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "baxter_react_controller");
    ros::NodeHandle _n("baxter_react_controller");

    printf("\n");
    ROS_INFO("READY! Waiting for control messages..\n");

    ros::spin();
    return 0;
}

