#include <gtest/gtest.h>

#include <baxter_collaboration_msgs/GoToPose.h>
#include <robot_utils/utils.h>
#include "react_controller/baxterChain.h"

class reactControllerTester
{
private:
    ros::NodeHandle  nh;

    std::string    limb;

    ros::Publisher  pub;

    std::vector<geometry_msgs::Pose> _wp;

public:
    reactControllerTester(std::string _limb) : nh("baxter_react_controller"), limb(_limb)
    {
        EXPECT_TRUE(importWayPoints(nh, _wp));

        pub = nh.advertise<baxter_collaboration_msgs::GoToPose>(
             "/baxter_react_controller/" + limb + "/go_to_pose", 10, true);
    }

    /**
     * Imports waypoints from the parameter server
     *
     * @param _nh the NodeHandle
     * @param _wp the waypoints extracted from the parameter server
     *
     * @return true/false if success/failure
     */
    bool importWayPoints(const ros::NodeHandle& _nh, std::vector<geometry_msgs::Pose>& _wp)
    {
        _wp.clear();
        XmlRpc::XmlRpcValue    waypoints;

        if(!_nh.getParam("/"+_nh.getNamespace()+"/waypoints/" + limb, waypoints))
        {
            ROS_INFO("No objects' database found in the parameter server. "
                     "Looked up param is %s", ("/"+_nh.getNamespace()+"/waypoints").c_str());
            return false;
        }

        EXPECT_EQ  (waypoints.getType(), XmlRpc::XmlRpcValue::TypeArray);
        EXPECT_TRUE(waypoints.size());

        for (int i = 0; i < waypoints.size(); ++i)
        {
            EXPECT_EQ(waypoints[i].getType(), XmlRpc::XmlRpcValue::TypeArray);
            EXPECT_EQ(waypoints[i].size()   , 7);

            geometry_msgs::Pose pose;
            pose.position.x    = waypoints[i][0];
            pose.position.y    = waypoints[i][1];
            pose.position.z    = waypoints[i][2];
            pose.orientation.x = waypoints[i][3];
            pose.orientation.y = waypoints[i][4];
            pose.orientation.z = waypoints[i][5];
            pose.orientation.w = waypoints[i][6];

            _wp.push_back(pose);
        }

        return true;
    }

    bool testWayPoints()
    {
        for (size_t i = 0; i < _wp.size(); ++i)
        {
            baxter_collaboration_msgs::GoToPose msg;
            msg.type      = "pose";
            msg.ctrl_mode =      1;

            msg.position    = _wp[i].position;
            msg.orientation = _wp[i].orientation;

            ROS_INFO("Testing waypoint %lu: %s", i, toString(_wp[i]).c_str());

            pub.publish(msg);
            ros::Duration(6.0).sleep();
        }

        return true;
    }

    ~reactControllerTester() {};
};

// Declare a test
TEST(ReactControllerTest, testWayPoints)
{
    reactControllerTester rct("right");

    EXPECT_TRUE(rct.testWayPoints());
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_react_controller");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
