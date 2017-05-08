#include <gtest/gtest.h>

#include <baxter_collaboration_msgs/GoToPose.h>
#include <baxter_collaboration_msgs/ArmState.h>
#include <robot_utils/utils.h>
#include "react_controller/baxterChain.h"

#include <memory>

class reactControllerTester
{
private:
    ros::NodeHandle  nh;

    std::string    limb;

    ros::Publisher  pub;
    ros::Subscriber sub;  // Subscriber for the arm state

    std::shared_ptr<baxter_collaboration_msgs::ArmState> arm_state_ptr;

    std::vector<geometry_msgs::Pose> _wp;

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
        EXPECT_TRUE(waypoints.size() > 0);

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
    };

    /**
     * Callback for the subscriber.
     *
     * @param msg the message received.
     */
    void armStateCb(const baxter_collaboration_msgs::ArmState& msg)
    {
        *arm_state_ptr = msg;
    };

public:
    reactControllerTester(std::string _limb) : nh("test_react_controller"), limb(_limb),
                                               arm_state_ptr(new baxter_collaboration_msgs::ArmState)
    {
        EXPECT_TRUE(importWayPoints(nh, _wp));

        pub = nh.advertise<baxter_collaboration_msgs::GoToPose>(
             "/baxter_react_controller/" + limb + "/go_to_pose", 10, true);

        sub = nh.subscribe("/baxter_react_controller/" + limb + "/state", SUBSCRIBER_BUFFER,
                                                  &reactControllerTester::armStateCb, this);
    };


    /*
     * Waits for states to be received from the subscriber.
     *
     * @return true  when expected state has been received
     * @return false if expected state has not been received
     */
    bool waitReactCtrlState(State _as, double _wait_time = 20.0)
    {
        ros::Time _init = ros::Time::now();

        ros::Rate r(100);
        while (ros::ok())
        {
            if (std::string(_as) == arm_state_ptr->state)     { return true; }

            ros::spinOnce();
            r.sleep();

            if ((ros::Time::now()-_init).toSec() > _wait_time)
            {
                ROS_WARN("No reactController state %s in %gs!", std::string(_as).c_str(), _wait_time);
                return false;
            }
        }

        return false;
    }

    /**
     * Tests waypoints against the ctrl thread.
     *
     * @return true/false if success/failure
     */
    void testWayPoints()
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

            ASSERT_TRUE(waitReactCtrlState(State(CTRL_RUNNING),  4.0));
            ASSERT_TRUE(waitReactCtrlState(State(   CTRL_DONE), 40.0));
        }
    };

    ~reactControllerTester() {};
};

// Declare a test
TEST(ReactControllerTest, testWayPoints)
{
    reactControllerTester rct("right");

    ROS_INFO("Waiting for robot to be ready..");
    // CTRL_DONE is there to test the tester, but it should be removed.
    bool test_waypoints = rct.waitReactCtrlState(State(CTRL_DONE), 10.0) ||
                          rct.waitReactCtrlState(State(START), 10.0);
    EXPECT_TRUE(test_waypoints);

    ROS_INFO("Done. Testing waypoints..");
    rct.testWayPoints();
    ROS_INFO("Done.");
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_react_controller");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
