#include <gtest/gtest.h>

#include <baxter_collaboration_msgs/GoToPose.h>
#include <baxter_collaboration_msgs/ArmState.h>
#include <baxter_core_msgs/AssemblyState.h>
#include <robot_utils/utils.h>
#include "react_controller/baxterChain.h"

#include   <memory>
#include <stdlib.h>

class reactControllerTester
{
private:
    ros::NodeHandle  nh;

    std::string    limb;

    ros::Publisher     pub;
    ros::Subscriber as_sub;  // Subscriber for the arm   state
    ros::Subscriber gz_sub;  // Subscriber for the robot state

    std::shared_ptr<baxter_collaboration_msgs::ArmState> as_state_ptr;
    std::shared_ptr< baxter_core_msgs::AssemblyState>    gz_state_ptr;

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
        as_state_ptr.reset(new baxter_collaboration_msgs::ArmState(msg));
    };

    void robotStateCb(const baxter_core_msgs::AssemblyState& msg)
    {
        gz_state_ptr.reset(new baxter_core_msgs::AssemblyState(msg));
    };

public:
    reactControllerTester(std::string _limb) : nh("test_react_controller"), limb(_limb),
                                               as_state_ptr(new baxter_collaboration_msgs::ArmState),
                                               gz_state_ptr(nullptr)
    {
        EXPECT_TRUE(importWayPoints(nh, _wp));

        pub = nh.advertise<baxter_collaboration_msgs::GoToPose>(
              "/baxter_react_controller/" + limb + "/go_to_pose", SUBSCRIBER_BUFFER, true);

        as_sub = nh.subscribe("/baxter_react_controller/" + limb + "/state", SUBSCRIBER_BUFFER,
                                                  &reactControllerTester::armStateCb, this);

        gz_sub = nh.subscribe("/robot/state", SUBSCRIBER_BUFFER,
                              &reactControllerTester::robotStateCb, this);
    };

    /**
     * This is very dirty because of how roslaunch and rostests are handled, but it does the job.
     * Wait for robot/state to be published, so that we know that the robot is up and running
     * on the Gazebo server. After this, the robot still needs to be enabled and untucked.
     *
     * @param _wait_time time to wait for the robot to be up in Gazebo
     * @return true/false if the robot was up within the given amount of time
     */
    bool waitBaxterGazeboReady(double _wait_time = 60.0)
    {
        ros::Time _init = ros::Time::now();

        ros::Rate r(100);
        while (ros::ok())
        {
            if (bool(gz_state_ptr))     { return true; }

            if ((ros::Time::now()-_init).toSec() > _wait_time)
            {
                ROS_WARN("No robot state in %gs!", _wait_time);
                return false;
            }

            ros::spinOnce();
            r.sleep();
        }

        return false;
    }

    /*
     * Waits for the robot to be enabled
     *
     * @param _wait_time time to wait for the robot to be up in Gazebo
     * @return true/false if the robot was enabled within the given amount of time
     */
    bool waitRobotEnabled(double _wait_time = 4.0)
    {
        clearRobotState();
        ros::Time _init = ros::Time::now();

        ros::Rate r(100);
        while (ros::ok())
        {
            if (isEnabled())     { return true; }

            if ((ros::Time::now()-_init).toSec() > _wait_time)
            {
                ROS_WARN("Robot not enabled in %gs!", _wait_time);
                return false;
            }

            ros::spinOnce();
            r.sleep();
        }

        return false;
    }

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
            if (std::string(_as) == as_state_ptr->state)     { return true; }

            if ((ros::Time::now()-_init).toSec() > _wait_time)
            {
                ROS_WARN("No reactController state %s in %gs!", std::string(_as).c_str(), _wait_time);
                return false;
            }

            ros::spinOnce();
            r.sleep();
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
            ASSERT_TRUE(waitReactCtrlState(State(   CTRL_DONE), 60.0));
        }
    };

    bool isEnabled()
    {
        if (not bool(gz_state_ptr))         { return false; }

        return gz_state_ptr -> enabled;
    };

    bool clearRobotState()
    {
        gz_state_ptr = nullptr;
        return true;
    };

    ~reactControllerTester() {};
};

// Declare a test
TEST(ReactControllerTest, testWayPoints)
{
    reactControllerTester rct("right");

    // DO NOT remove this sleep
    ros::Duration(0.1).sleep();

    ROS_INFO("Waiting for robot to be up..");
    ASSERT_TRUE(rct.waitBaxterGazeboReady());

    ROS_INFO("Enabling robot..");
    ASSERT_FALSE(system("rosrun baxter_tools enable_robot.py -e"));
    ASSERT_TRUE(rct.waitRobotEnabled());

    ROS_INFO("Untucking robot..");
    ASSERT_FALSE(system("rosrun baxter_tools tuck_arms.py -u"));

    ROS_INFO("Waiting for robot to be ready..");
    // CTRL_DONE is there to test the tester, but it should be removed.
    bool test_waypoints = rct.waitReactCtrlState(State(CTRL_DONE), 10.0) ||
                          rct.waitReactCtrlState(State(    START), 10.0);
    ASSERT_TRUE(test_waypoints);

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
