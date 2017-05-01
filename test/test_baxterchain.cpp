#include <gtest/gtest.h>
#include <ros/ros.h>

#include "react_controller/baxterChain.h"

using namespace std;
using namespace Eigen;

// Declare a test
TEST(BaxterChainTest, testCollisionPoints)
{
    vector<Vector3d> joints{Vector3d(0, 0, 0), Vector3d(0, 2, 0),
                            Vector3d(2, 2, 0), Vector3d(2, 0, 0)};

    vector<collisionPoint> coll_points;

    EXPECT_TRUE(computeCollisionPoints(joints, Vector3d(1, 1, 0), coll_points));
    EXPECT_EQ(coll_points[0].x[0],  0);
    EXPECT_EQ(coll_points[0].x[1],  1);
    EXPECT_EQ(coll_points[0].x[2],  0);
    EXPECT_EQ(coll_points[1].x[0],  1);
    EXPECT_EQ(coll_points[1].x[1],  2);
    EXPECT_EQ(coll_points[1].x[2],  0);
    EXPECT_EQ(coll_points[2].x[0],  2);
    EXPECT_EQ(coll_points[2].x[1],  1);
    EXPECT_EQ(coll_points[2].x[2],  0);

    EXPECT_EQ(coll_points[0].n[0],  1);
    EXPECT_EQ(coll_points[0].n[1],  0);
    EXPECT_EQ(coll_points[0].n[2],  0);
    EXPECT_EQ(coll_points[1].n[0],  0);
    EXPECT_EQ(coll_points[1].n[1], -1);
    EXPECT_EQ(coll_points[1].n[2],  0);
    EXPECT_EQ(coll_points[2].n[0], -1);
    EXPECT_EQ(coll_points[2].n[1],  0);
    EXPECT_EQ(coll_points[2].n[2],  0);

    coll_points.clear();

    EXPECT_TRUE(computeCollisionPoints(joints, Vector3d(4, 4, 0), coll_points));
    EXPECT_EQ(coll_points[0].x[0],  0);
    EXPECT_EQ(coll_points[0].x[1],  4);
    EXPECT_EQ(coll_points[0].x[2],  0);
    EXPECT_EQ(coll_points[1].x[0],  4);
    EXPECT_EQ(coll_points[1].x[1],  2);
    EXPECT_EQ(coll_points[1].x[2],  0);
    EXPECT_EQ(coll_points[2].x[0],  2);
    EXPECT_EQ(coll_points[2].x[1],  4);
    EXPECT_EQ(coll_points[2].x[2],  0);

    EXPECT_EQ(coll_points[0].n[0],  1);
    EXPECT_EQ(coll_points[0].n[1],  0);
    EXPECT_EQ(coll_points[0].n[2],  0);
    EXPECT_EQ(coll_points[1].n[0],  0);
    EXPECT_EQ(coll_points[1].n[1],  1);
    EXPECT_EQ(coll_points[1].n[2],  0);
    EXPECT_EQ(coll_points[2].n[0],  1);
    EXPECT_EQ(coll_points[2].n[1],  0);
    EXPECT_EQ(coll_points[2].n[2],  0);

    coll_points.clear();

    EXPECT_TRUE(computeCollisionPoints(joints, Vector3d(-1, -1, 0), coll_points));
    EXPECT_EQ(coll_points[0].x[0],  0);
    EXPECT_EQ(coll_points[0].x[1], -1);
    EXPECT_EQ(coll_points[0].x[2],  0);
    EXPECT_EQ(coll_points[1].x[0], -1);
    EXPECT_EQ(coll_points[1].x[1],  2);
    EXPECT_EQ(coll_points[1].x[2],  0);
    EXPECT_EQ(coll_points[2].x[0],  2);
    EXPECT_EQ(coll_points[2].x[1], -1);
    EXPECT_EQ(coll_points[2].x[2],  0);

    EXPECT_EQ(coll_points[0].n[0], -1);
    EXPECT_EQ(coll_points[0].n[1],  0);
    EXPECT_EQ(coll_points[0].n[2],  0);
    EXPECT_EQ(coll_points[1].n[0],  0);
    EXPECT_EQ(coll_points[1].n[1], -1);
    EXPECT_EQ(coll_points[1].n[2],  0);
    EXPECT_EQ(coll_points[2].n[0], -1);
    EXPECT_EQ(coll_points[2].n[1],  0);
    EXPECT_EQ(coll_points[2].n[2],  0);
}

#include <iostream>
TEST(BaxterChainTest, testRemoveSegmentRightArm)
{
    urdf::Model robot_model;
    string xml_string;
    ros::NodeHandle _n("baxter_react_controller");

    string urdf_xml,full_urdf_xml;
    _n.param<std::string>("urdf_xml",urdf_xml,"/robot_description");
    _n.searchParam(urdf_xml,full_urdf_xml);

    EXPECT_TRUE(_n.getParam(full_urdf_xml, xml_string));

    _n.param(full_urdf_xml,xml_string,std::string());
    robot_model.initString(xml_string);

    string base_link = "base";
    string  tip_link = "right_gripper";

    BaxterChain chain(robot_model, base_link, tip_link);
    EXPECT_EQ(chain.getNrOfJoints(), 7);

    Matrix4d H = chain.getH(chain.getNrOfJoints() - 2);

    chain.removeJoint();

    Matrix4d H_prime = chain.getH();

    EXPECT_EQ(chain.getNrOfJoints(), 6);

    cout << H << endl;
    cout << H_prime << endl;

    EXPECT_EQ(H, H_prime);
}

TEST(BaxterChainTest, testSegmentTypes)
{
    urdf::Model robot_model;
    string xml_string;
    ros::NodeHandle _n("baxter_react_controller");

    string urdf_xml,full_urdf_xml;
    _n.param<std::string>("urdf_xml",urdf_xml,"/robot_description");
    _n.searchParam(urdf_xml,full_urdf_xml);

    EXPECT_TRUE(_n.getParam(full_urdf_xml, xml_string));

    _n.param(full_urdf_xml,xml_string,std::string());
    robot_model.initString(xml_string);

    string base_link = "base";
    string  tip_link = "right_gripper";

    BaxterChain chain(robot_model, base_link, tip_link);

    EXPECT_EQ(chain.getSegment(0).getJoint().getType(), KDL::Joint::None);
    EXPECT_EQ(chain.getSegment(1).getJoint().getType(), KDL::Joint::None);
    EXPECT_EQ(chain.getSegment(2).getJoint().getType(), KDL::Joint::RotAxis);
    EXPECT_EQ(chain.getSegment(3).getJoint().getType(), KDL::Joint::RotAxis);
    EXPECT_EQ(chain.getSegment(4).getJoint().getType(), KDL::Joint::RotAxis);
    EXPECT_EQ(chain.getSegment(5).getJoint().getType(), KDL::Joint::RotAxis);
    EXPECT_EQ(chain.getSegment(6).getJoint().getType(), KDL::Joint::RotAxis);
    EXPECT_EQ(chain.getSegment(7).getJoint().getType(), KDL::Joint::RotAxis);
    EXPECT_EQ(chain.getSegment(8).getJoint().getType(), KDL::Joint::RotAxis);
    EXPECT_EQ(chain.getSegment(9).getJoint().getType(), KDL::Joint::None);
    EXPECT_EQ(chain.getSegment(10).getJoint().getType(), KDL::Joint::None);
    EXPECT_EQ(chain.getSegment(11).getJoint().getType(), KDL::Joint::None);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    ros::init(argc, argv, "baxter_react_controller");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
