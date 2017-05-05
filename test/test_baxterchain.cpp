#include <gtest/gtest.h>

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

    BaxterChain chainR(robot_model, base_link, tip_link);
    EXPECT_EQ(chainR.getNrOfJoints(),    7);
    EXPECT_EQ(chainR.getNrOfSegments(), 12);

    Matrix4d H5 = chainR.getH(5);
    Matrix4d H4 = chainR.getH(4);
    Matrix4d H3 = chainR.getH(3);
    Matrix4d H2 = chainR.getH(2);
    Matrix4d H1 = chainR.getH(1);
    Matrix4d H0 = chainR.getH(0);

    // 8 means Joint::None
    EXPECT_EQ(chainR.segments.back().getJoint().getType(), 8);
    chainR.removeSegment();
    EXPECT_EQ(chainR.getNrOfJoints(),    7);
    EXPECT_EQ(chainR.getNrOfSegments(), 11);

    chainR.removeJoint();
    EXPECT_EQ(chainR.getNrOfJoints(),    6);
    EXPECT_EQ(chainR.getNrOfSegments(),  8);
    EXPECT_EQ(H5, chainR.getH());

    chainR.removeJoint();
    EXPECT_EQ(H4, chainR.getH());
    chainR.removeJoint();
    EXPECT_EQ(H3, chainR.getH());
    chainR.removeJoint();
    EXPECT_EQ(H2, chainR.getH());
    chainR.removeJoint();
    EXPECT_EQ(H1, chainR.getH());
    chainR.removeJoint();
    EXPECT_EQ(H0, chainR.getH());

    tip_link = "left_gripper";

    BaxterChain chainL(robot_model, base_link, tip_link);
    EXPECT_EQ(chainL.getNrOfJoints(),    7);
    EXPECT_EQ(chainL.getNrOfSegments(), 12);

    H5 = chainL.getH(5);
    H4 = chainL.getH(4);
    H3 = chainL.getH(3);
    H2 = chainL.getH(2);
    H1 = chainL.getH(1);
    H0 = chainL.getH(0);

    // 8 means Joint::None
    EXPECT_EQ(chainL.segments.back().getJoint().getType(), 8);
    chainL.removeSegment();
    EXPECT_EQ(chainL.getNrOfJoints(),    7);
    EXPECT_EQ(chainL.getNrOfSegments(), 11);

    chainL.removeJoint();
    EXPECT_EQ(chainL.getNrOfJoints(),    6);
    EXPECT_EQ(chainL.getNrOfSegments(),  8);
    EXPECT_EQ(H5, chainL.getH());

    chainL.removeJoint();
    EXPECT_EQ(H4, chainL.getH());
    chainL.removeJoint();
    EXPECT_EQ(H3, chainL.getH());
    chainL.removeJoint();
    EXPECT_EQ(H2, chainL.getH());
    chainL.removeJoint();
    EXPECT_EQ(H1, chainL.getH());
    chainL.removeJoint();
    EXPECT_EQ(H0, chainL.getH());
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

    BaxterChain chainR(robot_model, base_link, tip_link);

    EXPECT_EQ(chainR.getSegment(0).getJoint().getType(), KDL::Joint::None);
    EXPECT_EQ(chainR.getSegment(1).getJoint().getType(), KDL::Joint::None);
    EXPECT_EQ(chainR.getSegment(2).getJoint().getType(), KDL::Joint::RotAxis);
    EXPECT_EQ(chainR.getSegment(3).getJoint().getType(), KDL::Joint::RotAxis);
    EXPECT_EQ(chainR.getSegment(4).getJoint().getType(), KDL::Joint::RotAxis);
    EXPECT_EQ(chainR.getSegment(5).getJoint().getType(), KDL::Joint::RotAxis);
    EXPECT_EQ(chainR.getSegment(6).getJoint().getType(), KDL::Joint::RotAxis);
    EXPECT_EQ(chainR.getSegment(7).getJoint().getType(), KDL::Joint::RotAxis);
    EXPECT_EQ(chainR.getSegment(8).getJoint().getType(), KDL::Joint::RotAxis);
    EXPECT_EQ(chainR.getSegment(9).getJoint().getType(), KDL::Joint::None);
    EXPECT_EQ(chainR.getSegment(10).getJoint().getType(), KDL::Joint::None);
    EXPECT_EQ(chainR.getSegment(11).getJoint().getType(), KDL::Joint::None);

    tip_link = "left_gripper";

    BaxterChain chainL(robot_model, base_link, tip_link);

    EXPECT_EQ(chainL.getSegment(0).getJoint().getType(), KDL::Joint::None);
    EXPECT_EQ(chainL.getSegment(1).getJoint().getType(), KDL::Joint::None);
    EXPECT_EQ(chainL.getSegment(2).getJoint().getType(), KDL::Joint::RotAxis);
    EXPECT_EQ(chainL.getSegment(3).getJoint().getType(), KDL::Joint::RotAxis);
    EXPECT_EQ(chainL.getSegment(4).getJoint().getType(), KDL::Joint::RotAxis);
    EXPECT_EQ(chainL.getSegment(5).getJoint().getType(), KDL::Joint::RotAxis);
    EXPECT_EQ(chainL.getSegment(6).getJoint().getType(), KDL::Joint::RotAxis);
    EXPECT_EQ(chainL.getSegment(7).getJoint().getType(), KDL::Joint::RotAxis);
    EXPECT_EQ(chainL.getSegment(8).getJoint().getType(), KDL::Joint::RotAxis);
    EXPECT_EQ(chainL.getSegment(9).getJoint().getType(), KDL::Joint::None);
    EXPECT_EQ(chainL.getSegment(10).getJoint().getType(), KDL::Joint::None);
    EXPECT_EQ(chainL.getSegment(11).getJoint().getType(), KDL::Joint::None);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    ros::init(argc, argv, "baxter_react_controller");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
