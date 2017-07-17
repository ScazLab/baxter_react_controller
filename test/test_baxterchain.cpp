#include <gtest/gtest.h>

#include "react_controller/baxterChain.h"

using namespace std;
using namespace Eigen;

BaxterChain getChain(const std::string &_tip_link)
{
    urdf::Model robot_model;
    string xml_string;
    ros::NodeHandle _n("baxter_react_controller");

    string urdf_xml,full_urdf_xml;
    _n.param<std::string>("urdf_xml",urdf_xml,"/robot_description");
    _n.searchParam(urdf_xml,full_urdf_xml);

    ROS_ASSERT(_n.getParam(full_urdf_xml, xml_string));

    _n.param(full_urdf_xml,xml_string,std::string());
    robot_model.initString(xml_string);

    return BaxterChain(robot_model, "base", _tip_link);
}

TEST(BaxterChainTest, testClass)
{
    BaxterChain chain;

    EXPECT_EQ(chain.getNrOfJoints(),   0);
    EXPECT_EQ(chain.getNrOfSegments(), 0);

    chain = getChain("right_gripper");

    EXPECT_EQ(chain.getNrOfJoints(),    7);
    EXPECT_EQ(chain.getNrOfSegments(), 12);

    EXPECT_EQ(chain.GeoJacobian().rows(), 6);
    EXPECT_EQ(chain.GeoJacobian().cols(), 7);

    // getH() is expected to be different from getH(6) because of
    // some segments that are attached at the end of the chain.
    EXPECT_EQ(chain.getH(), chain.getH(chain.getNrOfJoints()-1));
    // Eigen::IOFormat HeavyFmt(FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");
    // cout << "getH (): " << chain.getH( ).format(HeavyFmt) << endl;
    // cout << "getH(6): " << chain.getH(6).format(HeavyFmt) << endl;

    Eigen::VectorXd q_0(chain.getNrOfJoints());
    for (size_t i = 0; i < chain.getNrOfJoints(); ++i)
    {
        // This will initialize the joint in the
        // middle of its operational range
        q_0[i] = (chain.getMin(i)+chain.getMax(i))/2;
    }

    EXPECT_EQ(q_0.size(), chain.getAng().size());
    EXPECT_EQ(q_0, chain.getAng());
    for (size_t i = 0; i < chain.getNrOfJoints(); ++i)
    {
        EXPECT_EQ(q_0[i], chain.getAng()[i]) << "q_0[i] and chain.getAng()[i]  "
                                                "differ at idx " << i;
        EXPECT_EQ(q_0[i], chain.getAng(i))   << "q_0[i] and chain.getAng(i)    "
                                                "differ at idx " << i;
    }

    q_0[1] = 0.4;
    q_0[4] = 0.8;

    EXPECT_TRUE (chain.setAng(q_0));
    EXPECT_FALSE(chain.setAng(Eigen::VectorXd(chain.getNrOfJoints()+1)));
    EXPECT_EQ   (q_0, chain.getAng());

    q_0[1] =  100.0;
    q_0[4] = -100.0;
    EXPECT_TRUE (chain.setAng(q_0));
    EXPECT_NE   (q_0, chain.getAng());
    q_0[1] = chain.getMax(1);
    q_0[4] = chain.getMin(4);
    EXPECT_EQ   (q_0, chain.getAng());
    for (size_t i = 0; i < chain.getNrOfJoints(); ++i)
    {
        EXPECT_EQ(q_0[i], chain.getAng()[i]) << "q_0[i] and chain.getAng()[i]  "
                                                "differ at idx " << i;
        EXPECT_EQ(q_0[i], chain.getAng(i))   << "q_0[i] and chain.getAng(i)    "
                                                "differ at idx " << i;
    }
}

TEST(BaxterChainTest, testRemoveSegment)
{
    BaxterChain chainR(getChain("right_gripper"));
    BaxterChain chainL(getChain( "left_gripper"));

    // Right chain tests
    Matrix4d H6 = chainR.getH(6);
    Matrix4d H5 = chainR.getH(5);
    Matrix4d H4 = chainR.getH(4);
    Matrix4d H3 = chainR.getH(3);
    Matrix4d H2 = chainR.getH(2);
    Matrix4d H1 = chainR.getH(1);
    Matrix4d H0 = chainR.getH(0);

    chainR.removeSegment();
    EXPECT_EQ(chainR.getNrOfJoints(),      7);
    EXPECT_EQ(chainR.getNrOfSegments(),   11);

    chainR.removeJoint();
    EXPECT_EQ(chainR.getNrOfJoints(),      6);
    EXPECT_EQ(chainR.getNrOfSegments(),    8);
    EXPECT_EQ(chainR.GeoJacobian().cols(), 6);
    EXPECT_EQ(H5, chainR.getH()) << "getH (): " << chainR.getH( ) << endl
                                 << "     H5: " <<             H6 << endl;

    chainR.removeJoint();
    EXPECT_EQ(chainR.getNrOfJoints(),      5);
    EXPECT_EQ(chainR.getNrOfSegments(),    7);
    EXPECT_EQ(chainR.GeoJacobian().cols(), 5);
    EXPECT_EQ(H4, chainR.getH()) << "getH (): " << chainR.getH( ) << endl
                                 << "     H4: " <<             H5 << endl;

    chainR.removeJoint();
    EXPECT_EQ(chainR.getNrOfJoints(),      4);
    EXPECT_EQ(chainR.getNrOfSegments(),    6);
    EXPECT_EQ(chainR.GeoJacobian().cols(), 4);
    EXPECT_EQ(H3, chainR.getH()) << "getH (): " << chainR.getH( ) << endl
                                 << "     H3: " <<             H4 << endl;

    chainR.removeJoint();
    EXPECT_EQ(chainR.getNrOfJoints(),      3);
    EXPECT_EQ(chainR.getNrOfSegments(),    5);
    EXPECT_EQ(chainR.GeoJacobian().cols(), 3);
    EXPECT_EQ(H2, chainR.getH()) << "getH (): " << chainR.getH( ) << endl
                                 << "     H2: " <<             H3 << endl;

    chainR.removeJoint();
    EXPECT_EQ(chainR.getNrOfJoints(),      2);
    EXPECT_EQ(chainR.getNrOfSegments(),    4);
    EXPECT_EQ(chainR.GeoJacobian().cols(), 2);
    EXPECT_EQ(H1, chainR.getH()) << "getH (): " << chainR.getH( ) << endl
                                 << "     H1: " <<             H2 << endl;

    chainR.removeJoint();
    EXPECT_EQ(chainR.getNrOfJoints(),      1);
    EXPECT_EQ(chainR.getNrOfSegments(),    3);
    EXPECT_EQ(chainR.GeoJacobian().cols(), 1);
    EXPECT_EQ(H0, chainR.getH()) << "getH (): " << chainR.getH( ) << endl
                                 << "     H0: " <<             H1 << endl;

    // Left chain tests
    H6 = chainL.getH(6);
    H5 = chainL.getH(5);
    H4 = chainL.getH(4);
    H3 = chainL.getH(3);
    H2 = chainL.getH(2);
    H1 = chainL.getH(1);
    H0 = chainL.getH(0);

    chainL.removeSegment();
    EXPECT_EQ(chainL.getNrOfJoints(),      7);
    EXPECT_EQ(chainL.getNrOfSegments(),   11);

    chainL.removeJoint();
    EXPECT_EQ(chainL.getNrOfJoints(),      6);
    EXPECT_EQ(chainL.getNrOfSegments(),    8);
    EXPECT_EQ(chainL.GeoJacobian().cols(), 6);
    EXPECT_EQ(H5, chainL.getH()) << "getH (): " << chainL.getH( ) << endl
                                 << "     H5: " <<             H6 << endl;

    chainL.removeJoint();
    EXPECT_EQ(chainL.getNrOfJoints(),      5);
    EXPECT_EQ(chainL.getNrOfSegments(),    7);
    EXPECT_EQ(chainL.GeoJacobian().cols(), 5);
    EXPECT_EQ(H4, chainL.getH()) << "getH (): " << chainL.getH( ) << endl
                                 << "     H4: " <<             H5 << endl;

    chainL.removeJoint();
    EXPECT_EQ(chainL.getNrOfJoints(),      4);
    EXPECT_EQ(chainL.getNrOfSegments(),    6);
    EXPECT_EQ(chainL.GeoJacobian().cols(), 4);
    EXPECT_EQ(H3, chainL.getH()) << "getH (): " << chainL.getH( ) << endl
                                 << "     H3: " <<             H4 << endl;

    chainL.removeJoint();
    EXPECT_EQ(chainL.getNrOfJoints(),      3);
    EXPECT_EQ(chainL.getNrOfSegments(),    5);
    EXPECT_EQ(chainL.GeoJacobian().cols(), 3);
    EXPECT_EQ(H2, chainL.getH()) << "getH (): " << chainL.getH( ) << endl
                                 << "     H2: " <<             H3 << endl;

    chainL.removeJoint();
    EXPECT_EQ(chainL.getNrOfJoints(),      2);
    EXPECT_EQ(chainL.getNrOfSegments(),    4);
    EXPECT_EQ(chainL.GeoJacobian().cols(), 2);
    EXPECT_EQ(H1, chainL.getH()) << "getH (): " << chainL.getH( ) << endl
                                 << "     H1: " <<             H2 << endl;

    chainL.removeJoint();
    EXPECT_EQ(chainL.getNrOfJoints(),      1);
    EXPECT_EQ(chainL.getNrOfSegments(),    3);
    EXPECT_EQ(chainL.GeoJacobian().cols(), 1);
    EXPECT_EQ(H0, chainL.getH()) << "getH (): " << chainL.getH( ) << endl
                                 << "     H0: " <<             H1 << endl;
}

TEST(BaxterChainTest, testSegmentTypes)
{
    BaxterChain chainR(getChain("right_gripper"));
    BaxterChain chainL(getChain( "left_gripper"));

    // Right chain tests
    EXPECT_EQ(chainR.getSegment( 0).getJoint().getType(), KDL::Joint::None   );
    EXPECT_EQ(chainR.getSegment( 1).getJoint().getType(), KDL::Joint::None   );
    EXPECT_EQ(chainR.getSegment( 2).getJoint().getType(), KDL::Joint::RotAxis);
    EXPECT_EQ(chainR.getSegment( 3).getJoint().getType(), KDL::Joint::RotAxis);
    EXPECT_EQ(chainR.getSegment( 4).getJoint().getType(), KDL::Joint::RotAxis);
    EXPECT_EQ(chainR.getSegment( 5).getJoint().getType(), KDL::Joint::RotAxis);
    EXPECT_EQ(chainR.getSegment( 6).getJoint().getType(), KDL::Joint::RotAxis);
    EXPECT_EQ(chainR.getSegment( 7).getJoint().getType(), KDL::Joint::RotAxis);
    EXPECT_EQ(chainR.getSegment( 8).getJoint().getType(), KDL::Joint::RotAxis);
    EXPECT_EQ(chainR.getSegment( 9).getJoint().getType(), KDL::Joint::None   );
    EXPECT_EQ(chainR.getSegment(10).getJoint().getType(), KDL::Joint::None   );
    EXPECT_EQ(chainR.getSegment(11).getJoint().getType(), KDL::Joint::None   );

    // Left chain tests
    EXPECT_EQ(chainL.getSegment( 0).getJoint().getType(), KDL::Joint::None   );
    EXPECT_EQ(chainL.getSegment( 1).getJoint().getType(), KDL::Joint::None   );
    EXPECT_EQ(chainL.getSegment( 2).getJoint().getType(), KDL::Joint::RotAxis);
    EXPECT_EQ(chainL.getSegment( 3).getJoint().getType(), KDL::Joint::RotAxis);
    EXPECT_EQ(chainL.getSegment( 4).getJoint().getType(), KDL::Joint::RotAxis);
    EXPECT_EQ(chainL.getSegment( 5).getJoint().getType(), KDL::Joint::RotAxis);
    EXPECT_EQ(chainL.getSegment( 6).getJoint().getType(), KDL::Joint::RotAxis);
    EXPECT_EQ(chainL.getSegment( 7).getJoint().getType(), KDL::Joint::RotAxis);
    EXPECT_EQ(chainL.getSegment( 8).getJoint().getType(), KDL::Joint::RotAxis);
    EXPECT_EQ(chainL.getSegment( 9).getJoint().getType(), KDL::Joint::None   );
    EXPECT_EQ(chainL.getSegment(10).getJoint().getType(), KDL::Joint::None   );
    EXPECT_EQ(chainL.getSegment(11).getJoint().getType(), KDL::Joint::None   );
}

TEST(BaxterChainTest, testRVIZVisualization)
{
    BaxterChain chainR(getChain("right_gripper"));
    BaxterChain chainL(getChain( "left_gripper"));
    RVIZPublisher rviz_pub("rviz_tester");

    rviz_pub.start();

    vector <RVIZMarker> rviz_markersR = asRVIZMarkers(chainR, true, true, true);
    vector <RVIZMarker> rviz_markersL = asRVIZMarkers(chainL, true, true, true);

    rviz_markersR.insert(std::end(rviz_markersR),
                         std::begin(rviz_markersL), std::end(rviz_markersL));
    rviz_pub.setMarkers(rviz_markersR);

    ros::Duration(0.5).sleep();
}

#include <kdl/chainjnttojacsolver.hpp>

TEST(BaxterChainTest, testJacobians)
{
    BaxterChain chain(getChain("right_gripper"));

    std::shared_ptr<KDL::ChainJntToJacSolver> kdl_solver;
    kdl_solver.reset(new KDL::ChainJntToJacSolver(KDL::Chain(chain)));

    KDL::JntArray q(chain.getNrOfJoints());
    q.data = chain.getAng();

    KDL::Jacobian kdlJac(chain.getNrOfJoints());
    EXPECT_FALSE(kdl_solver->JntToJac(q, kdlJac)); // False means that it works

    EXPECT_EQ(chain.GeoJacobian(), kdlJac.data);
}

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>

TEST(BaxterChainTest, testFWDKin)
{
    BaxterChain chain(getChain("right_gripper"));

    std::shared_ptr<KDL::ChainFkSolverPos_recursive> kdl_solver;
    kdl_solver.reset(new KDL::ChainFkSolverPos_recursive(KDL::Chain(chain)));

    KDL::JntArray q(chain.getNrOfJoints());
    q.data = chain.getAng();

    KDL::Frame kdl_frame;
    EXPECT_FALSE(kdl_solver->JntToCart(q, kdl_frame)); // False means that it works
    EXPECT_EQ(toMatrix4d(chain.JntToCart()), toMatrix4d(kdl_frame)) << "Expected:\n" <<
              toMatrix4d(chain.JntToCart()) << "\nObtained:\n" <<  toMatrix4d(kdl_frame) << endl;
    EXPECT_EQ(chain.JntToCart(), kdl_frame) << "Expected:\n" <<
              chain.JntToCart() << "\nObtained:\n" <<  kdl_frame << endl;

    for (size_t i = 0; i <= chain.getNrOfSegments(); ++i)
    {
        EXPECT_FALSE(kdl_solver->JntToCart(q, kdl_frame, i)) << "[" << i << "]\n"; // False means that it works
        EXPECT_EQ(toMatrix4d(chain.JntToCart(i)), toMatrix4d(kdl_frame)) << "[" << i << "] Expected:\n" <<
                  toMatrix4d(chain.JntToCart(i)) << "\nObtained:\n" <<  toMatrix4d(kdl_frame) << endl;
        EXPECT_EQ(chain.JntToCart(i), kdl_frame) << "[" << i << "] Expected:\n" <<
                  chain.JntToCart(i) << "\nObtained:\n" <<  kdl_frame << endl;
    }
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    ros::init(argc, argv, "baxter_react_controller");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
