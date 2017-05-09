#include <gtest/gtest.h>

#include "react_controller/react_control_utils.h"

using namespace std;
using namespace Eigen;

// Declare a test
TEST(UtilsTest, toMatrix4d)
{
    KDL::Frame frame(KDL::Frame::Identity());
    Matrix4d   eigen(  Matrix4d::Identity());

    EXPECT_EQ(eigen, toMatrix4d(frame));
    EXPECT_EQ(eigen, toMatrix4d(frame.Inverse()));

    // Test some positions
    eigen(0,3) =  0.1;
    frame.p[0] =  0.1;
    EXPECT_EQ(eigen, toMatrix4d(frame)) << "Frame:\n" << toMatrix4d(frame)
                           << endl << endl << "Eigen:\n" << eigen << endl;

    eigen(1,3) = 0.01;
    frame.p[1] = 0.01;
    EXPECT_EQ(eigen, toMatrix4d(frame)) << "Frame:\n" << toMatrix4d(frame)
                           << endl << endl << "Eigen:\n" << eigen << endl;

    eigen(2,3) =  100;
    frame.p[2] =  100;
    EXPECT_EQ(eigen, toMatrix4d(frame)) << "Frame:\n" << toMatrix4d(frame)
                           << endl << endl << "Eigen:\n" << eigen << endl;

    // Test some rotations
    frame.M  = KDL::Rotation(0,1,0, 1,0,0, 0,0,-1 );
    eigen.block<3,3>(0,0) << 0,1,0, 1,0,0, 0,0,-1;
    EXPECT_EQ(eigen, toMatrix4d(frame)) << "Frame:\n" << toMatrix4d(frame)
                           << endl << endl << "Eigen:\n" << eigen << endl;

    frame.M  = KDL::Rotation(-1,0,0, 0,0,1, 0,1,0 );
    eigen.block<3,3>(0,0) << -1,0,0, 0,0,1, 0,1,0;
    EXPECT_EQ(eigen, toMatrix4d(frame)) << "Frame:\n" << toMatrix4d(frame)
                           << endl << endl << "Eigen:\n" << eigen << endl;

    // KDL rotation expects the three axis of rotations (ie the three columns)
    // Eigen instead expects the matrix to be assigned row by row
    frame.M  = KDL::Rotation(0,0,1, -1,0,0, 0,1,0 );
    eigen.block<3,3>(0,0) << 0,-1,0, 0,0,1, 1,0,0;
    EXPECT_EQ(eigen, toMatrix4d(frame)) << "Frame:\n" << toMatrix4d(frame)
                           << endl << endl << "Eigen:\n" << eigen << endl;
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
