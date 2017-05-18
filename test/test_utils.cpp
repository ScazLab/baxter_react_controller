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

void testAngularErrors(Matrix3d _a, Matrix3d _b, Vector3d _exp)
{
    Vector3d _a_b = angularError(_a, _b);
    Vector3d _b_a = angularError(_b, _a);

    ASSERT_EQ(0.0,  angularError(_a, _a).squaredNorm());
    ASSERT_EQ(0.0,  angularError(_b, _b).squaredNorm());
    ASSERT_EQ(0.0, (_a_b + _b_a).squaredNorm()) <<
               "Angular Error: [_a, _b][deg]: " << RAD2DEG * _a_b.transpose() <<
               "               [_b, _a][deg]: " << RAD2DEG * _b_a.transpose() << endl;

    ASSERT_NEAR((+1.0 * _exp).squaredNorm(), _a_b.squaredNorm(), 1e-6) <<
                 "[_a, _b] expected [deg]: " << (RAD2DEG *  _exp).transpose() <<
                           " actual [deg]: " << (RAD2DEG *  _a_b).transpose() << endl;
    ASSERT_NEAR((-1.0 * _exp).squaredNorm(), _b_a.squaredNorm(), 1e-6) <<
                 "[_b, _a] expected [deg]: " << (RAD2DEG * -_exp).transpose() <<
                           " actual [deg]: " << (RAD2DEG *  _b_a).transpose() << endl;
}

void evaluateAngularErrorsFromMat(Matrix3d _ori_mat, double _rot_x, double _rot_y, double _rot_z)
{
    // Let's test angular errors
    Matrix3d new_mat;
    Vector3d exp_err;

    cout << "Evaluating [" << _rot_x << ", " << _rot_y << ", " << _rot_z << "]\n";

    exp_err = DEG2RAD*Vector3d(-_rot_x, -_rot_y, -_rot_z);
    new_mat = AngleAxisd(_rot_x*DEG2RAD, Vector3d::UnitX())
            * AngleAxisd(_rot_y*DEG2RAD, Vector3d::UnitY())
            * AngleAxisd(_rot_z*DEG2RAD, Vector3d::UnitZ()) * _ori_mat;

    EXPECT_TRUE(new_mat.isUnitary()) << "new_mat:\n" << new_mat << endl;

    testAngularErrors(_ori_mat, new_mat, exp_err);
}

TEST(UtilsTest, angleAxisErrAng)
{
    // First, let's test constructors and stuff
    Vector3d P0(0.1, 0.2, 0.3);
    Matrix3d R0;
    R0 = AngleAxisd(45*DEG2RAD, Vector3d::UnitX())
       * AngleAxisd(90*DEG2RAD, Vector3d::UnitY())
       * AngleAxisd(60*DEG2RAD, Vector3d::UnitZ());
    EXPECT_TRUE(R0.isUnitary()) << "R0:\n" << R0 << endl;

    Matrix4d H0(Matrix4d::Identity());
    H0.block<3,3>(0,0) = R0;
    H0.block<3,1>(0,3) = P0;
    EXPECT_EQ((H0.block<3,3>(0,0)), R0);
    EXPECT_EQ((H0.block<3,1>(0,3)), P0);

    AngleAxisd AA0A(R0);
    AngleAxisd AA0B(H0.block<3,3>(0,0));
    EXPECT_EQ(AA0A.toRotationMatrix(), AA0B.toRotationMatrix());

    // Let's test angular errors
    Matrix3d R1;
    Vector3d R0R1_exp;

    // 45° rotation about x axis
    evaluateAngularErrorsFromMat(R0,  45.0,   0.0,   0.0);

    // -90° rotation about y axis
    evaluateAngularErrorsFromMat(R0,   0.0, -90.0,   0.0);

    // 180° rotation about all axes
    evaluateAngularErrorsFromMat(R0,   0.0,   0.0, 120.0);

    // 180° rotation about all axes
    // evaluateAngularErrorsFromMat(R0, 180.0, 180.0, 180.0);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
