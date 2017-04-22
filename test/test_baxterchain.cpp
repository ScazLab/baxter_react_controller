#include <gtest/gtest.h>

#include "react_controller/baxterChain.h"

using namespace std;
using namespace Eigen;

// Declare a test
TEST(BaxterChainTest, testCollisionPoints)
{
    vector<Vector3d> joints{Vector3d(0, 0, 0), Vector3d(0, 2, 0),
                            Vector3d(2, 2, 0), Vector3d(2, 0, 0)};

    vector<Vector3d> coll_points;
    vector<Vector3d>       norms;

    EXPECT_TRUE(computeCollisionPoint(joints, Vector3d(1, 1, 0), coll_points, norms));
    EXPECT_EQ(coll_points[0][0], 0);
    EXPECT_EQ(coll_points[0][1], 1);
    EXPECT_EQ(coll_points[0][2], 0);
    EXPECT_EQ(coll_points[1][0], 1);
    EXPECT_EQ(coll_points[1][1], 2);
    EXPECT_EQ(coll_points[1][2], 0);
    EXPECT_EQ(coll_points[2][0], 2);
    EXPECT_EQ(coll_points[2][1], 1);
    EXPECT_EQ(coll_points[2][2], 0);

    EXPECT_EQ(norms[0][0],  1);
    EXPECT_EQ(norms[0][1],  0);
    EXPECT_EQ(norms[0][2],  0);
    EXPECT_EQ(norms[1][0],  0);
    EXPECT_EQ(norms[1][1], -1);
    EXPECT_EQ(norms[1][2],  0);
    EXPECT_EQ(norms[2][0], -1);
    EXPECT_EQ(norms[2][1],  0);
    EXPECT_EQ(norms[2][2],  0);

    coll_points.clear();
    norms.clear();

    EXPECT_TRUE(computeCollisionPoint(joints, Vector3d(4, 4, 0), coll_points, norms));
    EXPECT_EQ(coll_points[0][0], 0);
    EXPECT_EQ(coll_points[0][1], 4);
    EXPECT_EQ(coll_points[0][2], 0);
    EXPECT_EQ(coll_points[1][0], 4);
    EXPECT_EQ(coll_points[1][1], 2);
    EXPECT_EQ(coll_points[1][2], 0);
    EXPECT_EQ(coll_points[2][0], 2);
    EXPECT_EQ(coll_points[2][1], 4);
    EXPECT_EQ(coll_points[2][2], 0);

    EXPECT_EQ(norms[0][0],  4);
    EXPECT_EQ(norms[0][1],  0);
    EXPECT_EQ(norms[0][2],  0);
    EXPECT_EQ(norms[1][0],  0);
    EXPECT_EQ(norms[1][1],  2);
    EXPECT_EQ(norms[1][2],  0);
    EXPECT_EQ(norms[2][0],  2);
    EXPECT_EQ(norms[2][1],  0);
    EXPECT_EQ(norms[2][2],  0);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    ros::init(argc, argv, "baxter_react_controller");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
