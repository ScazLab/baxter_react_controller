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

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    ros::init(argc, argv, "baxter_react_controller");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
