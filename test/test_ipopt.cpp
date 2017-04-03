#include <gtest/gtest.h>

#include "react_controller/ctrlThread.h"

using namespace std;

// Declare a test
TEST(IPOPTtest, testRightArm)
{
    CtrlThread arm("baxter_react_controller", "right", true, "base", "right_gripper", true);

    EXPECT_TRUE(arm.getInternalState());
}

TEST(IPOPTtest, testLeftArm)
{
    CtrlThread arm("baxter_react_controller",  "left", true, "base",  "left_gripper", true);

    EXPECT_TRUE(arm.getInternalState());
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    ros::init(argc, argv, "baxter_react_controller");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
