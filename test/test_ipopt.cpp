#include <gtest/gtest.h>

#include "react_controller/ctrlThread.h"

using namespace std;

// Declare a test
TEST(IPOPTtest, testRightArm10ms)
{
    CtrlThread arm("baxter_react_controller", "right", true, true);

    EXPECT_TRUE(arm.getInternalState());
}

TEST(IPOPTtest, testLeftArm10ms)
{
    CtrlThread arm("baxter_react_controller",  "left", true, true);

    EXPECT_TRUE(arm.getInternalState());
}

TEST(IPOPTtest, testRightArm20ms)
{
    CtrlThread arm("baxter_react_controller", "right", true, true, 0.02);

    EXPECT_TRUE(arm.getInternalState());
}

TEST(IPOPTtest, testLeftArm20ms)
{
    CtrlThread arm("baxter_react_controller",  "left", true, true, 0.02);

    EXPECT_TRUE(arm.getInternalState());
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    ros::init(argc, argv, "baxter_react_controller");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
