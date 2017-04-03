#include <gtest/gtest.h>

using namespace std;

// Declare a test
TEST(IPOPTtest, myTest)
{
    // string limb = "right";

    // CtrlThread arm("baxter_react_controller", limb, true, "base", limb+"_gripper", true);

    EXPECT_TRUE(true);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
