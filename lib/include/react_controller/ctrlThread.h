#include <robot_utils/utils.h>
#include <robot_interface/robot_interface.h>
#include <react_controller/controllerNLP.h>

class CtrlThread : public RobotInterface
{
private:
    BaxterChain *chain;

    Eigen::VectorXd x_0;  // Initial end-effector position
    Eigen::VectorXd x_t;  // Current end-effector position
    Eigen::VectorXd x_n;  // Desired next end-effector position
    Eigen::VectorXd x_d;  // Vector that stores the new target

    Eigen::VectorXd o_0;  // Initial end-effector orientation
    Eigen::VectorXd o_t;  // Current end-effector orientation
    Eigen::VectorXd o_n;  // Desired next end-effector orientation
    Eigen::VectorXd o_d;  // Vector that stores the new orientation

    Eigen::VectorXd q_dot;

    Eigen::MatrixXd vLimAdapted;

    ros::Subscriber ctrl_sub;

public:
    CtrlThread(const std::string& _name, const std::string& _limb, bool _no_robot,
               const std::string& _base_link, const std::string& _tip_link);

    void ctrlCb(const baxter_collaboration_msgs::GoToPose& _msg);

    Eigen::VectorXd solveIK(int &_exit_code);

    ~CtrlThread();
};
