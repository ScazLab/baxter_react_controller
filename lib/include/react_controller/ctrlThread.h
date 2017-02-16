#include <robot_interface/robot_interface.h>
#include <react_controller/controllerNLP.h>
#include <baxter_core_msgs/JointCommand.h>

class CtrlThread : public RobotInterface
{
private:
    std::string _urdf_param;
    KDL::JntArray _lb, _ub;
    BaxterChain _chain;
    ros::Subscriber _jntstate_sub;
    pthread_mutex_t _mutex_jnts;
    VectorXd x_0;  // Initial end-effector position
    VectorXd x_t;  // Current end-effector position
    VectorXd x_n;  // Desired next end-effector position
    VectorXd x_d;  // Vector that stores the new target

    VectorXd o_0;  // Initial end-effector orientation
    VectorXd o_t;  // Current end-effector orientation
    VectorXd o_n;  // Desired next end-effector orientation
    VectorXd o_d;  // Vector that stores the new orientation

    VectorXd q_dot;

    MatrixXd vLimAdapted;

public:
    CtrlThread(const std::string& base_link, const std::string& tip_link);
    VectorXd solveIK(int &_exit_code);
};
