#include <robot_utils/utils.h>
#include <robot_interface/robot_interface.h>
#include <react_controller/controllerNLP.h>
#include <tf/transform_datatypes.h>

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

    Eigen::VectorXd q_dot; // vector of initial joint angles in arm chain

    Eigen::MatrixXd vLimAdapted; // matrix of maximum joint velocities per joint

    double tol;         // tolerance for constraint violations
    double vMax;        // maximum velocity of joints
    double dT;          // time constraint for IpOpt solver time per optimization

public:
    CtrlThread(const std::string& _name, const std::string& _limb, bool _no_robot,
               const std::string& _base_link, const std::string& _tip_link, double tol = 1e-6,
               double vMax = 45.0, double dT = 0.01);

    /**
     * Overridden version of the robot_interfact function. Takes position and
     * orientation values and updates the variables x_n and o_n to be used by
     * IpOpt to obtain velocites for the next pose.
     *
     * @param p_ [position in the _ direction]
     * @param o_ [orientation in the _ direction]
     *
     * @return: true if successful, false if error
    */
    bool goToPoseNoCheck(double px, double py, double pz,
                         double ox, double oy, double oz, double ow);

    Eigen::VectorXd solveIK(int &_exit_code);

    ~CtrlThread();
};
