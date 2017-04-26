#include <robot_utils/utils.h>
#include <robot_interface/robot_interface.h>

#include "react_controller/controllerNLP.h"
#include "react_controller/avoidance.h"

class CtrlThread : public RobotInterface
{
private:
    BaxterChain *chain;

    Ipopt::SmartPtr<Ipopt::IpoptApplication> app;

    Ipopt::SmartPtr<ControllerNLP> nlp;

    bool is_debug;        // Flag to enable debug mode (without using the robot)
    bool internal_state;  // Flag to know the internal state. True if OK.

    Eigen::Vector3d    x_n;  // Desired next end-effector position
    Eigen::Quaterniond o_n;  // Desired next end-effector orientation

    Eigen::VectorXd q_dot; // vector of initial joint angles in arm chain

    Eigen::MatrixXd vLim; // matrix of maximum joint velocities per joint
    Eigen::MatrixXd vLimCollision; // matrix of maximum joint velocities per joint

    double dT;          // time constraint for IpOpt solver time per optimization
    double tol;         // tolerance for constraint violations
    double vMax;        // maximum velocity of joints
    bool coll_av;       // collision avoidance mode

public:
    CtrlThread(const std::string& _name, const std::string& _limb, bool _no_robot, double _ctrl_freq,
               bool _is_debug = false, double tol = 1e-7, double vMax = 60.0, bool _coll_av = false);

    /**
     * Initializes the IpoptApplication with default values for every time the solver
     * is called.
     *
     * @param _verbosity verbosity flag
     */
    void initializeNLP(bool _verbosity);

    /**
     * Overridden version of the robot_interface function. Takes position and
     * orientation values and updates the variables x_n and o_n to be used by
     * IpOpt to obtain velocites for the next pose.
     *
     * @param p_ [position in the _ direction]
     * @param o_ [orientation in the _ direction]
     *
     * @return true/false if success/failure
    */
    bool goToPoseNoCheck(double px, double py, double pz,
                         double ox, double oy, double oz, double ow);

    /**
     * Method used to debug IPOPT. Does not take any input arguments, because it
     * asks the solver to find a joint configuration suitable for the current
     * state of the chain, i.e. for the current position and orientation.
     *
     * @return true/false if success/failure
    */
    bool debugIPOPT();

    /**
     * Method used to get the internal state of the controller.
     * @return true/false if OK/notOK
     */
    bool getInternalState() { return internal_state; };

    Eigen::VectorXd solveIK(int &_exit_code);

    ~CtrlThread();
};
