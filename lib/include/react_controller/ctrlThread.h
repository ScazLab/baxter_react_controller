#include <robot_utils/utils.h>
#include <robot_interface/robot_interface.h>

#include "react_controller/controllerNLP.h"
#include "react_controller/avoidanceHandler.h"

class CtrlThread : public RobotInterface
{
private:
    BaxterChain *chain;

    Ipopt::SmartPtr<Ipopt::IpoptApplication> app;
    Ipopt::SmartPtr<ControllerNLP> nlp;

    bool       is_debug;  // Flag to enable debug mode (without using the robot)
    bool internal_state;  // Flag to know the internal state. True if OK.

    // IPOPT params
    bool               nlp_ctrl_ori;  // Flag to know if to control the orientation or not
    std::string nlp_derivative_test;  // String to enable the derivative test
    int             nlp_print_level;  // Print level of the IPOPT app

    Eigen::Vector3d    x_n;  // Desired next end-effector position
    Eigen::Quaterniond o_n;  // Desired next end-effector orientation

    Eigen::VectorXd q_dot;   // Vector of initial joint velocities in arm chain

    Eigen::MatrixXd      vLim; // matrix of maximum joint velocities per joint
    Eigen::MatrixXd vlim_coll; // matrix of maximum joint velocities per joint
                               // limited by the collision points

    std::vector<Eigen::Vector3d> obstacles; // Vector of 3D obstacles in the world reference frame

    double    dT;       // time constraint for IpOpt solver time per optimization
    double   tol;       // tolerance for constraint violations
    double  vMax;       // maximum velocity of joints
    bool coll_av;       // collision avoidance mode

public:
    CtrlThread(const std::string& _name, const std::string&        _limb,
                bool _use_robot =  true, double _ctrl_freq = THREAD_FREQ,
                bool  _is_debug = false, bool     _coll_av =       false,
                double     _tol =  1e-3, double      _vMax =      120.0);

    /**
     * Initializes the IpoptApplication with default values for every time the solver
     * is called.
     */
    void initializeNLP();

    /**
     * Reads some NLP options from the parameter server and initializes the app
     */
    void NLPOptionsFromParameterServer();

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
