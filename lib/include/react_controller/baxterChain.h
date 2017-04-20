#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <urdf/model.h>
#include <sensor_msgs/JointState.h>
#include <robot_utils/utils.h>

// Things to remember:
//  1 allList == quickList
//  2 N == DOF
//
//  3 HN == Identity matrix (for now) -> every time there is something
//                                       multiplied by HN, just discard it
//  3 H0 == Identity matrix (for now) -> every time there is something
//                                       multiplied by HN, just discard it
//
// New rule:
// 1 input parameters for functions are prefixed with a _ (e.g. _q_0),
// 2 members of the class do not have a _ (e.g. q)
//
// New rule:
// please put the bracket for a beginning of a function in a new line.
// I know that is a matter of personal choices, but this is what I use
// in my code and you would need to do it anyway when we'll integrate
// with my code

/**
 * Takes a KDL::Frame and returns a 4X4 pose Eigen::Matrix
 *
 * @param _f: KDL::Frame to turn into a pose Eigen::Matrix
 *
 * return: Eigen 4X4 pose matrix
*/
Eigen::Matrix4d KDLFrameToEigen(KDL::Frame _f);

/****************************************************************/
class BaxterChain : public KDL::Chain
{
private:

    std::vector<double> q;    // vector of joint angles in the arm chain
    KDL::JntArray lb, ub;     // lower bound, upper bound joint arrays

public:

    /**
     * Takes an arm chain and returns the KDL::Frame of the end effector w.r.t
     * the base of the arm.
     *
     * @param _q_in:      array of joint angles
     * @param _p_out:     KDL::Frame of end effector, populated by function
     * @param _segmentNr: segment to get frame for, default is end effector
     *
     * return:     true if successful, false if error
     */
    bool JntToCart(const KDL::JntArray& _q_in, KDL::Frame& _p_out, int _segNr=-1);

    /**
     * Gets all collision points and normal vectors for each segment in the arm.
     *
     * @return true if successful, flase if error
     */
    bool GetCollisionPoints();

    bool ComputeCollisionPoint(std::vector<Eigen::Vector3d> joints, Eigen::Vector3d coll_coords);

    /**
     * TODO
     * @param  q_in   [description]
     * @param  jac    [description]
     * @param  seg_nr [description]
     * @return        [description]
     */
    bool JntToJac(const KDL::JntArray& q_in, KDL::Jacobian& jac, int seg_nr=-1);

    /**
     * Constructor for Baxter Chain. Takes a urdf robot model and base/tip link to
     * intialize KDL::Chain. Automatically initializes q to average of lower and
     * upper bounds.
     *
     * @param _robot_model [urdf::Model of the robot]
     * @param _base_link [base link string of robot chain]
     * @param _tip_link [tip link string of robot chain]
     */
    BaxterChain(urdf::Model      _robot_model,
                const std::string& _base_link,
                const std::string&  _tip_link);


    /**
     * Constructor for Baxter Chain. Takes a urdf robot model and base/tip link to
     * intialize KDL::Chain. Initializes q to values in _q_0.
     *
     * @param _robot_model [urdf::Model of the robot]
     * @param _base_link [base link string of robot chain]
     * @param _tip_link [tip link string of robot chain]
     * @param _q_0 [vector of initial joint angles]
     */
    BaxterChain(urdf::Model      _robot_model,
                const std::string& _base_link,
                const std::string&  _tip_link,
                std::vector<double>      _q_0);

    /**
     * TODO
     * @param _robot_model [description]
     * @param _base_link   [description]
     * @param _tip_link    [description]
     */
    void initChain(urdf::Model      _robot_model,
                   const std::string& _base_link,
                   const std::string&  _tip_link);

    /**
     * Function to return the geometric Jacobian of the _i'th segment
     * in the chain, or the end effector if no parameter given.
     *
     * @param _i [index of segment to return]
     *
     * @return geometric Jacobian in he form of an Eigen Matrix
     */
    Eigen::MatrixXd GeoJacobian();
    Eigen::MatrixXd GeoJacobian(const unsigned int _i);

    /**
     * Function to get array of joint angles for the chain
     *
     * return: array of joint angles in the form of Eigen::Vector
     */
    Eigen::VectorXd     getAng();
    double              getAng(const unsigned int _i) { return getAng()[_i]; };

    /**
     * Functions to set the joint angles of the arm chain. Used
     * to update arm chain throughout control process.
     *
     * @param _q [array of angles]
     */
    bool     setAng(std::vector<double>     _q);
    bool     setAng(Eigen::VectorXd         _q);
    bool     setAng(sensor_msgs::JointState _q);

    /**
     * Functions to get pose matrix of a joint in the chain. If called
     * without a parameter, gets pose matrix of end effector.
     *
     * return: pose matrix of _i'th (or end effector) joint
     */
    Eigen::MatrixXd getH();
    Eigen::MatrixXd getH(const unsigned int _i);

    /**
     * Functions to get joint angle limits for the _i'th joint
     *
     * @param _i [number of the joint to get max/min limit]
     *
     * return: value of joint limit
     */
    double getMax(const unsigned int _i);
    double getMin(const unsigned int _i);

    ~BaxterChain();
};
