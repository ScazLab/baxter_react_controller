#ifndef __BAXTERCHAIN_H__
#define __BAXTERCHAIN_H__

#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <urdf/model.h>
#include <sensor_msgs/JointState.h>
#include <robot_utils/utils.h>
#include "react_controller/react_control_utils.h"

// New rule:
// 1 input parameters for functions are prefixed with a _ (e.g. _q_0),
// 2 members of the class do not have a _ (e.g. q)
//
// New rule:
// please put the bracket for a beginning of a function in a new line.
// I know that is a matter of personal choices, but this is what I use
// in my code and you would need to do it anyway when we'll integrate
// with my code

/****************************************************************/


class BaxterChain : public KDL::Chain
{
private:
    std::vector<double> q;    // vector of joint angles in the arm chain
    KDL::JntArray lb, ub;     // lower bound, upper bound joint arrays
    size_t nrOfJoints;
    size_t nrOfSegments;

public:

    std::vector<KDL::Segment> segments;

    /** CONSTRUCTORS **/
    BaxterChain();
    BaxterChain(const KDL::Chain& in);

    /**
     * Constructor for Baxter Chain. Takes a urdf robot model and base/tip link to
     * initialize KDL::Chain. Automatically initializes q to average of lower and
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
     * initialize KDL::Chain. Initializes q to values in _q_0.
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
     * Assignment operator
     */
    BaxterChain& operator = (const KDL::Chain& arg);
    // BaxterChain& operator = (const BaxterChain& arg);

    /**
     * Adds a new segment to the <strong>end</strong> of the chain.
     *
     * @param segment The segment to add
     */
    void addSegment(const KDL::Segment& segment);

    /**
     * Adds a complete chain to the <strong>end</strong> of the chain
     * The added chain is copied.
     *
     * @param chain The chain to add
     */
    void addChain(const KDL::Chain& chain);

    /**
     * Request the total number of joints in the chain.\n
     * <strong> Important:</strong> It is not the
     * same as the total number of segments since a segment does not
     * need to have a joint. This function is important when
     * creating a KDL::JntArray to use with this chain.
     * @return total nr of joints
     */
    size_t getNrOfJoints()const {return nrOfJoints;};

    /**
     * Request the total number of segments in the chain.
     * @return total number of segments
     */
    size_t getNrOfSegments()const {return nrOfSegments;};

    /**
     * Request the nr'd segment of the chain. There is no boundary
     * checking.
     *
     * @param nr the nr of the segment starting from 0
     *
     * @return a constant reference to the nr'd segment
     */
    const KDL::Segment& getSegment(size_t nr)const;

    /**
     * Takes an arm chain and returns the KDL::Frame of the end effector w.r.t
     * the base of the arm.
     *
     * @param _q_in:      array of joint angles
     * @param _p_out:     KDL::Frame of end effector, populated by function
     * @param _segmentNr: segment to get frame for, default is end effector
     *
     * return:     true/false if success/failure
     */
    bool JntToCart(const KDL::JntArray& _q_in, KDL::Frame& _p_out, int seg_nr=-1);

    /**
     * TODO
     * @param  q_in   [description]
     * @param  jac    [description]
     * @param  seg_nr [description]
     * @return        [description]
     */
    bool JntToJac(const KDL::JntArray& q_in, KDL::Jacobian& jac, int seg_nr=-1);

    /**
     * Gets all collision points and normal vectors for each segment in the arm.
     *
     * @return true/false if success/failure
     */
    bool GetJointPositions(std::vector<Eigen::Vector3d>& positions);

    /**
     * Function to return the geometric Jacobian of the _i'th segment
     * in the chain, or the end effector if no parameter given.
     *
     * @return geometric Jacobian in the form of an Eigen Matrix
     */
    Eigen::MatrixXd GeoJacobian();

    /**
     * Function to get array of joint angles for the chain
     *
     * return: array of joint angles in the form of Eigen::Vector
     */
    Eigen::VectorXd     getAng();
    double              getAng(const size_t _i) { return getAng()[_i]; };

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
     * Function that returns the current pose as a geometry_msgs::Pose
     *
     * @return the current pose as a geometry_msgs::Pose
     */
    geometry_msgs::Pose getPose();

    /**
     * Functions to get pose matrix of a joint in the chain. If called
     * without a parameter, gets pose matrix of end effector.
     *
     * return: pose matrix of _i'th (or end effector) joint
     */
    Eigen::Matrix4d getH();
    Eigen::Matrix4d getH(const size_t _i);

    void removeSegment();
    void removeJoint();

    /**
     * Functions to get joint angle limits for the _i'th joint
     *
     * @param _i [number of the joint to get max/min limit]
     *
     * return: value of joint limit
     */
    double getMax(const size_t _i);
    double getMin(const size_t _i);

    ~BaxterChain();
};

#endif
