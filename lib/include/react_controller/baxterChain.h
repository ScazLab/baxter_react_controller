#ifndef __BAXTERCHAIN_H__
#define __BAXTERCHAIN_H__

#include "gtest/gtest_prod.h"

#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <urdf/model.h>
#include <sensor_msgs/JointState.h>
#include <robot_utils/rviz_publisher.h>

#include "react_controller/react_control_utils.h"

/**
 * Class for encapsulating a KDL chain with its state
 */
class BaxterChain
{
private:
    size_t nrOfJoints;    // number of joints
    size_t nrOfSegments;  // number of segments

    Eigen::VectorXd q;    // vector of joint angles in the arm chain
    Eigen::VectorXd l;    // vector of lower bounds for the joints
    Eigen::VectorXd u;    // vector of upper bounds for the joints
    Eigen::VectorXd v;    // vector of joint velocities of the arm chain

    /**
     * Takes an arm chain and returns the KDL::Frame of the end effector w.r.t.
     * the base of the arm.
     *
     * @param _seg_nr segment to get frame for, default is end effector
     * return         the KDL::Frame of the end effector
     */
    KDL::Frame JntToCart(int _seg_nr=-1);

    /**
     * Computes the jacobian for the chain
     *
     * @param _seg_nr segment to get jacobian for, default is end effector
     * return         the jacobian
     */
    KDL::Jacobian JntToJac(int _seg_nr=-1);

    /**
     * Let's add a number of friend tests to test the private methods of this class (without ROS).
     */
    FRIEND_TEST(BaxterChainTest, testFWDKin);

public:
    /**
     * Array of KDL::Segments that compose the chain.
     */
    std::vector<KDL::Segment> segments;

    /** CONSTRUCTORS **/
    BaxterChain();
    BaxterChain(const KDL::Chain& in);

    /**
     * Takes a urdf robot model and base/tip link to initialize KDL::Chain.
     * Automatically initializes q to average of lower and upper bounds.
     *
     * @param _robot  [urdf::Model of the robot]
     * @param _base   [base link string of robot chain]
     * @param _tip    [tip link string of robot chain]
     */
    BaxterChain(urdf::Model        _robot,
                const std::string&  _base,
                const std::string&   _tip);

    /**
     * Takes a urdf robot model and base/tip link to initialize KDL::Chain.
     * Initializes q to values in _q_0.
     *
     * @param _robot  [urdf::Model of the robot]
     * @param _base   [base link string of robot chain]
     * @param _tip    [tip link string of robot chain]
     * @param _q_0    [vector of initial joint angles]
     */
    BaxterChain(urdf::Model        _robot,
                const std::string&  _base,
                const std::string&   _tip,
                std::vector<double> _q_0);

    /**
     * Resets the chain
     *
     * @return true/false if success/failure
     */
    bool resetChain();

    /**
     * Cast to KDL::Chain
     */
    operator KDL::Chain();

    /**
     * Assignment operator
     */
    BaxterChain& operator=(const KDL::Chain&  _ch);
    BaxterChain& operator=(const BaxterChain& _ch);

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
     * Request the nth segment of the chain. There is no boundary
     * checking.
     *
     * @param nr the nr of the segment starting from 0
     * @return a constant reference to the nth segment
     */
    const KDL::Segment& getSegment(size_t nr)const;

    /**
     * Function to return the geometric Jacobian of the _i'th segment
     * in the chain, or the end effector if no parameter given.
     *
     * @return geometric Jacobian in the form of an Eigen Matrix
     */
    Eigen::MatrixXd GeoJacobian();

    /**
     * Gets the joint angles in the arm chain
     *
     * @return array of joint angles as an Eigen::VectorXd
     */
    Eigen::VectorXd getAng() { return q; };

    /**
     * Gets the configuration of i-th joint
     *
     * @param _i joint to return the configuration of
     * @return   the joint configuration
     */
    double getAng(const size_t _i) { return getAng()[_i]; };

    /**
     * Gets the joint velocities in the arm chain
     *
     * @return array of joint velocities as an Eigen::VectorXd
     */
    Eigen::VectorXd getVel()    { return v; };

    /**
     * Functions to set the joint angles of the arm chain.
     *
     * @param _q  vector of joint positions (in rad)
     * @return    true/false if success/failure
     */
    bool     setAng(Eigen::VectorXd         _q);
    bool     setAng(sensor_msgs::JointState _j);

    /**
     * Functions to set the joint velocities of the arm chain.
     *
     * @param _v  vector of joint velocities
     * @return    true/false if success/failure
     */
    bool     setVel(Eigen::VectorXd         _v);

    /**
     * Function that returns the current pose as a geometry_msgs::Pose
     *
     * @return the current pose as a geometry_msgs::Pose
     */
    geometry_msgs::Pose getPose();

    /**
     * Gets pose matrix of chain end effector.
     *
     * @return pose matrix of end-effector joint
     */
    Eigen::Matrix4d getH();

    /**
     * Gets pose matrix of the i'th joint in the chain. The means it gets the
     * transformation matrix of the last segment before the next joint. For example,
     * if the first segment is a joint, the second is not a joint, and the third is a joint,
     * getH(0) will return the transformation matrix as if the second segment were the
     * end effector of the chain.
     *
     * @param _i [index of joint in chain]
     *
     * @return pose matrix of _i'th joint
     */
    Eigen::Matrix4d getH(const size_t _i);

    /**
     * Removes a segment from the chain. The segment may or may not include a joint.
     * Decrements nrOfSegments and if there is a joint also being removed, decrements
     * nrOfJoints and pops a value off of q.
     */
    void removeSegment();

    /**
     * Removes a joint from the chain. The joint may not be the last segment in the chain,
     * so the function removes segments until it removes a segment that also includes a joint.
     * Decrements nrOfSegments and nrOfJoints appropriately.
     */
    void removeJoint();

    /**
     * Functions to get joint angle limits for the _i'th joint
     *
     * @param _i [number of the joint to get max/min limit]
     *
     * @return value of joint limit
     */
    double getMax(const size_t _i);
    double getMin(const size_t _i);


    bool is_between(Eigen::Vector3d _a, Eigen::Vector3d _b, Eigen::Vector3d _c);

    /**
     * Computes a collision point given the coordinates of the obstacle.
     *
     * @param  _obstacle_wrf    3D coordinates of the obstacle in the world reference frame
     * @param  _coll_point_erf  collisionPoint onto the limb in the end-effector reference frame
     * @return                  true on success, false otherwise
     */
    bool obstacleToCollisionPoint(const Eigen::Vector3d& _obstacle_wrf,
                                  collisionPoint&      _coll_point_erf);

    /**
     * Friend function to convert the baxter chain as a set of RVIZmarkers for
     * visualization in RVIZ.
     *
     * @param _chain      the BaxterChain to visualize.
     * @param _pub_joints if to publish the joints as a set of spheres (default  true)
     * @param _pub_links  if to publish the  links as a set of  sticks (default  true)
     * @param _pub_ori    if to publish the end-effector orientation   (default false)
     * @return            a vector of RVIZ Markers.
     *                    They can be directly provided to an RVIZPublisher object.
     */
    friend std::vector<RVIZMarker> asRVIZMarkers(BaxterChain _chain,
                                                 bool _pub_joints =  true,
                                                 bool _pub_links  =  true,
                                                 bool _pub_ori    = false);

    ~BaxterChain();
};

#endif
