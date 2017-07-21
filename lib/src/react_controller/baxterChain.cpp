#include <ros/ros.h>

#include "react_controller/baxterChain.h"

using namespace Eigen;
using namespace   std;

/**************************************************************************/
/*                            BaxterChain                                 */
/**************************************************************************/

BaxterChain::BaxterChain(): nrOfJoints(0), nrOfSegments(0), segments(0)
{

}

BaxterChain::BaxterChain(const KDL::Chain& in): BaxterChain()
{
    for(size_t i=0; i<in.getNrOfSegments(); ++i)
    {
        this->addSegment(in.getSegment(i));
    }
}

BaxterChain::BaxterChain(urdf::Model _robot, const string& _base,
                         const string& _tip): BaxterChain()
{
    // Read joints and links from URDF
    ROS_INFO("Reading joints and links from URDF, from %s to %s link",
                                         _base.c_str(), _tip.c_str());

    KDL::Tree tree;
    if (not kdl_parser::treeFromUrdfModel(_robot, tree))
    {
        ROS_FATAL("Failed to extract KDL tree from xml robot description");
    }

    KDL::Chain chain;
    if (not tree.getChain(_base, _tip, chain))
    {
        ROS_FATAL("Couldn't find chain %s to %s",_base.c_str(),_tip.c_str());
    }
    *this = chain;

    // Read upper and lower bounds
    boost::shared_ptr<const urdf::Joint> joint;
    uint joint_num=0;

    for (size_t i = 0; i < segments.size(); ++i)
    {
        // ROS_INFO("[%lu]: name %s,\tJoint name %s", i, segments[i].getName().c_str(),
        //                                    segments[i].getJoint().getName().c_str());
        joint = _robot.getJoint(segments[i].getJoint().getName());

        if (joint->type != urdf::Joint::UNKNOWN &&
            joint->type != urdf::Joint::FIXED)
        {
            ++joint_num;
            double lower, upper, velocity;

            int hasLimits = 0;
            if (joint->type != urdf::Joint::CONTINUOUS )
            {
                if (joint->safety)
                {
                    lower = max(joint->limits->lower,
                                joint->safety->soft_lower_limit);
                    upper = min(joint->limits->upper,
                                joint->safety->soft_upper_limit);
                }
                else
                {
                    lower = joint->limits->lower;
                    upper = joint->limits->upper;
                }

                velocity = joint->limits->velocity;
                hasLimits = 1;
            }

            if (hasLimits)
            {
                q_l(joint_num-1) =    lower;
                q_u(joint_num-1) =    upper;
                v_l(joint_num-1) = velocity;
            }
            else
            {
                q_l(joint_num-1) = numeric_limits<double>::lowest();
                q_u(joint_num-1) = numeric_limits<double>::   max();
                v_l(joint_num-1) = numeric_limits<double>::   max();
            }

            ROS_DEBUG_STREAM("IK Using joint "<<joint->name<<" "<<
                              q_l(joint_num-1)<<" "<<q_u(joint_num-1));
        }
    }

    // ROS_INFO_STREAM("Velocity limits: " << v_l.transpose());

    // Assign default values for q
    for (size_t i = 0; i < getNrOfJoints(); ++i)
    {
        // This will initialize the joint in the
        // middle of its operational range
        q[i] = (q_l[i]+q_u[i])/2;
    }
}

BaxterChain::BaxterChain(urdf::Model _robot, const   string& _base,
                         const string& _tip, const VectorXd&  _q_0):
                         BaxterChain(_robot, _base, _tip)

{
    ROS_ASSERT(int(getNrOfJoints()) == _q_0.size());

    q = _q_0;
}

bool BaxterChain::resetChain()
{
    nrOfJoints=0;
    nrOfSegments=0;
    segments.resize(0);
    q  .resize(0);
    q_l.resize(0);
    q_u.resize(0);
    v  .resize(0);
    v_l.resize(0);

    return true;
}

BaxterChain::operator KDL::Chain()
{
    KDL::Chain res;

    for (size_t i = 0; i < getNrOfSegments(); ++i)
    {
        res.addSegment(getSegment(i));
    }

    return res;
}

BaxterChain& BaxterChain::operator=(const KDL::Chain& _ch)
{
    resetChain();
    for(size_t i=0; i<_ch.getNrOfSegments(); ++i)
    {
        addSegment(_ch.getSegment(i));
    }

    for (size_t i = 0; i < getNrOfJoints(); ++i)
    {
        q  [i] = 0.0;
        q_l[i] = 0.0;
        q_u[i] = 0.0;
        v  [i] = 0.0;
        v_l[i] = 0.0;
    }

    return *this;
}

BaxterChain& BaxterChain::operator=(const BaxterChain& _ch)
{
    // self-assignment check
    if (this != &_ch)
    {
        resetChain();
        for(size_t i=0; i<_ch.getNrOfSegments(); ++i)
        {
            addSegment(_ch.getSegment(i));
        }

        for (size_t i = 0; i < getNrOfJoints(); ++i)
        {
            q  [i] = _ch.q  [i];
            q_l[i] = _ch.q_l[i];
            q_u[i] = _ch.q_u[i];
            v  [i] = _ch.v  [i];
            v_l[i] = _ch.v_l[i];
        }
    }

    return *this;
}

void BaxterChain::addSegment(const KDL::Segment& _seg)
{
    segments.push_back(_seg);
    nrOfSegments++;

    if(_seg.getJoint().getType()!=KDL::Joint::None)
    {
        nrOfJoints++;

        q  .conservativeResize(getNrOfJoints());
        q_l.conservativeResize(getNrOfJoints());
        q_u.conservativeResize(getNrOfJoints());
        v  .conservativeResize(getNrOfJoints());
        v_l.conservativeResize(getNrOfJoints());
    }
}

void BaxterChain::addChain(const KDL::Chain& _ch)
{
    for(size_t i=0; i<_ch.getNrOfSegments(); ++i)
    {
        this->addSegment(_ch.getSegment(i));
    }
}

const KDL::Segment& BaxterChain::getSegment(size_t nr)const
{
    return segments[nr];
}

MatrixXd BaxterChain::GeoJacobian()
{
    return JntToJac().data;
}

bool BaxterChain::setAng(const sensor_msgs::JointState& _q)
{
    if (_q.position.size() != getNrOfJoints()) { return false; }
    if (_q.velocity.size() != getNrOfJoints()) { return false; }

    VectorXd new_q(getNrOfJoints()), new_v(getNrOfJoints());

    for (size_t i = 0; i < getNrOfJoints(); ++i)
    {
        new_q[i] = _q.position[i];
        new_v[i] = _q.velocity[i];
    }

    return setAng(new_q) && setVel(new_v);
}

bool BaxterChain::setAng(const VectorXd& _q)
{
    if (_q.size() != int(getNrOfJoints()))     { return false; }

    // Check for consistency (each joint should be lower than its
    // upper limit and bigger than its lower limit)
    bool res = true;

    for (size_t i = 0; i < getNrOfJoints(); ++i)
    {
        if      (_q[i]>q_u[i])   { res = false; q[i] = q_u[i]; }
        else if (_q[i]<q_l[i])   { res = false; q[i] = q_l[i]; }
        else                     {              q[i] =  _q[i]; }
    }

    return res;
}

bool BaxterChain::setAng(const VectorXd& _q,
                         const VectorXd& _l,
                         const VectorXd& _u)
{
    if (_q.size() != int(getNrOfJoints()) ||
        _l.size() != int(getNrOfJoints()) ||
        _u.size() != int(getNrOfJoints())   )     { return false; }

    for (size_t i = 0; i < getNrOfJoints(); ++i)
    {
        q_l[i] = _l[i];
        q_u[i] = _u[i];
    }

    return setAng(_q);
}

bool BaxterChain::setVel(const VectorXd& _v)
{
    if (_v.size() != int(getNrOfJoints()))     { return false; }

    v = _v;
    return true;
}

KDL::Frame BaxterChain::JntToCart(int _seg_nr)
{
    if (_seg_nr<0) { _seg_nr = int(getNrOfSegments()); }

    KDL::Frame H = KDL::Frame::Identity();

    if (_seg_nr>int(getNrOfSegments()))    { return H; }

    int j=0;
    for (size_t i=0; i < size_t(_seg_nr); ++i)
    {
        if (getSegment(i).getJoint().getType()!=KDL::Joint::None)
        {
            H = H*getSegment(i).pose(q(j));
            ++j;
        }
        else
        {
            H = H*getSegment(i).pose(0.0);
        }
    }

    return H;
}

KDL::Jacobian BaxterChain::JntToJac(int _seg_nr)
{
    if (_seg_nr<0) { _seg_nr = int(getNrOfSegments()); }

    KDL::Jacobian J(getNrOfJoints());
    SetToZero(J);

    if (_seg_nr>int(getNrOfSegments()))    { return J; }

    KDL::Frame T_tmp(KDL::Frame::Identity());
    KDL::Frame total(KDL::Frame::Identity());

    KDL::Twist t_tmp;
    SetToZero(t_tmp);

    int j=0, k=0;

    for (size_t i=0; i<size_t(_seg_nr); ++i)
    {
        //Calculate new Frame_base_ee
        if (getSegment(i).getJoint().getType()!=KDL::Joint::None)
        {
            //pose of the new end-point expressed in the base
            total = T_tmp*getSegment(i).pose(q(j));
            //changing base of new segment's twist to base frame if it is not locked
            t_tmp = T_tmp.M*getSegment(i).twist(q(j),1.0);
        }
        else
        {
            total = T_tmp*getSegment(i).pose(0.0);
        }

        //Changing Refpoint of all columns to new ee
        changeRefPoint(J, total.p-T_tmp.p, J);

        //Only increase jointnr if the segment has a joint
        if (getSegment(i).getJoint().getType()!=KDL::Joint::None)
        {
            //Only put the twist inside if it is not locked
            J.setColumn(k++,t_tmp);
            ++j;
        }

        T_tmp = total;
    }

    return J;
}

geometry_msgs::Pose BaxterChain::getPose()
{
    geometry_msgs::Pose result;

    Matrix4d H = getH();
    result.position.x = H(0,3);
    result.position.y = H(1,3);
    result.position.z = H(2,3);

    Quaterniond o(H.block<3,3>(0,0));
    result.orientation.x = o.x();
    result.orientation.y = o.y();
    result.orientation.z = o.z();
    result.orientation.w = o.w();

    return result;
}

Matrix4d BaxterChain::getH()
{
    return toMatrix4d(JntToCart());
}

Matrix4d BaxterChain::getH(const size_t _i)
{
    // if i > than num_joints
    ROS_ASSERT_MSG(_i < getNrOfJoints(), "_i %lu, num_joints %lu", _i, getNrOfJoints());

    KDL::Frame F;

    size_t j=0, s=0;
    for (s=0; s<getNrOfSegments(); ++s)
    {
        if (getSegment(s).getJoint().getType()!=KDL::Joint::None)
        {
            if (j == _i + 1) { break; }
            ++j;
        }
    }

    return toMatrix4d(JntToCart(s));
}

void BaxterChain::removeSegment()
{
    if(segments.back().getJoint().getType()!=KDL::Joint::None)
    {
        --nrOfJoints;
        q  .conservativeResize(getNrOfJoints());
        q_l.conservativeResize(getNrOfJoints());
        q_u.conservativeResize(getNrOfJoints());
        v  .conservativeResize(getNrOfJoints());
        v_l.conservativeResize(getNrOfJoints());
    }
    segments.pop_back();
    --nrOfSegments;

    return;
}

void BaxterChain::removeJoint()
{
    while(true)
    {
        if(segments.back().getJoint().getType()!=KDL::Joint::None)
        {
            removeSegment();
            break;
        }
        else
        {
            removeSegment();
        }
    }

    return;
}

bool BaxterChain::is_between(Eigen::Vector3d _a, Eigen::Vector3d _b, Eigen::Vector3d _c)
{
    double dot_product = (_b - _a).dot(_c - _a);

    if (dot_product > 0 && dot_product < (_a - _b).norm())
    {
        return true;
    }

    return false;
}

bool BaxterChain::obstacleToCollisionPoint(const Obstacle& _obstacle,
                                           CollisionPoint&  _coll_pt)
{
    _coll_pt.o_wrf = _obstacle.x_wrf;
    _coll_pt.size  = _obstacle.size;

    // Project the point onto the last segment of the chain
    Vector3d pos_ee           = getH(getNrOfJoints()-1).block<3,1>(0,3);
    Vector3d pos_ee_minus_one = getH(getNrOfJoints()-2).block<3,1>(0,3);

    _coll_pt.x_wrf = projectOntoSegment(pos_ee_minus_one, pos_ee, _obstacle.x_wrf);
    _coll_pt.n_wrf = _obstacle.x_wrf - _coll_pt.x_wrf;

    // Convert the collision point from the wrf to end-effector reference frame
    changeFoR(_coll_pt.x_wrf, getH(), _coll_pt.x_erf);

    // Convert the obstacle point from the wrf to end-effector reference frame
    Vector3d obstacle_erf;
    changeFoR(_obstacle.x_wrf, getH(), obstacle_erf);

    // Compute the norm vector in the end-effector reference frame
    _coll_pt.n_erf = ( obstacle_erf - _coll_pt.x_erf);
    ROS_ASSERT_MSG(_coll_pt.n_wrf.norm() -   _coll_pt.n_erf.norm() < EPSILON,
                  "_coll_pt.n_wrf.norm(): %g _coll_pt.n_erf.norm(): %g",
                   _coll_pt.n_wrf.norm(),    _coll_pt.n_erf.norm() );

    // Compute the "distance"
    // ROS_INFO_STREAM("      _obstacle.x_wrf: " <<_obstacle.x_wrf.transpose() <<
    //                      ", _coll_pt.x_wrf: " << _coll_pt.x_wrf.transpose());
    // ROS_INFO("_coll_pt.n_wrf.norm(): %g,  _coll_pt.size: %g",
    //           _coll_pt.n_wrf.norm(),      _coll_pt.size);
    _coll_pt.dist = _coll_pt.n_wrf.norm() - _coll_pt.size;

    if (not is_between(pos_ee, pos_ee_minus_one, _coll_pt.x_wrf))
    {
        // ROS_INFO("(pos_ee           - _coll_pt.x_wrf).norm(): %g"
        //          "(pos_ee_minus_one - _coll_pt.x_wrf).norm(): %g",
        //           (pos_ee           - _coll_pt.x_wrf).norm(),
        //           (pos_ee_minus_one - _coll_pt.x_wrf).norm());
        _coll_pt.dist += min((pos_ee           - _coll_pt.x_wrf).norm(),
                             (pos_ee_minus_one - _coll_pt.x_wrf).norm());
    }

    // Normalize the norm vectors
    _coll_pt.n_wrf = _coll_pt.n_wrf / _coll_pt.n_wrf.norm();
    _coll_pt.n_erf = _coll_pt.n_erf / _coll_pt.n_erf.norm();

    ROS_ASSERT(_coll_pt.n_wrf.norm() - 1 < EPSILON);
    ROS_ASSERT(_coll_pt.n_erf.norm() - 1 < EPSILON);

    // Compute the magnitude
    double alpha = 4.0;
    double thres = 0.4;

    if (_coll_pt.dist > thres)
    {
        _coll_pt.mag = 0.0;
    }
    else
    {
        _coll_pt.mag = 1.0/(1.0+exp((_coll_pt.dist*(2.0/thres)-1.0)*alpha));
    }

    ROS_ASSERT(_coll_pt.mag >= 0.0 && _coll_pt.mag <= 1.0);

    return true;
}

BaxterChain::~BaxterChain()
{
    return;
}

std::vector<RVIZMarker> asRVIZMarkers(BaxterChain _chain, bool _pub_joints,
                                         bool _pub_links, bool _pub_ori)
{
    std::vector<RVIZMarker> res;

    if (_pub_joints || _pub_links)
    {
        std::vector<geometry_msgs::Point> joint_positions;

        for (size_t i = 0; i < _chain.getNrOfJoints(); ++i)
        {
            geometry_msgs::Point joint_position;
            Vector3d joint_pos = _chain.getH(i).block<3,1>(0,3);
            joint_position.x = joint_pos(0);
            joint_position.y = joint_pos(1);
            joint_position.z = joint_pos(2);

            joint_positions.push_back(joint_position);
        }

        if (_pub_links)
        {
            res.push_back(RVIZMarker(joint_positions, ColorRGBA(), 0.012,
                          visualization_msgs::Marker::LINE_STRIP));
        }

        if (_pub_joints)
        {
            res.push_back(RVIZMarker(joint_positions, ColorRGBA(1.0, 0.0, 1.0), 0.025));
        }
    }

    if (_pub_ori)
    {
        // We can represent the end-effector's reference frame as
        // three RVIZMarkers with type ARROW
        for (int i = 0; i < 3; ++i)
        {
            geometry_msgs::Pose pt;

            Eigen::Matrix4d H = _chain.getH();
            Eigen::Quaterniond q(H.block<3,3>(0,0));
            // ROS_INFO_STREAM("[q]: " << q.vec().transpose() << " " << q.w());

            // For axis y and z, let's add a small rotation to the quaternion
            if      (i == 1)
            {
                q = Eigen::Quaterniond(H.block<3,3>(0,0) *
                                       AngleAxisd( 0.5*M_PI, Vector3d::UnitZ()));
            }
            else if (i == 2)
            {
                q = Eigen::Quaterniond(H.block<3,3>(0,0) *
                                       AngleAxisd(-0.5*M_PI, Vector3d::UnitY()));
            }

            q.normalize();

            pt.position.x = H(0,3);
            pt.position.y = H(1,3);
            pt.position.z = H(2,3);
            pt.orientation.x = q.x();
            pt.orientation.y = q.y();
            pt.orientation.z = q.z();
            pt.orientation.w = q.w();

            ColorRGBA col(0.2, 0.2, 0.2, 1.0);

            if      (i == 0)
            {
                col.col.r = 0.8;
            }
            else if (i == 1)
            {
                col.col.g = 0.8;
            }
            else if (i == 2)
            {
                col.col.b = 0.8;
            }

            res.push_back(RVIZMarker(pt, col, 0.1, visualization_msgs::Marker::ARROW));
        }
    }

    return res;
}
