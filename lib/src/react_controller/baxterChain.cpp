#include <ros/ros.h>
#include <kdl/chainfksolverpos_recursive.hpp>

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
    ROS_INFO("Reading joints and links from URDF, from %s link to %s link",
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
        // ROS_INFO("Segment %lu, name %s,\tJoint name %s", i, segments[i].getName().c_str(),
        //                                         segments[i].getJoint().getName().c_str());
        joint = _robot.getJoint(segments[i].getJoint().getName());

        if (joint->type != urdf::Joint::UNKNOWN &&
            joint->type != urdf::Joint::FIXED)
        {
            joint_num++;
            float lower, upper;
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

                hasLimits = 1;
            }

            if (hasLimits)
            {
                l(joint_num-1) = lower;
                u(joint_num-1) = upper;
            }
            else
            {
                l(joint_num-1) = numeric_limits<float>::lowest();
                u(joint_num-1) = numeric_limits<float>::max();
            }

            ROS_DEBUG_STREAM("IK Using joint "<<joint->name<<" "<<
                              l(joint_num-1)<<" "<<u(joint_num-1));
        }
    }

    // Assign default values for q
    for (size_t i = 0; i < getNrOfJoints(); ++i)
    {
        // This will initialize the joint in the
        // middle of its operational range
        q[i] = (l[i]+u[i])/2;
    }
}

BaxterChain::BaxterChain(urdf::Model _robot, const string& _base,
                         const string& _tip, vector<double> _q_0):
                         BaxterChain(_robot, _base, _tip)

{
    // TODO : instead of assert, just
    // place a ROS_ERROR and fill q with defaults.
    ROS_ASSERT(getNrOfJoints() == _q_0.size());

    for (size_t i = 0; i < getNrOfJoints(); ++i)
    {
        q[i] = _q_0[i];
    }
}

bool BaxterChain::resetChain()
{
    nrOfJoints=0;
    nrOfSegments=0;
    segments.resize(0);
    q.resize(0);
    l.resize(0);
    u.resize(0);
    v.resize(0);

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
        q[i] = 0;
        l[i] = 0;
        u[i] = 0;
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
            q[i] = _ch.q[i];
            l[i] = _ch.l[i];
            u[i] = _ch.u[i];
        }
    }

    return *this;
}

void BaxterChain::addSegment(const KDL::Segment& segment)
{
    segments.push_back(segment);
    nrOfSegments++;
    if(segment.getJoint().getType()!=KDL::Joint::None)
    {
        nrOfJoints++;

        l.conservativeResize(getNrOfJoints());
        u.conservativeResize(getNrOfJoints());
        q.conservativeResize(getNrOfJoints());
        v.conservativeResize(getNrOfJoints());
    }
}

void BaxterChain::addChain(const KDL::Chain& chain)
{
    for(size_t i=0; i<chain.getNrOfSegments(); ++i)
    {
        this->addSegment(chain.getSegment(i));
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

bool BaxterChain::setAng(sensor_msgs::JointState _q)
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

bool BaxterChain::setAng(VectorXd _q)
{
    if (_q.size() != int(getNrOfJoints()))     { return false; }

    q = _q;
    return true;
}

bool BaxterChain::setVel(VectorXd _v)
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

bool BaxterChain::GetJointPositions(vector<Vector3d>& positions)
{

    size_t segmentNr=getNrOfSegments();

    KDL::JntArray jnts(getNrOfJoints());

    for (size_t i = 0; i < getNrOfJoints() - 1; ++i)
    {
        jnts(i) = q[i];
    }

    int j=0;
    KDL::Frame frame(KDL::Frame::Identity());

    for (size_t i=0; i<segmentNr; ++i)
    {
        if (getSegment(i).getJoint().getType()!=KDL::Joint::None)
        {
            KDL::Vector   posKDL = frame.p;
            Vector3d posEig;
            tf::vectorKDLToEigen(posKDL, posEig);
            positions.push_back(posEig);
            frame = frame*getSegment(i).pose(jnts(j));
            ++j;
        }
        else
        {
            frame = frame*getSegment(i).pose(0.0);
        }
    }

    return true;
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
        l.conservativeResize(getNrOfJoints());
        u.conservativeResize(getNrOfJoints());
        q.conservativeResize(getNrOfJoints());
        v.conservativeResize(getNrOfJoints());
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

double BaxterChain::getMax(const size_t _i)
{
    return u[_i];
}

double BaxterChain::getMin(const size_t _i)
{
    return l[_i];
}

bool BaxterChain::obstacleToCollisionPoint(const Eigen::Vector3d& _obstacle_wrf,
                                           collisionPoint&      _coll_point_erf)
{
    int num_jnts = getNrOfJoints();

    std::vector<Vector3d> positions;
    GetJointPositions(positions);

    // Project the point onto the last segment of the chain
    Vector3d pos_ee = getH().block<3,1>(0,3);
    Vector3d coll_pt_wrf = projectOntoSegment(positions[num_jnts - 1], pos_ee, _obstacle_wrf);

    // Convert the collision point from the wrf to end-effector reference frame
    Vector4d tmp(0, 0, 0, 1);
    tmp.block<3,1>(0,0) = coll_pt_wrf;

    changeFoR(coll_pt_wrf, getH(), _coll_point_erf.x);

    Vector3d obstacle_erf;
    changeFoR(_obstacle_wrf, getH(), obstacle_erf);

    // Compute the norm vector in the end-effector reference frame
    _coll_point_erf.n  = ( obstacle_erf - _coll_point_erf.x);

    double dist = _coll_point_erf.n.norm();
    _coll_point_erf.n /= dist;

    double thres = 0.3;
    // Compute the magnitude
    if (dist > thres)
    {
        _coll_point_erf.m = 0.0;
    }
    else
    {
        _coll_point_erf.m = (thres - dist) / thres;
    }


    // ROS_INFO("coll point %zu at x: %g y: %g z: %g", i,
    //           _coll_points[i].x(0), _coll_points[i].x(1), _coll_points[i].x(2));
    // ROS_INFO("      norm %zu at x: %g y: %g z: %g", i,
    //           _coll_points[i].n(0), _coll_points[i].n(1), _coll_points[i].n(2));

    return true;
}

BaxterChain::~BaxterChain()
{
    return;
}
