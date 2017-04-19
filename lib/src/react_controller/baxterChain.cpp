#include <assert.h>
#include <deque>
#include <ros/ros.h>

#include <react_controller/baxterChain.h>
#include <react_controller/mathUtils.h>
#include <eigen_conversions/eigen_kdl.h>

#include <kdl/chainfksolverpos_recursive.hpp>

using namespace Eigen;
using namespace   KDL;
using namespace   std;

Matrix4d KDLFrameToEigen(KDL::Frame _f)
{
    Matrix4d result;
    result.setIdentity();

    //get pose matrix
    KDL::Rotation rotKDL = _f.M;
    KDL::Vector   posKDL = _f.p;

    //convert to Eigen matrix
    Eigen::Quaterniond quatEig;
    Eigen::Vector3d posEig;

    tf::quaternionKDLToEigen(rotKDL, quatEig);
    tf::vectorKDLToEigen(posKDL, posEig);

    Matrix3d rot = quatEig.toRotationMatrix();

    result.block<3,3>(0,0) = rot;
    result.block<3,1>(0,3) = posEig;

    return result;
}

/**************************************************************************/
/*                            BaxterChain                                 */
/**************************************************************************/
BaxterChain::BaxterChain(urdf::Model _robot_model,
                         const string& _base_link,
                          const string& _tip_link)
{
    initChain(_robot_model, _base_link, _tip_link);

    for (size_t i = 0; i < getNrOfJoints(); ++i)
    {
        // This will initialize the joint in the
        // middle of its operational range
        q.push_back((lb.data[i]+ub.data[i])/2);
    }
}

BaxterChain::BaxterChain(urdf::Model _robot_model,
                         const string& _base_link,
                          const string& _tip_link,
                         std::vector<double> _q_0)
{
    initChain(_robot_model, _base_link, _tip_link);

    // TODO : better interface: instead of assert, just
    // place a ROS_ERROR and fill q with zeros.
    ROS_ASSERT(getNrOfJoints() == _q_0.size());

    for (size_t i = 0; i < getNrOfJoints(); ++i)
    {
        q.push_back(_q_0[i]);
    }
}

void BaxterChain::initChain(urdf::Model _robot_model,
                       const std::string& _base_link,
                        const std::string& _tip_link)
{
    ROS_INFO("Reading joints and links from URDF");
    KDL::Tree tree;

    if (!kdl_parser::treeFromUrdfModel(_robot_model, tree))
      ROS_FATAL("Failed to extract kdl tree from xml robot description");

    if(!tree.getChain(_base_link, _tip_link, *this))
      ROS_FATAL("Couldn't find chain %s to %s",_base_link.c_str(),_tip_link.c_str());

    std::vector<KDL::Segment> kdl_chain_segs = segments;

    boost::shared_ptr<const urdf::Joint> joint;

    std::vector<double> l_bounds, u_bounds;

    lb.resize(getNrOfJoints());
    ub.resize(getNrOfJoints());

    uint joint_num=0;

    for(unsigned int i = 0; i < kdl_chain_segs.size(); ++i)
    {
        joint = _robot_model.getJoint(kdl_chain_segs[i].getJoint().getName());

        if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
        {
            joint_num++;
            float lower, upper;
            int hasLimits = 0;

            if ( joint->type != urdf::Joint::CONTINUOUS )
            {
                if(joint->safety)
                {
                    lower = std::max(joint->limits->lower, joint->safety->soft_lower_limit);
                    upper = std::min(joint->limits->upper, joint->safety->soft_upper_limit);
                }
                else
                {
                    lower = joint->limits->lower;
                    upper = joint->limits->upper;
                }

                hasLimits = 1;
            }

            if(hasLimits)
            {
                lb(joint_num-1)=lower;
                ub(joint_num-1)=upper;
            }
            else
            {
                lb(joint_num-1)=std::numeric_limits<float>::lowest();
                ub(joint_num-1)=std::numeric_limits<float>::max();
            }

            ROS_INFO_STREAM("IK Using joint "<<joint->name<<" "<<
                                               lb(joint_num-1)<<" "<<
                                               ub(joint_num-1));
        }
    }


}

// MatrixXd  BaxterChain::GeoJacobian(const unsigned int i)
// {
//     // yAssert(i<N);

//     Matrix J(6,i+1);
//     Matrix PN,Z;
//     Vector w;

//     deque<Matrix> intH;
//     intH.push_back(H0);

//     for (size_t j=0; j<=i; j++)
//         intH.push_back(intH[j]*_q[j].getH(true));

//     PN=intH[i+1];
//     if (i>=N-1)
//         PN=PN*HN;

//     for (size_t j=0; j<=i; j++)
//     {
//         Z=intH[j];
//         w=cross(Z,2,PN-Z,3);

//         J(0,j)=w[0];
//         J(1,j)=w[1];
//         J(2,j)=w[2];
//         J(3,j)=Z(0,2);
//         J(4,j)=Z(1,2);
//         J(5,j)=Z(2,2);
//     }

//     return J;
// }



MatrixXd BaxterChain::GeoJacobian()
{

    KDL::Jacobian J;
    J.resize(getNrOfJoints());
    KDL::JntArray jnts(getNrOfJoints());

    for (size_t i = 0, _i = getNrOfJoints(); i < _i; ++i)
    {
        jnts(i) = q[i];
    }

    JntToJac(jnts, J);
    return J.data;
}

VectorXd BaxterChain::getAng()
{
    return Map<VectorXd>(q.data(), q.size());
}

bool BaxterChain::setAng(sensor_msgs::JointState _q)
{
    std::vector<double> angles;
    for (size_t i = 0; i < getNrOfJoints(); ++i)
    {
        angles.push_back(_q.position[i]);
    }
    setAng(angles);
    return true;
}

bool BaxterChain::setAng(Eigen::VectorXd _q)
{
    q = std::vector<double>(_q.data(), _q.data()+_q.size());
    return true;
}

bool BaxterChain::setAng(std::vector<double> _q)
{
    q = _q;
    return true;
}

bool BaxterChain::JntToCart(const JntArray& _q_in, Frame& _p_out, int _seg_nr)
{
    unsigned int segmentNr;
    if(_seg_nr<0)
        segmentNr=getNrOfSegments();
    else
        segmentNr = _seg_nr;

    _p_out = Frame::Identity();

    if(_q_in.rows()!=getNrOfJoints())
        return false;
    else if(segmentNr>getNrOfSegments())
        return false;
    else
    {
        int j=0;
        for(unsigned int i=0;i<segmentNr;i++)
        {
            if(getSegment(i).getJoint().getType()!=Joint::None)
            {
                _p_out = _p_out*getSegment(i).pose(_q_in(j));
                j++;
            }
            else
            {
                _p_out = _p_out*getSegment(i).pose(0.0);
            }
        }
        return true;
    }
}

bool BaxterChain::JntToJac(const JntArray& q_in, Jacobian& jac, int seg_nr)
{
    unsigned int segmentNr;
    if(seg_nr<0)
         segmentNr=getNrOfSegments();
    else
         segmentNr = seg_nr;

    //Initialize Jacobian to zero since only segmentNr colunns are computed
    SetToZero(jac) ;

    if(q_in.rows()!=getNrOfJoints()||getNrOfJoints()!=jac.columns())
        return false;
    else if(segmentNr>getNrOfSegments())
        return false;

    KDL::Frame T_tmp;
    KDL::Twist t_tmp;
    std::vector<bool> locked_joints_(getNrOfJoints(), false);

    T_tmp = KDL::Frame::Identity();
    SetToZero(t_tmp);
    int j=0;
    int k=0;
    Frame total;
    for (unsigned int i=0;i<segmentNr;i++) {
        //Calculate new Frame_base_ee
        if(getSegment(i).getJoint().getType()!=KDL::Joint::None){
            //pose of the new end-point expressed in the base
            total = T_tmp*getSegment(i).pose(q_in(j));
            //changing base of new segment's twist to base frame if it is not locked
            //t_tmp = T_tmp.M*chain.getSegment(i).twist(1.0);
            if(!locked_joints_[j])
                t_tmp = T_tmp.M*getSegment(i).twist(q_in(j),1.0);
        }else{
            total = T_tmp*getSegment(i).pose(0.0);

        }

        //Changing Refpoint of all columns to new ee
        changeRefPoint(jac,total.p-T_tmp.p,jac);

        //Only increase jointnr if the segment has a joint
        if(getSegment(i).getJoint().getType()!=KDL::Joint::None){
            //Only put the twist inside if it is not locked
            if(!locked_joints_[j])
                jac.setColumn(k++,t_tmp);
            j++;
        }

        T_tmp = total;
    }
    return true;
}

MatrixXd BaxterChain::getH()
{
    return getH(q.size() - 1);
}

MatrixXd BaxterChain::getH(const unsigned int _i)
{
    //num joints in chain
    size_t num_joints = q.size();

    // TODO also here, remove the assert, place a ROS_ERROR, and return
    // if i > than num_joints
    ROS_ASSERT_MSG(_i < num_joints, "_i %i, num_joints %lu", _i, num_joints);

    KDL::JntArray jnts(getNrOfJoints());

    for (size_t i = 0; i < _i; ++i)
    {
        jnts(i) = q[i];
    }

    KDL::Frame frame;
    JntToCart(jnts,frame);

    // if (!JntToCart(jnts, frame))
    // {
    //     ROS_ERROR("Something went wrong with jnt to cart");
    //     ROS_ERROR("getNrOfJoints %u getNrOfSegments %u jnts.size() %u",
    //                      getNrOfJoints(), getNrOfSegments(), jnts.rows());
    // }

    // return KDLFrameToEigen(getSegment(_i).pose(q[_i]));
    return KDLFrameToEigen(frame);
}

double BaxterChain::getMax(const unsigned int _i) {
    return ub.data[_i];
}

double BaxterChain::getMin(const unsigned int _i) {
    return lb.data[_i];
}

BaxterChain::~BaxterChain()
{
    return;
}
