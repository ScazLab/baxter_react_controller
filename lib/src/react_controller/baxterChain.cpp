#include <assert.h>
#include <deque>
#include <ros/ros.h>

#include <react_controller/baxterChain.h>
#include <react_controller/mathUtils.h>
#include <eigen_conversions/eigen_kdl.h>

using namespace Eigen;
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
    size_t DOF = q.size();

    MatrixXd J(6,DOF);
    MatrixXd PN,Z;
    VectorXd w;

    std::deque<MatrixXd> intH;
    intH.push_back(getH(0));

    for (size_t i=0; i<DOF; i++)
    {
        intH.push_back(intH[i]*getH(i));
    }

    PN=intH[DOF];

    for (size_t i=0; i<DOF; i++)
    {
        Z=intH[i];
        w=cross(Z,2,PN-Z,3); // TODO define cross

        J(0,i)=w[0];
        J(1,i)=w[1];
        J(2,i)=w[2];
        J(3,i)=Z(0,2);
        J(4,i)=Z(1,2);
        J(5,i)=Z(2,2);
    }
    return J;
}

VectorXd BaxterChain::getAng()
{
    return Map<VectorXd>(q.data(), q.size());
}

bool BaxterChain::setAng(sensor_msgs::JointState jnt_state) {
    std::vector<double> angles;
    for (size_t i = 0; i < getNrOfJoints(); ++i) {
        angles.push_back(jnt_state.position[i]);
    }
    setAng(angles);
    return true;
}

bool BaxterChain::setAng(Eigen::VectorXd _q)
{
    std::vector<double> v(*_q.data(), _q.size());
    q = v;
    return true;
}

bool BaxterChain::setAng(std::vector<double> _q)
{
    q = _q;
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

    return KDLFrameToEigen(getSegment(_i).pose(q[_i]));
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
