#include <assert.h>
#include <deque>
#include <ros/ros.h>

#include <react_controller/baxterChain.h>
#include <react_controller/mathUtils.h>
#include <eigen_conversions/eigen_kdl.h>

using namespace Eigen;

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
BaxterChain::BaxterChain(KDL::Chain _chain) : KDL::Chain(_chain)
{
    for (size_t i = 0; i < getNrOfJoints(); ++i)
    {
        q.push_back(0.0);
    }
}

BaxterChain::BaxterChain(KDL::Chain _chain, std::vector<double> _q_0)  : KDL::Chain(_chain)
{
    // TODO : better interface: instead of assert, just
    // place a ROS_ERROR and fill q with zeros.
    assert(getNrOfJoints() == _q_0.size());

    for (size_t i = 0; i < getNrOfJoints(); ++i)
    {
        q.push_back(_q_0[i]);
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
    assert(_i < num_joints);

    return KDLFrameToEigen(getSegment(_i).pose(0.0));
}

BaxterChain::~BaxterChain()
{
    return;
}
