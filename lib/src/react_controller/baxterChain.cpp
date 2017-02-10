#include <assert.h>
#include <deque>

#include <react_controller/baxterChain.h>
#include <eigen_conversions/eigen_kdl.h>

BaxterChain::BaxterChain() {

}

BaxterChain::BaxterChain(KDL::Chain _chain) : KDL::Chain(_chain) {
    // cycle through the chain to get the number of joints
    // int nS = ?chain numbersegments
    int num_joints = _chain.getNrOfJoints();

    for (int i = 0; i < num_joints; ++i)
    {
        _q.push_back(0.0);
    }


}

BaxterChain::BaxterChain(KDL::Chain _chain, std::vector<double> _joint_angles_0)  : KDL::Chain(_chain) {
    // cycle through the chain to get the number of joints
    // int nS = ?chain numbersegments
    int num_joints = _chain.getNrOfJoints();
    assert(num_joints == _joint_angles_0.size());

    for (int i = 0; i < num_joints; ++i)
    {
        _q.push_back(_joint_angles_0[i]);
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

//     for (unsigned int j=0; j<=i; j++)
//         intH.push_back(intH[j]*_q[j].getH(true));

//     PN=intH[i+1];
//     if (i>=N-1)
//         PN=PN*HN;

//     for (unsigned int j=0; j<=i; j++)
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

    int DOF = _q.size();

    MatrixXd J(6,DOF);
    MatrixXd PN,Z;
    VectorXd w;

    std::deque<MatrixXd> intH;
    intH.push_back(getH(0));

    for (unsigned int i=0; i<N; i++) {
        intH.push_back(intH[i]*getH(getSegment(i)));
    }

    PN=intH[N]*HN;

    for (unsigned int i=0; i<DOF; i++)
    {
        unsigned int j=hash[i];

        Z=intH[j];
        w=cross(Z,2,PN-Z,3);

        J(0,i)=w[0];
        J(1,i)=w[1];
        J(2,i)=w[2];
        J(3,i)=Z(0,2);
        J(4,i)=Z(1,2);
        J(5,i)=Z(2,2);
    }
}



VectorXd BaxterChain::getAng() {
    return Map<VectorXd>(_q.data(), _q.size());
}

// std::vector<double> BaxterChain::getAngStd() {
//     return _joint_angles;
// }

bool BaxterChain::setAng(std::vector<double> joint_angles) {
    _q = joint_angles;
    return true;
}

MatrixXd BaxterChain::getH() {

    int num_joints = _q.size();

    return getH(num_joints - 1);
}

MatrixXd BaxterChain::getH(const unsigned int i) {
    //num joints in chain
    int num_joints = _q.size();

    assert(i < num_joints);

    //get i'th segment frame w.r.t base frame
    KDL::Segment seg = getSegment(i);
    return getH(seg);
}

MatrixXd BaxterChain::getH(KDL::Segment seg) {
    KDL::Frame f = seg.pose(0.0);

    //get pose matrix
    KDL::Rotation mat = f.M;
    KDL::Vector posKDL = f.p;
    //convert to Eigen matrix
    Eigen::Quaterniond quart;
    Eigen::Vector3d posEig;
    tf::quaternionKDLToEigen(mat, quart);
    tf::vectorKDLToEigen(posKDL, posEig);
    Matrix3d rot = quart.toRotationMatrix();
    Matrix4d trans;
    trans.setIdentity();
    trans.block<3,3>(0,0) = rot;
    trans.block<3,1>(0,3) = posEig;
    return trans;
}

BaxterChain::~BaxterChain() {
    return;
}
