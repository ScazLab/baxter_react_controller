#include "react_controller/avoidanceHandler.h"

using namespace std;

AvoidanceHandlerAbstract::AvoidanceHandlerAbstract(const BaxterChain &_chain,
                                                   const std::vector<collisionPoint> &_collPoints,
                                                   const std::string _type) :
                                                   chain(_chain), collPoints(_collPoints), type(_type)
{
    if (!collPoints.empty())
    {
        BaxterChain customChain = chain;
        for (size_t i = 0, num_jnts = chain.getNrOfJoints(); i < num_jnts - 1; ++i)
        {
            customChain.removeJoint();
        }

        for (size_t i = 0, num_jnts = chain.getNrOfJoints(),
             num_segs = customChain.getNrOfSegments();i < num_jnts - 1; ++i)
        {
            customChain.addSegment(chain.getSegment(num_segs++));

            while (chain.getSegment(num_segs).getJoint().getType() == KDL::Joint::None)
            {
                customChain.addSegment(chain.getSegment(num_segs++));
            }

            // Instantiates a new chain, copying from the old (full) one
            BaxterChain nextChain = customChain;
            Eigen::Matrix4d HN;
            HN.setIdentity();
            computeFoR((collPoints[i]).x,(collPoints[i]).n,HN);

            KDL::Vector x, y, z, pos;
            tf::vectorEigenToKDL(HN.block<3,1>(0,0), x);
            tf::vectorEigenToKDL(HN.block<3,1>(0,1), y);
            tf::vectorEigenToKDL(HN.block<3,1>(0,2), z);
            tf::vectorEigenToKDL(HN.block<3,1>(0,3), pos);
            KDL::Rotation rot = KDL::Rotation(x, y, z);
            KDL::Segment s = KDL::Segment(KDL::Joint(KDL::Joint::None),
                                            KDL::Frame(rot, pos));
            nextChain.addSegment(s);
            ctrlPointChains.push_back(nextChain);
        }
    }
}


// deque<Eigen::VectorXd> AvoidanceHandlerAbstract::getCtrlPointsPosition()
// {
//     deque<Eigen::VectorXd> ctrlPoints;
//     for (size_t i = 0; i<ctrlPointChains.size(); i++)
//         ctrlPoints.push_back(ctrlPointChains[i].EndEffPosition());
//     return ctrlPoints;
// }

Eigen::MatrixXd AvoidanceHandlerAbstract::getVLIM(const Eigen::MatrixXd &v_lim)
{
    return v_lim;
}

bool AvoidanceHandlerAbstract::computeFoR(const Eigen::VectorXd &pos,
                                          const Eigen::VectorXd &norm,
                                                Eigen::Matrix4d &FoR)
{
    Eigen::Vector3d zeros;
    zeros.setZero();
    if (norm == zeros)
    {
        FoR.setIdentity();
        return false;
    }

    Eigen::Vector3d x(0,0,0), y(0,0,0), z(0,0,0);

    z = norm;
    if (z[0] == 0.0)
    {
        z[0] = 0.00000001;    // Avoid the division by 0
    }
    y[0] = -z[2]/z[0]; //y is in normal plane
    y[2] = 1; //this setting is arbitrary
    x = -1*(z.cross(y));

    // Let's make them unitary vectors:
    x = x / x.norm();
    y = y / y.norm();
    z = z / z.norm();

    FoR.setIdentity();
    FoR.block<3,1>(0,0) = x;
    FoR.block<3,1>(0,1) = y;
    FoR.block<3,1>(0,2) = z;
    FoR.block<3,1>(0,3) = pos;

    return true;
}

AvoidanceHandlerAbstract::~AvoidanceHandlerAbstract()
{

}

/****************************************************************/
/****************************************************************/
AvoidanceHandlerTactile::AvoidanceHandlerTactile(const BaxterChain &_chain,
                                                 const std::vector<collisionPoint> &_collPoints) :
                                                 AvoidanceHandlerAbstract(_chain, _collPoints, "tactile")
{
    avoidingSpeed = 50;
    // produce collisionPoint.m * avoidingSpeed deg/s repulsive speed

    // parameters.unput("avoidingSpeed");
    // parameters.put("avoidingSpeed",avoidingSpeed);
}

Eigen::MatrixXd AvoidanceHandlerTactile::getVLIM(const Eigen::MatrixXd &v_lim)
{
    // printMessage(2,"AvoidanceHandlerTactile::getVLIM\n");
    Eigen::MatrixXd VLIM = v_lim;
    for (size_t i = 0; i < ctrlPointChains.size(); i++)
    {
        // printMessage(2,"Chain with control point - index %d (last index %d), nDOF: %d.\n",
        //                  i,ctrlPointChains.size()-1,ctrlPointChains[i].getNrOfSegments());
        Eigen::MatrixXd tmp = ctrlPointChains[i].GeoJacobian();
        Eigen::MatrixXd J = tmp.block(0, 0, 3, ctrlPointChains[i].getNrOfJoints()-1); //.submatrix(0,2,0,ctrlPointChains[i].getNrOfSegments()-1); //first 3 rows ~ dPosition/dJoints
        Eigen::VectorXd normal = ctrlPointChains[i].getH().col(2).block<3,1>(0,0);//.subEigen::VectorXd(0,2); //get the end-effector frame of the standard or custom chain (control point derived from skin), takes the z-axis (3rd column in transform matrix) ~ normal, only its first three elements of the 4 in the homogenous transf. format
        Eigen::VectorXd s = (J.transpose()*normal) * avoidingSpeed * collPoints[i].m; //project movement along the normal into joint velocity space and scale by default avoidingSpeed and m of skin (or PPS) activation

        s = s * -1.0; //we reverse the direction to obtain joint velocities that bring about avoidance
        // printMessage(2,"s * (-1) -> joint contributions toward avoidance: \n (%s) \n",s.toString(3,3).c_str());
        // cout << s;
        for (size_t j = 0; j < size_t(s.rows()); j++)
        {
            // printMessage(2,"        Joint: %d, s[j]: %f, limits before: Min: %f, Max: %f\n",j,s[j],VLIM(j,0),VLIM(j,1));
            if (s[j] >= 0.0) //joint contributes to avoidance, we will set the min velocity accordingly
            {
                s[j] = std::min(v_lim(j,1),s[j]); //make sure new min vel is <= max vel
                VLIM(j,0) = std::max(VLIM(j,0),s[j]); // set min vel to max of s[j] and current limit ~ avoiding action
                VLIM(j,1) = std::max(VLIM(j,0),VLIM(j,1)); //make sure current max is at least equal to current min
                // printMessage(2,"            s>=0 clause, joint contributes to avoidance, adjusting Min; limits after: Min: %f, Max: %f\n",VLIM(j,0),VLIM(j,1));
            }
            else //joint acts to bring control point toward obstacle - we will shape the max vel
            {
                s[j] = std::max(v_lim(j,0),s[j]);
                VLIM(j,1) = std::min(VLIM(j,1),s[j]);
                VLIM(j,0) = std::min(VLIM(j,0),VLIM(j,1));
                // printMessage(2,"            s<0 clause, joint contributes to approach, adjusting Max; limits after: Min: %f, Max: %f\n",VLIM(j,0),VLIM(j,1));
            }
        }
    }

    return VLIM;
};

AvoidanceHandlerTactile::~AvoidanceHandlerTactile()
{

}
