#include "react_controller/avoidanceHandler.h"

using namespace std;

AvoidanceHandler::AvoidanceHandler(const BaxterChain &_chain,
                                   const vector<collisionPoint> &_collPoints,
                                   const string _type) :
                                   chain(_chain), collPoints(_collPoints), type(_type)
{
    if (not collPoints.empty())
    {
        BaxterChain customChain = chain;

        for (size_t i = 0; i < chain.getNrOfJoints() - 1; ++i)
        {
            customChain.removeJoint();
        }

        for (size_t i = 0; i < customChain.getNrOfJoints() - 1; ++i)
        {
            size_t num_segs = customChain.getNrOfSegments();
            customChain.addSegment(chain.getSegment(num_segs++));

            while (chain.getSegment(num_segs).getJoint().getType() == KDL::Joint::None)
            {
                customChain.addSegment(chain.getSegment(num_segs++));
            }

            // Instantiates a new chain, copying from the old (full) one
            BaxterChain nextChain = customChain;
            Eigen::Matrix4d HN(Eigen::Matrix4d::Identity());
            computeFoR(collPoints[i].x, collPoints[i].n, HN);

            KDL::Vector x, y, z, pos;
            tf::vectorEigenToKDL(HN.block<3,1>(0,0), x);
            tf::vectorEigenToKDL(HN.block<3,1>(0,1), y);
            tf::vectorEigenToKDL(HN.block<3,1>(0,2), z);
            tf::vectorEigenToKDL(HN.block<3,1>(0,3), pos);
            KDL::Rotation rot = KDL::Rotation(x, y, z);
            KDL::Segment s = KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame(rot, pos));
            nextChain.addSegment(s);
            ctrlPointChains.push_back(nextChain);
        }
    }
}

// deque<Eigen::VectorXd> AvoidanceHandler::getCtrlPointsPosition()
// {
//     deque<Eigen::VectorXd> ctrlPoints;
//     for (size_t i = 0; i<ctrlPointChains.size(); ++i)
//         ctrlPoints.push_back(ctrlPointChains[i].EndEffPosition());
//     return ctrlPoints;
// }

Eigen::MatrixXd AvoidanceHandler::getV_LIM(const Eigen::MatrixXd &v_lim)
{
    return v_lim;
}

bool AvoidanceHandler::computeFoR(const Eigen::VectorXd &pos,
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
    x    = -1*(z.cross(y));

    // Let's make them unitary vectors:
    x = x / x.norm();
    y = y / y.norm();
    z = z / z.norm();

    FoR.setIdentity();
    FoR.block<3,1>(0,0) =   x;
    FoR.block<3,1>(0,1) =   y;
    FoR.block<3,1>(0,2) =   z;
    FoR.block<3,1>(0,3) = pos;

    return true;
}

AvoidanceHandler::~AvoidanceHandler()
{

}

/****************************************************************/
/****************************************************************/
AvoidanceHandlerTactile::AvoidanceHandlerTactile(const BaxterChain &_chain,
                                                 const vector<collisionPoint> &_collPoints) :
                                                 AvoidanceHandler(_chain, _collPoints, "tactile"),
                                                 avoidingSpeed(50.0)
{

}

Eigen::MatrixXd AvoidanceHandlerTactile::getV_LIM(const Eigen::MatrixXd &v_lim)
{
    Eigen::MatrixXd V_LIM = v_lim;

    for (size_t i = 0; i < ctrlPointChains.size(); ++i)
    {
        // ROS_INFO("Chain with control point - index %d (last index %d), nDOF: %d.",
        //           i, ctrlPointChains.size()-1, ctrlPointChains[i].getNrOfJoints());
        // First 3 rows ~ dPosition/dJoints
        Eigen::MatrixXd J_xyz = ctrlPointChains[i].GeoJacobian().block(0, 0, 3, ctrlPointChains[i].getNrOfJoints());

        // Get the end-effector frame of the standard or custom chain (control point derived from skin),
        // takes the z-axis (3rd column in transform matrix) ~ normal, only its first three elements of the
        // four in the homogeneous transformation format
        Eigen::VectorXd nrm = ctrlPointChains[i].getH().col(2).block<3,1>(0,0);//.subEigen::VectorXd(0,2);

        // Project movement along the normal into joint velocity space and scale by default
        // avoidingSpeed and m of skin (or PPS) activation
        Eigen::VectorXd s = (J_xyz.transpose()*nrm) * avoidingSpeed * collPoints[i].m;

        s = s * -1.0; // we reverse the direction to obtain joint velocities that bring about avoidance
        // ROS_INFO_STREAM << "s * (-1) -> joint contributions toward avoidance: " << s.transpose();
        //
        for (size_t j = 0; j < size_t(s.rows()); ++j)
        {
            // ROS_INFO("Joint: %d, s[j]: %f, limits before: Min: %f, Max: %f",j,s[j],V_LIM(j,0),V_LIM(j,1));
            if (s[j] >= 0.0) //joint contributes to avoidance, we will set the min velocity accordingly
            {
                s[j]       = min(v_lim(j,1),       s[j]); // make sure new min vel is <= max vel
                V_LIM(j,0) = max(V_LIM(j,0),       s[j]); // set min vel to max of s[j] and current limit ~ avoiding action
                V_LIM(j,1) = max(V_LIM(j,0), V_LIM(j,1)); // make sure current max is at least equal to current min
                // ROS_INFO(" s>=0 clause, joint contributes to avoidance,"
                //          " adjusting Min; limits after: Min: %f, Max: %f",V_LIM(j,0),V_LIM(j,1));
            }
            else //joint acts to bring control point toward obstacle - we will shape the max vel
            {
                s[j]       = max(v_lim(j,0),       s[j]);
                V_LIM(j,1) = min(V_LIM(j,1),       s[j]);
                V_LIM(j,0) = min(V_LIM(j,0), V_LIM(j,1));
                // ROS_INFO("s<0 clause, joint contributes to approach, adjusting Max;"
                //          " limits after: Min: %f, Max: %f",V_LIM(j,0),V_LIM(j,1));
            }
        }
    }

    return V_LIM;
};

AvoidanceHandlerTactile::~AvoidanceHandlerTactile()
{

}
