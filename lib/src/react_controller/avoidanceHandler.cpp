#include "react_controller/avoidanceHandler.h"

using namespace   std;
using namespace Eigen;

AvoidanceHandler::AvoidanceHandler(const BaxterChain &_chain,
                                   const vector<Vector3d> &_obstacles,
                                   const string _type) :
                                   chain(_chain), type(_type)
{
    // ROS_INFO_STREAM("Chain Angles: " << chain.getAng().transpose());

    std::vector<BaxterChain>    tmpCC;  // temporary array of control chains
    std::vector<CollisionPoint> tmpCP;  // temporary array of collision points

    for (size_t i = 0; i < _obstacles.size(); ++i)
    {
        // Let's start by creating a custom chain with only one joint
        BaxterChain customChain;

        Eigen::VectorXd angles(1);
        angles[0] = chain.getAng(0);

        while (customChain.getNrOfJoints() == 0)
        {
            customChain.addSegment(chain.getSegment(customChain.getNrOfSegments()));
        }

        customChain.setAng(angles);

        // This while loop incrementally creates bigger chains (up to the end-effector)
        while (customChain.getNrOfSegments() < _chain.getNrOfSegments())
        {
            angles.conservativeResize(angles.rows()+1);
            angles[angles.rows()-1] = chain.getAng(customChain.getNrOfJoints());

            customChain.addSegment(chain.getSegment(customChain.getNrOfSegments()));

            customChain.setAng(angles);

            while (chain.getSegment(customChain.getNrOfSegments()).getJoint().getType() == KDL::Joint::None)
            {
                if (customChain.getNrOfSegments()==chain.getNrOfSegments())  { break; };

                customChain.addSegment(chain.getSegment(customChain.getNrOfSegments()));
            }

            // ROS_INFO_STREAM("Get Angles:  " << customChain.getAng().transpose());
            // ROS_INFO_STREAM("Real Angles: " <<       chain.getAng().transpose());

            // Compute collision points
            // obstacles are expressed in the world reference frame [WRF]
            // coll_pt is in the end-effector reference frame [ERF]
            CollisionPoint coll_pt;

            if (customChain.obstacleToCollisionPoint(_obstacles[i], coll_pt))
            {
                tmpCP.push_back(coll_pt);

                // create new segment to add to the custom chain that ends up in the collision point
                Matrix4d HN(Matrix4d::Identity());
                // Compute new segment to add to the chain
                computeFoR(coll_pt.x_erf, coll_pt.n_erf, HN);
                KDL::Segment s = KDL::Segment(KDL::Joint(KDL::Joint::None), toKDLFrame(HN));

                BaxterChain chainToAdd = customChain;
                chainToAdd.addSegment(s);
                tmpCC.push_back(chainToAdd);
                // ROS_INFO("adding chain with %zu joints and %zu segments", chainToAdd.getNrOfJoints(), chainToAdd.getNrOfSegments());
            }

            // ROS_INFO("tmpCP.size %lu tmpCC.size %lu", tmpCP.size(), tmpCC.size());
        }

        // Cycle through the newfound collision points to find the max, and stick to that one
        double max_mag = 0.0;
        int    max_idx =  -1;
        string  cp_str =  "";

        for (size_t i = 0; i < tmpCP.size(); ++i)
        {
            cp_str = cp_str + " " + toString(tmpCP[i].mag);

            if (tmpCP[i].mag > 1e-2)
            {
                if (tmpCP[i].mag > max_mag)
                {
                    max_mag = tmpCP[i].mag;
                    max_idx =     int(i);
                }
            }
        }

        if (max_idx != -1)
        {
            // ROS_INFO("Collision points with magnitude: %s Selected: %i", cp_str.c_str(), max_idx);

            collPoints.push_back(tmpCP[max_idx]);
            ctrlChains.push_back(tmpCC[max_idx]);
        }

        tmpCC.clear();
        tmpCP.clear();
    }

}

std::vector<BaxterChain> AvoidanceHandler::getCtrlChains()
{
    return ctrlChains;
}

std::vector<CollisionPoint> AvoidanceHandler::getCtrlPoints()
{
    return collPoints;
}

MatrixXd AvoidanceHandler::getV_LIM(const MatrixXd &v_lim)
{
    return v_lim;
}

bool AvoidanceHandler::computeFoR(const VectorXd &pos,
                                  const VectorXd &norm,
                                        Matrix4d &FoR)
{
    Vector3d zeros;
    zeros.setZero();

    if (norm == zeros)
    {
        FoR.setIdentity();
        return false;
    }

    Vector3d x(0,0,0), y(0,0,0), z(0,0,0);

    z = norm;
    if (z[0] == 0.0)
    {
        z[0] = 0.00000001;    // Avoid the division by 0
    }

    y[0] =      -z[2]/z[0]; // y is in normal plane
    y[2] =               1; // this setting is arbitrary
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

std::vector<RVIZMarker> AvoidanceHandler::toRVIZMarkers()
{
    std::vector<RVIZMarker> res;

    vector <RVIZMarker> rvzcc;

    for (size_t i = 0; i < ctrlChains.size(); ++i)
    {
        rvzcc = asRVIZMarkers(ctrlChains[i], false, false, true);

        // Let's use the magnitude of the collision point as length
        // of the arrow to be displayed to RVIZ
        for (int j = 0; j < 3; ++j)
        {
            rvzcc[j].size *= collPoints[i].mag;
        }

        res.insert(std::end(res),
                   std::begin(rvzcc), std::end(rvzcc));
    }

    return res;
}

AvoidanceHandler::~AvoidanceHandler()
{

}

/****************************************************************/
/****************************************************************/
AvoidanceHandlerTactile::AvoidanceHandlerTactile(const BaxterChain &_chain,
                                                 const vector<Vector3d> &_obstacles) :
                                                 AvoidanceHandler(_chain, _obstacles, "tactile"),
                                                 avoidingSpeed(0.25)
{

}

MatrixXd AvoidanceHandlerTactile::getV_LIM(const MatrixXd &v_lim)
{
    MatrixXd V_LIM = v_lim;

    for (size_t i = 0; i < collPoints.size(); ++i)
    {
        // ROS_INFO("Chain with control point - index %d (last index %d), nDOF: %d.",
        //           i, ctrlChains.size()-1, ctrlChains[i].getNrOfJoints());
        // First 3 rows ~ dPosition/dJoints
        MatrixXd J_xyz = ctrlChains[i].GeoJacobian().block(0, 0, 3, ctrlChains[i].getNrOfJoints());

        // Get the end-effector frame of the standard or custom chain (control point derived from skin),
        // takes the z-axis (3rd column in transform matrix) ~ normal, only its first three elements of the
        // four in the homogeneous transformation format
        VectorXd nrm = ctrlChains[i].getH().block<3,1>(0,2);

        // Project movement along the normal into joint velocity space and scale by default
        // avoidingSpeed and m of skin (or PPS) activation
        VectorXd s = J_xyz.transpose()*nrm;

        for (size_t j = 0; j < size_t(s.rows()); ++j)
        {
            double sj =       s[j];
            double vm = V_LIM(j,0);
            double vM = V_LIM(j,1);

            if (s[j] >= 0.0)
            {
                double tmp = v_lim(j,1)*(1.0 - collPoints[i].mag);

                V_LIM(j,1) = std::min(V_LIM(j,1),        tmp);
                V_LIM(j,0) = std::min(V_LIM(j,0), V_LIM(j,1));
                ROS_INFO("s[%lu]: %g   \t[avoidance], adjusting min. "
                         "Limits: [%g %g]->[%g %g]",j, sj, vm, vM, V_LIM(j,0),V_LIM(j,1));
            }
            else
            {
                double tmp = v_lim(j,0)*(1.0 - collPoints[i].mag);

                V_LIM(j,0) = std::max(V_LIM(j,0),        tmp);
                V_LIM(j,1) = std::max(V_LIM(j,0), V_LIM(j,1));
                ROS_INFO("s[%lu]: %g   \t[ approach], adjusting max. "
                         "Limits: [%g %g]->[%g %g]",j, sj, vm, vM, V_LIM(j,0),V_LIM(j,1));
            }
        }

        // LEGACY CODE
        // // Project movement along the normal into joint velocity space and scale by default
        // // avoidingSpeed and m of skin (or PPS) activation
        // VectorXd s = (J_xyz.transpose()*nrm) * avoidingSpeed * collPoints[i].mag;

        // s = s * -1.0; // we reverse the direction to obtain joint velocities that bring about avoidance
        // // ROS_INFO_STREAM("s*(-1): " << s.transpose());

        // for (size_t j = 0; j < size_t(s.rows()); ++j)
        // {
        //     double sj =       s[j];
        //     double vm = V_LIM(j,0);
        //     double vM = V_LIM(j,1);

        //     if (s[j] >= 0.0) //joint contributes to avoidance, we will set the min velocity accordingly
        //     {
        //         s[j]       = min(v_lim(j,1),       s[j]); // make sure new min vel is <= max vel
        //         V_LIM(j,0) = max(V_LIM(j,0),       s[j]); // set min vel to max of s[j] and current limit ~ avoiding action
        //         V_LIM(j,1) = max(V_LIM(j,0), V_LIM(j,1)); // make sure current max is at least equal to current min
        //         ROS_INFO("s[%lu]: %g   \t[avoidance], adjusting min. "
        //                  "Limits: [%g %g]->[%g %g]",j, sj, vm, vM, V_LIM(j,0),V_LIM(j,1));
        //     }
        //     else //joint acts to bring control point toward obstacle - we will shape the max vel
        //     {
        //         s[j]       = max(v_lim(j,0),       s[j]);
        //         V_LIM(j,1) = min(V_LIM(j,1),       s[j]);
        //         V_LIM(j,0) = min(V_LIM(j,0), V_LIM(j,1));
        //         ROS_INFO("s[%lu]: %g   \t[ approach], adjusting max. "
        //                  "Limits: [%g %g]->[%g %g]",j, sj, vm, vM, V_LIM(j,0),V_LIM(j,1));
        //     }
        // }
    }

    return V_LIM;
};

AvoidanceHandlerTactile::~AvoidanceHandlerTactile()
{

}
