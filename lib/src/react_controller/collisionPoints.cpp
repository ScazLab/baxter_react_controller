#include <assert.h>
#include <deque>
#include <ros/ros.h>

#include <react_controller/baxterChain.h>
#include <eigen_conversions/eigen_kdl.h>

#include <kdl/chainfksolverpos_recursive.hpp>

using namespace   std;

bool computeCollisionPoints(const std::vector<Eigen::Vector3d>&      joints,
                            const             Eigen::Vector3d & coll_coords,
                            std::vector<collisionPoint>&    collisionPoints)
{
    collisionPoints.clear();

    for (size_t i = 0; i < joints.size() - 1; ++i)
    {
        Eigen::Vector3d ab = joints[i + 1] - joints[i];
        Eigen::Vector3d ap = coll_coords - joints[i];
        Eigen::Vector3d coll_pt = joints[i] + ((ap).dot(ab)) / ((ab).dot(ab)) * ab;
        collisionPoint c;
        c.x = coll_pt;
        c.magnitude = (coll_coords - coll_pt).norm();
        c.n = (coll_coords - coll_pt) / c.magnitude;
        collisionPoints.push_back(c);

        // ROS_INFO("coll point %zu at x: %g y: %g z: %g", i, collisionPoints[i].x(0), collisionPoints[i].x(1), collisionPoints[i].x(2));
        // ROS_INFO("      norm %zu at x: %g y: %g z: %g", i, collisionPoints[i].n(0), collisionPoints[i].n(1), collisionPoints[i].n(2));
    }
    // printf("\n");

    return true;
}

bool BaxterChain::GetJointPositions(std::vector<Eigen::Vector3d>& positions)
{
    Eigen::Vector3d point(0.40, -0.25, 0.45);

    unsigned int segmentNr=getNrOfSegments();

    KDL::JntArray jnts(getNrOfJoints());

    for (size_t i = 0; i < q.size() - 1; ++i)
    {
        jnts(i) = q[i];
    }

    int j=0;
    KDL::Frame frame(KDL::Frame::Identity());

    for (unsigned int i=0; i<segmentNr; ++i)
    {
        if (getSegment(i).getJoint().getType()!=KDL::Joint::None)
        {
            frame = frame*getSegment(i).pose(jnts(j));
            KDL::Vector   posKDL = frame.p;
            Eigen::Vector3d posEig;
            tf::vectorKDLToEigen(posKDL, posEig);
            positions.push_back(posEig);
            j++;
        }
        else
        {
            frame = frame*getSegment(i).pose(0.0);
        }
    }

    // for (size_t i = 0; i < positions.size(); ++i)
    // {
    //     ROS_INFO("joint %zu at x: %g y: %g z: %g", i, positions[i](0), positions[i](1), positions[i](2));
    // }

    // std::vector<Eigen::Vector3d> coll_points;
    // std::vector<Eigen::Vector3d> norms;
    // for (size_t i = 0; i < joint_pos.size() - 1; ++i)
    // {
    //     Eigen::Vector3d ab = joint_pos[i + 1] - joint_pos[i];
    //     Eigen::Vector3d ap = point - joint_pos[i];
    //     Eigen::Vector3d coll_pt = joint_pos[i] + ((ap).dot(ab)) / ((ab).dot(ab)) * ab;
    //     coll_points.push_back(coll_pt);
    //     norms.push_back(point - coll_pt);
    //     ROS_INFO("coll point %zu at x:%f y:%f z:%f", i, coll_points[i](0), coll_points[i](1), coll_points[i](2));
    //     ROS_INFO("norm %zu at x:%f y:%f z:%f", i, norms[i](0), norms[i](1), norms[i](2));
    // }

    return true;
}

