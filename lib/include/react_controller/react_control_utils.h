#ifndef __REACT_CONTROL_UTILS_H__
#define __REACT_CONTROL_UTILS_H__

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <boost/scoped_ptr.hpp>

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>

#include <eigen_conversions/eigen_kdl.h>

#include <ros/ros.h>

/**
 * [SVD description]
 * @param in [description]
 * @param U  [description]
 * @param S  [description]
 * @param V  [description]
 */
void SVD(const Eigen::MatrixXd &in, Eigen::MatrixXd &U, Eigen::VectorXd &S, Eigen::MatrixXd &V);

/**
 * [dcm2axis description]
 * @param  R [description]
 * @return   [description]
 */
Eigen::VectorXd dcm2axis(const Eigen::MatrixXd &R);

/**
 * [axis2dcm description]
 * @param  v [description]
 * @return   [description]
 */
Eigen::MatrixXd axis2dcm(const Eigen::VectorXd &v);

/**
 * [cross description]
 * @param  A    [description]
 * @param  colA [description]
 * @param  B    [description]
 * @param  colB [description]
 * @return      [description]
 */
Eigen::Vector3d cross(const Eigen::MatrixXd &A, int colA, const Eigen::MatrixXd &B, int colB);

/**
 * [skew description]
 * @param  w [description]
 * @return   [description]
 */
Eigen::Matrix3d skew(const Eigen::Vector3d &w);

struct collisionPoint
{
        // iCub::skinDynLib::SkinPart skin_part;
        Eigen::VectorXd x; //position (x,y,z) in the FoR of the respective skin part
        Eigen::VectorXd n; //direction of normal vector at that point - derived from taxel normals, pointing out of the skin
        double magnitude;  // ~ activation level from probabilistic representation in pps - likelihood of collision
};

/**
 * Takes a KDL::Frame and returns a 4X4 pose Eigen::Matrix
 *
 * @param _f: KDL::Frame to turn into a pose Eigen::Matrix
 *
 * return: Eigen 4X4 pose matrix
*/
Eigen::Matrix4d KDLFrameToEigen(KDL::Frame _f);

/**
 * TODO documentation
 * @param  joints      [description]
 * @param  coll_coords [description]
 * @param  coll_points [description]
 * @param  norms       [description]
 * @return             true/false if success/failure
 */
bool computeCollisionPoints(const std::vector<Eigen::Vector3d>&      joints,
                            const             Eigen::Vector3d & coll_coords,
                            std::vector<collisionPoint>&   _collisionPoints);

#endif
