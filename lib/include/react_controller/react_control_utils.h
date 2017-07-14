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

#define RAD2DEG (180.0 /  M_PI)
#define DEG2RAD ( M_PI / 180.0)

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

/**
 * Takes a KDL::Frame and returns a 4X4 pose Eigen::Matrix
 *
 * @param _f KDL::Frame to turn into a pose Eigen::Matrix
 * return    Eigen 4X4 pose matrix
*/
Eigen::Matrix4d toMatrix4d(KDL::Frame _f);

/**
 * Computes the angular error between two rotation matrices.
 * Angular error computation is taken from the following book:
 * Robotics, Modelling, Planning and Control, Siciliano & Sciavicco, page 111.
 *
 * @param  _a The  first rotation matrix (ie the reference)
 * @param  _b The second rotation matrix
 * @return    the angular error
 */
Eigen::Vector3d angularError(const Eigen::Matrix3d& _a, const Eigen::Matrix3d& _b);

/**
 * Computes the angular error between two unit quaternions.
 * Angular error computation is taken from the following book:
 * Robotics, Modelling, Planning and Control, Siciliano & Sciavicco, page 111.
 *
 * @param  _a The  first quaternion (ie the reference)
 * @param  _b The second quaternion
 * @return    the angular error
 */
Eigen::Vector3d angularError(const Eigen::Quaterniond& _a, const Eigen::Quaterniond& _b);

/**
 * TODO documentation
 */
struct collisionPoint
{
    Eigen::Vector3d x_erf; // position (x,y,z) in the end effector reference frame
    Eigen::Vector3d n_erf; // direction of normal vector in the end effector reference frame

    Eigen::Vector3d x_wrf; // position (x,y,z) in the world reference frame
    Eigen::Vector3d n_wrf; // direction of normal vector in the world reference frame

    Eigen::Vector3d o_wrf; // position (x,y,z) of the obstacle in the world reference frame

    double m; // activation level (magnitude)
};

/**
 * TODO documentation
 * @param  orig      [description]
 * @param  transform [description]
 * @param  new_pt    [description]
 * @return           [description]
 */
bool changeFoR(const Eigen::Vector3d orig, const Eigen::Matrix4d transform, Eigen::Vector3d &new_pt);

/**
 * Takes a 4x4 Eigen matrix and converts it to a KDL Frame.
 *
 * @param  mat          4d matrix to convert
 * @return              KDL Frame of converted matrix
 */
KDL::Frame toKDLFrame(Eigen::Matrix4d mat);

/**
 * TODO documentation
 * @param  vec [description]
 * @return     [description]
 */
Eigen::VectorXd stdToEigen(std::vector<double> vec);

/**
 * Projects a 3D point onto a line composed of base and tip
 * @param  base  the base of the line
 * @param  tip   the  tip of the line
 * @param  point the point to project
 * @return       the projected  point
 */
Eigen::Vector3d projectOntoSegment(Eigen::Vector3d base, Eigen::Vector3d tip, Eigen::Vector3d point);

#endif
