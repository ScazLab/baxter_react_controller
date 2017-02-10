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

using namespace Eigen;

void SVD(const MatrixXd &in, MatrixXd &U, VectorXd &S, MatrixXd &V);
VectorXd dcm2axis(const MatrixXd &R);
MatrixXd axis2dcm(const VectorXd &v);
Vector3d cross(const MatrixXd &A, int colA, const MatrixXd &B, int colB);
