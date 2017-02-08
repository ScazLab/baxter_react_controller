#include <Eigen/Dense>
#include <Eigen/SVD>

using namespace Eigen;

void SVD(const MatrixXd &in, MatrixXd &U, VectorXd &S, MatrixXd &V);
VectorXd dcm2axis(const MatrixXd &R);
MatrixXd axis2dcm(const VectorXd &v);