#include "react_controller/react_control_utils.h"

using namespace Eigen;

void SVD(const MatrixXd &in, Matrix3d &U, Vector3d &S, Matrix3d &V)
{
    Eigen::JacobiSVD< Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> > svd(in, Eigen::ComputeThinU | Eigen::ComputeThinV);

    U = svd.matrixU();

    S = svd.singularValues();

    V = svd.matrixV();

    return;
}

Vector3d cross(const MatrixXd &A, int colA, const MatrixXd &B, int colB)
{

    Vector3d v;
    v[0]=A(1,colA)*B(2,colB)-A(2,colA)*B(1,colB);
    v[1]=A(2,colA)*B(0,colB)-A(0,colA)*B(2,colB);
    v[2]=A(0,colA)*B(1,colB)-A(1,colA)*B(0,colB);

    return v;
}

MatrixXd axis2dcm(const VectorXd &v)
{
    MatrixXd R(4, 4); R.setIdentity();

    double theta=v[3];
    if (theta==0.0)
        return R;

    double c=cos(theta);
    double s=sin(theta);
    double C=1.0-c;

    double xs =v[0]*s;
    double ys =v[1]*s;
    double zs =v[2]*s;
    double xC =v[0]*C;
    double yC =v[1]*C;
    double zC =v[2]*C;
    double xyC=v[0]*yC;
    double yzC=v[1]*zC;
    double zxC=v[2]*xC;

    R(0,0)=v[0]*xC+c;
    R(0,1)=xyC-zs;
    R(0,2)=zxC+ys;
    R(1,0)=xyC+zs;
    R(1,1)=v[1]*yC+c;
    R(1,2)=yzC-xs;
    R(2,0)=zxC-ys;
    R(2,1)=yzC+xs;
    R(2,2)=v[2]*zC+c;

    return R;
}

VectorXd dcm2axis(const Eigen::MatrixXd &R)
{

    VectorXd v(4);
    v[0]=R(2,1)-R(1,2);
    v[1]=R(0,2)-R(2,0);
    v[2]=R(1,0)-R(0,1);
    v[3]=0.0;
    double r=v.norm();
    double theta=atan2(0.5*r,0.5*(R(0,0)+R(1,1)+R(2,2)-1));

    if (r<1e-9) {
        // if we enter here, then
        // R is symmetric; this can
        // happen only if the rotation
        // angle is 0 (R=I) or 180 degrees
        Matrix3d A=R.block(0,0,3,3);
        Matrix3d U, V;
        Vector3d S;

        // A=I+sin(theta)*S+(1-cos(theta))*S^2
        // where S is the skew matrix.
        // Given a point x, A*x is the rotated one,
        // hence if Ax=x then x belongs to the rotation
        // axis. We have therefore to find the kernel of
        // the linear application (A-I).
        Matrix3d I; I.setIdentity();
        SVD(A-I,U,S,V);

        v[0]=V(0,2);
        v[1]=V(1,2);
        v[2]=V(2,2);
        r=v.norm();
   }

   v=(1.0/r)*v;
   v[3]=theta;

   return v;
}

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
