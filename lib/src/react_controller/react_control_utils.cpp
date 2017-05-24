#include "react_controller/react_control_utils.h"

using namespace Eigen;

Vector3d cross(const MatrixXd &A, int colA, const MatrixXd &B, int colB)
{
    Vector3d v;
    v[0]=A(1,colA)*B(2,colB)-A(2,colA)*B(1,colB);
    v[1]=A(2,colA)*B(0,colB)-A(0,colA)*B(2,colB);
    v[2]=A(0,colA)*B(1,colB)-A(1,colA)*B(0,colB);

    return v;
}

Matrix3d skew(const Vector3d &w)
{
    Matrix3d S;

    S(0,0)=S(1,1)=S(2,2)=0.0;
    S(1,0)= w[2]; S(0,1)=-S(1,0);
    S(2,0)=-w[1]; S(0,2)=-S(2,0);
    S(2,1)= w[0]; S(1,2)=-S(2,1);

    return S;
}

Matrix4d toMatrix4d(KDL::Frame _f)
{
    Matrix4d result(Matrix4d::Identity());

    Vector3d p(VectorXd::Map(&_f.p.data[0], 3   ));
    Matrix3d r(MatrixXd::Map(&_f.M.data[0], 3, 3).transpose());

    result.block<3,3>(0,0) = r;
    result.block<3,1>(0,3) = p;

    return result;
}

Vector3d angularError(const Matrix3d& _a, const Matrix3d& _b)
{
    AngleAxisd angErr((_a)*(_b.transpose()));

    return angErr.axis() * angErr.angle();
}

Vector3d angularError(const Quaterniond& _a, const Quaterniond& _b)
{
    return _b.w()*_a.vec() - _a.w()*_b.vec() -skew(_a.vec())*_b.vec();
}

bool computeCollisionPoints(const std::vector<Vector3d>&      _joints,
                            const             Vector3d & _coll_coords,
                            std::vector<collisionPoint>& _coll_points)
{
    _coll_points.clear();

    for (size_t i = 0; i < _joints.size() - 1; ++i)
    {
        Vector3d ab = _joints[i + 1] - _joints[i];
        Vector3d ap = _coll_coords - _joints[i];
        Vector3d coll_pt = _joints[i] + ((ap).dot(ab)) / ((ab).dot(ab)) * ab;

        collisionPoint cp;
        cp.x = coll_pt;
        cp.m = (_coll_coords - coll_pt).norm();
        cp.n = (_coll_coords - coll_pt) / cp.m;

        _coll_points.push_back(cp);

        // ROS_INFO("coll point %zu at x: %g y: %g z: %g", i,
        //           _coll_points[i].x(0), _coll_points[i].x(1), _coll_points[i].x(2));
        // ROS_INFO("      norm %zu at x: %g y: %g z: %g", i,
        //           _coll_points[i].n(0), _coll_points[i].n(1), _coll_points[i].n(2));
    }
    // printf("\n");

    return true;
}
