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

bool changeFoR(const Vector3d orig, const Matrix4d transform, Vector3d &new_pt)
{
    Vector4d tmp(0, 0, 0, 1);
    tmp.block<3,1>(0,0) = orig;
    new_pt = (transform.inverse() * tmp).block<3,1>(0,0);
    return true;
}

KDL::Frame toKDLFrame(Matrix4d mat)
{
    KDL::Vector x, y, z, pos;

    tf::vectorEigenToKDL(mat.block<3,1>(0,0), x);
    tf::vectorEigenToKDL(mat.block<3,1>(0,1), y);
    tf::vectorEigenToKDL(mat.block<3,1>(0,2), z);
    KDL::Rotation rot = KDL::Rotation(x, y, z);

    tf::vectorEigenToKDL(mat.block<3,1>(0,3), pos);

    return KDL::Frame(rot, pos);
}

VectorXd stdToEigen(std::vector<double> vec)
{
    VectorXd res(vec.size());

    for (size_t i = 0; i < vec.size(); ++i)
    {
        res(i) = vec[i];
    }

    return res;
}

Vector3d projectOntoSegment(Vector3d base, Vector3d tip, Vector3d point)
{
    Vector3d ab =   tip - base;
    Vector3d ap = point - base;

    return base + ((ap).dot(ab)) / ((ab).dot(ab)) * ab;
}

std::vector<Obstacle> readFromParamServer(XmlRpc::XmlRpcValue _param)
{
    std::vector<Obstacle> res;

    ROS_ASSERT(_param.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(_param.size() > 0);

    for (int i = 0; i < _param.size(); ++i)
    {
        ROS_ASSERT(_param[i].getType() == XmlRpc::XmlRpcValue::TypeArray);
        ROS_ASSERT(_param[i].size()    == 4);

        res.push_back(Obstacle(_param[i][3], Vector3d(_param[i][0], _param[i][1], _param[i][2])));
    }

    return res;
}
