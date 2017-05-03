#include <iostream>

#include <math.h>
#include <ros/ros.h>

#include "react_controller/controllerNLP.h"
#include "react_controller/react_control_utils.h"

using namespace Eigen;
using namespace   std;

/****************************************************************/
ControllerNLP::ControllerNLP(BaxterChain chain_, double dt_, bool ctrl_ori_) :
                             chain(chain_), dt(dt_), ctrl_ori(ctrl_ori_),
                             q_0(chain_.getNrOfJoints()), v_0(chain_.getNrOfJoints()),
                             J_0_xyz(3,chain_.getNrOfJoints()), J_0_ang(3,chain_.getNrOfJoints()),
                             v_e(chain.getNrOfJoints()), q_lim(chain.getNrOfJoints(),2),
                             v_lim(chain_.getNrOfJoints(),2), bounds(chain.getNrOfJoints(),2),
                             qGuard(chain.getNrOfJoints()),
                             qGuardMinExt(chain.getNrOfJoints()), qGuardMinInt(chain.getNrOfJoints()),
                             qGuardMinCOG(chain.getNrOfJoints()), qGuardMaxExt(chain.getNrOfJoints()),
                             qGuardMaxInt(chain.getNrOfJoints()), qGuardMaxCOG(chain.getNrOfJoints())
{
    v_0.setZero();
    v_e.setZero();

    R_e.setIdentity();
    R_r.setIdentity();

    for (size_t r=0; r<chain.getNrOfJoints(); r++)
    {
        /* angle bounds */
        q_lim(r,0)=chain.getMin(r);
        q_lim(r,1)=chain.getMax(r);

        v_lim(r,1)=std::numeric_limits<double>::max();
        v_lim(r,0)=-v_lim(r,1);
    }
    bounds=v_lim;

    computeGuard();
}

/****************************************************************/
void ControllerNLP::computeGuard()
{
    double guardRatio=0.1;

    for (size_t i=0; i<chain.getNrOfJoints(); ++i)
    {
        qGuard[i]=0.25*guardRatio*(chain.getMax(i)-chain.getMin(i));

        qGuardMinExt[i]=chain.getMin(i)+qGuard[i];
        qGuardMinInt[i]=qGuardMinExt[i]+qGuard[i];
        qGuardMinCOG[i]=0.5*(qGuardMinExt[i]+qGuardMinInt[i]);

        qGuardMaxExt[i]=chain.getMax(i)-qGuard[i];
        qGuardMaxInt[i]=qGuardMaxExt[i]-qGuard[i];
        qGuardMaxCOG[i]=0.5*(qGuardMaxExt[i]+qGuardMaxInt[i]);
    }
}

/****************************************************************/
void ControllerNLP::computeBounds()
{
    bounds.resize(chain.getNrOfJoints(), 2);

    for (size_t i=0; i<chain.getNrOfJoints(); ++i)
    {
        double qi=q_0[i];
        if ((qi>=qGuardMinInt[i]) && (qi<=qGuardMaxInt[i]))
        {
            bounds(i,0)=bounds(i,1)=1.0;
        }
        else if (qi<qGuardMinInt[i])
        {
            bounds(i,0)=(qi<=qGuardMinExt[i]?0.0:
                         0.5*(1.0+tanh(+10.0*(qi-qGuardMinCOG[i])/qGuard[i])));
            bounds(i,1)=1.0;
        }
        else
        {
            bounds(i,0)=1.0;
            bounds(i,1)=(qi>=qGuardMaxExt[i]?0.0:
                         0.5*(1.0+tanh(-10.0*(qi-qGuardMaxCOG[i])/qGuard[i])));
        }
    }
    for (size_t i=0; i<chain.getNrOfJoints(); ++i)
    {
        bounds(i,0)*=v_lim(i,0);
        bounds(i,1)*=v_lim(i,1);
    }
}

/****************************************************************/
void ControllerNLP::set_x_r(const Eigen::Vector3d &_p_r, const Eigen::Quaterniond &_o_r)
{
    ROS_ASSERT(p_r.size() == _p_r.size());

    p_r = _p_r;
    o_r = _o_r;
    R_r =  o_r.toRotationMatrix();

    skew_nr = skew(R_r.col(0));
    skew_sr = skew(R_r.col(1));
    skew_ar = skew(R_r.col(2));
}

/****************************************************************/
void ControllerNLP::set_v_lim(const MatrixXd &_v_lim)
{
    v_lim=CTRL_DEG2RAD*_v_lim;
}

/****************************************************************/
void ControllerNLP::set_ctrl_ori(const bool _ctrl_ori)
{
    ctrl_ori=_ctrl_ori;
}

/****************************************************************/
void ControllerNLP::set_dt(const double _dt)
{
    ROS_ASSERT(dt>0.0);
    dt=_dt;
}

/****************************************************************/
void ControllerNLP::set_v_0(const VectorXd &_v_0)
{
    v_0=_v_0;
}

/****************************************************************/
void ControllerNLP::init()
{
    q_0= chain.getAng();

    Matrix4d H_0= chain.getH();
    R_0= H_0.block<3,3>(0,0);
    p_0= H_0.col(3).block<3,1>(0,0);

    // cout << "H_0: \n" << H_0 << endl;
    // cout << "R_0: \n" << R_0 << endl;
    // cout << "p_0: \t" << p_0.transpose() << endl;

    MatrixXd J_0=chain.GeoJacobian();
    J_0_xyz=J_0.block(0,0,3,chain.getNrOfJoints());
    J_0_ang=J_0.block(3,0,3,chain.getNrOfJoints());

    // cout << "J_0:    \n" << J_0     << endl;
    // cout << "J_0_xyz:\n" << J_0_xyz << endl;
    // cout << "J_0_ang:\n" << J_0_ang << endl;

    computeBounds();
}

/****************************************************************/
VectorXd ControllerNLP::get_result() const
{
    return v_e;
}

/****************************************************************/
bool ControllerNLP::get_nlp_info(Ipopt::Index &n, Ipopt::Index &m, Ipopt::Index &nnz_jac_g,
                  Ipopt::Index &nnz_h_lag, IndexStyleEnum &index_style)
{

    n=chain.getNrOfJoints();

    // reaching in position
    m=1; nnz_jac_g=n;

    nnz_h_lag=0;
    index_style=TNLP::C_STYLE;
    return true;
}

/****************************************************************/
bool ControllerNLP::get_bounds_info(Ipopt::Index n, Ipopt::Number *x_l, Ipopt::Number *x_u,
                                    Ipopt::Index m, Ipopt::Number *g_l, Ipopt::Number *g_u)
{

    for (Ipopt::Index i=0; i<n; ++i)
    {
        x_l[i]=bounds(i,0);
        x_u[i]=bounds(i,1);
    }

    // reaching in position
    g_l[0]=g_u[0]=0.0;

    return true;
}

/****************************************************************/
bool ControllerNLP::get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number *x,
                        bool init_z, Ipopt::Number *z_L, Ipopt::Number *z_U,
                        Ipopt::Index m, bool init_lambda, Ipopt::Number *lambda)
{
    for (Ipopt::Index i=0; i<n; ++i)
    {
        x[i]=std::min(std::max(bounds(i,0),v_0[i]),bounds(i,1));
    }
    return true;
}

/************************************************************************/
void ControllerNLP::computeQuantities(const Ipopt::Number *x, const bool new_x)
{
    if (new_x)
    {
        for (int i=0; i<v_e.size(); ++i)
        {
            v_e[i]=x[i];
        }

        Vector3d  ww = J_0_ang*v_e;
        double theta =  ww.norm();
        if (theta > 0.0) { ww /= theta; }
        AngleAxisd aa_e(theta * dt, ww);
        // cout << "aa_e: \t" << aa_e.axis().transpose() << " " << aa_e.angle() << endl;

        R_e = aa_e.toRotationMatrix() * R_0;
        // cout << "R_e: \n" << R_e << endl;
        p_e = p_0 + dt * (J_0_xyz * v_e);

        err_xyz = p_r-p_e;

        AngleAxisd aa_err((R_r)*(R_e.transpose()));
        err_ang = aa_err.axis() * sin(aa_err.angle());

        // cout << " aa_err: " << aa_err.axis().transpose() << " " << aa_err.angle() << endl;
        // cout << "err_ang: " << err_ang.transpose() << endl;

        MatrixXd L=-0.5*(skew_nr*skew(R_e.col(0))+
                         skew_sr*skew(R_e.col(1))+
                         skew_ar*skew(R_e.col(2)));

        Derr_ang=-dt*(L*J_0_ang);
    }
}

/****************************************************************/
bool ControllerNLP::eval_f(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
            Ipopt::Number &obj_value)
{
    computeQuantities(x,new_x);
    obj_value=(ctrl_ori?err_ang.squaredNorm():0.0);
    // ROS_INFO("err_ang.squaredNorm() %g", err_ang.squaredNorm());
    return true;
}

/****************************************************************/
bool ControllerNLP::eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                 Ipopt::Number *grad_f)
{
    computeQuantities(x,new_x);
    for (Ipopt::Index i=0; i<n; ++i)
    {
        grad_f[i]=(ctrl_ori?2.0*err_ang.dot(Derr_ang.col(i)):0.0);
    }

    return true;
}

// /****************************************************************/
bool ControllerNLP::eval_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
            Ipopt::Index m, Ipopt::Number *g)
{
    computeQuantities(x,new_x);

    // reaching in position
    g[0]=err_xyz.squaredNorm();
    // ROS_INFO("err_xyz.squaredNorm() %g", g[0]);

    return true;
}

/****************************************************************/
bool ControllerNLP::eval_jac_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index *iRow,
                Ipopt::Index *jCol, Ipopt::Number *values)
{
    if (values==NULL)
    {
        Ipopt::Index idx=0;

        // reaching in position
        for (Ipopt::Index i=0; i<n; ++i)
        {
            iRow[i]=0; jCol[i]=i;
            idx++;
        }
    }
    else
    {
        computeQuantities(x,new_x);

        Ipopt::Index idx=0;

        // reaching in position
        for (Ipopt::Index i=0; i<n; ++i)
        {
            values[i]=-2.0*dt*(err_xyz.dot(J_0_xyz.col(i)));
            idx++;
        }
    }

    return true;
}

/****************************************************************/
void ControllerNLP::finalize_solution(Ipopt::SolverReturn status, Ipopt::Index n,
                                      const Ipopt::Number *x, const Ipopt::Number *z_L,
                                      const Ipopt::Number *z_U, Ipopt::Index m,
                                      const Ipopt::Number *g, const Ipopt::Number *lambda,
                                      Ipopt::Number obj_value, const Ipopt::IpoptData *ip_data,
                                      Ipopt::IpoptCalculatedQuantities *ip_cq)
{
    for (Ipopt::Index i=0; i<n; ++i)
    {
        v_e[i]=x[i];
    }

    // printf("\n");

    switch(status)
    {
        case Ipopt::SUCCESS             : break;
        case Ipopt::CPUTIME_EXCEEDED    : ROS_WARN("Maximum CPU time exceeded.");  break;
        case Ipopt::LOCAL_INFEASIBILITY : ROS_WARN("Algorithm converged to a point of local infeasibility. "
                                                   "Problem may be infeasible.");  break;
        case Ipopt::DIVERGING_ITERATES  : ROS_WARN("Iterates divering; problem might be unbounded."); break;
        default                         : ROS_WARN("IPOPT Failed. Error code: %i", status);
        // Error codes: https://www.coin-or.org/Ipopt/doxygen/classorg_1_1coinor_1_1Ipopt.html
    }

    Eigen::VectorXd pos_err = (p_e-p_r) * 1000.0;
    ROS_INFO("  pos err [mm]: %s\tsquared norm [mm]: %g", toString(std::vector<double>(pos_err.data(),
                                    pos_err.data() + pos_err.size())).c_str(), pos_err.squaredNorm());

    if (ctrl_ori)
    {
        Quaterniond o_e(R_e);

        // cout << "o_r: " << o_r.vec().transpose() << " " << o_r.w() << endl;
        // cout << "o_e: " << o_e.vec().transpose() << " " << o_e.w() << endl;
        ROS_INFO("  ori err [quaternion dot product]: %g err_ang %g", o_e.dot(o_r), err_ang.squaredNorm());
    }

    // ROS_INFO("  initial  position: %s", toString(std::vector<double>(p_0.data(),
    //                                           p_0.data() + p_0.size())).c_str());
    // ROS_INFO("  desired  position: %s", toString(std::vector<double>(p_r.data(),
    //                                           p_r.data() + p_r.size())).c_str());
    // ROS_INFO("  computed position: %s", toString(std::vector<double>(p_e.data(),
    //                                           p_e.data() + p_e.size())).c_str());

    // VectorXd j(chain.getNrOfJoints());

    // for (size_t i = 0; i < chain.getNrOfJoints(); ++i)
    // {
    //     j(i) = q_0[i] + (dt * v_e[i]);
    // }

    // ROS_INFO("initial  joint vels: %s", toString(std::vector<double>(v_0.data(),
    //                                          v_0.data() + v_0.size())).c_str());
    // ROS_INFO("initial joint state: %s", toString(std::vector<double>(q_0.data(),
    //                                          q_0.data() + q_0.size())).c_str());
    // ROS_INFO("computed joint vels: %s", toString(std::vector<double>(v_e.data(),
    //                                          v_e.data() + v_e.size())).c_str());
    // ROS_INFO("computed next state: %s", toString(std::vector<double>(j.data(),
    //                                            j.data() + j.size())).c_str());
}

ControllerNLP::~ControllerNLP()
{
    return;
}

