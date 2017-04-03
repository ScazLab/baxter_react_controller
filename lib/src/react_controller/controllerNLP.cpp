#include <math.h>
#include <ros/ros.h>

#include "react_controller/controllerNLP.h"
#include "react_controller/mathUtils.h"

using namespace Eigen;
using namespace   std;

/****************************************************************/
ControllerNLP::ControllerNLP(BaxterChain chain_, double dt_, bool ctrl_ori_) :
                                chain(chain_), dt(dt_), ctrl_ori(ctrl_ori_)
{
    xr.resize(6); //xr.setZero(); set_xr(xr);

    v_0.resize(chain.getNrOfJoints()); v_0.setZero();
    v=v_0;
    He.resize(4, 4);
    He.setZero();
    He(3,3)=1.0;

    q_lim.resize(chain.getNrOfJoints(),2);
    v_lim.resize(chain.getNrOfJoints(),2);
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
    qGuard.resize(chain.getNrOfJoints());
    qGuardMinExt.resize(chain.getNrOfJoints());
    qGuardMinInt.resize(chain.getNrOfJoints());
    qGuardMinCOG.resize(chain.getNrOfJoints());
    qGuardMaxExt.resize(chain.getNrOfJoints());
    qGuardMaxInt.resize(chain.getNrOfJoints());
    qGuardMaxCOG.resize(chain.getNrOfJoints());

    for (size_t i=0, DOF=chain.getNrOfJoints(); i<DOF; i++)
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

    for (size_t i=0, DOF=chain.getNrOfJoints(); i<DOF; i++)
    {
        double qi=q_0[i];
        if ((qi>=qGuardMinInt[i]) && (qi<=qGuardMaxInt[i]))
            bounds(i,0)=bounds(i,1)=1.0;
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
    };
    for (size_t i=0; i<chain.getNrOfJoints(); i++)
    {
        bounds(i,0)*=v_lim(i,0);
        bounds(i,1)*=v_lim(i,1);
    }
}

/****************************************************************/
MatrixXd ControllerNLP::v2m(const VectorXd &x)
{
    ROS_ASSERT(x.size()>=6);
    Vector4d ang; ang.setZero();
    ang.block<3,1>(0, 0) = x.tail(3);
    double ang_mag=ang.norm();
    if (ang_mag>0.0)
        ang/=ang_mag;
    ang(3, 0) = ang_mag;
    MatrixXd H = axis2dcm(ang);
    H(0,3)=x[0];
    H(1,3)=x[1];
    H(2,3)=x[2];
    return H;
}

/****************************************************************/
MatrixXd ControllerNLP::skew(const VectorXd &w)
{
    ROS_ASSERT(w.size()>=3);
    MatrixXd S(3,3);
    S(0,0)=S(1,1)=S(2,2)=0.0;
    S(1,0)= w[2]; S(0,1)=-S(1,0);
    S(2,0)=-w[1]; S(0,2)=-S(2,0);
    S(2,1)= w[0]; S(1,2)=-S(2,1);
    return S;
}

/****************************************************************/
void ControllerNLP::set_xr(const VectorXd &_xr)
{
    ROS_ASSERT(xr.size() == _xr.size());
    xr=_xr;

    Hr=v2m(xr);
    pr=xr.block<3,1>(0, 0);
    skew_nr=skew(Hr.col(0));
    skew_sr=skew(Hr.col(1));
    skew_ar=skew(Hr.col(2));
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
    H_0=   chain.getH();
    R_0= H_0.block(0,0,3,3);
    x_0= H_0.col(3).block<3,1>(0, 0);

    MatrixXd J0=chain.GeoJacobian();
    J0_xyz=J0.block(0,0,3,chain.getNrOfJoints());
    J0_ang=J0.block(3,0,3,chain.getNrOfJoints());

    // ROS_INFO("           q_0: %s", toString(std::vector<double>(q_0.data(),
    //                                         q_0.data() + q_0.size())).c_str());
    // ROS_INFO("           H_0: %s", toString(std::vector<double>(H_0.data(),
    //                                         H_0.data() + H_0.size())).c_str());

    computeBounds();
}

/****************************************************************/
VectorXd ControllerNLP::get_result() const
{
    return v;
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

    for (Ipopt::Index i=0; i<n; i++)
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
    for (Ipopt::Index i=0; i<n; i++)
        x[i]=std::min(std::max(bounds(i,0),v_0[i]),bounds(i,1));
    return true;
}

/************************************************************************/
void ControllerNLP::computeQuantities(const Ipopt::Number *x, const bool new_x)
{
    if (new_x)
    {
        for (int i=0; i<v.size(); i++) v[i]=x[i];

        MatrixXd sub = R_0+dt*(skew(J0_ang*v)*R_0);
        He.block<3,3>(0, 0) = sub;
        pe=x_0+dt*(J0_xyz*v);
        He(0,3)=pe[0];
        He(1,3)=pe[1];
        He(2,3)=pe[2];

        err_xyz=pr-pe;
        err_ang=dcm2axis(Hr*He.transpose());
        err_ang*=err_ang[3];
        err_ang = err_ang.block<3, 1>(0, 0);

        MatrixXd L=-0.5*(skew_nr*skew(He.col(0))+
                       skew_sr*skew(He.col(1))+
                       skew_ar*skew(He.col(2)));
        Derr_ang=-dt*(L*J0_ang);
    }
}

/****************************************************************/
bool ControllerNLP::eval_f(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
            Ipopt::Number &obj_value)
{
    computeQuantities(x,new_x);
    obj_value=(ctrl_ori?err_ang.squaredNorm():0.0);
    return true;
}

/****************************************************************/
bool ControllerNLP::eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                 Ipopt::Number *grad_f)
{
    computeQuantities(x,new_x);
    for (Ipopt::Index i=0; i<n; i++) {
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
        for (Ipopt::Index i=0; i<n; i++)
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
        for (Ipopt::Index i=0; i<n; i++)
        {
            values[i]=-2.0*dt*(err_xyz.dot(J0_xyz.col(i)));
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
    for (Ipopt::Index i=0; i<n; i++)
        v[i]=x[i];

    switch(status) {
        case Ipopt::SUCCESS             : break;
        case Ipopt::CPUTIME_EXCEEDED    : ROS_WARN("Maximum CPU time exceeded.");  break;
        case Ipopt::LOCAL_INFEASIBILITY : ROS_WARN("Algorithm converged to a point of local infeasibility. "
                                                   "Problem may be infeasible.");  break;
        default                         : ROS_WARN("IPOPT Failed. Error code: %i", status);
        // Error codes: https://www.coin-or.org/Ipopt/doxygen/classorg_1_1coinor_1_1Ipopt.html
    }

    printf("\n");
    ROS_INFO("  initial  position: %s", toString(std::vector<double>(x_0.data(),
                                              x_0.data() + x_0.size())).c_str());
    ROS_INFO("  desired  position: %s", toString(std::vector<double>(pr.data(),
                                               pr.data() + pr.size())).c_str());
    ROS_INFO("  computed position: %s", toString(std::vector<double>(pe.data(),
                                               pe.data() + pe.size())).c_str());

    VectorXd j(chain.getNrOfJoints());

    for (size_t i = 0, _i = chain.getNrOfJoints(); i < _i; ++i)
    {
        j(i) = q_0[i] + (dt * v[i]);
    }

    ROS_INFO("initial  joint vels: %s", toString(std::vector<double>(v_0.data(),
                                                v_0.data() + v_0.size())).c_str());
    ROS_INFO("initial joint state: %s", toString(std::vector<double>(q_0.data(),
                                              q_0.data() + q_0.size())).c_str());
    ROS_INFO("computed joint vels: %s", toString(std::vector<double>(v.data(),
                                                v.data() + v.size())).c_str());
    ROS_INFO("computed next state: %s", toString(std::vector<double>(j.data(),
                                                j.data() + j.size())).c_str());
}

ControllerNLP::~ControllerNLP()
{
    return;
}

