#include <math.h>
#include <ros/ros.h>

#include "react_controller/controllerNLP.h"
#include "react_controller/mathUtils.h"

using namespace Eigen;

/****************************************************************/
ControllerNLP::ControllerNLP(BaxterChain chain_) : chain(chain_)
{
    xr.resize(6); xr.setZero();
    set_xr(xr);

    v0.resize(chain.getNrOfJoints()); v0.setZero();
    v=v0;
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

    computeSelfAvoidanceConstraints();
    computeGuard();

    hitting_constraints=false;
    orientation_control=true;
    dt=0.01;
}

/****************************************************************/
void ControllerNLP::computeSelfAvoidanceConstraints()
{
    double joint1_0, joint1_1;
    double joint2_0, joint2_1;
    joint1_0= 28.0*CTRL_DEG2RAD;
    joint1_1= 23.0*CTRL_DEG2RAD;
    joint2_0=-37.0*CTRL_DEG2RAD;
    joint2_1= 80.0*CTRL_DEG2RAD;
    shou_m=(joint1_1-joint1_0)/(joint2_1-joint2_0);
    shou_n=joint1_0-shou_m*joint2_0;

    double joint3_0, joint3_1;
    double joint4_0, joint4_1;
    joint3_0= 85.0*CTRL_DEG2RAD;
    joint3_1=105.0*CTRL_DEG2RAD;
    joint4_0= 90.0*CTRL_DEG2RAD;
    joint4_1= 40.0*CTRL_DEG2RAD;
    elb_m=(joint4_1-joint4_0)/(joint3_1-joint3_0);
    elb_n=joint4_0-elb_m*joint3_0;
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

    for (size_t i=0; i<chain.getNrOfJoints(); i++)
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

    for (size_t i=0; i<chain.getNrOfJoints(); i++)
    {
        double qi=q0[i];
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
    Vector4d ang; ang.setZero();
    ang.block(0, 0, 3, 1) = x.tail(3);
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
    xr=_xr;

    Hr=v2m(xr);
    pr=xr.block(0, 0, 3, 1);
    skew_nr=skew(Hr.col(0));
    skew_sr=skew(Hr.col(1));
    skew_ar=skew(Hr.col(2));
}

/****************************************************************/
void ControllerNLP::set_v_limInDegPerSecond(const MatrixXd &_v_lim)
{
    v_lim=CTRL_DEG2RAD*_v_lim;
}

/****************************************************************/
void ControllerNLP::set_hitting_constraints(const bool _hitting_constraints)
{
    hitting_constraints=_hitting_constraints;
}

/****************************************************************/
void ControllerNLP::set_orientation_control(const bool _orientation_control)
{
    orientation_control=_orientation_control;
}

/****************************************************************/
void ControllerNLP::set_dt(const double _dt)
{
    dt=_dt;
}

/****************************************************************/
void ControllerNLP::set_v0InDegPerSecond(const VectorXd &_v0)
{
    v0=CTRL_DEG2RAD*_v0;
}

/****************************************************************/
void ControllerNLP::init()
{
    q0=chain.getAng();
    H0=chain.getH();
    R0=H0.block(0,0,3,3);
    p0=H0.col(3).block(0, 0, 3, 1);
    std::cout << H0 << "\n";

    MatrixXd J0=chain.GeoJacobian();
    J0_xyz=J0.block(0,0,3,chain.getNrOfJoints());
    J0_ang=J0.block(3,0,3,chain.getNrOfJoints());

    computeBounds();
}

/****************************************************************/
VectorXd ControllerNLP::get_resultInDegPerSecond() const
{
    return CTRL_RAD2DEG*v;
}

//     /****************************************************************/
//     Property ControllerNLP::getParameters() const
//     {
//         Property parameters;
//         parameters.put("dt",dt);
//         return parameters;
//     }

/****************************************************************/
bool ControllerNLP::get_nlp_info(Ipopt::Index &n, Ipopt::Index &m, Ipopt::Index &nnz_jac_g,
                  Ipopt::Index &nnz_h_lag, IndexStyleEnum &index_style)
{
    ROS_INFO("get_nlp_info");

    n=chain.getNrOfJoints();

    // reaching in position
    m=1; nnz_jac_g=n;

    if (hitting_constraints)
    {
        // shoulder's cables length
        m+=3; nnz_jac_g+=2+3+2;

        // avoid hitting torso
        m+=1; nnz_jac_g+=2;

        // avoid hitting forearm
        m+=2; nnz_jac_g+=2+2;
    }

    nnz_h_lag=0;
    index_style=TNLP::C_STYLE;
    return true;
}

/****************************************************************/
bool ControllerNLP::get_bounds_info(Ipopt::Index n, Ipopt::Number *x_l, Ipopt::Number *x_u,
                     Ipopt::Index m, Ipopt::Number *g_l, Ipopt::Number *g_u)
{
    ROS_INFO("get_bounds_info");

    for (Ipopt::Index i=0; i<n; i++)
    {
        x_l[i]=bounds(i,0);
        x_u[i]=bounds(i,1);
    }

    // reaching in position
    g_l[0]=g_u[0]=0.0;

    if (hitting_constraints)
    {
        // shoulder's cables length
        g_l[1]=-347.00*CTRL_DEG2RAD;
        g_u[1]=std::numeric_limits<double>::max();
        g_l[2]=-366.57*CTRL_DEG2RAD;
        g_u[2]=112.42*CTRL_DEG2RAD;
        g_l[3]=-66.60*CTRL_DEG2RAD;
        g_u[3]=213.30*CTRL_DEG2RAD;

        // avoid hitting torso
        g_l[4]=shou_n;
        g_u[4]=std::numeric_limits<double>::max();

        // avoid hitting forearm
        g_l[5]=-std::numeric_limits<double>::max();
        g_u[5]=elb_n;
        g_l[6]=-elb_n;
        g_u[6]=std::numeric_limits<double>::max();
    }

    return true;
}

/****************************************************************/
bool ControllerNLP::get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number *x,
                        bool init_z, Ipopt::Number *z_L, Ipopt::Number *z_U,
                        Ipopt::Index m, bool init_lambda, Ipopt::Number *lambda)
{
    for (Ipopt::Index i=0; i<n; i++)
        x[i]=std::min(std::max(bounds(i,0),v0[i]),bounds(i,1));
    return true;
}

/************************************************************************/
void ControllerNLP::computeQuantities(const Ipopt::Number *x, const bool new_x)
{
    // ROS_INFO("computeQuantities");

    if (new_x)
    {
        for (size_t i=0; i<6; i++)
            v[i]=x[i];

        std::cout << v << "\n";
        std::cout << "Just printed v\n";

        MatrixXd sub = R0+dt*(skew(J0_ang*v)*R0);
        He.block<3,3>(0, 0) = sub;
        VectorXd pe=p0+dt*(J0_xyz*v);
        He(0,3)=pe[0];
        He(1,3)=pe[1];
        He(2,3)=pe[2];

        err_xyz=pr-pe;
        std::cout << p0 << "\n";
        std::cout << "Just printed p0\n";
        std::cout << pr << "\n";
        std::cout << "Just printed pr\n";
        std::cout << pe << "\n";
        std::cout << "Just printed pe\n";
        err_ang=dcm2axis(Hr*He.transpose());
        err_ang*=err_ang[3];
        err_ang.resize(3);

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
    obj_value=(orientation_control?err_ang.squaredNorm():0.0);
    ROS_INFO("obj_value: %g", obj_value);
    return true;
}

/****************************************************************/
bool ControllerNLP::eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                 Ipopt::Number *grad_f)
{
    computeQuantities(x,new_x);
    for (Ipopt::Index i=0; i<n; i++) {
        grad_f[i]=(orientation_control?2.0*err_ang.dot(Derr_ang.col(i)):0.0);
    }

    ROS_INFO("grad_f: %g %g %g %g %g %g %g", grad_f[0], grad_f[1], grad_f[2], grad_f[3], grad_f[4], grad_f[5], grad_f[6]);
    return true;
}

// /****************************************************************/
bool ControllerNLP::eval_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
            Ipopt::Index m, Ipopt::Number *g)
{
    computeQuantities(x,new_x);

    // reaching in position
    g[0]=err_xyz.squaredNorm();
    ROS_INFO("err_xyz norm: %g", g[0]);

    if (hitting_constraints)
    {
        // shoulder's cables length
        g[1]=1.71*(q0[3+0]+dt*x[3+0]-(q0[3+1]+dt*x[3+1]));
        g[2]=1.71*(q0[3+0]+dt*x[3+0]-(q0[3+1]+dt*x[3+1])-(q0[3+2]+dt*x[3+2]));
        g[3]=q0[3+1]+dt*x[3+1]+q0[3+2]+dt*x[3+2];

        // avoid hitting torso
        g[4]=q0[3+1]+dt*x[3+1]-shou_m*(q0[3+2]+dt*x[3+2]);

        // avoid hitting forearm
        g[5]=-elb_m*(q0[3+3+0]+dt*x[3+3+0])+q0[3+3+1]+dt*x[3+3+1];
        g[6]=elb_m*(q0[3+3+0]+dt*x[3+3+0])+q0[3+3+1]+dt*x[3+3+1];
    }
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

        if (hitting_constraints)
        {
            // shoulder's cables length
            iRow[idx]=1; jCol[idx]=3+0; idx++;
            iRow[idx]=1; jCol[idx]=3+1; idx++;

            iRow[idx]=2; jCol[idx]=3+0; idx++;
            iRow[idx]=2; jCol[idx]=3+1; idx++;
            iRow[idx]=2; jCol[idx]=3+2; idx++;

            iRow[idx]=3; jCol[idx]=3+1; idx++;
            iRow[idx]=3; jCol[idx]=3+2; idx++;

            // avoid hitting torso
            iRow[idx]=4; jCol[idx]=3+1; idx++;
            iRow[idx]=4; jCol[idx]=3+2; idx++;

            // avoid hitting forearm
            iRow[idx]=5; jCol[idx]=3+3+0; idx++;
            iRow[idx]=5; jCol[idx]=3+3+1; idx++;

            iRow[idx]=6; jCol[idx]=3+3+0; idx++;
            iRow[idx]=6; jCol[idx]=3+3+1; idx++;
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
        ROS_INFO("values: %g %g %g %g %g %g %g", values[0], values[1], values[2], values[3], values[4], values[5], values[6]);

        if (hitting_constraints)
        {
            // shoulder's cables length
            values[idx++]=1.71*dt;
            values[idx++]=-1.71*dt;

            values[idx++]=1.71*dt;
            values[idx++]=-1.71*dt;
            values[idx++]=-1.71*dt;

            values[idx++]=dt;
            values[idx++]=dt;

            // avoid hitting torso
            values[idx++]=dt;
            values[idx++]=-shou_m*dt;

            // avoid hitting forearm
            values[idx++]=-elb_m*dt;
            values[idx++]=dt;

            values[idx++]=elb_m*dt;
            values[idx++]=dt;
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
}

ControllerNLP::~ControllerNLP() {
    return;
}

