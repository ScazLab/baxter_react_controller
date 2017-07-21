#include <math.h>
#include <ros/ros.h>

#include "react_controller/controllerNLP.h"
#include "react_controller/react_control_utils.h"

using namespace Eigen;
using namespace   std;

ControllerNLP::ControllerNLP(BaxterChain chain_, double dt_, bool ctrl_ori_) :
                             chain(chain_), int_status(true), dt(dt_), ctrl_ori(ctrl_ori_), print_level(0),
                             pid(10.0), q_0(chain_.getNrOfJoints()), v_0(chain_.getNrOfJoints()),
                             J_0_xyz(3,chain_.getNrOfJoints()), J_0_ang(3,chain_.getNrOfJoints()),
                             v_e(chain_.getNrOfJoints()), q_lim(chain_.getNrOfJoints(),2),
                             v_lim(chain_.getNrOfJoints(),2), bounds(chain_.getNrOfJoints(),2),
                             qGuard(chain_.getNrOfJoints()),
                             qGuardMinExt(chain_.getNrOfJoints()), qGuardMinInt(chain_.getNrOfJoints()),
                             qGuardMinCOG(chain_.getNrOfJoints()), qGuardMaxExt(chain_.getNrOfJoints()),
                             qGuardMaxInt(chain_.getNrOfJoints()), qGuardMaxCOG(chain_.getNrOfJoints())
{
    v_0.setZero();
    v_e.setZero();

    R_e.setIdentity();
    R_r.setIdentity();

    for (size_t r=0; r<chain_.getNrOfJoints(); r++)
    {
        /* angle bounds */
        q_lim(r,0)=chain.getMin(r);
        q_lim(r,1)=chain.getMax(r);

        v_lim(r,1)=std::numeric_limits<double>::max();
        v_lim(r,0)=-v_lim(r,1);
    }
    bounds=v_lim;
}

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

void ControllerNLP::computeBounds()
{
    computeGuard();

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

    ROS_INFO_STREAM_COND(print_level>=8, "bounds before" << endl << bounds);

    for (size_t i=0; i<chain.getNrOfJoints(); ++i)
    {
        bounds(i,0)*=v_lim(i,0);
        bounds(i,1)*=v_lim(i,1);
    }

    ROS_INFO_STREAM_COND(print_level>=4, "Bounds:" << endl << bounds);

    // for (size_t i=0; i<chain.getNrOfJoints(); ++i)
    // {
    //     bounds(i,0)=v_lim(i,0);
    //     bounds(i,1)=v_lim(i,1);
    // }
}

void ControllerNLP::set_x_r(const Eigen::Vector3d &_p_r, const Eigen::Quaterniond &_o_r)
{
    p_r = _p_r;
    o_r = _o_r;
    R_r =  o_r.toRotationMatrix();

    skew_nr = skew(R_r.col(0));
    skew_sr = skew(R_r.col(1));
    skew_ar = skew(R_r.col(2));
}

void ControllerNLP::set_v_lim(const MatrixXd &_v_lim)
{
    v_lim = DEG2RAD*_v_lim;

    // if (print_level >= 2)
    // {
    //     // Print stuff in a single line for convenience
    //     MatrixXd vlim = v_lim;
    //     vlim.transposeInPlace();
    //     VectorXd vlim_vec(Map<VectorXd>(vlim.data(), vlim.cols()*vlim.rows()));

    //     ROS_INFO_STREAM("vlim: " << vlim_vec.transpose());
    // }
}

void ControllerNLP::set_ctrl_ori(const bool _ctrl_ori)
{
    ctrl_ori=_ctrl_ori;
}

void ControllerNLP::set_dt(const double _dt)
{
    ROS_ASSERT(dt>0.0);

    dt = _dt;
    ROS_INFO_COND(print_level>=12, "dT set to: %g", dt);
}

void ControllerNLP::set_v_0(const VectorXd &_v_0)
{
    ROS_ASSERT(v_0.size() == _v_0.size());
    v_0 = _v_0;
}

void ControllerNLP::set_print_level(size_t _print_level)
{
    print_level = _print_level;
}

void ControllerNLP::init()
{
    int_status = true;

    q_0 = chain.getAng();
    // v_0 = chain.getVel();

    ROS_INFO_STREAM_COND(print_level>=2, "q_0: [" << q_0.transpose() << "]");
    ROS_INFO_STREAM_COND(print_level>=2, "v_0: [" << v_0.transpose() << "]");

    Matrix4d H_0 = chain.getH();
    R_0 = H_0.block<3,3>(0,0);
    p_0 = H_0.block<3,1>(0,3);

    ROS_INFO_STREAM_COND(print_level>=6, "H_0: \n" << H_0);
    ROS_INFO_STREAM_COND(print_level>=6, "R_0: \n" << R_0);
    ROS_INFO_STREAM_COND(print_level>=6, "p_0: \t" << p_0.transpose());

    MatrixXd J_0 = chain.GeoJacobian();
    J_0_xyz = J_0.block(0,0,3,chain.getNrOfJoints());
    J_0_ang = J_0.block(3,0,3,chain.getNrOfJoints());

    ROS_INFO_STREAM_COND(print_level>=6, "J_0:    \n" << J_0    );
    ROS_INFO_STREAM_COND(print_level>=3, "J_0_xyz:\n" << J_0_xyz);

    ROS_INFO_STREAM_COND(print_level>=3 && ctrl_ori, "J_0_ang:\n" << J_0_ang);

    computeBounds();
}

VectorXd ControllerNLP::get_est_vels()
{
    return v_e;
}

VectorXd ControllerNLP::get_est_conf()
{
    return q_0 + (pid * dt * v_e);
}

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

bool ControllerNLP::get_bounds_info(Ipopt::Index n, Ipopt::Number *x_l, Ipopt::Number *x_u,
                                    Ipopt::Index m, Ipopt::Number *g_l, Ipopt::Number *g_u)
{
    for (Ipopt::Index i=0; i<n; ++i)
    {
        x_l[i]=bounds(i,0);
        x_u[i]=bounds(i,1);

        if (x_l[i] > x_u[i])
        {
            ROS_ERROR("Inconsistent variable bounds! Joint: %i x_l[i]: %g x_u[i]: %g",
                                                              int(i), x_l[i], x_u[i]);
            ROS_ERROR("qGuard: %g Max: %g Min: %g", qGuard[i], chain.getMax(i), chain.getMin(i));
            ROS_ERROR("qGuardMinExt: %g qGuardMinInt: %g qGuardMinCOG: %g",
                       qGuardMinExt[i], qGuardMinInt[i], qGuardMinCOG[i]);
            ROS_ERROR("qGuardMaxExt: %g qGuardMaxInt: %g qGuardMaxCOG: %g",
                       qGuardMaxExt[i], qGuardMaxInt[i], qGuardMaxCOG[i]);
            ROS_ASSERT(false);
        }
    }

    // reaching in position
    g_l[0]=-1e-11;
    g_u[0]=+1e-11;

    return true;
}

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

void ControllerNLP::computeQuantities(const Ipopt::Number *x, const bool new_x)
{
    if (new_x)
    {
        // Let's update the estimated velocities
        for (int i=0; i<v_e.size(); ++i)
        {
            v_e[i]=x[i];
        }

        p_e = p_0 + pid * dt * (J_0_xyz * v_e);

        err_xyz = p_r-p_e;

        ROS_INFO_STREAM_COND(print_level>=4, "    v_e: " <<     v_e.transpose());
        ROS_INFO_STREAM_COND(print_level>=4, "err_xyz: " << err_xyz.transpose() <<
                                        " squaredNorm: " << err_xyz.squaredNorm());
        if (ctrl_ori)
        {
            // Now, let's compute the position and orientation errors
            // See https://math.stackexchange.com/questions/773902/integrating-body-angular-velocity/2176586#217658
            Vector3d  w_e = J_0_ang*v_e;          // rotational (angular) speed
            double theta =   w_e.norm();
            if (theta > 0.0) { w_e /= theta; }

            AngleAxisd w_e_aa(theta * pid * dt, w_e);   // angular increment in axis angle representation
            ROS_INFO_STREAM_COND(print_level>=5, "w_e_aa: \t" <<
                                 w_e_aa.axis().transpose() << " " << w_e_aa.angle());

            R_e = w_e_aa.toRotationMatrix() * R_0;
            ROS_INFO_STREAM_COND(print_level>=5, "R_e: \n" << R_e);

            Quaterniond o_e(R_e);

            err_ang = angularError(R_r, R_e);
            // err_ang = angularError(o_r, o_e);
            ROS_INFO_STREAM_COND(print_level>=4, "err_ang: " << err_ang.transpose() <<
                                            " squaredNorm: " << err_ang.squaredNorm());

            MatrixXd L=-0.5*(skew_nr*skew(R_e.col(0))+
                             skew_sr*skew(R_e.col(1))+
                             skew_ar*skew(R_e.col(2)));

            Derr_ang=-pid*dt*(L*J_0_ang);
        }
    }
}

bool ControllerNLP::eval_f(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
            Ipopt::Number &obj_value)
{
    computeQuantities(x,new_x);
    obj_value=(ctrl_ori?err_ang.squaredNorm():0.0);
    // ROS_INFO("err_ang.squaredNorm() %g", err_ang.squaredNorm());
    return true;
}

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

bool ControllerNLP::eval_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
            Ipopt::Index m, Ipopt::Number *g)
{
    computeQuantities(x,new_x);

    // reaching in position
    g[0]=err_xyz.squaredNorm();
    // ROS_INFO("err_xyz.squaredNorm() %g", g[0]);

    return true;
}

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
            values[i]=-2.0*pid*dt*(err_xyz.dot(J_0_xyz.col(i)));
            idx++;
        }
    }

    return true;
}

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
    string print_str = "";
    if (status != Ipopt::SUCCESS) { print_str = "IPOPT Failed. Error code: " + toString(status) + ". "; }

    switch(status)
    {
        case Ipopt::SUCCESS                  : break;
        case Ipopt::CPUTIME_EXCEEDED         : ROS_WARN((print_str + "Maximum CPU time exceeded.").c_str()); break;
        case Ipopt::LOCAL_INFEASIBILITY      : ROS_WARN((print_str + "Algorithm converged to a point of local infeasibility. "
                                                        "Problem may be infeasible.").c_str()); break;
        case Ipopt::DIVERGING_ITERATES       : ROS_WARN((print_str + "Iterates divering; problem might be unbounded.").c_str()); break;
        case Ipopt::STOP_AT_ACCEPTABLE_POINT : ROS_WARN((print_str + "Solved to acceptable level.").c_str()); break;
        default : break;
        // Error codes: https://www.coin-or.org/Ipopt/doxygen/classorg_1_1coinor_1_1Ipopt.html
        //    see also: https://www.coin-or.org/Doxygen/CoinAll/_ip_alg_types_8hpp-source.html#l00022
        // enum SolverReturn {
        //     SUCCESS, MAXITER_EXCEEDED, CPUTIME_EXCEEDED, STOP_AT_TINY_STEP, STOP_AT_ACCEPTABLE_POINT,
        //     LOCAL_INFEASIBILITY, USER_REQUESTED_STOP, FEASIBLE_POINT_FOUND, DIVERGING_ITERATES,
        //     RESTORATION_FAILURE, ERROR_IN_STEP_COMPUTATION, INVALID_NUMBER_DETECTED,
        //     TOO_FEW_DEGREES_OF_FREEDOM, INVALID_OPTION, OUT_OF_MEMORY, INTERNAL_ERROR, UNASSIGNED
        // };
    }

    Eigen::VectorXd pos_0rr = (p_0-p_r) * 1000.0;
    Eigen::VectorXd pos_err = (p_e-p_r) * 1000.0;

    ROS_INFO_STREAM_COND(print_level>=1, "ref  pos [p_r]: " << p_r.transpose());
    ROS_INFO_STREAM_COND(print_level>=1, "init err [p_0]: " << pos_0rr.transpose() <<
                                         "\t\tsqNorm[mm]: " << pos_0rr.squaredNorm());
    ROS_INFO_STREAM_COND(print_level>=1, "est  err [p_e]: " << pos_err.transpose() <<
                                         "\t\tsqNorm[mm]: " << pos_err.squaredNorm());

    // ROS_INFO_STREAM("  pos 0rr [mm]: " << pos_0rr.transpose() <<
    //            "\tsquared norm [mm]: " << pos_0rr.squaredNorm());

    if (status == Ipopt::SUCCESS)
    {
        ROS_INFO_STREAM("IPOPT succeeded! pos err (sq.norm) [mm]: " << pos_err.squaredNorm());
    }
    else
    {
        ROS_INFO_STREAM_COND(print_level>=2, "  pos err [mm]: " << pos_err.transpose() <<
                                        "\tsquared norm [mm]: " << pos_err.squaredNorm());
    }

    if (ctrl_ori)
    {
        Quaterniond o_e(R_e);
        // Quaterniond o_0(R_0);

        // ROS_INFO_STREAM("init ori [o_0]: " << o_0.vec().transpose() << " " << o_0.w());
        // ROS_INFO_STREAM("ref  ori [o_r]: " << o_r.vec().transpose() << " " << o_r.w());
        // ROS_INFO_STREAM("est  ori [o_e]: " << o_e.vec().transpose() << " " << o_e.w());
        // ROS_INFO("ori 0rr [rad?]: %g\t0rr_ang %g",
        //           o_0.dot(o_r), angularError(o_r, o_0).squaredNorm());
        ROS_INFO("ori err [rad?]: %g\terr_ang %g",
                  o_e.dot(o_r),                err_ang.squaredNorm());

        // ROS_INFO_STREAM("R_0: \n" << R_0);
        // ROS_INFO_STREAM("o_0: \n" << o_0.toRotationMatrix());
        // ROS_INFO_STREAM("o_r: \n" << o_r.toRotationMatrix());
    }

    VectorXd q_e = get_est_conf();

    ROS_INFO_STREAM_COND(print_level>=2, "v_0: " << v_0.transpose());
    ROS_INFO_STREAM_COND(print_level>=2, "q_0: " << q_0.transpose());
    ROS_INFO_STREAM_COND(print_level>=2, "v_e: " << v_e.transpose());
    // ROS_WARN_STREAM_COND(v_e.norm()>=0.5, "v_e norm is high: " << v_e.norm());
    ROS_INFO_STREAM_COND(print_level>=2, "q_e: " << VectorXd(q_e).transpose());

    if (print_level>=1)
    {
        // This is basically a check that the positional component of the jacobian is correct.
        // It compares the estimated position at the end effector if computed with the jacobian
        // p_e = p_0 + pid * dt * (J_0_xyz * v_e);
        // with the one computed with standard kinematics, i.e. with the new estimated joint conf:
        // q_e = q_0 + pid * dt * v_e -> p_ee = K_p(q_e)
        // For dt small enough, p_e should be very close to p_ee
        BaxterChain ch(chain);
        ch.setAng(q_e);
        Vector3d p_ee = ch.getH().block<3,1>(0,3);

        if ((p_e - p_ee).squaredNorm() >= 1e-3)
        {
            ROS_ERROR_STREAM("q_ee: " << ch.getAng().transpose());
            ROS_ERROR_STREAM("p_e : " << p_e .transpose());
            ROS_ERROR_STREAM("p_ee: " << p_ee.transpose());
            ROS_ERROR_STREAM("(p_e - p_ee).squaredNorm(): " <<
                              (p_e - p_ee).squaredNorm());

            int_status = false;
        }
    }

    if (status == Ipopt::SUCCESS)
    {
        ROS_ASSERT_MSG(pos_err.squaredNorm() < 10, "This should never happen!");
    }

}

ControllerNLP::~ControllerNLP()
{
    return;
}

