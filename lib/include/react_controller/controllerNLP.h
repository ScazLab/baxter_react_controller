#ifndef __REACTIPOPT_H__
#define __REACTIPOPT_H__

#include <IpTNLP.hpp>
#include <IpIpoptApplication.hpp>
#include <Eigen/Dense>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include <math.h>

#include "react_controller/baxterChain.h"

#define CTRL_RAD2DEG (180.0 / M_PI)
#define CTRL_DEG2RAD (M_PI/180.0)

/****************************************************************/
class ControllerNLP : public Ipopt::TNLP
{
private:
    // Chain to solve the IK against
    BaxterChain chain;

    // Period of the control thread
    double dt;

    // Flag to know if to control the orientation or not
    bool ctrl_ori;

    Eigen::Vector3d     p_0;  // Initial 3D position
    Eigen::Quaterniond  o_0;  // Initial 3D orientation
    Eigen::VectorXd     q_0;  // Initial ND joint configuration
    Eigen::VectorXd     v_0;  // Initial ND joint velocities
    Eigen::Matrix4d     H_0;  // Initial 4x4 transform matrix
    Eigen::Matrix3d     R_0;  // Initial 3x3 rotation matrix
    Eigen::MatrixXd  J0_xyz;  // Initial Jacobian (positional component)
    Eigen::MatrixXd  J0_ang;  // Initial Jacobian (orientational component)

    Eigen::VectorXd x_r;      // Reference 6D  pose
    Eigen::Vector3d p_r;      // Reference 3D  position
    Eigen::Matrix4d H_r;      // Reference 4x4 transform matrix

    Eigen::Vector3d p_e;      // Estimated 3D position
    Eigen::Matrix4d H_e;      // Estimated 4x4 transform matrix

    Eigen::VectorXd err_xyz;  // Positional error
    Eigen::VectorXd err_ang;  // Orientational error
    Eigen::MatrixXd Derr_ang; // Derivative of the orientational error

    Eigen::MatrixXd  skew_nr;
    Eigen::MatrixXd  skew_sr;
    Eigen::MatrixXd  skew_ar;
    Eigen::MatrixX2d q_lim;
    Eigen::MatrixX2d v_lim;
    Eigen::VectorXd  v;

    Eigen::MatrixX2d bounds;

    Eigen::VectorXd qGuard;
    Eigen::VectorXd qGuardMinExt;
    Eigen::VectorXd qGuardMinInt;
    Eigen::VectorXd qGuardMinCOG;
    Eigen::VectorXd qGuardMaxExt;
    Eigen::VectorXd qGuardMaxInt;
    Eigen::VectorXd qGuardMaxCOG;

    /****************************************************************/
    void computeGuard();
    void computeBounds();
    Eigen::MatrixXd v2m(const Eigen::VectorXd &x);
    Eigen::MatrixXd skew(const Eigen::VectorXd &w);

public:
    ControllerNLP(BaxterChain chain_, double dt_ = 0.01, bool ctrl_ori_ = false);

    void init();
    Eigen::VectorXd get_result() const;

    void computeQuantities(const Ipopt::Number *x, const bool new_x);
    bool eval_f(Ipopt::Index n, const Ipopt::Number *x, bool new_x, Ipopt::Number &obj_value);
    bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number *grad_f);
    bool eval_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,Ipopt::Index m, Ipopt::Number *g);
    bool eval_jac_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x, Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index *iRow,
                    Ipopt::Index *jCol, Ipopt::Number *values);
    void finalize_solution(Ipopt::SolverReturn status, Ipopt::Index n, const Ipopt::Number *x, const Ipopt::Number *z_L,
                           const Ipopt::Number *z_U, Ipopt::Index m, const Ipopt::Number *g, const Ipopt::Number *lambda,
                           Ipopt::Number obj_value, const Ipopt::IpoptData *ip_data, Ipopt::IpoptCalculatedQuantities *ip_cq);

    void set_x_r(const Eigen::VectorXd &_x_r);
    void set_v_lim(const Eigen::MatrixXd &_v_lim);
    void set_ctrl_ori(const bool _ctrl_ori);
    void set_dt(const double _dt);
    void set_v_0(const Eigen::VectorXd &_v_0);

    // Property getParameters() const;
    bool get_nlp_info(Ipopt::Index &n, Ipopt::Index &m, Ipopt::Index &nnz_jac_g,
                      Ipopt::Index &nnz_h_lag, IndexStyleEnum &index_style);
    bool get_bounds_info(Ipopt::Index n, Ipopt::Number *x_l, Ipopt::Number *x_u,
                         Ipopt::Index m, Ipopt::Number *g_l, Ipopt::Number *g_u);
    bool get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number *x,
                            bool init_z, Ipopt::Number *z_L, Ipopt::Number *z_U,
                            Ipopt::Index m, bool init_lambda, Ipopt::Number *lambda);
    ~ControllerNLP();
};

#endif
