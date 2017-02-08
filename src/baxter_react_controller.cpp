#include <stdio.h>

#include <ros/ros.h>
#include "react_controller/reactIpOpt.h"

using namespace std;

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "baxter_react_controller");
    ros::NodeHandle _n("baxter_react_controller");

    double tol = 1e-6;
    double dT = 0.001;
    VectorXd xr(6);
    MatrixXd vLimAdapted(3, 3);
    VectorXd q_dot(6);
    VectorXd res(3);

    bool verbosity = false;
    bool controlMode = true;
    bool hittingConstraints = true;
    bool orientationControl = true;
    int _exit_code;


    Ipopt::SmartPtr<Ipopt::IpoptApplication> app=new Ipopt::IpoptApplication;
    app->Options()->SetNumericValue("tol",tol);
    app->Options()->SetNumericValue("constr_viol_tol",1e-6);
    app->Options()->SetIntegerValue("acceptable_iter",0);
    app->Options()->SetStringValue("mu_strategy","adaptive");
    app->Options()->SetIntegerValue("max_iter",std::numeric_limits<int>::max());
    app->Options()->SetNumericValue("max_cpu_time",0.75*dT);
    app->Options()->SetStringValue("nlp_scaling_method","gradient-based");
    app->Options()->SetStringValue("hessian_approximation","limited-memory");
    app->Options()->SetStringValue("derivative_test",verbosity?"first-order":"none");
    app->Options()->SetIntegerValue("print_level",verbosity?5:0);
    app->Initialize();

    Ipopt::SmartPtr<ControllerNLP> nlp;
    // if (controlMode == "positionDirect") //in this mode, ipopt will use the qIntegrated values to update its copy of chain
    //      nlp=new ControllerNLP(*virtualArmChain);
    // else
    //      nlp=new ControllerNLP(*(arm->asChain()));
    KDL::Chain chain;
    nlp=new ControllerNLP(chain);
    nlp->set_hitting_constraints(hittingConstraints);
    nlp->set_orientation_control(orientationControl);
    nlp->set_dt(dT);
    nlp->set_xr(xr);
    nlp->set_v_limInDegPerSecond(vLimAdapted);
    nlp->set_v0InDegPerSecond(q_dot);
    nlp->init();

    _exit_code=app->OptimizeTNLP(GetRawPtr(nlp));

    res=nlp->get_resultInDegPerSecond();

    printf("\n");
    ROS_INFO("READY! Waiting for control messages..\n");

    ros::spin();
    return 0;
}

