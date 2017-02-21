#include <react_controller/ctrlThread.h>

using namespace              std;
using namespace      sensor_msgs;
using namespace baxter_core_msgs;
using namespace            Eigen;

CtrlThread::CtrlThread(const std::string& _name, const std::string& _limb, bool _no_robot,
                        const std::string& _base_link, const std::string& _tip_link, double _tol,
                        double _vMax, double _dT) : RobotInterface(_name, _limb), tol(_tol), vMax(_vMax),
                        dT(_dT)
{
    urdf::Model robot_model;
    std::string xml_string;

    std::string urdf_xml,full_urdf_xml;
    _n.param<std::string>("urdf_xml",urdf_xml,"/robot_description");
    _n.searchParam(urdf_xml,full_urdf_xml);

    ROS_DEBUG("Reading xml file from parameter server");
    if (!_n.getParam(full_urdf_xml, xml_string))
    {
        ROS_FATAL("Could not load the xml from parameter server: %s", urdf_xml.c_str());
        return;
    }

    _n.param(full_urdf_xml,xml_string,std::string());
    robot_model.initString(xml_string);

    chain = new BaxterChain(robot_model, _base_link, _tip_link);

    x_0.resize(3); x_0.setZero();
    x_t.resize(3); x_t.setZero();
    x_n.resize(3); x_n.setZero();
    x_d.resize(3); x_d.setZero();

    o_n.resize(4); o_n.setZero();

    std::string topic = "/" + getName() + "/" + getLimb() + "/ipopt";
    ctrl_sub      = _n.subscribe(topic, SUBSCRIBER_BUFFER, &CtrlThread::ctrlCb, this);
    ROS_INFO("[%s] Created cartesian controller that listens to : %s",
                                    getLimb().c_str(), topic.c_str());
}

void CtrlThread::ctrlCb(const baxter_collaboration_msgs::GoToPose& _msg)
{
    // x_d[0] = _msg.pose_stamp.pose.position.x;
    // x_d[1] = _msg.pose_stamp.pose.position.y;
    // x_d[2] = _msg.pose_stamp.pose.position.z;
    // o_n[0] = -0.128;
    // o_n[1] = 0.99;
    // o_n[2] = -0.018;
    // o_n[3] = 0.022;

    // int exit_code;
    // solveIK(exit_code);
}

bool CtrlThread::goToPoseNoCheck(double px, double py, double pz,
                                 double ox, double oy, double oz, double ow)
{
    x_n[0] = px;
    x_n[1] = py;
    x_n[2] = pz;
    o_n[0] = ox;
    o_n[1] = oy;
    o_n[2] = oz;
    o_n[3] = ow;

    int exit_code;
    Eigen::VectorXd joint_velocities_eigen = solveIK(exit_code);

    std::vector<double> joint_velocities_std;

    for (int i = 0; i < joint_velocities_eigen.col(0).size(); i++)
    {
        joint_velocities_std.push_back(joint_velocities_eigen[i]);
    }

    ROS_INFO_THROTTLE(0.25, "%g %g %g %g %g %g %g", joint_velocities_std[0], joint_velocities_std[1], joint_velocities_std[2], joint_velocities_std[3], joint_velocities_std[4], joint_velocities_std[5], joint_velocities_std[6]);

    // if (!goToJointConfNoCheck(joint_velocities_std)) return false;

    return true;

}

VectorXd CtrlThread::solveIK(int &_exit_code)
{
    VectorXd res(chain->getNrOfJoints()); res.setZero();

    if (!waitForJointAngles()) {
        return res;
    }

    chain->setAng(getJointStates());

    VectorXd xr(7);
    xr.block<3, 1>(0, 0) = x_n;
    xr.block<4, 1>(3, 0) = o_n;

    MatrixXd vLimAdapted;
    vLimAdapted.resize(chain->getNrOfJoints(), 2);
    for (size_t r = 0, DOF = chain->getNrOfJoints(); r < DOF; ++r)
    {
        vLimAdapted(r, 0) = -vMax;
        vLimAdapted(r, 1) = vMax;
    }
    q_dot.resize(chain->getNrOfJoints());
    q_dot.setZero();

    bool verbosity = false;
    // bool controlMode = true;
    bool hittingConstraints = false;
    bool orientationControl = false;

    Ipopt::SmartPtr<Ipopt::IpoptApplication> app=new Ipopt::IpoptApplication;
    app->Options()->SetNumericValue("tol",tol);
    app->Options()->SetNumericValue("constr_viol_tol",1e-6);
    app->Options()->SetIntegerValue("acceptable_iter",0);
    app->Options()->SetStringValue("mu_strategy","adaptive");
    app->Options()->SetIntegerValue("max_iter",std::numeric_limits<int>::max());
    app->Options()->SetNumericValue("max_cpu_time", dT);
    app->Options()->SetStringValue("nlp_scaling_method","gradient-based");
    app->Options()->SetStringValue("hessian_approximation","limited-memory");
    app->Options()->SetStringValue("derivative_test",verbosity?"first-order":"none");
    app->Options()->SetIntegerValue("print_level",verbosity?5:0);
    app->Initialize();

    Ipopt::SmartPtr<ControllerNLP> nlp;
    nlp=new ControllerNLP(*chain);
    nlp->set_hitting_constraints(hittingConstraints);
    nlp->set_orientation_control(orientationControl);
    nlp->set_dt(dT);
    nlp->set_xr(xr);
    nlp->set_v_limInDegPerSecond(vLimAdapted);
    nlp->set_v0InDegPerSecond(q_dot);
    nlp->init();

    _exit_code=app->OptimizeTNLP(GetRawPtr(nlp));

    res=nlp->get_resultInDegPerSecond();

    if(verbosity >= 1)
    {
        ROS_INFO("x_n: %g %g %g\tx_d: %g %g %g\tdT: %g",
                  x_n[0], x_n[1], x_n[2], x_d[0], x_d[1], x_d[2], dT);
        ROS_INFO("x_0: %g %g %g\tx_t: %g %g %g",
                  x_0[0], x_0[1], x_0[2], x_t[0], x_t[1], x_t[2]);
        ROS_INFO("norm(x_n-x_t): %g\tnorm(x_d-x_n): %g\tnorm(x_d-x_t): %g",
                    (x_n-x_t).norm(), (x_d-x_n).norm(), (x_d-x_t).norm());
        ROS_INFO("Result (solved velocities (deg/s)): %g %g %g %g %g %g %g",
                    res[0], res[1], res[2], res[3], res[4], res[5], res[6]);
    }

    return res;
}

CtrlThread::~CtrlThread()
{
    if (chain)
    {
        delete chain;
        chain = 0;
    }
}
