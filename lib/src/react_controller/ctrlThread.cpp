#include <react_controller/ctrlThread.h>

using namespace              std;
using namespace      sensor_msgs;
using namespace baxter_core_msgs;
using namespace            Eigen;

CtrlThread::CtrlThread(const std::string& _name, const std::string& _limb, bool _no_robot,
                       const std::string& _base_link, const std::string& _tip_link, double _tol,
                       double _vMax, double _dT) : RobotInterface(_name, _limb, _no_robot),
                       tol(_tol), vMax(_vMax), dT(_dT)
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

    o_n.resize(3); o_n.setZero();
}

bool CtrlThread::goToPoseNoCheck(double px, double py, double pz,
                                 double ox, double oy, double oz, double ow)
{
    x_n[0] = px;
    x_n[1] = py;
    x_n[2] = pz;

    tf::Quaternion q(ox, oy, oz, ow);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    o_n[0] = roll;
    o_n[1] = pitch;
    o_n[2] = yaw;

    int exit_code = -1;
    Eigen::VectorXd joint_velocities_eigen = solveIK(exit_code);

    KDL::JntArray jnts(chain->getNrOfJoints());
    VectorXd angles = chain->getAng();

    for (size_t i = 0, _i = chain->getNrOfJoints(); i < _i; ++i)
    {
        jnts(i) = angles[i] + (dT * joint_velocities_eigen(i));
    }

    KDL::Frame frame;
    chain->JntToCart(jnts,frame);

    sensor_msgs::JointState cj = getJointStates();

    ROS_INFO("x_c: %g %g %g", getPos().x, getPos().y, getPos().z);
    ROS_INFO("x_n: %g %g %g", px, py, pz);
    ROS_INFO("x_f: %g %g %g\n", frame.p[0], frame.p[1], frame.p[2]);

    ROS_INFO("joint_angles    getAng: %g %g %g %g %g %g %g", angles[0], angles[1], angles[2], angles[3], angles[4], angles[5], angles[6]);
    ROS_INFO("joint_angles JntStates: %g %g %g %g %g %g %g", cj.position[0], cj.position[1], cj.position[2], cj.position[3], cj.position[4], cj.position[5], cj.position[6]);
    ROS_INFO("joint       velocities: %g %g %g %g %g %g %g", joint_velocities_eigen(0), joint_velocities_eigen(1), joint_velocities_eigen(2), joint_velocities_eigen(3), joint_velocities_eigen(4), joint_velocities_eigen(5), joint_velocities_eigen(6));
    ROS_INFO("joint_angles nextState: %g %g %g %g %g %g %g\n", jnts(0), jnts(1), jnts(2), jnts(3), jnts(4), jnts(5), jnts(6));

    if (exit_code != 0) return false;

    std::vector<double> joint_velocities_std;

    for (int i = 0; i < joint_velocities_eigen.col(0).size(); i++)
    {
        joint_velocities_std.push_back(joint_velocities_eigen[i]);
    }

    if (!goToJointConfNoCheck(joint_velocities_std)) return false;

    return true;

}

VectorXd CtrlThread::solveIK(int &_exit_code)
{
    VectorXd res(chain->getNrOfJoints()); res.setZero();

    if (!waitForJointAngles()) {
        return res;
    }

    chain->setAng(getJointStates());

    VectorXd xr(6);
    xr.block<3, 1>(0, 0) = x_n;
    xr.block<3, 1>(3, 0) = o_n;

    MatrixXd vLimAdapted;
    vLimAdapted.resize(chain->getNrOfJoints(), 2);
    for (size_t r = 0, DOF = chain->getNrOfJoints(); r < DOF; ++r)
    {
        vLimAdapted(r, 0) = -vMax;
        vLimAdapted(r, 1) = vMax;
    }
    q_dot.resize(chain->getNrOfJoints());
    q_dot.setZero();

    bool verbosity = true;
    // bool controlMode = true;
    bool ctrlOri = false;

    Ipopt::SmartPtr<Ipopt::IpoptApplication> app=new Ipopt::IpoptApplication;
    app->Options()->SetNumericValue("tol",tol);
    app->Options()->SetNumericValue("constr_viol_tol",1e-6);
    app->Options()->SetIntegerValue("acceptable_iter",0);
    app->Options()->SetStringValue ("mu_strategy","adaptive");
    app->Options()->SetIntegerValue("max_iter",std::numeric_limits<int>::max());
    app->Options()->SetNumericValue("max_cpu_time", 0.95 * dT);
    app->Options()->SetStringValue ("nlp_scaling_method","gradient-based");
    app->Options()->SetStringValue ("hessian_approximation","limited-memory");
    app->Options()->SetStringValue ("derivative_test",verbosity?"first-order":"none");
    app->Options()->SetIntegerValue("print_level",verbosity?5:0);
    app->Initialize();

    Ipopt::SmartPtr<ControllerNLP> nlp;
    nlp=new ControllerNLP(*chain);
    nlp->set_ctrl_ori(ctrlOri);
    nlp->set_dt(dT);
    nlp->set_xr(xr);
    nlp->set_v_lim(vLimAdapted);
    nlp->set_v_0(q_dot);
    nlp->init();

    _exit_code=app->OptimizeTNLP(GetRawPtr(nlp));

    res=CTRL_RAD2DEG * nlp->get_result();

    if(verbosity)
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
