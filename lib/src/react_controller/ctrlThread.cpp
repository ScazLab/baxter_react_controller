#include <react_controller/ctrlThread.h>

using namespace              std;
using namespace      sensor_msgs;
using namespace baxter_core_msgs;
using namespace            Eigen;

CtrlThread::CtrlThread(const std::string& _name, const std::string& _limb, bool _no_robot,
                       const std::string& _base_link, const std::string& _tip_link, bool _is_debug,
                       double _tol, double _vMax, double _dT) :
                       RobotInterface(_name, _limb, _no_robot, true, false, true, true),
                       is_debug(_is_debug), tol(_tol), vMax(_vMax), dT(_dT)
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

    q_dot.resize(chain->getNrOfJoints());
    q_dot.setZero();

    if (is_debug == true)
    {
        if (goToPoseNoCheck()) ROS_INFO("Success! IPOPT works.");
        else                   ROS_ERROR("IPOPT does not work!");

        if (chain)
        {
            delete chain;
            chain = 0;
        }
    }

    if (!noRobot())
    {
        waitForJointAngles();
        chain->setAng(getJointStates());

        ROS_INFO("Current Pose: %s", toString(getPose()).c_str());
    }
}

bool CtrlThread::goToPoseNoCheck()
{
    if (!noRobot())
    {
        if (!waitForJointAngles())
        {
            return false;
        }

        chain->setAng(getJointStates());
    }

    KDL::JntArray jnts(chain->getNrOfJoints());
    VectorXd angles = chain->getAng();

    for (size_t i = 0, _i = chain->getNrOfJoints(); i < _i; ++i)
    {
        jnts(i) = angles[i];
    }

    KDL::Frame frame;
    chain->JntToCart(jnts,frame);

    double ox, oy, oz, ow;
    frame.M.GetQuaternion(ox, oy, oz, ow);

    return goToPoseNoCheck(frame.p[0], frame.p[1], frame.p[2]+0.025, ox, oy, oz, ow);
    // return goToPoseNoCheck(frame.p[0], frame.p[1], frame.p[2], ox, oy, oz, ow);
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

    if (!noRobot())
    {
        if (!waitForJointAngles())
        {
            return false;
        }

        chain->setAng(getJointStates());
    }

    int exit_code = -1;
    Eigen::VectorXd est_vels = solveIK(exit_code);
    q_dot = est_vels;

    // KDL::JntArray jnts(chain->getNrOfJoints());
    // VectorXd angles = chain->getAng();

    // for (size_t i = 0, _i = chain->getNrOfJoints(); i < _i; ++i)
    // {
    //     jnts(i) = angles[i] + (dT * est_vels(i));
    // }

    // KDL::Frame frame;
    // chain->JntToCart(jnts,frame);

    // sensor_msgs::JointState curr_jnts = getJointStates();

    // ROS_INFO("current  position: [%g, %g, %g]", getPos().x, getPos().y, getPos().z);
    // ROS_INFO("desired  position: [%g, %g, %g]", px, py, pz);
    // ROS_INFO("computed position: [%g, %g, %g]", frame.p[0], frame.p[1], frame.p[2]);

    // ROS_INFO("initial joint state   [rad]: %s  ", toString(std::vector<double>(angles.data(),
    //                                                  angles.data() + angles.size())).c_str());
    // ROS_INFO("computed joint vels [rad/s]: %s  ", toString(std::vector<double>(est_vels.data(),
    //                                                est_vels.data() + est_vels.size())).c_str());
    // ROS_INFO("computed next state   [rad]: %s\n", toString(std::vector<double>(jnts.data.data(),
    //                                               jnts.data.data() + jnts.data.size())).c_str());

    // if (exit_code != 0 && exit_code != -4)  return false;
    if (exit_code != 0)                     return false;
    if (is_debug)                           return  true;

    // VectorXd est_vels(chain->getNrOfJoints());
    // est_vels.setZero();
    // est_vels[5] = 0.1;

    if (!goToJointConfNoCheck(std::vector<double>(est_vels.data(), est_vels.data() + est_vels.size()))) return false;

    return true;

}

VectorXd CtrlThread::solveIK(int &_exit_code)
{
    size_t DoFs = chain->getNrOfJoints();
    VectorXd res(DoFs); res.setZero();

    VectorXd xr(6);
    xr.block<3, 1>(0, 0) = x_n;
    xr.block<3, 1>(3, 0) = o_n;

    MatrixXd vLimAdapted(DoFs, 2);
    for (size_t r = 0; r < DoFs; ++r)
    {
        vLimAdapted(r, 0) = -vMax;
        vLimAdapted(r, 1) =  vMax;
    }

    bool verbosity =  true;
    bool ctrlOri   = false;

    Ipopt::SmartPtr<Ipopt::IpoptApplication> app=new Ipopt::IpoptApplication;
    app->Options()->SetNumericValue("tol",tol);
    app->Options()->SetNumericValue("constr_viol_tol",1e-6);
    // app->Options()->SetIntegerValue("acceptable_iter",0);
    app->Options()->SetStringValue ("mu_strategy","adaptive");
    app->Options()->SetStringValue ("linear_solver", "ma57");
    app->Options()->SetIntegerValue("max_iter",std::numeric_limits<int>::max());
    app->Options()->SetNumericValue("max_cpu_time", 0.95 * dT);
    // app->Options()->SetStringValue ("nlp_scaling_method","gradient-based");
    app->Options()->SetStringValue ("hessian_approximation","limited-memory");
    // app->Options()->SetStringValue ("derivative_test",verbosity?"first-order":"none");
    app->Options()->SetStringValue ("derivative_test","none");
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

    return nlp->get_result();
}

CtrlThread::~CtrlThread()
{
    if (chain)
    {
        delete chain;
        chain = 0;
    }
}
