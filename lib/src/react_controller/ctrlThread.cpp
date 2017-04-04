#include <react_controller/ctrlThread.h>

using namespace              std;
using namespace      sensor_msgs;
using namespace baxter_core_msgs;
using namespace            Eigen;

CtrlThread::CtrlThread(const std::string& _name, const std::string& _limb, bool _no_robot,
                       bool _is_debug, double _tol, double _vMax, double _dT) :
                       RobotInterface(_name, _limb, _no_robot, true, false, true, true),
                       is_debug(_is_debug), internal_state(true), tol(_tol), vMax(_vMax), dT(_dT)
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

    string base_link = "base";
    string  tip_link = getLimb()+"_gripper";

    chain = new BaxterChain(robot_model, base_link, tip_link);

    x_0.resize(3); x_0.setZero();
    x_t.resize(3); x_t.setZero();
    x_n.resize(3); x_n.setZero();
    x_d.resize(3); x_d.setZero();

    o_n.resize(3); o_n.setZero();

    q_dot.resize(chain->getNrOfJoints());
    q_dot.setZero();

    bool verbosity =  true;
    // bool ctrlOri = false;
    initializeApp(verbosity);
    // initializeNLP(ctrlOri);

    if (is_debug == true)
    {
        if (debugIPOPT()) ROS_INFO("Success! IPOPT works.");
        else              ROS_ERROR("IPOPT does not work!");

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

void CtrlThread::initializeApp(bool verbosity) {
    app=new Ipopt::IpoptApplication;
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
    app->Options()->SetIntegerValue("print_level",verbosity & !is_debug?5:0);
    app->Initialize();
}

void CtrlThread::initializeNLP(bool ctrlOri) {

    nlp=new ControllerNLP(*chain);
    nlp->set_ctrl_ori(ctrlOri);
    nlp->set_dt(dT);
    MatrixXd vLimAdapted(chain->getNrOfJoints(), 2);

    for (size_t r = 0, DoFs = chain->getNrOfJoints(); r < DoFs; ++r)
    {
        vLimAdapted(r, 0) = -vMax;
        vLimAdapted(r, 1) =  vMax;
    }

    nlp->set_v_lim(vLimAdapted);
}

bool CtrlThread::debugIPOPT()
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

    double offs_x =    0;
    double offs_y =    0;
    double offs_z =    0;
    int     range =    2;
    int   counter =    0;
    bool   result = true;

    // Let's do all the test together
    // The number of test performed is 2^range
    for (int i = 0; i < range; ++i)
    {
        offs_x = i * 0.001;

        for (int j = 0; j < range; ++j)
        {
            offs_y = j * 0.001;

            for (int k = 0; k < range; ++k)
            {
                offs_z = k * 0.001;
                result = goToPoseNoCheck(frame.p[0] + offs_x,
                                         frame.p[1] + offs_y,
                                         frame.p[2] + offs_z,
                                         ox, oy, oz, ow);
                if (result == false)
                {
                    ROS_ERROR("Test number %i , result %s", counter, result==true?"TRUE":"FALSE");
                }
                else
                {
                    ROS_WARN("Test number %i , result %s", counter, result==true?"TRUE":"FALSE");
                }

                ++counter;
                internal_state = internal_state & result;
            }
        }
    }

    return internal_state;
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

    if (exit_code != 0 && is_debug)        return false;
    if (exit_code != 0 && exit_code != -4) return false;
    if (is_debug)                          return  true;

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


    nlp=new ControllerNLP(*chain);
    nlp->set_ctrl_ori(false);
    nlp->set_dt(dT);
    MatrixXd vLimAdapted(chain->getNrOfJoints(), 2);

    for (size_t r = 0, DoFs = chain->getNrOfJoints(); r < DoFs; ++r)
    {
        vLimAdapted(r, 0) = -vMax;
        vLimAdapted(r, 1) =  vMax;
    }

    nlp->set_v_lim(vLimAdapted);
    nlp->set_xr(xr);
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
