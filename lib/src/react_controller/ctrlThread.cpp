#include <react_controller/ctrlThread.h>

using namespace              std;
using namespace      sensor_msgs;
using namespace baxter_core_msgs;
using namespace            Eigen;

CtrlThread::CtrlThread(const std::string& _name, const std::string& _limb, bool _no_robot, double _ctrl_freq,
                       bool _is_debug, double _tol, double _vMax) :
                       RobotInterface(_name, _limb, _no_robot, _ctrl_freq, true, false, true, true), chain(0),
                       is_debug(_is_debug), internal_state(true), dT(1000.0/_ctrl_freq), tol(_tol), vMax(_vMax)
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

    vLimAdapted.resize(chain->getNrOfJoints(), 2);
    for (size_t r = 0, DoFs = chain->getNrOfJoints(); r < DoFs; ++r)
    {
        vLimAdapted(r, 0) = -vMax;
        vLimAdapted(r, 1) =  vMax;
    }

    initializeApp(false);

    if (is_debug == true)
    {
        if (debugIPOPT()) ROS_INFO("Success! IPOPT works.");
        else              ROS_ERROR("IPOPT does not work!");
    }

    std::vector<double> positions_init;

    if (!isNoRobot())
    {
        waitForJointAngles();
        chain->setAng(getJointStates());

        for (size_t i = 0; i < chain->getNrOfJoints(); ++i)
        {
            positions_init.push_back(chain->getAng(i));
        }

        ROS_INFO("Current Pose: %s", toString(getPose()).c_str());
    }

    std::vector<Eigen::Vector3d> positions;
    chain->GetJointPositions(positions);

    Eigen::Vector3d point(0.40, -0.25, 0.45);
    std::vector<collisionPoint> collisionPoints;
    computeCollisionPoints(positions, point, collisionPoints);
    AvoidanceHandlerAbstract *avhdl;
    avhdl = new AvoidanceHandlerTactile(*chain, collisionPoints, false);
    cout << vLimAdapted << "\n";
    vLimAdapted = avhdl->getVLIM(vLimAdapted);

    cout << vLimAdapted << "\n";

    for (size_t i = 0; i < positions.size(); ++i)
    {
        ROS_INFO("joint %zu at x: %g y: %g z: %g", i, positions[i](0), positions[i](1), positions[i](2));
    }

    if (is_debug == true)
    {
        if (chain)
        {
            delete chain;
            chain = 0;
        }
    }
}

void CtrlThread::initializeApp(bool _verbosity)
{
    app=new Ipopt::IpoptApplication;
    app->Options()->SetNumericValue("tol", tol/10);
    app->Options()->SetNumericValue("constr_viol_tol", tol);
    app->Options()->SetNumericValue("acceptable_tol", tol/10);
    app->Options()->SetIntegerValue("acceptable_iter",0);
    app->Options()->SetStringValue ("mu_strategy","adaptive");
    if (is_debug == false) { app->Options()->SetStringValue ("linear_solver", "ma57"); }
    app->Options()->SetNumericValue("max_cpu_time", 0.95 * dT);
    // app->Options()->SetStringValue ("nlp_scaling_method","gradient-based");
    app->Options()->SetStringValue ("hessian_approximation","limited-memory");
    // app->Options()->SetStringValue ("derivative_test",verbosity?"first-order":"none");
    app->Options()->SetStringValue ("derivative_test","none");
    app->Options()->SetIntegerValue("print_level",(_verbosity && !is_debug)?4:0);
    app->Initialize();
}

bool CtrlThread::debugIPOPT()
{
    if (!isNoRobot())
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

    double    offs_x =     0;
    double    offs_y =     0;
    double    offs_z =     0;
    int      counter =     0;
    bool      result =  true;
    int   n_failures =     0;

    std::vector<double> increment{0.001, 0.004};

    // Let's do all the test together
    // The number of test performed is 2^2^increment.size()

    for (int i = -1; i < 2; ++i) // -1, 0, +1
    {
        for (int j = -1; j < 2; ++j) // -1, 0, +1
        {
            for (int k = -1; k < 2; ++k) // -1, 0, +1
            {
                for (size_t p = 0; p < increment.size(); ++p)
                {
                    offs_x = i * increment[p];
                    offs_y = j * increment[p];
                    offs_z = k * increment[p];

                    result = goToPoseNoCheck(frame.p[0] + offs_x,
                                             frame.p[1] + offs_y,
                                             frame.p[2] + offs_z,
                                             ox, oy, oz, ow);
                    q_dot.setZero(); // in simulation, we always start from 0 velocities

                    if (result == false)
                    {
                        ROS_ERROR("[%s] Test number %i , dT %g, offset [%g %g %g], result %s",
                                   getLimb().c_str(), counter, dT, offs_x, offs_y, offs_z,
                                   result==true?"TRUE":"FALSE");
                        n_failures++;
                    }
                    else
                    {
                        ROS_WARN("[%s] Test number %i , dT %g, offset [%g %g %g], result %s",
                                  getLimb().c_str(), counter, dT, offs_x, offs_y, offs_z,
                                  result==true?"TRUE":"FALSE");
                    }

                    ++counter;
                    internal_state = internal_state & result;
                }
            }
        }
    }

    if (n_failures)
    {
        ROS_ERROR("[%s] Number of failures: %i", getLimb().c_str(), n_failures);
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

    if (!isNoRobot())
    {
        if (!waitForJointAngles())
        {
            return false;
        }

        chain->setAng(getJointStates());
    }

    int exit_code = -1;

    // ROS_INFO("actual joint  pos: %s", toString(std::vector<double>(_q.position.data(),
    //                                             _q.position.data() + _q.position.size())).c_str());

    Eigen::VectorXd est_vels = solveIK(exit_code);

    std::vector<double> des_poss(chain->getNrOfJoints());

    for (size_t i = 0, _i = chain->getNrOfJoints(); i < _i; ++i)
    {
        des_poss[i] = chain->getAng(i) + (dT * est_vels[i]);
    }

    // q_dot = est_vels;

    ROS_INFO("sending joint position: %s", toString(des_poss).c_str());

    if (exit_code != 0 && is_debug)        return false;
    if (exit_code == 4 && is_debug)        return false;
    if (exit_code != 0 && exit_code != -4) return false;
    if (is_debug)                          return  true;

    // suppressCollisionAv();
    if (!goToJointConfNoCheck(des_poss)) return false;

    return true;

}

VectorXd CtrlThread::solveIK(int &_exit_code)
{
    size_t DoFs = chain->getNrOfJoints();
    VectorXd res(DoFs); res.setZero();

    VectorXd xr(6);
    xr.block<3, 1>(0, 0) = x_n;
    xr.block<3, 1>(3, 0) = o_n;

    Ipopt::SmartPtr<ControllerNLP> nlp = new ControllerNLP(*chain);
    nlp->set_ctrl_ori(false);
    nlp->set_dt(dT);
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
