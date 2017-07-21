#include "react_controller/ctrlThread.h"

using namespace   std;
using namespace Eigen;

CtrlThread::CtrlThread(const string& _name, const string& _limb, bool _use_robot, double _ctrl_freq,
                       bool _is_debug, bool _coll_av, double _tol, double _vMax) :
                       RobotInterface(_name, _limb, _use_robot, _ctrl_freq, true, false, true, true),
                       chain(0), is_debug(_is_debug), internal_state(true), ctrl_ori(false),
                       derivative_test(false), dT(1.0/_ctrl_freq),
                       tol(_tol), vMax(_vMax), coll_av(_coll_av)
{
    urdf::Model robot_model;
    string       xml_string;
    string         urdf_xml;
    string    full_urdf_xml;

    nh.param<string>("urdf_xml", urdf_xml, "/robot_description");
    nh.searchParam  ( urdf_xml , full_urdf_xml);

    ROS_DEBUG("Reading xml file from parameter server");
    if (!nh.getParam(full_urdf_xml, xml_string))
    {
        ROS_FATAL("Could not load the xml from parameter server: %s", urdf_xml.c_str());
        return;
    }

    nh.param(full_urdf_xml,xml_string,string());
    robot_model.initString(xml_string);

    string base_link = "base";
    string  tip_link = getLimb()+"_gripper";

    chain = new BaxterChain(robot_model, base_link, tip_link);

    x_n.setZero();

    q_dot.resize(chain->getNrOfJoints());
    q_dot.setZero();

    vLim.resize(chain->getNrOfJoints(), 2);
    for (size_t r = 0; r < chain->getNrOfJoints(); ++r)
    {
        // Let's find the most conservative choice between the limits from URDF
        // and the one in ctrlThread.
        double lim = std::min(vMax, RAD2DEG*chain->getVLim(r));
        vLim(r, 0) = -lim;
        vLim(r, 1) =  lim;
    }

    initializeNLP();

    if (waitForJointAngles(2.0))   { chain->setAng(getJointStates()); }
    else                           {              setUseRobot(false); }

    if (isRobotUsed()) { ROS_INFO("Curr pose: %s", toString(RobotInterface::getPose()).c_str()); }
    else               { ROS_INFO("Curr pose: %s", toString(         chain->getPose()).c_str()); }

    if (is_debug == true)
    {
        if (debugIPOPT()) { ROS_INFO("Success! IPOPT works."); }
        else              { ROS_ERROR("IPOPT does not work!"); }
    }
}

void CtrlThread::initializeNLP()
{
    app=new Ipopt::IpoptApplication;
    app->Options()->SetNumericValue(            "tol", tol);
    app->Options()->SetNumericValue("constr_viol_tol", tol);
    app->Options()->SetNumericValue( "acceptable_tol", tol);
    app->Options()->SetIntegerValue("acceptable_iter",  10);
    app->Options()->SetStringValue ( "mu_strategy", "adaptive");
    // if (is_debug == false) { app->Options()->SetStringValue ("linear_solver", "ma57"); }
    app->Options()->SetNumericValue("max_cpu_time", 0.95 * dT);
    // app->Options()->SetStringValue ("nlp_scaling_method","gradient-based");
    app->Options()->SetStringValue ("hessian_approximation","limited-memory");
}

void CtrlThread::NLPOptionsFromParameterServer()
{
    nh.param<bool>("ctrl_ori", ctrl_ori, false);
    nh.param<bool>("derivative_test", derivative_test, false);
    nh.param<int> ("print_level", print_level, 0);

    if (print_level >= 3)
    {
        ROS_INFO("[NLP]                  dT: %g", dT);
        ROS_INFO("[NLP]         Print Level: %i", print_level);
        ROS_INFO("[NLP] Orientation Control: %s", ctrl_ori?"on":"off");
        ROS_INFO("[NLP]     Derivative Test: %s", derivative_test?"first-order":"none");
    }

    setCtrlType(ctrl_ori?"pose":"position");

    app->Options()->SetStringValue ("derivative_test", derivative_test?"first-order":"none");
    app->Options()->SetIntegerValue(    "print_level",     print_level);
    app->Initialize();

    // Read the obstacles from the parameter server
    XmlRpc::XmlRpcValue obstacles_db;
    if(nh.getParam("/"+getName()+"/obstacles", obstacles_db))
    {
        obstacles = readFromParamServer(obstacles_db);
    }
}

bool CtrlThread::goToPoseNoCheck(double px, double py, double pz,
                                 double ox, double oy, double oz, double ow)
{
    x_n = Vector3d(px, py, pz);
    o_n = Quaterniond(ow, ox, oy, oz);

    if ((not is_debug) && waitForJointAngles(2.0))   { chain->setAng(getJointStates()); }

    // ROS_INFO("actual joint  pos: %s", toString(vector<double>(_q.position.data(),
    //                                _q.position.data() + _q.position.size())).c_str());
    nlp = new ControllerNLP(*chain);

    // Solve the task
    int exit_code = -1;
    VectorXd est = solveIK(exit_code);

    publishRVIZMarkers();

    if (exit_code != 0)    { ROS_WARN("Exit code: %i", exit_code); }

    // if (exit_code != 0 && exit_code != 4 && exit_code != -4) { return false; }
    if (is_debug) { return true; }

    // if (exit_code == -4)    // Maximum CPU time exceeded
    // {
    //     if (print_level >= 1)   { printf("\n"); }

    //     return  false;
    // }

    if (exit_code ==  2)    // Local infeasibility
    {
        if (print_level >= 1)   { printf("\n"); }

        return  false;
    }

    // if (exit_code == 0)
    {
        q_dot = nlp->get_est_vels();
    }

    if (isRobotUsed() && not nlp->get_int_status())
    {
        // suppressCollisionAv();

        ROS_INFO_STREAM(" send:  " << est.transpose() <<
                        " mode:  " <<   getCtrlMode() << endl);
        if (goToJointConfNoCheck(est))     { return true; }
    }

    return true;
}

VectorXd CtrlThread::solveIK(int &_exit_code)
{
    NLPOptionsFromParameterServer();

    nlp->set_print_level(size_t(print_level));

    if (coll_av)
    {
        avhdl = std::make_unique<AvoidanceHandlerTactile>(*chain, obstacles, print_level);
        vlim_coll = avhdl->getV_LIM(DEG2RAD * vLim) * RAD2DEG;

        nlp->set_v_lim(vlim_coll);
    }
    else
    {
        nlp->set_v_lim(vLim);
    }

    nlp->set_ctrl_ori(ctrl_ori);
    nlp->set_dt(dT);
    nlp->set_x_r(x_n, o_n);
    nlp->set_v_0(q_dot);
    // nlp->set_v_0(chain->getVel());
    nlp->init();

    _exit_code=app->OptimizeTNLP(GetRawPtr(nlp));

    if (getCtrlMode() == human_robot_collaboration_msgs::GoToPose::VELOCITY_MODE)
    {
        return nlp->get_est_vels();
    }
    else
    {
        return nlp->get_est_conf();
    }
}

void CtrlThread::publishRVIZMarkers()
{
    vector <RVIZMarker> rviz_markers;
    for (size_t i = 0; i < obstacles.size(); ++i)
    {
        rviz_markers.push_back(RVIZMarker(obstacles[i].x_wrf,
                                          ColorRGBA(1.0, 0.0, 1.0),
                                          obstacles[i].size,
                                          visualization_msgs::Marker::SPHERE));
    }

    vector <RVIZMarker> rviz_chain = asRVIZMarkers(*chain);
    rviz_markers.insert(std::end(rviz_markers),
                        std::begin(rviz_chain), std::end(rviz_chain));

    if (coll_av)
    {
        vector <RVIZMarker> rviz_coll = avhdl->toRVIZMarkers();

        rviz_markers.insert(std::end(rviz_markers),
                            std::begin( rviz_coll), std::end(rviz_coll));
    }

    // Let's publish the set of markers to RVIZ
    rviz_pub.setMarkers(rviz_markers);
}

bool CtrlThread::debugIPOPT()
{
    if (isRobotNotUsed())
    {
        VectorXd q(7);
        if (getLimb() == "left") { q << -0.08, -1.0, -1.19, 1.94,  0.67, 1.03, -0.50; }
        else                     { q <<  0.08, -1.0,  1.19, 1.94, -0.67, 1.03,  0.50; }

        chain->setAng(q);
        ROS_INFO("New  pose: %s", toString(chain->getPose()).c_str());
    }

    Matrix4d    H_ee(chain->getH());
    Matrix3d    R_ee= H_ee.block<3,3>(0,0);
    Vector3d    p_ee= H_ee.block<3,1>(0,3);
    Quaterniond o_ee(R_ee);
    o_ee.normalize();

    // internal_state = goToPoseNoCheck(p_ee[0], p_ee[1], p_ee[2] + 0.02,
    //                                  o_ee.x(), o_ee.y(), o_ee.z(), o_ee.w());
    // return internal_state;

    double    offs_x =     0;
    double    offs_y =     0;
    double    offs_z =     0;
    int      counter =     0;
    bool      result =  true;
    int   n_failures =     0;

    vector<double> increment{0.001, 0.004};

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

                    result = goToPoseNoCheck(p_ee[0] + offs_x,
                                             p_ee[1] + offs_y,
                                             p_ee[2] + offs_z,
                                             o_ee.x(), o_ee.y(), o_ee.z(), o_ee.w());
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

CtrlThread::~CtrlThread()
{
    if (chain)
    {
        delete chain;
        chain = 0;
    }
}
