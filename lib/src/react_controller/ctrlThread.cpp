#include "react_controller/ctrlThread.h"
#include <iostream>

using namespace              std;
using namespace      sensor_msgs;
using namespace baxter_core_msgs;
using namespace            Eigen;

CtrlThread::CtrlThread(const std::string& _name, const std::string& _limb, bool _use_robot, double _ctrl_freq,
                       bool _is_debug, double _tol, double _vMax, bool _coll_av) :
                       RobotInterface(_name, _limb, _use_robot, _ctrl_freq, true, false, true, true), chain(0),
                       is_debug(_is_debug), internal_state(true), nlp_ctrl_ori(false), nlp_derivative_test("none"),
                       nlp_print_level(0), dT(1000.0/_ctrl_freq), tol(_tol), vMax(_vMax), coll_av(_coll_av)
{
    urdf::Model robot_model;
    std::string  xml_string;

    std::string urdf_xml,full_urdf_xml;
    nh.param<std::string>("urdf_xml",urdf_xml,"/robot_description");
    nh.searchParam(urdf_xml,full_urdf_xml);

    ROS_DEBUG("Reading xml file from parameter server");
    if (!nh.getParam(full_urdf_xml, xml_string))
    {
        ROS_FATAL("Could not load the xml from parameter server: %s", urdf_xml.c_str());
        return;
    }

    nh.param(full_urdf_xml,xml_string,std::string());
    robot_model.initString(xml_string);

    string base_link = "base";
    string  tip_link = getLimb()+"_gripper";

    chain = new BaxterChain(robot_model, base_link, tip_link);

    x_n.setZero();

    q_dot.resize(chain->getNrOfJoints());
    q_dot.setZero();

    vLim.resize(chain->getNrOfJoints(), 2);
    for (size_t r = 0, DoFs = chain->getNrOfJoints(); r < DoFs; ++r)
    {
        vLim(r, 0) = -vMax;
        vLim(r, 1) =  vMax;
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
    app->Options()->SetNumericValue(            "tol", tol    );
    app->Options()->SetNumericValue("constr_viol_tol", tol*100);
    app->Options()->SetNumericValue( "acceptable_tol", tol    );
    app->Options()->SetIntegerValue("acceptable_iter",      10);
    app->Options()->SetStringValue ( "mu_strategy", "adaptive");
    // if (is_debug == false) { app->Options()->SetStringValue ("linear_solver", "ma57"); }
    app->Options()->SetNumericValue("max_cpu_time", 0.95 * dT / 1000.0);
    // app->Options()->SetStringValue ("nlp_scaling_method","gradient-based");
    app->Options()->SetStringValue ("hessian_approximation","limited-memory");
}

void CtrlThread::NLPOptionsFromParameterServer()
{
    bool derivative_test = false;
    nh.param<bool>("ctrl_ori", nlp_ctrl_ori, false);
    nh.param<bool>("derivative_test", derivative_test, false);
    nh.param<int> ("print_level", nlp_print_level, 0);

    nlp_derivative_test = derivative_test?"first-order":"none";

    if (nlp_print_level > 0)
    {
        ROS_INFO("[NLP]         Print Level: %i", nlp_print_level);
        ROS_INFO("[NLP] Orientation Control: %s", nlp_ctrl_ori?"on":"off");
        ROS_INFO("[NLP]     Derivative Test: %s", nlp_derivative_test.c_str());
    }

    if (nlp_ctrl_ori == true) {     setCtrlType("pose"); }
    else                      { setCtrlType("position"); }

    app->Options()->SetStringValue ("derivative_test", nlp_derivative_test);
    app->Options()->SetIntegerValue(    "print_level",     nlp_print_level);
    app->Initialize();
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

bool CtrlThread::goToPoseNoCheck(double px, double py, double pz,
                                 double ox, double oy, double oz, double ow)
{
    x_n = Vector3d(px, py, pz);
    o_n = Quaterniond(ow, ox, oy, oz);

    if ((not is_debug) && waitForJointAngles(2.0))   { chain->setAng(getJointStates()); }

    // ROS_INFO("actual joint  pos: %s", toString(std::vector<double>(_q.position.data(),
    //                                             _q.position.data() + _q.position.size())).c_str());
    nlp = new ControllerNLP(*chain);

    int exit_code = -1;
    Eigen::VectorXd est_vels = solveIK(exit_code);
    q_dot = est_vels;

    std::vector<double> des_poss(chain->getNrOfJoints());
    for (size_t i = 0; i < chain->getNrOfJoints(); ++i)
    {
        des_poss[i] = chain->getAng(i) + (dT * est_vels[i]);
    }

    // ROS_INFO("sending joint position: %s", toString(des_poss).c_str());

    if (exit_code != 0 && exit_code != 4 && exit_code != -4) { return false; }
    if (is_debug)                                            { return  true; }

    if (isRobotUsed())
    {
        // suppressCollisionAv();
        if (goToJointConfNoCheck(des_poss))   { return true; }
    }

    return false;
}

VectorXd CtrlThread::solveIK(int &_exit_code)
{
    NLPOptionsFromParameterServer();

    if (coll_av)
    {
        std::vector<Eigen::Vector3d> positions;
        chain->GetJointPositions(positions);

        Eigen::Vector3d point(0.63, -0.17, 0.0);
        std::vector<collisionPoint> collisionPoints;
        computeCollisionPoints(positions, point, collisionPoints);
        AvoidanceHandler *avhdl;
        avhdl = new AvoidanceHandlerTactile(*chain, collisionPoints);
        vLimCollision = avhdl->getV_LIM(vLim);
        cout << vLimCollision << endl;
        nlp->set_v_lim(vLimCollision);
    }
    else
    {
        nlp->set_v_lim(vLim);
    }

    nlp->set_ctrl_ori(nlp_ctrl_ori);
    nlp->set_dt(dT);
    nlp->set_x_r(x_n, o_n);
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
