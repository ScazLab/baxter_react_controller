#include <react_controller/ctrlThread.h>

using namespace sensor_msgs;
using namespace baxter_core_msgs;
using namespace Eigen;

CtrlThread::CtrlThread(const std::string& _base_link, const std::string& _tip_link) :
                                  RobotInterface("baxter_react_controller", "left")
{
    ros::NodeHandle node_handle("~");

    urdf::Model robot_model;
    std::string xml_string;

    std::string urdf_xml,full_urdf_xml;
    node_handle.param<std::string>("urdf_xml",urdf_xml,"/robot_description");
    node_handle.searchParam(urdf_xml,full_urdf_xml);

    ROS_DEBUG_NAMED("trac_ik","Reading xml file from parameter server");
    if (!node_handle.getParam(full_urdf_xml, xml_string))
    {
        ROS_FATAL_NAMED("trac_ik","Could not load the xml from parameter server: %s", urdf_xml.c_str());
        return;
    }

    node_handle.param(full_urdf_xml,xml_string,std::string());
    robot_model.initString(xml_string);

    ROS_DEBUG_STREAM_NAMED("trac_ik","Reading joints and links from URDF");

    KDL::Tree tree;

    if (!kdl_parser::treeFromUrdfModel(robot_model, tree))
      ROS_FATAL("Failed to extract kdl tree from xml robot description");

    KDL::Chain kdl_chain;

    if(!tree.getChain(_base_link, _tip_link, kdl_chain))
      ROS_FATAL("Couldn't find chain %s to %s",_base_link.c_str(),_tip_link.c_str());

    std::vector<KDL::Segment> kdl_chain_segs = kdl_chain.segments;

    boost::shared_ptr<const urdf::Joint> joint;

    std::vector<double> l_bounds, u_bounds;

    lb.resize(kdl_chain.getNrOfJoints());
    ub.resize(kdl_chain.getNrOfJoints());

    uint joint_num=0;
    for(unsigned int i = 0; i < kdl_chain_segs.size(); ++i)
    {
        joint = robot_model.getJoint(kdl_chain_segs[i].getJoint().getName());

        if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
        {
            joint_num++;
            float lower, upper;
            int hasLimits = 0;

            if ( joint->type != urdf::Joint::CONTINUOUS )
            {
                if(joint->safety)
                {
                    lower = std::max(joint->limits->lower, joint->safety->soft_lower_limit);
                    upper = std::min(joint->limits->upper, joint->safety->soft_upper_limit);
                }
                else
                {
                    lower = joint->limits->lower;
                    upper = joint->limits->upper;
                }

                hasLimits = 1;
            }

            if(hasLimits)
            {
                lb(joint_num-1)=lower;
                ub(joint_num-1)=upper;
            }
            else
            {
                lb(joint_num-1)=std::numeric_limits<float>::lowest();
                ub(joint_num-1)=std::numeric_limits<float>::max();
            }

            ROS_INFO_STREAM("IK Using joint "<<joint->name<<" "<<lb(joint_num-1)<<" "<<ub(joint_num-1));
        }
    }

    chain = new BaxterChain(kdl_chain);

    x_0.resize(3); x_0.setZero();
    x_t.resize(3); x_t.setZero();
    x_n.resize(3); x_n.setZero();
    x_d.resize(3); x_d.setZero();

    o_n.resize(3); o_n.setZero();
    // TODO REsize also the other orientations?

    int exit_code;
    solveIK(exit_code);
}

VectorXd CtrlThread::solveIK(int &_exit_code)
{
    double tol  =  1e-6;
    double vMax =  45.0;
    double dT   = 0.001;
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
    VectorXd res(chain->getNrOfJoints()); res.setZero();

    bool verbosity = true;
    // bool controlMode = true;
    bool hittingConstraints = false;
    bool orientationControl = true;

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
    nlp=new ControllerNLP(*chain, lb, ub);
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
        ROS_INFO("x_n: %g %g %g\tx_d: %g %g %g\tdT: %g", x_n[0], x_n[1], x_n[2], x_d[0], x_d[1], x_d[2], dT);
        ROS_INFO("x_0: %g %g %g\tx_t: %g %g %g", x_0[0], x_0[1], x_0[2], x_t[0], x_t[1], x_t[2]);
        ROS_INFO("norm(x_n-x_t): %g\tnorm(x_d-x_n): %g\tnorm(x_d-x_t): %g",
                    (x_n-x_t).norm(), (x_d-x_n).norm(), (x_d-x_t).norm());
        ROS_INFO("Result (solved velocities (deg/s)): %g %g %g %g %g %g %g",res[0], res[1], res[2], res[3], res[4], res[5], res[6]);
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
