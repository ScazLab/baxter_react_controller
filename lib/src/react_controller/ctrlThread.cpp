#include <react_controller/ctrlThread.h>

using namespace sensor_msgs;
using namespace baxter_core_msgs;

CtrlThread::CtrlThread(const std::string& base_link, const std::string& tip_link) :
        RobotInterface("test", "test")
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

    if(!tree.getChain(base_link, tip_link, _chain))
      ROS_FATAL("Couldn't find chain %s to %s",base_link.c_str(),tip_link.c_str());

    std::vector<KDL::Segment> chain_segs = _chain.segments;

    boost::shared_ptr<const urdf::Joint> joint;

    std::vector<double> l_bounds, u_bounds;

    _lb.resize(_chain.getNrOfJoints());
    _ub.resize(_chain.getNrOfJoints());

    uint joint_num=0;
    for(unsigned int i = 0; i < chain_segs.size(); ++i) {
      joint = robot_model.getJoint(chain_segs[i].getJoint().getName());
      if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {
        joint_num++;
        float lower, upper;
        int hasLimits;
        if ( joint->type != urdf::Joint::CONTINUOUS ) {
          if(joint->safety) {
            lower = std::max(joint->limits->lower, joint->safety->soft_lower_limit);
            upper = std::min(joint->limits->upper, joint->safety->soft_upper_limit);
          } else {
            lower = joint->limits->lower;
            upper = joint->limits->upper;
          }
          hasLimits = 1;
        }
        else {
          hasLimits = 0;
        }
        if(hasLimits) {
          _lb(joint_num-1)=lower;
          _ub(joint_num-1)=upper;
        }
        else {
          _lb(joint_num-1)=std::numeric_limits<float>::lowest();
          _ub(joint_num-1)=std::numeric_limits<float>::max();
        }
        ROS_INFO_STREAM("IK Using joint "<<joint->name<<" "<<_lb(joint_num-1)<<" "<<_ub(joint_num-1));
      }
    }

    _jntstate_sub  = node_handle.subscribe("/robot/joint_states",
                                    SUBSCRIBER_BUFFER, &CtrlThread::jointStatesCb, this);
}

void CtrlThread::solveIK()
{
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
    // nlp=new ControllerNLP(_chain, _lb, _ub);
    // nlp->set_hitting_constraints(hittingConstraints);
    // nlp->set_orientation_control(orientationControl);
    // nlp->set_dt(dT);
    // nlp->set_xr(xr);
    // nlp->set_v_limInDegPerSecond(vLimAdapted);
    // nlp->set_v0InDegPerSecond(q_dot);
    // nlp->init();

    // _exit_code=app->OptimizeTNLP(GetRawPtr(nlp));

    // res=nlp->get_resultInDegPerSecond();
    ROS_DEBUG("HELLO");
}

void CtrlThread::jointStatesCb(const sensor_msgs::JointState& msg)
{
    JointCommand joint_cmd;

    if (msg.name.size() >= joint_cmd.names.size())
    {
        pthread_mutex_lock(&_mutex_jnts);
        std::vector<double> angles;
        for (size_t i = 0; i < joint_cmd.names.size(); ++i)
        {
            for (size_t j = 0; j < msg.name.size(); ++j)
            {
                if (joint_cmd.names[i] == msg.name[j])
                {
                    angles.push_back(msg.position[j]);
                }
            }
        }
        _chain.setAng(angles);
        pthread_mutex_unlock(&_mutex_jnts);
    }

    return;
}
