#include <robot_interface/robot_interface.h>
#include <react_controller/reactIpOpt.h>
#include <baxter_core_msgs/JointCommand.h>

class CtrlThread : public RobotInterface
{
private:
    std::string _urdf_param;
    KDL::JntArray _lb, _ub;
    BaxterChain _chain;
    ros::Subscriber _jntstate_sub;
    pthread_mutex_t _mutex_jnts;
    
public:
    CtrlThread(const std::string& base_link, const std::string& tip_link);
    void solveIK();
    void jointStatesCb(const sensor_msgs::JointState& msg);
};