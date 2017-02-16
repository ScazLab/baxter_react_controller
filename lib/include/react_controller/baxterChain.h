#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <urdf/model.h>

// Things to remember:
//  1 allList == quickList
//  2 N == DOF
//
//  3 HN == Identity matrix (for now) -> every time there is something
//                                       multiplied by HN, just discard it
//  3 H0 == Identity matrix (for now) -> every time there is something
//                                       multiplied by HN, just discard it
//
// New rule:
// 1 input parameters for functions are prefixed with a _ (e.g. _q_0),
// 2 members of the class do not have a _ (e.g. q)
//
// New rule:
// please put the bracket for a beginning of a function in a new line.
// I know that is a matter of personal choices, but this is what I use
// in my code and you would need to do it anyway when we'll integrate
// with my code

Eigen::Matrix4d KDLFrameToEigen(KDL::Frame _f);

/****************************************************************/
class BaxterChain : public KDL::Chain
{
private:
    std::vector<double> q;
    KDL::JntArray lb, ub;

public:
    // TODO documentation
    BaxterChain(urdf::Model _robot_model,
           const std::string& _base_link,
           const std::string&  _tip_link);

    BaxterChain(urdf::Model _robot_model,
           const std::string& _base_link,
           const std::string&  _tip_link,
                std::vector<double> _q_0);

    void initChain(urdf::Model _robot_model,
              const std::string& _base_link,
               const std::string& _tip_link);

    Eigen::MatrixXd GeoJacobian();
    Eigen::MatrixXd GeoJacobian(const unsigned int _i);

    bool     setAng(std::vector<double> _q);

    Eigen::VectorXd     getAng();

    Eigen::MatrixXd getH();
    Eigen::MatrixXd getH(const unsigned int _i);

    double getMax(const unsigned int _i);
    double getMin(const unsigned int _i);

    ~BaxterChain();
};
