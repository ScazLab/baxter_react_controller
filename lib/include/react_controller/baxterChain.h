#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>

#include <Eigen/Dense>

using namespace Eigen;


/****************************************************************/
class BaxterChain : public KDL::Chain
{
    std::vector<double> _joint_angles;

    public:
    BaxterChain();
    VectorXd getAngEigen();
    std::vector<double> getAngStd();
    bool setAng(std::vector<double> joint_angles);
    MatrixXd getH();
    ~BaxterChain();
};

MatrixXd GeoJacobian(BaxterChain &chain);
