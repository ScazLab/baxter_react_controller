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
private:
    std::vector<double> _q;

public:

    BaxterChain();
    BaxterChain(KDL::Chain _chain);
    BaxterChain(KDL::Chain _chain, std::vector<double> _joint_angles_0);
    VectorXd getAng();
    // std::vector<double> getAngStd();
    bool setAng(std::vector<double> joint_angles);
    MatrixXd getH();
    MatrixXd getH(const unsigned int i);
    MatrixXd getH(KDL::Segment seg);
    MatrixXd GeoJacobian();
    MatrixXd GeoJacobian(const unsigned int i);
    ~BaxterChain();
};

MatrixXd GeoJacobian(BaxterChain &chain);
