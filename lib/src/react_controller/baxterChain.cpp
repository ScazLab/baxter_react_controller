#include <react_controller/baxterChain.h>

MatrixXd GeoJacobian(BaxterChain &chain) {
    std::unique_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
    KDL::Jacobian J;
    KDL::JntArray arr;
    std::vector<double> angles = chain.getAngStd();
    for (int i = 0; i < angles.size(); ++i) {
        arr(i) = angles[i];
    }
    jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(chain));
    jnt_to_jac_solver_->JntToJac(arr, J);
    return J.data;
}

VectorXd BaxterChain::getAngEigen() {
    return Map<VectorXd>(_joint_angles.data(), _joint_angles.size());
}

std::vector<double> BaxterChain::getAngStd() {
    return _joint_angles;
}

bool BaxterChain::setAng(std::vector<double> joint_angles) {
    _joint_angles = joint_angles;
    return true;
}

MatrixXd BaxterChain::getH() {

}

BaxterChain::BaxterChain() {

}

BaxterChain::~BaxterChain() {
    return;
}