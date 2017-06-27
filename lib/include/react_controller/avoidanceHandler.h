#ifndef __AVOIDANCEHANDLER_H__
#define __AVOIDANCEHANDLER_H__

#include <vector>

#include <stdarg.h>

#include "react_controller/baxterChain.h"

/****************************************************************/
class AvoidanceHandler
{
private:
    BaxterChain chain;

    /**
     * Creates a full transform as given by a DCM matrix at the pos and norm w.r.t.
     * the original frame, from the pos and norm (one axis set arbitrarily)
     * @param  pos  [description]
     * @param  norm [description]
     * @param  FoR  [description]
     * @return      [description]
     */
    bool computeFoR(const Eigen::VectorXd &pos,
                    const Eigen::VectorXd &norm,
                          Eigen::Matrix4d &FoR);

protected:
    std::string type;

    std::vector<BaxterChain>    ctrlPointChains;
    std::vector<collisionPoint>      collPoints;

public:
    AvoidanceHandler(const BaxterChain &_chain,
                             const std::vector<Eigen::Vector3d> &_obstacles,
                             const std::string _type = "none");

    std::string getType() { return type; };

    // std::deque<Eigen::VectorXd> getCtrlPointsPosition();

    virtual Eigen::MatrixXd getV_LIM(const Eigen::MatrixXd &v_lim);

    ~AvoidanceHandler();
};

/****************************************************************/
class AvoidanceHandlerTactile : public virtual AvoidanceHandler
{
private:
    double avoidingSpeed;

public:
    AvoidanceHandlerTactile(const BaxterChain &_chain,
                            const std::vector<Eigen::Vector3d> &_obstacles);

    ~AvoidanceHandlerTactile();

    Eigen::MatrixXd getV_LIM(const Eigen::MatrixXd &v_lim);
};


#endif
