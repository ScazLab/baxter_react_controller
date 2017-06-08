#ifndef __AVOIDANCEHANDLER_H__
#define __AVOIDANCEHANDLER_H__

#include <vector>
#include <deque>

#include <stdarg.h>
#include <string>

#include <algorithm>

#include "react_controller/baxterChain.h"

/****************************************************************/
class AvoidanceHandlerAbstract
{
private:
    BaxterChain chain;

protected:
    std::vector<collisionPoint> collPoints;
    std::string type;

    std::deque<BaxterChain>     ctrlPointChains;

    /**
     * Creates a full transform as given by a DCM matrix at the pos and norm w.r.t. the original frame,
     * from the pos and norm (one axis set arbitrarily)
     * @param  pos  [description]
     * @param  norm [description]
     * @param  FoR  [description]
     * @return      [description]
     */
    bool computeFoR(const Eigen::VectorXd &pos,
                    const Eigen::VectorXd &norm,
                          Eigen::Matrix4d &FoR);

public:
    AvoidanceHandlerAbstract(const BaxterChain &_chain,
                             const std::vector<collisionPoint> &_collPoints,
                             const std::string _type = "none");

    std::string getType() { return type; };

    // std::deque<Eigen::VectorXd> getCtrlPointsPosition();

    virtual Eigen::MatrixXd getVLIM(const Eigen::MatrixXd &v_lim);

    ~AvoidanceHandlerAbstract();
};

/****************************************************************/
class AvoidanceHandlerTactile : public virtual AvoidanceHandlerAbstract
{
private:
    double avoidingSpeed;

public:
    AvoidanceHandlerTactile(const BaxterChain &_chain,
                            const std::vector<collisionPoint> &_collPoints);

    ~AvoidanceHandlerTactile();

    Eigen::MatrixXd getVLIM(const Eigen::MatrixXd &v_lim);
};


#endif
