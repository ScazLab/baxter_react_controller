#ifndef __AVOIDANCEHANDLER_H__
#define __AVOIDANCEHANDLER_H__

#include <vector>
#include <deque>

#include <stdarg.h>
#include <string>

#include <algorithm>

#include <react_controller/baxterChain.h>

// #include <yarp/sig/all.h>
// #include <yarp/math/Math.h>
// #include <yarp/os/Property.h>

// #include <iCub/iKin/iKinFwd.h>
// #include <iCub/skinDynLib/common.h>

/****************************************************************/
class AvoidanceHandlerAbstract
{


public:
    AvoidanceHandlerAbstract(const BaxterChain &_chain, const std::vector<collisionPoint_t> &_collisionPoints, const unsigned int _verbosity=0);

    std::string getType() const;

    // virtual yarp::os::Property getParameters() const;

    // virtual void setParameters(const yarp::os::Property &parameters);

    // std::deque<Eigen::VectorXd> getCtrlPointsPosition();

    virtual Eigen::MatrixXd getVLIM(const Eigen::MatrixXd &v_lim);

    virtual ~AvoidanceHandlerAbstract();

 protected:
    std::string type;
    BaxterChain chain;
    const std::vector<collisionPoint_t> &collisionPoints;
    unsigned int verbosity;
    std::deque<BaxterChain> ctrlPointChains;
    // yarp::os::Property parameters;

    bool computeFoR(const Eigen::VectorXd &pos, const Eigen::VectorXd &norm, Eigen::Matrix4d &FoR);

    /**
    * Prints a message according to the verbosity level:
    * @param l is the level of verbosity: if verbosity >= l, something is printed
    * @param f is the text. Please use c standard (like printf)
    */
    int printMessage(const unsigned int l, const char *f, ...) const;

};



/****************************************************************/
class AvoidanceHandlerTactile : public virtual AvoidanceHandlerAbstract
{

public:
    AvoidanceHandlerTactile(const BaxterChain &_chain,const std::vector<collisionPoint_t> &_collisionPoints,const unsigned int _verbosity=0);
    // void setParameters(const yarp::os::Property &parameters);
    Eigen::MatrixXd getVLIM(const Eigen::MatrixXd &v_lim);

protected:
    double avoidingSpeed;


};


#endif
