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

    // Print level to be used throughout the code
    int print_level;

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

    std::vector<BaxterChain>    ctrlChains;
    std::vector<CollisionPoint> collPoints;

public:
    AvoidanceHandler(BaxterChain _chain,
                     std::vector<Obstacle> _obstacles,
                     std::string _type = "none",
                     int _print_level = 0);

    std::string getType() { return type; };

    virtual Eigen::MatrixXd getV_LIM(const Eigen::MatrixXd &v_lim);

    /**
     * Gets the control chains
     *
     * @return an std::vector of all the control chains
     */
    std::vector<BaxterChain> getCtrlChains();

    /**
     * Gets the control points
     *
     * @return an std::vector of all the control points
     */
    std::vector<CollisionPoint> getCtrlPoints();


    /**
     * Convert the control chains to a set of RVIZmarkers for
     * visualization in RVIZ.
     *
     * @return  a vector of RVIZ Markers.
     *          They can be directly provided to an RVIZPublisher object.
     */
    std::vector<RVIZMarker> toRVIZMarkers();

    virtual ~AvoidanceHandler();
};

/****************************************************************/
class AvoidanceHandlerTactile : public virtual AvoidanceHandler
{
private:
    double avoidingSpeed;

public:
    AvoidanceHandlerTactile(BaxterChain _chain,
                            std::vector<Obstacle> _obstacles,
                            int _print_level = 0);

    Eigen::MatrixXd getV_LIM(const Eigen::MatrixXd &v_lim);

    ~AvoidanceHandlerTactile();
};


#endif
