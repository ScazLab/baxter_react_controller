#include <Eigen/Dense>
#include <math.h>
#include <ros/ros.h>

#include "react_controller/avoidance.h"

using namespace std;

AvoidanceHandlerAbstract::AvoidanceHandlerAbstract(const BaxterChain &_chain, const std::vector<collisionPoint> &_collisionPoints,const unsigned int _verbosity) :  chain(_chain), collisionPoints(_collisionPoints), verbosity(_verbosity)
{
        if (!collisionPoints.empty()){
            BaxterChain customChain = chain;
            for (size_t i = 0, num_jnts = chain.getNrOfJoints(); i < num_jnts - 1; ++i) {
                customChain.removeJoint();
            }
            for (size_t i = 0, num_jnts = chain.getNrOfJoints(), num_segs = customChain.getNrOfSegments(); i < num_jnts - 1; ++i) {
                customChain.addSegment(chain.getSegment(num_segs++));
                while (chain.getSegment(num_segs).getJoint().getType() == KDL::Joint::None) {
                    customChain.addSegment(chain.getSegment(num_segs++));
                }
                BaxterChain nextChain = customChain; //instantiates a new chain, copying from the old (full) one
                Eigen::Matrix4d HN;
                HN.setIdentity();
                computeFoR((collisionPoints[i]).x,(collisionPoints[i]).n,HN);

                KDL::Vector x, y, z, pos;
                tf::vectorEigenToKDL(HN.block<3,1>(0,0), x);
                tf::vectorEigenToKDL(HN.block<3,1>(0,1), y);
                tf::vectorEigenToKDL(HN.block<3,1>(0,2), z);
                tf::vectorEigenToKDL(HN.block<3,1>(0,3), pos);
                KDL::Rotation rot = KDL::Rotation(x, y, z);
                KDL::Segment s = KDL::Segment(KDL::Joint(KDL::Joint::None),
                                                KDL::Frame(rot, pos));
                nextChain.addSegment(s);
                ctrlPointChains.push_back(nextChain);

            }
        }
        type="none";
}


/****************************************************************/
string AvoidanceHandlerAbstract::getType() const
{
    return type;
}

/****************************************************************/
// Property AvoidanceHandlerAbstract::getParameters() const
// {
//     return parameters;
// }

/****************************************************************/
// void AvoidanceHandlerAbstract::setParameters(const Property &parameters)
// {
//     this->parameters=parameters;
// }


/****************************************************************/
// deque<Eigen::VectorXd> AvoidanceHandlerAbstract::getCtrlPointsPosition()
// {
//     deque<Eigen::VectorXd> ctrlPoints;
//     for (size_t i=0; i<ctrlPointChains.size(); i++)
//         ctrlPoints.push_back(ctrlPointChains[i].EndEffPosition());
//     return ctrlPoints;
// }

/****************************************************************/
Eigen::MatrixXd AvoidanceHandlerAbstract::getVLIM(const Eigen::MatrixXd &v_lim)
{
    return v_lim;
}

/****************************************************************/
AvoidanceHandlerAbstract::~AvoidanceHandlerAbstract()
{
    ctrlPointChains.clear();
}


int AvoidanceHandlerAbstract::printMessage(const unsigned int l, const char *f, ...) const
{
    if (verbosity>=l)
    {
        fprintf(stdout,"[%s] ",type.c_str());

        va_list ap;
        va_start(ap,f);
        int ret=vfprintf(stdout,f,ap);
        va_end(ap);
        return ret;
    }
    else
        return -1;
};


//creates a full transform as given by a DCM matrix at the pos and norm w.r.t. the original frame, from the pos and norm (one axis set arbitrarily)
bool AvoidanceHandlerAbstract::computeFoR(const Eigen::VectorXd &pos, const Eigen::VectorXd &norm, Eigen::Matrix4d &FoR)
{
    Eigen::Vector3d zeros;
    zeros.setZero();
    if (norm == zeros)
    {
        FoR.setIdentity();
        return false;
    }

    Eigen::Vector3d x(0,0,0), y(0,0,0), z(0,0,0);

    z = norm;
    if (z[0] == 0.0)
    {
        z[0] = 0.00000001;    // Avoid the division by 0
    }
    y[0] = -z[2]/z[0]; //y is in normal plane
    y[2] = 1; //this setting is arbitrary
    x = -1*(z.cross(y));

    // Let's make them unitary vectors:
    x = x / x.norm();
    y = y / y.norm();
    z = z / z.norm();

    FoR.setIdentity();
    FoR.block<3,1>(0,0) = x;
    FoR.block<3,1>(0,1) = y;
    FoR.block<3,1>(0,2) = z;
    FoR.block<3,1>(0,3) = pos;

    return true;
}



/****************************************************************/
AvoidanceHandlerTactile::AvoidanceHandlerTactile(const BaxterChain &_chain,const std::vector<collisionPoint> &_collisionPoints,const unsigned int _verbosity): AvoidanceHandlerAbstract(_chain,_collisionPoints,_verbosity)
{
    type="tactile";

    avoidingSpeed = 50;
    // produce collisionPoint.magnitude * avoidingSpeed deg/s repulsive speed

    // parameters.unput("avoidingSpeed");
    // parameters.put("avoidingSpeed",avoidingSpeed);
}

/****************************************************************/
// void AvoidanceHandlerTactile::setParameters(const Property &parameters)
// {
//     if (parameters.check("avoidingVelocity"))
//     {
//         avoidingSpeed=parameters.find("avoidingSpeed").asDouble();
//         this->parameters.unput("avoidingSpeed");
//         this->parameters.put("avoidingSpeed",avoidingSpeed);
//     }
// }

/****************************************************************/
Eigen::MatrixXd AvoidanceHandlerTactile::getVLIM(const Eigen::MatrixXd &v_lim)
{
        // printMessage(2,"AvoidanceHandlerTactile::getVLIM\n");
        Eigen::MatrixXd VLIM=v_lim;
        for (size_t i=0; i<ctrlPointChains.size(); i++)
        {
            // printMessage(2,"Chain with control point - index %d (last index %d), nDOF: %d.\n",i,ctrlPointChains.size()-1,ctrlPointChains[i].getNrOfSegments());
            Eigen::MatrixXd tmp = ctrlPointChains[i].GeoJacobian();
            Eigen::MatrixXd J = tmp.block(0, 0, 3, ctrlPointChains[i].getNrOfJoints()-1); //.submatrix(0,2,0,ctrlPointChains[i].getNrOfSegments()-1); //first 3 rows ~ dPosition/dJoints
            Eigen::VectorXd normal = ctrlPointChains[i].getH().col(2).block<3,1>(0,0);//.subEigen::VectorXd(0,2); //get the end-effector frame of the standard or custom chain (control point derived from skin), takes the z-axis (3rd column in transform matrix) ~ normal, only its first three elements of the 4 in the homogenous transf. format
            Eigen::VectorXd s=(J.transpose()*normal) * avoidingSpeed * collisionPoints[i].magnitude; //project movement along the normal into joint velocity space and scale by default avoidingSpeed and magnitude of skin (or PPS) activation
            if (verbosity>=2){
                // printf("J for positions at control point:\n %s \nJ.transposed:\n %s \nNormal at control point: (%s), norm: %f \n",J.toString(3,3).c_str(),J.transposed().toString(3,3).c_str(), normal.toString(3,3).c_str(),norm(normal));
                // printf("s = (J.transposed()*normal) * avoidingSpeed * collisionPoints[i].magnitude \n (%s)T = (%s)T * %f * %f\n",s.toString(3,3).c_str(),(J.transposed()*normal).toString(3,3).c_str(),avoidingSpeed,collisionPoints[i].magnitude);
            }
            s = s * -1.0; //we reverse the direction to obtain joint velocities that bring about avoidance
            // printMessage(2,"s * (-1) -> joint contributions toward avoidance: \n (%s) \n",s.toString(3,3).c_str());
            // cout << s;
            for (int j=0; j<s.size(); j++)
            {
                // printMessage(2,"        Joint: %d, s[j]: %f, limits before: Min: %f, Max: %f\n",j,s[j],VLIM(j,0),VLIM(j,1));
                if (s[j]>=0.0) //joint contributes to avoidance, we will set the min velocity accordingly
                {
                    s[j]=std::min(v_lim(j,1),s[j]); //make sure new min vel is <= max vel
                    VLIM(j,0)=std::max(VLIM(j,0),s[j]); // set min vel to max of s[j] and current limit ~ avoiding action
                    VLIM(j,1)=std::max(VLIM(j,0),VLIM(j,1)); //make sure current max is at least equal to current min
                    // printMessage(2,"            s>=0 clause, joint contributes to avoidance, adjusting Min; limits after: Min: %f, Max: %f\n",VLIM(j,0),VLIM(j,1));
                }
                else //joint acts to bring control point toward obstacle - we will shape the max vel
                {
                    s[j]=std::max(v_lim(j,0),s[j]);
                    VLIM(j,1)=std::min(VLIM(j,1),s[j]);
                    VLIM(j,0)=std::min(VLIM(j,0),VLIM(j,1));
                    // printMessage(2,"            s<0 clause, joint contributes to approach, adjusting Max; limits after: Min: %f, Max: %f\n",VLIM(j,0),VLIM(j,1));
                }
            }
        }

        return VLIM;
};









