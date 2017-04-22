#include <Eigen/Dense>

#include "react_controller/avoidance.h"

using namespace std;

// using namespace yarp::sig;
// using namespace yarp::math;
// using namespace yarp::os;
// using namespace iCub::iKin;
// using namespace iCub::skinDynLib;

AvoidanceHandlerAbstract::AvoidanceHandlerAbstract(const BaxterChain &_chain, const std::vector<collisionPoint_t> &_collisionPoints,const unsigned int _verbosity) :  chain(_chain), collisionPoints(_collisionPoints), verbosity(_verbosity)
{
        if (!collisionPoints.empty()){
            for(std::vector<collisionPoint_t>::const_iterator it = collisionPoints.begin(); it != collisionPoints.end(); ++it) {
                // size_t dim = chain.getNrOfSegments();
                BaxterChain customChain = chain; //instantiates a new chain, copying from the old (full) one

                // if (verbosity >= 5){
                //     printf("Full chain has %d DOF \n",chain.getNrOfSegments());
                //     printf("chain.getH() (end-effector): \n %s \n",chain.getH().toString(3,3).c_str());
                // }
                // int linkNrForCurrentSkinPartFrame = 0;
                // if (verbosity >= 5){
                //     if (dim ==7){
                //         printf("SkinPart %s, linkNum %d, chain.getH() (skin part frame): \n %s \n",SkinPart_s[(*it).skin_part].c_str(),SkinPart_2_LinkNum[(*it).skin_part].linkNum,chain.getH(SkinPart_2_LinkNum[(*it).skin_part].linkNum).toString(3,3).c_str());
                //     }
                //     else if (dim == 10){
                //         printf("SkinPart %s, linkNum %d + 3, chain.getH() (skin part frame): \n %s \n",SkinPart_s[(*it).skin_part].c_str(),SkinPart_2_LinkNum[(*it).skin_part].linkNum,chain.getH(SkinPart_2_LinkNum[(*it).skin_part].linkNum + 3).toString(3,3).c_str());
                //     }
                // }
                Eigen::MatrixXd JfullChain = chain.GeoJacobian(); //6 rows, n columns for every active DOF
                //printf("GeoJacobian matrix for canonical end-effector (palm): \n %s \n",JfullChain.toString(3,3).c_str());

                // Remove all the more distal links after the collision point
                // if the skin part is a hand, no need to remove any links from the chain
                // if (((*it).skin_part == SKIN_LEFT_FOREARM) ||  ((*it).skin_part == SKIN_RIGHT_FOREARM)){
                //     if(dim == 10){
                //         customChain.rmLink(9); customChain.rmLink(8);
                //         // we keep link 7 from elbow to wrist - it is getH(7) that is the FoR at the wrist in which forearm skin is expressed; and we want to keep the elbow joint part of the game
                        // printMessage(2,"obstacle threatening skin part %s, blocking links 8 and 9 on subchain for avoidance\n",SkinPart_s[(*it).skin_part].c_str());
                //     }
                //     else if(dim==7){
                //         customChain.rmLink(6); customChain.rmLink(5);
                //         // we keep link 4 from elbow to wrist - it is getH(4) that is the FoR at the wrist in which forearm skin is expressed; and we want to keep the elbow joint part of the game
                //         customChain.blockLink(6); customChain.blockLink(5);//wrist joints
                        // printMessage(2,"obstacle threatening skin part %s, blocking links 5 and 6 on subchain for avoidance\n",SkinPart_s[(*it).skin_part].c_str());

                //     }
                // }
                // else if (((*it).skin_part == SKIN_LEFT_UPPER_ARM) ||  ((*it).skin_part == SKIN_RIGHT_UPPER_ARM)){
                //     if(dim == 10){
                //         customChain.rmLink(9); customChain.rmLink(8);customChain.rmLink(7);customChain.rmLink(6);
                        // printMessage(2,"obstacle threatening skin part %s, blocking links 6-9 on subchain for avoidance\n",SkinPart_s[(*it).skin_part].c_str());
                //     }
                //     else if(dim==7){
                //         customChain.rmLink(6); customChain.rmLink(5);customChain.rmLink(4);customChain.rmLink(3);
                        // printMessage(2,"obstacle threatening skin part %s, blocking links 3-6 on subchain for avoidance\n",SkinPart_s[(*it).skin_part].c_str());
                //     }
                // }


                // SetHN to move the end effector toward the point to be controlled - the average locus of collision threat from safety margin
                Eigen::Matrix4d HN;
                HN.setIdentity();
                computeFoR((*it).x,(*it).n,HN);
                // printMessage(5,"HN matrix at collision point w.r.t. local frame: \n %s \n",HN.toString(3,3).c_str());
                // customChain.setHN(HN); //setting the end-effector transform to the collision point w.r.t subchain
                // if (verbosity >=5){
                //     Eigen::MatrixXd H = customChain.getH();
                //     printf("H matrix at collision point w.r.t. root: \n %s \n",H.toString(3,3).c_str());
                // }

                // printMessage(5,"Normal at collision point w.r.t. skin part frame: %s, norm %f.\n",(*it).n.toString(3,3).c_str(),yarp::math::norm((*it).n));
                // //yarp::sig::Vector normalAtCollisionInRootFoR = chain_local.getH() * (*it).n;
                // //normalAtCollisionInRootFoR.subVector(0,3); //take out the dummy element from homogenous transform
                // printMessage(5,"Normal at collision point w.r.t. Root: %s, norm %f.\n",normalAtCollisionInRootFoR.toString(3,3).c_str(),yarp::math::norm(normalAtCollisionInRootFoR));

                //for testing
                //yarp::sig::Vector normalAtCollisionInEndEffFrame(4,0.0);
                //normalAtCollisionInEndEffFrame(2) = 1.0; //z-axis ~ normal
                //yarp::sig::Vector normalAtCollisionInRootFoR_2 = chain_local.getHN() * normalAtCollisionInEndEffFrame;

                ctrlPointChains.push_back(customChain);

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
AvoidanceHandlerTactile::AvoidanceHandlerTactile(const BaxterChain &_chain,const std::vector<collisionPoint_t> &_collisionPoints,const unsigned int _verbosity): AvoidanceHandlerAbstract(_chain,_collisionPoints,_verbosity)
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
            Eigen::MatrixXd J = tmp.block(3,ctrlPointChains[i].getNrOfSegments()-1,0,0); //.submatrix(0,2,0,ctrlPointChains[i].getNrOfSegments()-1); //first 3 rows ~ dPosition/dJoints
            Eigen::VectorXd normal = ctrlPointChains[i].getH().col(2);//.subEigen::VectorXd(0,2); //get the end-effector frame of the standard or custom chain (control point derived from skin), takes the z-axis (3rd column in transform matrix) ~ normal, only its first three elements of the 4 in the homogenous transf. format
            Eigen::VectorXd s=(J.transpose()*normal) * avoidingSpeed * collisionPoints[i].magnitude; //project movement along the normal into joint velocity space and scale by default avoidingSpeed and magnitude of skin (or PPS) activation
            if (verbosity>=2){
                // printf("J for positions at control point:\n %s \nJ.transposed:\n %s \nNormal at control point: (%s), norm: %f \n",J.toString(3,3).c_str(),J.transposed().toString(3,3).c_str(), normal.toString(3,3).c_str(),norm(normal));
                // printf("s = (J.transposed()*normal) * avoidingSpeed * collisionPoints[i].magnitude \n (%s)T = (%s)T * %f * %f\n",s.toString(3,3).c_str(),(J.transposed()*normal).toString(3,3).c_str(),avoidingSpeed,collisionPoints[i].magnitude);
            }
            s = s * -1.0; //we reverse the direction to obtain joint velocities that bring about avoidance
            // printMessage(2,"s * (-1) -> joint contributions toward avoidance: \n (%s) \n",s.toString(3,3).c_str());

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









