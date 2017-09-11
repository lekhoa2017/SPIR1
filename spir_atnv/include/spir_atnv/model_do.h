#ifndef MODEL_DO_H
#define MODEL_DO_H
#include "spir_atnv/control.h"
#include "sensor_msgs/Imu.h"
#include "ros/ros.h"
#include "Eigen/Dense"
#include <math.h>
#include <queue>
#include <stdexcept>
#include <boost/concept_check.hpp>
#include "spir_atnv/ThrustStamped.h"
#include "spir_atnv/kinematics.h"
#include "spir_atnv/calc_coeff_matrix.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"
#include "std_msgs/Float32MultiArray.h"
/** @class model_do.h
 *  @brief
 *
 *
 *
 *  @author Wenjie Lu
 *  @contact Wenjie.Lu@uts.edu.au
 *  @date 08/06/2017
 *  @version 1.0.0
 *  @bug Currently the No known bugs.
 *  @todo Add description to the method
 *
 */
/* Calculate the control input (i.e angular accelerations) based on IMU readings.
 * model based + disturbace observer
*/
class ModelDO : public Control
{
public:
    //Constructor
    ModelDO ( ros::NodeHandle *, ros::NodeHandle * );
private:
    // Variables
        
    // control parameters
    Eigen::Matrix<double,6,6> k1;
    Eigen::Matrix<double,6,6> k2;
    Eigen::Matrix<double,6,6> c; // used to desgin L
    
    // model parameters
    Eigen::Matrix<double,6,6> M;
    
    double DGB;                                       // disturbance between buoyancy and center of mass mutipled with gravity
    double BuoF;
    Eigen::Matrix<double,3,1> eta2_ini;               // this determines the vector from gravity to buoyancy center, in body frame
    Eigen::Matrix<double,3,1> zGB;               // this determines the vector from gravity to buoyancy center, in body frame
    
    // control auxiliary variables
    Eigen::Matrix<double,6,1> augmented_disturbance; // estimated augmented disturbance 
    Eigen::Matrix<double,6,1> G;
    Eigen::Matrix<double,6,1> tau;
    

    // disturbance augment function
    Eigen::Matrix<double,6,1> P(); // augment function to formulate augmented_disturbance
    
    Eigen::Matrix<double,6,6> L(); // gain function for update disturbance estimation
    Eigen::Matrix<double,3,1> additionalBuo(); // gain function for update disturbance estimation
    
    // Calulation functions
    Eigen::Matrix<double,6,1> getDerivativeAugmentedDisturbance(); // get time derivative of augmented_disturbance
    Eigen::Matrix<double,6,1> getDesiaredAcc(); // get desired acceleration in linear and angualr velocity
    Eigen::Matrix<double,6,1> getControlGF(); // get control in generalized forces
    void updateControlParam(); // get time derivative of augmented_disturbance
    void retrieveParams();
    // initialize parameters
    void initialize();
    
};

#endif
