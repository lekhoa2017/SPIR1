#ifndef RISE_PID_H
#define RISE_PID_H
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
/** @class rise_pid.h
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
class RisePid : public Control
{
public:
    //Constructor
    RisePid ( ros::NodeHandle *, ros::NodeHandle * );
private:
    // Variables
        
    // control parameters
    Eigen::Matrix<double,6,6> k;
    Eigen::Matrix<double,6,6> b;
    Eigen::Matrix<double,6,6> a;
    Eigen::Matrix<double,6,6> c; // used to desgin L

    
    // control auxiliary variables
    Eigen::Matrix<double,6,1> integral;
    Eigen::Matrix<double,6,1> tau;
    

    
    // Calulation functions
    Eigen::Matrix<double,6,1> getControlGF(); // get control in generalized forces
    Eigen::Matrix<double,6,1> getS(); // get control in generalized forces
    Eigen::Matrix<double,6,1> sign(Eigen::Matrix<double,6,1> s); // get sign
    void updateControlParam(); // get time derivative of augmented_disturbance
    void retrieveParams();
    // initialize parameters
    void initialize();
    
};

#endif
