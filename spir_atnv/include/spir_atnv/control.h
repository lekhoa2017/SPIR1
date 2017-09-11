#ifndef CONTROL_H
#define CONTROL_H
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
/** @class control.h
 *  @brief control abstract class
 *
 *
 *
 *  @author Wenjie Lu
 *  @contact Wenjie.Lu@uts.edu.au
 *  @date 16/06/2017
 *  @version 1.0.0
 *  @bug Currently the No known bugs.
 *  @todo Add description to the method
 *
 */
/* Calculate the control input (i.e angular accelerations) based on IMU readings.
 * model based + disturbace observer
*/
class Control
{
public:
    //Constructor
    Control ( ros::NodeHandle *, ros::NodeHandle * );

    /* callback function to robot state topic
     * 12*1 vector
     * position : eta1 (in inertial frame)
     * attitude : eta2 (in inertial frame)
     * linear velocity : velB (in body frame)
     * angular velocity : alphaB (in body frame)
     */
    void callBackState ( const std_msgs::Float32MultiArray::ConstPtr& msg );
    
    /* callback function to desired pose, velocities, accelerations
     * 18*1
     * position : eta1 (in inertial frame)
     * attitude : eta2 (in inertial frame)
     * linear velocity : velB (in body frame)
     * angular velocity : alphaB (in body frame)
     */
    void callBackStateDesired ( const std_msgs::Float32MultiArray::ConstPtr& msg );
    
    // node handles
    ros::NodeHandle n_;
    ros::NodeHandle nh;

    /* get control : desired thruster forces 
     */
    void updateThrusterControl() ;

protected:
    // Variables
    
    // timestamp, is updated when robot state message is available
    ros::Time time_stamp;	
    // mapping matrix B; 
    Eigen::Matrix<double,6,8> coeffMat;
    // bound of thruster forces
    Eigen::Matrix<double,8,1> UB;
    Eigen::Matrix<double,8,1> LB;
    Eigen::Matrix<double,8,1> UB_DB; // deadband
    Eigen::Matrix<double,8,1> LB_DB;

    // desired pose and velocity
    Eigen::Matrix<double,3,1> eta1_d;
    Eigen::Matrix<double,3,1> eta2_d;
    Eigen::Matrix<double,3,1> velB_d;
    Eigen::Matrix<double,3,1> omegaB_d;
    Eigen::Matrix<double,3,1> accB_d;
    Eigen::Matrix<double,3,1> alphaB_d;
    

    // robot state	
    Eigen::Matrix<double,3,1> eta1;
    Eigen::Matrix<double,3,1> eta2;
    Eigen::Matrix<double,3,1> velB;
    Eigen::Matrix<double,3,1> omegaB;

    // constaints on generalized input
    double max_thruster_val;
    Eigen::Matrix<double,6,1> max_gf ; // positve, max value of generalized force
    double gw;
    
    Eigen::Matrix<double,6,1> tau;
    Eigen::Matrix<double,8,1> thruster_tau;
    // ROS messages and publishers
    ros::Publisher control_pub;
    
    // Functions

    // Retrieving data from message/parameters
    void retrieveRotMat();
    void retrieveParams();

    
    // Calulation functions
    Eigen::Matrix<double,6,1> imposeSaturation(Eigen::Matrix<double,6,1> gf); // the saturation threshould is given by max_gf
    Eigen::Matrix<double,12,1> getError(); // get desired acceleration in linear and angualr velocity
    virtual Eigen::Matrix<double,6,1> getControlGF() = 0; // get control in generalized forces
    virtual void updateControlParam() = 0; // get control in generalized forces
    
    double calcTimeStep();
    
    // 
    void generalizedForces2ThrustersForces ( const Eigen::Matrix<double,6,1>& gf,spir_atnv::ThrustStamped& thrust,std::string suffix );
    Eigen::Matrix<double,8,1> solveQP(const Eigen::Matrix<double,6,1>& gf);
    Eigen::Matrix<double,16,1> getBoundMixedInteger(double a,double b,double c,double d);
    
    // initialize parameters
    void initializeParameters();
    
};

#endif
