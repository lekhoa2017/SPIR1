#ifndef ROBUST_INTEGRAL_CONTROL_FULL_DOF_H
#define ROBUST_INTEGRAL_CONTROL_FULL_DOF_H


#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PointStamped.h"
#include "ros/ros.h"
#include "Eigen/Dense"
#include <math.h>
#include <queue>
#include <stdexcept>
#include "spir_atnv/ThrustStamped.h"
#include "spir_atnv/calc_coeff_matrix.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"
#include "spir_atnv/kalman.hpp"
#include "spir_atnv/PileStateStamped.h"
#include <vector>
#include <string>
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"


/** @class robust_integral_control_full_dof.h
 *  @brief robust integral control for full dof based on the inputs from IMU and
 *  This node is built upon robot_integral_control
 *
 *
 *
 *  @author Wenjie Lu
 *  @contact wenjie.lu@uts.edu.au
 *  @date 2nd Sep 2016
 *  @version 1.0.0
 *  @version 1.1.0
 * 		subscribe to topic "pile_state_full" published by node cluster_dbscan
 *  @bug Currently the No known bugs.
 *  @todo Change sign() function to change very small values to 1 or -1.
 *        If sin(theta) (in calcW()) is very small, modify it.
 * 	  Currently the goal (xyGoalQueue) is initialized in the constructor,
 * 	  change it to launch file fasion or ros topic fasion
 *
 */

class RobIntControlFullDof
{
public:
    //Constructor
    RobIntControlFullDof ( ros::NodeHandle *, ros::NodeHandle * );

    void callBack_imu ( const sensor_msgs::Imu::ConstPtr& msg );
    void callBack_xy ( const geometry_msgs::PointStamped::ConstPtr& msg );
    void callBack_xyGoal ( const geometry_msgs::PointStamped::ConstPtr& msg );
    void callBack_z ( const std_msgs::Float32MultiArray::ConstPtr& msg );
    void callBack_full ( const spir_atnv::PileStateStamped::ConstPtr& msg );
    void callBack_atnv_trigger ( const std_msgs::Bool::ConstPtr& msg );
    void callBack_z_d ( const std_msgs::Float32MultiArray::ConstPtr& msg );

    ros::NodeHandle n_;
    ros::NodeHandle nh;

    Eigen::Matrix<double,6,1> calcControlOutput();
    /*  Calculate control inputs required to keep IMU/robot level in the x and y directions. See
    "robust_integral_control.h" for full mathematical explanation:

    u2(t) = k * [e(t) - e(0)] + integral{k * e(T) + B * sign[e(t)]} * dT + f1

    e(t) = a * W - R12 * omega

    W is calcualted based on the rotation between [0 0 -1]^T gravity vector and
    [0 0 -1]^T vector in IMU frame * the rotation of yaw in imu frame to align the heading with pile
    */


private:
    /* Functions */

    // Retrieving and store data from message/parameters
    void retrieveRotMat();
    void retrieveParams();
    template <typename MSGTYPE>
    void storeMsgs ( const MSGTYPE&, std::queue <MSGTYPE>&, MSGTYPE& );

    // Time related
    template <typename MSGTYPE>
    double calcTimeStep ( const MSGTYPE&, const MSGTYPE& );
    double calcTimeStepIntegral();
    double calcTimeStep();

    // Initializer
    KalmanFilter initializeKf ( std::string suffix );
    void initializeKfState ( KalmanFilter& kf, double state );

    // Setter, getter
    void setYawGoal();
    tf::Quaternion getCOMOrientation();

    // Convertor and calculator
    void string2Matrix ( Eigen::MatrixXd& matrixOut, std::string str, std::string delim );
    void generalizedForces2ThrustersForces ( const Eigen::Matrix<double,6,1>& gf,spir_atnv::ThrustStamped& thrust,std::string suffix );
    Eigen::Matrix<double,3,3> skewMatrix ( Eigen::Matrix<double,3,1> );
    Eigen::Matrix<double,3,1> sign ( Eigen::Matrix<double,3,1> );
    Eigen::Matrix<double,3,3> convQuatToRot ( tf::StampedTransform );
    void normalizeVector ( tf::Vector3& W );

    // Error calculator for function calcControlOutput()
    Eigen::Matrix<double,3,1> calcTranslateW();
    Eigen::Matrix<double,3,1> calcRotateW();
    Eigen::Matrix<double,3,1> calcTranslateE();
    Eigen::Matrix<double,3,1> calcRotateE();



    // publisher
    void robot_state_publish();
    /* Variables */

    // Tranformation related
    Eigen::Matrix<double,3,3> rotMat_imu;      // Rotation matrix
    Eigen::Matrix<double,3,3> rotMat_lidar;
    Eigen::Matrix<double,6,8> coeffMat;
    tf::StampedTransform comToIMU;		// tf
    tf::StampedTransform comToLidar;
    tf::StampedTransform comToDS;
    
    // Integral
    Eigen::Matrix<double,3,1> integralTranslate;
    Eigen::Matrix<double,3,1> integralRotate;

    // For function calcControlOutput()
    Eigen::Matrix<double,3,1> angVel;       // Angular Velocity
    Eigen::Matrix<double,3,1> initialE;      // e(0)
    Eigen::Matrix<double,3,1> f1;
    double yawGoal;        // goal in yaw
    int status;
    // Parameters for calcControlOutput()
    double k_r;
    double b_r;
    double a_r;
    double c_r;
    double k_t;
    double b_t;
    double a_t;
    double c_t;
    double z_t;
    double g;

    double z_goal;
    double pitch_goal;
    double roll_goal;

    
    double integral_x_max;
    double integral_y_max;
    double integral_z_max;
    double integral_s_max;
    bool auto_mode;
    bool leved;

    // ROS messages and publishers
    ros::Publisher control_pub;
    ros::Publisher robot_status_pub;
    sensor_msgs::Imu initialImuMsg;
    geometry_msgs::PointStamped initialXyMsg;
    geometry_msgs::PointStamped initialZMsg;
    spir_atnv::PileStateStamped initialFullMsg;
    sensor_msgs::Imu currentMsg;
    sensor_msgs::Imu previousMsg;

    // Queue to store current and previous message
    std::queue <sensor_msgs::Imu> imuMsgQueue;
    std::queue <geometry_msgs::PointStamped> xyMsgQueue;
    std::queue <geometry_msgs::PointStamped> zMsgQueue;
    std::queue <geometry_msgs::PointStamped> xyGoalQueue;
    std::queue <spir_atnv::PileStateStamped> fullMsgQueue;
    std::queue <ros::Time> previousLoopTimeQueue;

    //Filters
    KalmanFilter kfx;
    KalmanFilter kfy;
    KalmanFilter kfz;

};

#endif
