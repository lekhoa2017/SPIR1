#ifndef ROBUST_INTEGRAL_CONTROL_H
#define ROBUST_INTEGRAL_CONTROL_H


#include "sensor_msgs/Imu.h"
#include "ros/ros.h"
#include "Eigen/Dense"
#include <math.h>
#include <queue>
#include <stdexcept>
#include "spir_atnv/ThrustStamped.h"
#include "spir_atnv/calc_coeff_matrix.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include "spir_atnv/kalman.hpp"
#include <sensor_msgs/Joy.h>
/** @class robust_integral_control.h
 *  @brief
 *
 *
 *
 *  @author Brendan Emery
 *  @contact Brendan.emery@student.uts.edu.au
 *  @date 20/10/2015
 *  @version 1.0.0
 *  @bug Currently the No known bugs.
 *  @todo Change sign() function to change very small values to 1 or -1.
 *        If sin(theta) (in calcW()) is very small, modify it.
 *
 */
/* Calculate the control input (i.e angular accelerations) based on IMU readings.
    The control input u2(t) is a (3x1) matrix that is calculated according to:

    u2(t) = k * [e(t) - e(0)] + integral{k * e(T) + B * sign[e(t)]} * dT + f1

    e(t) = a * W - R12 * omega

    W = skew([0; 0; 1]) * R12 * C

    sin(theta) = norm(W)

    W = (arcsin(sin(theta)) * W) / sin(theta)

    C = [-sin(pitch); sin(roll) * cos(pitch); cos(roll) * cos(pitch)]

    where:
        skew([x1; x2; x3]) = [0, -x3, x2;
                              x3, 0, -x1;
                              -x2, x1, 0]

        k = constant          ==> gain
        B = constant          ==> gain
        a = constant          ==> gain
        roll = variable       ==> angle of rotation about x axis
        pitch = variable      ==> angle of rotation about y axis
        R12 = (3x3) matrix    ==> rotation matrix denoting IMU orientation
                                  relative to the underwater robot body frame.
        omega = (3x1) vector  ==> angular velocity of IMU
        f1 = (3x1) vector     ==> Values from dynamic model of robot. If robot has not been modelled,
                                  default value is [0; 0; 0]

*/
class RobIntControl
{
public:
    //Constructor
    RobIntControl ( ros::NodeHandle *, ros::NodeHandle * );

    void callBack_imu ( const sensor_msgs::Imu::ConstPtr& msg );
    void callBack_z ( const std_msgs::Float32MultiArray::ConstPtr& msg );
    void callBack_z_d ( const std_msgs::Float32MultiArray::ConstPtr& msg );
    void callBack_atnv_trigger ( const std_msgs::Bool::ConstPtr& msg );
    void callBack_joy ( const sensor_msgs::Joy::ConstPtr& msg );
    
    ros::NodeHandle n_;
    ros::NodeHandle nh;

    Eigen::Matrix<double,3,1> calcControlOutput();
    /*  Calculate control inputs required to keep IMU/robot level in the x and y directions. See
    "robust_integral_control.h" for full mathematical explanation:

    u2(t) = k * [e(t) - e(0)] + integral{k * e(T) + B * sign[e(t)]} * dT + f1

    e(t) = a * W - R12 * omega

    W = skew([0; 0; 1]) * R12 * C

    C = [-sin(pitch); sin(roll) * cos(pitch); cos(roll) * cos(pitch)]
    */


private:
    // Functions

    // Retrieving data from message/parameters
    void retrieveRotMat();
    void retrieveParams();
    void storeMsgs ( sensor_msgs::Imu );


    // Calulation functions
    Eigen::Matrix<double,3,3> skewMatrix ( Eigen::Matrix<double,3,1> );
    Eigen::Matrix<double,3,1> sign ( Eigen::Matrix<double,3,1> );
    Eigen::Matrix<double,3,1> calcW ( sensor_msgs::Imu );
    Eigen::Matrix<double,3,1> calcE ( sensor_msgs::Imu );
    Eigen::Matrix<double,3,3> convQuatToRot ( tf::StampedTransform );
    void normalizeVector ( tf::Vector3& W );
    bool isInBound(double value, double bound);
    double calcTimeStep();
    void getRPY(double& r, double& p,double& y);

    // Variables
    Eigen::Matrix<double,3,3> rotMat;      // Rotation matrix
    Eigen::Matrix<double,3,1> angVel;       // Angular Velocity
    Eigen::Matrix<double,3,1> initialE;      // e(0)
    Eigen::Matrix<double,3,1> integral;
    Eigen::Matrix<double,3,1> f1;
    Eigen::Matrix<double,6,8> coeffMat;
    double k;
    double b;
    double a;
    double c;
    double z_goal;
    double pitch_goal;
    double roll_goal;
    double yaw_goal;
    double z_vel_goal;
    double pitch_vel_goal;
    double roll_vel_goal;
    double yaw_vel_goal;
    
    double z;
    double integral_z;
    double e_z;

    double z_k;
    double z_b;
    double z_a;
    double z_c;
    bool auto_mode;
    bool joy_mode;
    double max_thruster_val;
    double max_integral_z;
    double max_integral_s;
    double max_force_stabilization;
    double max_torque_depth;

    double current_yaw;
    double yaw0;
    bool overLimitFlag;
    bool yawLock;

    // ROS messages and publishers
    ros::Publisher imu_control_pub;
    ros::Publisher yaw_desired_pub;    
    sensor_msgs::Imu initialMsg;
    sensor_msgs::Imu currentMsg;
    sensor_msgs::Imu previousMsg;
    std_msgs::Float32MultiArray zMsg;
    // Queue to store current and previous message
    std::queue <sensor_msgs::Imu> msgQueue;
    std::queue <geometry_msgs::PointStamped> zMsgQueue;
    tf::StampedTransform comToIMU;
    tf::StampedTransform comToDS;
    tf::Transform comToWorld;

    KalmanFilter initializeKf ( std::string suffix );
    void initializeKfState ( KalmanFilter& kf, double state );
    void string2Matrix ( Eigen::MatrixXd& matrixOut, std::string str, std::string delim );

    KalmanFilter kfz;

};

#endif
