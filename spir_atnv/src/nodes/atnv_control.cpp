#include "spir_atnv/robust_integral_control_full_dof.h"
#include "ros/ros.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/Int32.h"
#include "spir_atnv/PileStateStamped.h"
#include "std_msgs/Float32MultiArray.h"
/** @file atnv_control.cpp
 *  @brief Calculates the required thruster commands for the short distance
 *  autonomous navigation.
 *
 *  This node uses the RobIntControlFullDoF class to calculate forces and torques
 *  at the centre of mass required to autonomously navigate the robot toward a pile and lock
 *  based on data from the IMU and the pile position from message point_to_go.
 *
 *  It then converts these forces using the CalcCoeffMatrix class into
 *  individual thruster commands.
 *
 *  @author Wenjie Lu and Brendan Emery
 *  @date 2nd Sep 2016
 *  @version 1.0.0
 *  @bug Currently no known bugs.
 *  @todo Currently no todos
 */

int main ( int argc, char **argv )
{
    ros::init ( argc, argv, "autonomous_navigation" );
    ros::NodeHandle n;
    ros::NodeHandle nh ( "~" );

    // Instantiate class object
    RobIntControlFullDof myObj ( &n, &nh );

    ros::Subscriber xyGoal_sub = n.subscribe<geometry_msgs::PointStamped>
                                 ( "xyGoal", 1, &RobIntControlFullDof::callBack_xyGoal,
                                   &myObj );
    //ros::Subscriber xy_sub = n.subscribe<geometry_msgs::PointStamped>
    //                          ("xy", 1000, &RobIntControlFullDof::callBack_xy,
    //                           &myObj);
    ros::Subscriber full_sub = n.subscribe<spir_atnv::PileStateStamped>
                               ( "pile_state_full", 1, &RobIntControlFullDof::callBack_full,
                                 &myObj );

    ros::Subscriber z_sub = n.subscribe<std_msgs::Float32MultiArray>
                            ( "depth", 1, &RobIntControlFullDof::callBack_z,
                              &myObj );
    ros::Subscriber z_d_sub = n.subscribe<std_msgs::Float32MultiArray>
                            ( "depth_desired", 1, &RobIntControlFullDof::callBack_z_d,
                              &myObj );
    ros::Subscriber imu_sub = n.subscribe<sensor_msgs::Imu>
                              ( "imu/data", 1, &RobIntControlFullDof::callBack_imu,
                                &myObj );
			      
    ros::Subscriber auto_mode_trigger_sub = n.subscribe<std_msgs::Bool>
                                            ( "/autonavigation", 1, &RobIntControlFullDof::callBack_atnv_trigger,
                                                    &myObj );
			      
    ros::Rate loop_rate ( 10 );

    while ( ros::ok() ) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}
