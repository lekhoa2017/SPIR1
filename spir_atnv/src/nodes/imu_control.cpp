#include "spir_atnv/robust_integral_control.h"
#include "ros/ros.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"

/** @file imu_control.cpp
 *  @brief Calculates the required thruster commands to stabilise roll and
 *         pitch using a robust integral algorithm.
 *
 *  This node uses the RobIntControl class to calculate the stabilisation forces
 *  at the centre of mass required to keep the robot level about roll and pitch,
 *  based off data from the IMU.
 *
 *  It then converts these forces using the CalcCoeffMatrix class into
 *  individual thruster commands.
 *
 *  @author Brendan Emery
 *  @date Jan 2016
 *  @version 1.0.0
 *  @bug Currently no known bugs.
 *  @todo Currently no todos
 */

int main ( int argc, char **argv )
{
    ros::init ( argc, argv, "imu_control" );
    ros::NodeHandle n;
    ros::NodeHandle nh ( "~" );

    // Instantiate class object
    RobIntControl myObj ( &n, &nh );

    ros::Subscriber imu_sub = n.subscribe<sensor_msgs::Imu>
                              ( "imu/data", 1, &RobIntControl::callBack_imu,
                                &myObj );


    ros::Subscriber depth_sub = n.subscribe<std_msgs::Float32MultiArray>
                                ( "/depth", 1, &RobIntControl::callBack_z,
                                  &myObj );
    ros::Subscriber auto_mode_trigger_sub = n.subscribe<std_msgs::Bool>
                                            ( "/autonavigation", 1, &RobIntControl::callBack_atnv_trigger,
                                                    &myObj );
    
    ros::Subscriber depth_desired_sub = n.subscribe<std_msgs::Float32MultiArray>
                                            ( "/depth_desired", 1, &RobIntControl::callBack_z_d,
                                                    &myObj );
    ros::Subscriber joySub = n.subscribe<sensor_msgs::Joy> ( "joy_thruster", 1,
							     &RobIntControl::callBack_joy, &myObj );

    ros::Rate loop_rate ( 15 );

    while ( ros::ok() ) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}
