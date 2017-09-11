#include "spir_atnv/model_do.h"
#include "spir_atnv/rise_pid.h"
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
/** @file atnv_control.cpp
 *  @brief Calculates the required thruster commands given robot state and desired robot state
 *
 *  @author Wenjie Lu
 *  @date 9/6/2017
 *  @version 1.0.0
 *  @bug Currently no known bugs.
 *  @todo Currently no todos
 */

int main ( int argc, char **argv )
{
    ros::init ( argc, argv, "model_diaturbance_observer_control" );
    ros::NodeHandle n;
    ros::NodeHandle nh ( "~" );

    // Instantiate class object
    ModelDO controller ( &n, &nh );

    ros::Subscriber state_desired_sub = n.subscribe<std_msgs::Float32MultiArray>
                                 ( "body_state_desired", 1, &Control::callBackStateDesired,
                                   dynamic_cast<Control*> (&controller) );
		 
    ros::Subscriber state_sub = n.subscribe<std_msgs::Float32MultiArray>
                               ( "robot_state_body", 1, &Control::callBackState,
                                 dynamic_cast<Control*> (&controller)  );
	      
    ros::Rate loop_rate ( 10 );

    while ( ros::ok() ) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

