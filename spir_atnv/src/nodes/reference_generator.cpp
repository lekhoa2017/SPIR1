#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include <sensor_msgs/Joy.h>
/** @file reference_generator.cpp
 *  @brief Calculates the required thruster commands given robot state and desired robot state
 *
 *  @author Wenjie Lu
 *  @date 9/6/2017
 *  @version 1.0.0
 *  @bug Currently no known bugs.
 *  @todo Currently no todos
 */

// Global robot state message
std_msgs::Float32MultiArray robotState;

// Global robot state reference publisher
ros::Publisher stateReferencePub;

    double z_vel_d = 0;
    double r_vel_d = 0;
    double p_vel_d = 0; 
    double y_vel_d = 0;
    
    double z_d = 0;
    double r_d = 0;
    double p_d = 0;
    double y_d = 0; 

/**
* @brief Callback function to convert joystick commands into stamped wrench.
*
* @param joy Joy message subscribed to over joy topic that contains thruster
*        commands and other messages
*/
void cb_joy ( const sensor_msgs::Joy::ConstPtr& joy );

/**
* @brief Callback function to topic vel_desired.
*
* @param vel vel message subscribed to over std_msgs::Float32MultiArray topic that contains desired
* velocity in z, r, p, y.
*/
void cb_vel_desired ( const std_msgs::Float32MultiArray::ConstPtr& vel );

/**
* @brief Callback function to topic vel_desired.
*
* @param state state message subscribed to over std_msgs::Float32MultiArray topic that contains robot state
* pose, velocities, and acceleration
* : eta1, eta2, velB, omegaB, accB, alphaB
*/
void cb_body_state ( const std_msgs::Float32MultiArray::ConstPtr& state );

/**
 * @brief ROS node main function
 */
int main ( int argc, char **argv )
{
    ros::init ( argc, argv, "reference_generator" );
    ros::NodeHandle n;
    ros::NodeHandle hn ( "~" );

    // Set up publisher and subscriber
    ros::Subscriber joySub = n.subscribe<sensor_msgs::Joy> ( "joy_thruster", 1, cb_joy );
    ros::Subscriber velDesiredSub = n.subscribe<std_msgs::Float32MultiArray> ( "vel_desired_joy", 1, cb_vel_desired );
    ros::Subscriber stateSub = n.subscribe<std_msgs::Float32MultiArray> ( "robot_state_body", 1, cb_body_state );

    stateReferencePub = n.advertise<std_msgs::Float32MultiArray> ( "body_state_desired", 1 );
    // Enter eternal depths of ROS spin

    ros::spin();
}

void cb_joy ( const sensor_msgs::Joy::ConstPtr& joy )
{
}

bool isInBound ( double value, double bound )
{
    bound = std::fabs ( bound );
    value = std::fabs ( value );
    if ( value>bound )
        return false;
    else
        return true;
}

void cb_vel_desired ( const std_msgs::Float32MultiArray::ConstPtr& vel )
{
    z_vel_d = vel->data[0];
    r_vel_d = vel->data[1];
    p_vel_d = vel->data[2];
    y_vel_d = vel->data[3];

    
    if ( isInBound ( z_vel_d,0.01 ) )
    {
        z_vel_d = 0;
    }
    else
    {
        z_d = robotState.data[2];
    }


    if ( isInBound ( p_vel_d,0.01 ) )
    {
        p_vel_d = 0;
    }
    else
    {
        p_d = robotState.data[4];
    }

    if ( isInBound ( r_vel_d,0.01 ) )
    {
        r_vel_d = 0;
    }
    else
    {
        r_d = robotState.data[3];
    }

    if ( isInBound ( y_vel_d,0.01 ) )
    {
        y_vel_d = 0;
    }
    else
    {
        y_d = robotState.data[5];
    }
    
    std_msgs::Float32MultiArray state_reference_msg;
    state_reference_msg.data.resize(18);
    state_reference_msg.data[0] = 0;
    state_reference_msg.data[1] = 0;
    state_reference_msg.data[2] = z_d;
    state_reference_msg.data[3] = r_d;
    state_reference_msg.data[4] = p_d;
    state_reference_msg.data[5] = y_d;
    state_reference_msg.data[6] = 0;
    state_reference_msg.data[7] = 0;
    state_reference_msg.data[8] = z_vel_d*2;
    state_reference_msg.data[9] = r_vel_d;    
    state_reference_msg.data[10] = p_vel_d;
    state_reference_msg.data[11] = y_vel_d*2;
    state_reference_msg.data[12] = 0;
    state_reference_msg.data[13] = 0;
    state_reference_msg.data[14] = 0;
    state_reference_msg.data[15] = 0;
    state_reference_msg.data[16] = 0;
    state_reference_msg.data[17] = 0;
    
    stateReferencePub.publish(state_reference_msg);
}

void cb_body_state ( const std_msgs::Float32MultiArray::ConstPtr& state )
{
	robotState = *state;
}
