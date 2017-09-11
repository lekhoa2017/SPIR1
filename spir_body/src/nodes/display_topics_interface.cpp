#include "ros/ros.h"
#include "spir_body/ThrustStamped.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Bool.h"
#include <visualization_msgs/Marker.h>
#include "nav_msgs/Path.h"
#include "geometry_msgs/PointStamped.h"
#include <sstream>
#include "spir_body/PileStateStamped.h"
#include "geometry_msgs/WrenchStamped.h"
#include "Eigen/Dense"
#include "spir_body/calc_coeff_matrix.h"

/** @file display_topics_interface.cpp
 *  @brief ROS node for visulization in RVIZ.
 *  1. thruster forces
 *  2. auto navigation mode through display a line between targeted pile and the robot body
 *
 *
 *  @author Wenjie Lu
 *  @contact wenjie.lu@uts.edu.au
 *  @date 14/09/2016
 *  @version 1.0.0
 *  @bug Currently no known bugs.
 *  @todo May include more topics to display
 *
 */

// Global variables
ros::Publisher pub_thrust_marker;
ros::Publisher pub_line2target;
ros::Publisher pub_targeted_pile_pos;
ros::Publisher pub_merged_force;
double arrow_scale;
double line_width;
bool isAutoNav;

Eigen::Matrix<double,6,8> coeffMat;

/**
 * @brief ThrusterStamped callback function
 *
 * This function publishes markers corresponding to each thruster's force output for visualization in RVIZ.
 *
 * @param msgIn Pointer to ThrustStamped message containing thrust info
 */
void cb_joy_thrust ( const spir_body::ThrustStamped::ConstPtr& msgIn )
{

    spir_body::ThrustStamped thrust;
    thrust = *msgIn;
    // Create marker
    visualization_msgs::Marker marker;
    marker.header.stamp = thrust.header.stamp;
    marker.ns = "thrust";
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    marker.scale.x = 0.030; // Change this to set arrow diameter
    marker.scale.y = 0.060; // Arrow head diameter
    marker.scale.z = 0; // Arrow head length (0=default)
    marker.color.a = 0.75;
    marker.color.r = 255/255;
    marker.color.g =   0/255;
    marker.color.b = 255/255;

    geometry_msgs::Point pt0, pt1;
    pt0.x = 0;
    pt0.y = 0;
    pt0.z = 0;
    pt1.x = 0;
    pt1.y = 0;
    pt1.z = 1;

    for ( int i=0; i<8; i++ ) {
        marker.header.frame_id = thrust.name[i];
        pt1.z = thrust.thrust[i] * arrow_scale;
        marker.points.clear();
        marker.points.push_back ( pt0 );
        marker.points.push_back ( pt1 );
        marker.id = i;
        pub_thrust_marker.publish ( marker );
    }
    return;
}


/**
 * @brief ThrusterStamped callback function
 *
 * This function publishes markers corresponding to each thruster's force output for visualization in RVIZ.
 *
 * @param msgIn Pointer to ThrustStamped message containing thrust info
 */
void cb_thruster_commands_out ( const spir_body::ThrustStamped::ConstPtr&  msgIn )
{


    // convert into Eigen::matrix format
    Eigen::Matrix<double,8,1> thrusters_forces;
    //ROS_INFO_STREAM(msg1.thrust.size());
    for ( unsigned int i=0; i<8; i++ ) {
        thrusters_forces ( i,0 ) = msgIn->thrust[i];
    }

    // convert thruster forces back to generalized forces
    Eigen::Matrix<double,6,1> generalized_forces;
    generalized_forces = coeffMat*thrusters_forces;


    geometry_msgs::WrenchStamped gf;
    gf.header = msgIn->header;
    gf.header.frame_id = "COM";
    gf.wrench.force.x = generalized_forces(0,0);
    gf.wrench.force.y = generalized_forces(1,0);
    gf.wrench.force.z = generalized_forces(2,0);
    gf.wrench.torque.x = generalized_forces(3,0);
    gf.wrench.torque.y = generalized_forces(4,0);
    gf.wrench.torque.z = generalized_forces(5,0);
    
    pub_merged_force.publish(gf);

}


/**
 * @brief PointStamped callback function
 * Publish path message to drew a line between the lidar center and targeted pile in rviz for visualization
 *
 * @param msgIn pointer to PointStamped message with information about targeted position
 */
void cb_pile_state ( const geometry_msgs::PointStamped::ConstPtr msgIn )
{
    geometry_msgs::PointStamped pile_state;
    pile_state = *msgIn;

    if ( !isAutoNav ) {
        pile_state.point.x = 0;
        pile_state.point.y = 0;
        pile_state.point.z = 0;
    }


    nav_msgs::Path path;
    path.header = pile_state.header;
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;
    path.poses.push_back ( pose );

    pose.pose.position.x = pile_state.point.x;
    pose.pose.position.y = pile_state.point.y;
    pose.pose.position.z = pile_state.point.z;
    path.poses.push_back ( pose );

    pub_line2target.publish ( path );
    return;
}

/**
 * @brief PointStamped callback function
 * Publish path message to drew a line between the lidar center and targeted pile in rviz for visualization
 *
 * @param msgIn pointer to PointStamped message with information about targeted position
 */
void cb_pile_state_full ( const spir_body::PileStateStamped::ConstPtr msgIn )
{
    spir_body::PileStateStamped pile_state;
    pile_state = *msgIn;

    if ( !isAutoNav ) {
        pile_state.point.x = 0;
        pile_state.point.y = 0;
        pile_state.point.z = 0;
    }


    nav_msgs::Path path;
    path.header = pile_state.header;
    path.header.frame_id = "world";
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;
    path.poses.push_back ( pose );

    pose.pose.position.x = pile_state.projection.x;
    pose.pose.position.y = pile_state.projection.y;
    pose.pose.position.z = 0;
    path.poses.push_back ( pose );

    pub_targeted_pile_pos.publish ( path );

    nav_msgs::Path path_1;


    path_1.header = pile_state.header;
    path_1.header.frame_id = "world";
    pose.pose.position.x = 0.1;
    pose.pose.position.y = 0.1;
    pose.pose.position.z = 0;
    path_1.poses.push_back ( pose );

    if ( !isAutoNav ) {
        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        pose.pose.position.z = 0;
    } else {
        pose.pose.position.x = pile_state.projection.x;
        pose.pose.position.y = pile_state.projection.y;
        pose.pose.position.z = 0;
    }
    path_1.poses.push_back ( pose );

    pub_line2target.publish ( path_1 );

    return;
}

/**
 * @brief Bool callback function
 *
 * This function
 *
 * @param msgIn Pointer to Bool message containing navigation mode
 */
void cb_autonavigation ( const std_msgs::Bool::ConstPtr msgIn )
{
    isAutoNav = msgIn->data;

    //ROS_INFO_STREAM ("mode " << isAutoNav );
    return;
}

/**
 * @brief ROS node main function
 */
int main ( int argc, char **argv )
{

    /* Initialise node */
    ros::init ( argc, argv, "display_topics_interface" );
    ros::NodeHandle n;
    ros::NodeHandle nh ( "~" );

    nh.param<double> ( "arrow_scale", arrow_scale, 0.1 );
    nh.param<double> ( "line_width",line_width, 0.3 );
    isAutoNav = false;

    /* subscriber */
    ros::Subscriber sub_joy_thrust = n.subscribe ( "thrust_command_joy", 1, cb_joy_thrust );
    //ros::Subscriber sub_targeted_pile = n.subscribe ( "pile_state", 1, cb_pile_state );
    ros::Subscriber sub_autonavigatoin = n.subscribe ( "autonavigation", 1, cb_autonavigation );
    ros::Subscriber sub_pile_state_full = n.subscribe ( "pile_state_full", 1, cb_pile_state_full );
    ros::Subscriber sub_thruster_commands_out = n.subscribe ( "thruster_commands_out",1,cb_thruster_commands_out );


    /* publisher */
    pub_thrust_marker = n.advertise<visualization_msgs::Marker> ( "thrust_visualization", 1 );
    pub_targeted_pile_pos = n.advertise<nav_msgs::Path> ( "targeted_pile_pos", 1 );
    pub_line2target = n.advertise<nav_msgs::Path> ( "ray_to_targeted_pile", 1 );
    pub_merged_force = n.advertise<geometry_msgs::WrenchStamped> ( "thruster_commands_out_wrench", 1 );


    CalcCoeffMatrix myObj;
    coeffMat = myObj.getCoeffMat();


    ros::spin();
    return 0;
}


