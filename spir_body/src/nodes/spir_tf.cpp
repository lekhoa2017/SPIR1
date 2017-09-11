#include "spir_body/dbscan.h"
#include "spir_body/distance.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include <iostream>
#include <cmath>
#include "ros/ros.h"
#include <boost/bind.hpp>
#include "tf/tf.h"
#include "tf/transform_listener.h"
#include <tf/transform_broadcaster.h>
#include <queue>
#include <iostream>
#include <boost/thread/thread.hpp>

/** @file spir_tf.cpp
*  @brief use imu/data; imu_payload/data; depth/data; depth_desired/data; sonar/scan to publish
*  transform between body and world; between payload and world; between world and waterline; between world and desired depth
*
*  @author Wenjie Lu
*  @date 19 Oct 2016
*
*  @bug
*  @todo Currently no todos
*/

// forward declearation for callback functions
void cb_imu ( const sensor_msgs::Imu::ConstPtr& msg );
void cb_imu2 ( const sensor_msgs::Imu::ConstPtr& msg );
void cb_depth ( const std_msgs::Float32MultiArray::ConstPtr& msg );
void cb_depth_desired ( const std_msgs::Float32MultiArray::ConstPtr& msg );
void cb_sonar ( const sensor_msgs::PointCloud::ConstPtr& msg );

void retrieveRotMat();
tf::Quaternion getCOMOrientation();
tf::Quaternion getArmsOrientation();


sensor_msgs::Imu imu_msg;
sensor_msgs::Imu imu2_msg;

tf::StampedTransform comToIMU;		// tf
tf::StampedTransform comToLidar;
tf::StampedTransform armToIMU2;
tf::StampedTransform comToSonar;
tf::StampedTransform comToDepthSensor;

std::queue<std_msgs::Float32MultiArray> depthMsgQueue;
double intensity_cut;
double min_dist;

double seabed;
bool worldReady;
bool worldAligned;
double worldYaw;
double worldYawDiff;

int main ( int argc, char **argv )
{
    ros::init ( argc, argv, "spir_tf" );
    ros::NodeHandle n;
    ros::NodeHandle nh ( "~" );
    nh.param ( "intensity_cut", intensity_cut, 110.0); //Sonar intensity threshold
    nh.param ( "min_dist", min_dist, 0.15); //Sonar physical distance threshold
    seabed = 0.0;
    ros::Subscriber imuSub = n.subscribe<sensor_msgs::Imu> ( "imu/data", 1, cb_imu );
    ros::Subscriber imu2Sub = n.subscribe<sensor_msgs::Imu> ( "imu_payload/data", 1, cb_imu2 );
    ros::Subscriber depthSub = n.subscribe<std_msgs::Float32MultiArray> ( "depth", 1, cb_depth );
    ros::Subscriber depth_desiredSub = n.subscribe<std_msgs::Float32MultiArray> ( "depth_desired", 1, cb_depth_desired );
    ros::Subscriber sonarSub = n.subscribe<sensor_msgs::PointCloud> ( "SONAR/scan", 1, cb_sonar );

    worldReady=false;
    worldAligned=false;
    worldYawDiff = 0.0;
    ros::Rate loop_rate ( 10 );
    retrieveRotMat();
    while ( ros::ok() ) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void cb_imu ( const sensor_msgs::Imu::ConstPtr& msg )
// callback function for touchpad message
{
    imu_msg =  *msg;

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin ( tf::Vector3 ( 0, 0, 0.0 ) );

    // transformation from inertial to IMU
    tf::Quaternion quat = getCOMOrientation();
    double roll, pitch, yaw;
   tf::Matrix3x3 ( quat.inverse()).getRPY ( roll, pitch, yaw );

    //ROS_INFO_STREAM("roll, pitch, yaw"<< roll <<"   "<<pitch <<"   "<<yaw);
    
    
    if (!worldAligned)
    {
      worldReady = true;
      worldYaw = yaw;
    }
    
    transform.setRotation ( quat );
    br.sendTransform ( tf::StampedTransform ( transform, ros::Time::now(), "COM", "world" ) );

}

void cb_imu2 ( const sensor_msgs::Imu::ConstPtr& msg )
// callback function for touchpad message
{
    imu2_msg =  *msg;
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin ( tf::Vector3 ( 0, 0, 0) );

    // transformation from inertial to IMU
    tf::Quaternion quat = getArmsOrientation();
    // get optimal roational axis

    double roll, pitch, yaw;
    tf::Matrix3x3 ( quat ).getRPY ( roll, pitch, yaw );
    
    if(worldReady&&!worldAligned)
    {
      worldAligned=true;
      worldYawDiff = worldYaw-yaw;
    }
    
    tf::Quaternion    quatTFy = tf::createQuaternionFromYaw ( worldYawDiff );
    quatTFy.normalize();
    
    tf::Quaternion quatTFgoal;

    quatTFgoal = quat*quatTFy;
    
    transform.setRotation ( quatTFgoal );
    
    br.sendTransform ( tf::StampedTransform ( transform.inverse(), ros::Time::now(), "world", "Arms" ) );
}

void cb_depth ( const std_msgs::Float32MultiArray::ConstPtr& msg )
// broadcast the waterline frame; need to convert raw depth readings into COM frame first
{
    static tf::TransformBroadcaster br;
    tf::Vector3 point;
    point[0] = 0;
    point[1] = 0;
    point[2] = msg->data[2]; //raw depth readings
    point = tf::Transform(getCOMOrientation().inverse().normalize())*(comToDepthSensor*point);
    tf::Transform transform;
    //transform.setOrigin ( tf::Vector3 ( 0, 0, msg->data[2]-0.2 ) ); //Manually tune the visualisation to display with 25cm offset
    transform.setOrigin ( tf::Vector3 ( 0, 0, point[2]-0.3 ) );
    tf::Quaternion q;
    q.setRPY(0,0,0);
    transform.setRotation(q);
    br.sendTransform ( tf::StampedTransform ( transform, ros::Time::now(), "world", "waterline" ) );
}

void cb_depth_desired ( const std_msgs::Float32MultiArray::ConstPtr& msg )
// callback function for touchpad message
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin ( tf::Vector3 ( 0, 0, msg->data[0] ) ); //Manually tune the visualisation to display with 15cm offset
    tf::Quaternion q;
    q.setRPY(0,0,0);
    transform.setRotation(q);
    br.sendTransform ( tf::StampedTransform ( transform, ros::Time::now(), "world", "waterline_desired" ) );
}
void cb_sonar ( const sensor_msgs::PointCloud::ConstPtr& msg )
// callback function for touchpad message
{
    double max_intensity=intensity_cut;
    int indx=-1;
    tf::Vector3 point;
    for (unsigned int i=0;i<msg->points.size();i++)
    {
      // check the intensity
      if (msg->channels[0].values[i]<intensity_cut)
	continue;

      if (sqrt(pow(msg->points[i].x,2.0)+pow(msg->points[i].y,2.0))<min_dist)
	continue;

      if (max_intensity < msg->channels[0].values[i])
      {
        max_intensity = msg->channels[0].values[i];
        indx=i;
      }else{break;}

      if (indx>0)
      {
        point[0] = msg->points[indx].x;
        point[1] = msg->points[indx].y;
        point[2] = 0.0;

        ROS_INFO_STREAM("point"<<" "<<point[0]<<" "<<point[1]);
        point = tf::Transform(getCOMOrientation().inverse().normalize())*(comToSonar*point);

        std_msgs::Float32MultiArray msg_TransformedSonar;
        msg_TransformedSonar.data.resize(4);
        msg_TransformedSonar.data[2] = point[2];

        if (depthMsgQueue.size()<2)
        {
          depthMsgQueue.push(msg_TransformedSonar);
        }else
        {
          depthMsgQueue.pop();
          depthMsgQueue.push(msg_TransformedSonar);
        }

}
}
        if (depthMsgQueue.size()>0)
        {
            double z = depthMsgQueue.front().data[2]+depthMsgQueue.back().data[2];
            z = z/2.0;
            seabed = seabed*0.1+0.9*z;
            ROS_INFO_STREAM("seabed value"<<" "<<seabed);
            static tf::TransformBroadcaster br;
            tf::Transform transform;
            transform.setOrigin ( tf::Vector3 ( 0, 0, seabed ) ); //Negative to show it visually below the robot in RVIZ
            tf::Quaternion q;
            q.setRPY(0,0,0);
            transform.setRotation(q);
            br.sendTransform ( tf::StampedTransform ( transform, ros::Time::now(), "world", "seabed" ) );
        }


}

tf::Quaternion getArmsOrientation()
{
    tf::Quaternion quatOut;


    sensor_msgs::Imu imuMsg = imu2_msg;

    tf::Quaternion quat;
    quat.setX ( imuMsg.orientation.y );
    quat.setY ( imuMsg.orientation.z );
    quat.setZ ( imuMsg.orientation.w );
    quat.setW ( imuMsg.orientation.x );


    tf::Quaternion quat_inv = quat.inverse();

    // Rename and invert the axes via Euler.

    tf::Vector3 axis = quat_inv.getAxis();
    double angle = quat_inv.getAngle();
    axis[0] = - axis[0];
    axis[2] = - axis[2];

    tf::Quaternion quat_right_inv ( axis,angle );
    // If the model has been turned around (180º) then invert the rotations around the Y axis

    // Convert the Euler angles back to Quaternion

    quatOut = armToIMU2 * quat_right_inv;
    quatOut.normalize();

    return quatOut;
}

tf::Quaternion getCOMOrientation()
{
    tf::Quaternion quatOut;


    sensor_msgs::Imu imuMsg = imu_msg;

    tf::Quaternion quat;
    quat.setX ( imuMsg.orientation.y );
    quat.setY ( imuMsg.orientation.z );
    quat.setZ ( imuMsg.orientation.w );
    quat.setW ( imuMsg.orientation.x );


    tf::Quaternion quat_inv = quat.inverse();

    // Rename and invert the axes via Euler.

    tf::Vector3 axis = quat_inv.getAxis();
    double angle = quat_inv.getAngle();
    axis[0] = - axis[0];
    axis[2] = - axis[2];

    tf::Quaternion quat_right_inv ( axis,angle );
    // If the model has been turned around (180º) then invert the rotations around the Y axis

    // Convert the Euler angles back to Quaternion

    quatOut = comToIMU * quat_right_inv;
    quatOut.normalize();

    return quatOut;
}



void retrieveRotMat()
// Eventually will get rotation matrix from urdf
{
    tf::TransformListener listener;

    while ( !listener.canTransform ( "COM", "IMU", ros::Time ( 0 ) ) && ros::ok() ) {
        ros::Duration ( 1 ).sleep();
        if ( listener.canTransform ( "COM", "IMU", ros::Time ( 0 ) ) ) {
            break;
        }
        ros::Duration ( 1 ).sleep();
        if ( listener.canTransform ( "COM", "IMU", ros::Time ( 0 ) ) ) {
            break;
        }
        ros::Duration ( 1 ).sleep();
        if ( listener.canTransform ( "COM", "IMU", ros::Time ( 0 ) ) ) {
            break;
        }
        ros::Duration ( 1 ).sleep();
        ROS_INFO ( "Cannot retrieve TF's! Ensure that the TF publisher\
			node is running" );
    }
    listener.lookupTransform ( "COM", "IMU", ros::Time ( 0 ), comToIMU );


    while ( !listener.canTransform ( "COM", "laser", ros::Time ( 0 ) ) && ros::ok() ) {
        ros::Duration ( 1 ).sleep();
        if ( listener.canTransform ( "COM", "laser", ros::Time ( 0 ) ) ) {
            break;
        }
        ros::Duration ( 1 ).sleep();
        if ( listener.canTransform ( "COM", "laser", ros::Time ( 0 ) ) ) {
            break;
        }
        ros::Duration ( 1 ).sleep();
        if ( listener.canTransform ( "COM", "laser", ros::Time ( 0 ) ) ) {
            break;
        }
        ros::Duration ( 1 ).sleep();
        ROS_INFO ( "Cannot retrieve TF's! Ensure that the TF publisher\
			node is running" );
    }
    listener.lookupTransform ( "COM", "laser", ros::Time ( 0 ), comToLidar );

    while ( !listener.canTransform ( "Arms", "IMU2", ros::Time ( 0 ) ) && ros::ok() ) {
        ros::Duration ( 1 ).sleep();
        if ( listener.canTransform ( "Arms", "IMU2", ros::Time ( 0 ) ) ) {
            break;
        }
        ros::Duration ( 1 ).sleep();
        if ( listener.canTransform ( "Arms", "IMU2", ros::Time ( 0 ) ) ) {
            break;
        }
        ros::Duration ( 1 ).sleep();
        if ( listener.canTransform ( "Arms", "IMU2", ros::Time ( 0 ) ) ) {
            break;
        }
        ros::Duration ( 1 ).sleep();
        ROS_INFO ( "Cannot retrieve TF's! Ensure that the TF publisher\
			node is running" );
    }
    listener.lookupTransform ( "Arms", "IMU2", ros::Time ( 0 ), armToIMU2 );

        while ( !listener.canTransform ( "COM", "sonar", ros::Time ( 0 ) ) && ros::ok() ) {
        ros::Duration ( 1 ).sleep();
        if ( listener.canTransform ( "COM", "sonar", ros::Time ( 0 ) ) ) {
            break;
        }
        ros::Duration ( 1 ).sleep();
        if ( listener.canTransform ( "COM", "sonar", ros::Time ( 0 ) ) ) {
            break;
        }
        ros::Duration ( 1 ).sleep();
        if ( listener.canTransform ( "COM", "sonar", ros::Time ( 0 ) ) ) {
            break;
        }
        ros::Duration ( 1 ).sleep();
        ROS_INFO ( "Cannot retrieve TF's! Ensure that the TF publisher\
			node is running" );
    }
    listener.lookupTransform ( "COM", "sonar", ros::Time ( 0 ), comToSonar );

    while ( !listener.canTransform ( "COM", "DS", ros::Time ( 0 ) ) && ros::ok() ) {
        ros::Duration ( 1 ).sleep();
        if ( listener.canTransform ( "COM", "DS", ros::Time ( 0 ) ) ) {
            break;
        }
        ros::Duration ( 1 ).sleep();
        if ( listener.canTransform ( "COM", "DS", ros::Time ( 0 ) ) ) {
            break;
        }
        ros::Duration ( 1 ).sleep();
        if ( listener.canTransform ( "COM", "DS", ros::Time ( 0 ) ) ) {
            break;
        }
        ros::Duration ( 1 ).sleep();
        ROS_INFO ( "Cannot retrieve TF's! Ensure that the TF publisher\
			node is running" );
    }
    listener.lookupTransform ( "COM", "DS", ros::Time ( 0 ), comToDepthSensor );

}
