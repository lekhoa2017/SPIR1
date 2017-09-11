#include "spir_atnv/kinematics.h"
#include <ros/ros.h>
#include <tf/transform_listener.h>						
#include <cmath>								
#include "tf/tf.h"
/** @file	kinematics.cpp	
 *  @brief 	
 *
 *  @author 	Wenjie Lu
 *  @date 	08/06/2017
 *  @version 	1.0.0
 *  @bug 	No known bugs.
 *  @todo	 
 */

tf::Vector3 rot2rpy(tf::Matrix3x3 R)						// Convert a rotation matrix to vector of roll, pitch, yaw angles
{
	double roll = atan2(R[2][1],R[2][2]);
	double pitch;
	double yaw = atan2(R[1][0],R[0][0]);

	if(cos(yaw) == 0) pitch = atan2(-1*R[2][0],(R[1][0])/(sin(yaw)));
	else pitch = atan2(-1*R[2][0],(R[0][0])/(cos(yaw)));

	tf::Vector3 rpy;
	rpy.setValue(roll,pitch,yaw);
	
	return rpy;
}

Eigen::Matrix<double,3,3> rpy2rot(Eigen::Matrix<double,3,1> eta2)
{
	Eigen::Matrix<double,3,3> output;
	tf::Quaternion aQuat;
	aQuat.setRPY(eta2(0),eta2(1),eta2(2));
	tf::Transform aTF;
	aTF.setRotation(aQuat);
	tf::Matrix3x3 aRot = aTF.getBasis();
	
	for (int i=0;i<3;i++)
		for (int j=0;j<3;j++)
			output(i,j) = aRot[i][j];
	return output;
}
