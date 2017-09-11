#ifndef KINEMATICS_H
#define KINEMATICS_H
#include "Eigen/Dense"
#include "tf/tf.h"

tf::Vector3 rot2rpy(tf::Matrix3x3 R);					// Convert a rotation matrix to vector of roll, pitch, yaw angles

Eigen::Matrix<double,3,3> rpy2rot(Eigen::Matrix<double,3,1> eta2);

#endif