#ifndef CALC_COEFF_MATRIX_H
#define CALC_COEFF_MATRIX_H

#include "Eigen/Dense"
#include "ros/ros.h"
#include "spir_body/calc_coeff_matrix.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"

/** @class calc_coeff_matrix.h
 *  @brief Calculates the coefficient matrix based on a given transform listener.
 *
 *
 *
 *  @author Brendan Emery
 *  @contact Brendan.emery@student.uts.edu.au
 *  @date 27/10/2015
 *  @version 1.0.0
 *  @bug Currently the no known bugs.
 *  @todo Currently no new todo's
 */

 class CalcCoeffMatrix
 {
    public:
        /**
         * @brief Constructor
         *
         * @param tf::TransformListener used to get information from urdf via
         *        robot_state_publisher.
         */
        CalcCoeffMatrix();

        /**
         * @brief Accesses the coefficient matrix used to convert forces at the
         *        centre of mass to thruster forces.
         *
         * @return Returns a 6x8 eigen matrix containing the coefficient matrix.
         */
        Eigen::Matrix<double,6,8> getCoeffMat();

    private:
        ////////////////////// Class Functions ////////////////////////

        // Forms coefficient matrix based on values in the URDF
        void formCoeffMatrix();

        // Extracts rotation matrices for all required transforms from urdf
        void extractRotMat();

        /* Extract the distance between the origins of the relative frames from
        the URDF TF's
        */
        Eigen::Matrix<double,3,1> calcRelativePosition(tf::StampedTransform);

        /* Converts the quaternion stored in the StampedTransform into a
        rotation matrix */
        Eigen::Matrix<double,3,3> convQuatToRot(tf::StampedTransform);

        ////////////////////// Class Variables /////////////////////////

        /* Rotation matrices from centre of geometry frame to individual
           thruster frame */
        Eigen::Matrix<double,3,3> comToT1RotMat;
        Eigen::Matrix<double,3,3> comToT2RotMat;
        Eigen::Matrix<double,3,3> comToT3RotMat;
        Eigen::Matrix<double,3,3> comToT4RotMat;
        Eigen::Matrix<double,3,3> comToT5RotMat;
        Eigen::Matrix<double,3,3> comToT6RotMat;
        Eigen::Matrix<double,3,3> comToT7RotMat;
        Eigen::Matrix<double,3,3> comToT8RotMat;

        // Position vector from COM to thruster in the COM frame
        Eigen::Matrix<double,3,1> comToT1PosVec;
        Eigen::Matrix<double,3,1> comToT2PosVec;
        Eigen::Matrix<double,3,1> comToT3PosVec;
        Eigen::Matrix<double,3,1> comToT4PosVec;
        Eigen::Matrix<double,3,1> comToT5PosVec;
        Eigen::Matrix<double,3,1> comToT6PosVec;
        Eigen::Matrix<double,3,1> comToT7PosVec;
        Eigen::Matrix<double,3,1> comToT8PosVec;

        Eigen::Matrix<double,6,8> coeffMat;
        tf::TransformListener listener;
};

#endif // CALC_COEFF_MATRIX_H

