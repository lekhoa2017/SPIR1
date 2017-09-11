#include "spir_body/calc_coeff_matrix.h"

CalcCoeffMatrix::CalcCoeffMatrix()
{
    extractRotMat();
    formCoeffMatrix();
}

void CalcCoeffMatrix::formCoeffMatrix()
{
    /* Enter force vectors of thrusters in the COM frame into the coefficient
       matrix
       */
    coeffMat.block<3,1>(0,0) = comToT1RotMat.block<3,1>(0,2);
    coeffMat.block<3,1>(0,1) = comToT2RotMat.block<3,1>(0,2);
    coeffMat.block<3,1>(0,2) = comToT3RotMat.block<3,1>(0,2);
    coeffMat.block<3,1>(0,3) = comToT4RotMat.block<3,1>(0,2);
    coeffMat.block<3,1>(0,4) = comToT5RotMat.block<3,1>(0,2);
    coeffMat.block<3,1>(0,5) = comToT6RotMat.block<3,1>(0,2);
    coeffMat.block<3,1>(0,6) = comToT7RotMat.block<3,1>(0,2);
    coeffMat.block<3,1>(0,7) = comToT8RotMat.block<3,1>(0,2);

    /* Enter the moment vectors of thrusters in the COM frame into the
       coefficient matrix
       */
    coeffMat.block<3,1>(3,0) = comToT1PosVec.cross(comToT1RotMat.block<3,1>(0,2));
    coeffMat.block<3,1>(3,1) = comToT2PosVec.cross(comToT2RotMat.block<3,1>(0,2));
    coeffMat.block<3,1>(3,2) = comToT3PosVec.cross(comToT3RotMat.block<3,1>(0,2));
    coeffMat.block<3,1>(3,3) = comToT4PosVec.cross(comToT4RotMat.block<3,1>(0,2));
    coeffMat.block<3,1>(3,4) = comToT5PosVec.cross(comToT5RotMat.block<3,1>(0,2));
    coeffMat.block<3,1>(3,5) = comToT6PosVec.cross(comToT6RotMat.block<3,1>(0,2));
    coeffMat.block<3,1>(3,6) = comToT7PosVec.cross(comToT7RotMat.block<3,1>(0,2));
    coeffMat.block<3,1>(3,7) = comToT8PosVec.cross(comToT8RotMat.block<3,1>(0,2));
}




// Extract the rotation matrices for all the transforms required from the URDF TF's
void CalcCoeffMatrix::extractRotMat()
{
    // Initialise transforms
    tf::StampedTransform comToT1;
    tf::StampedTransform comToT2;
    tf::StampedTransform comToT3;
    tf::StampedTransform comToT4;
    tf::StampedTransform comToT5;
    tf::StampedTransform comToT6;
    tf::StampedTransform comToT7;
    tf::StampedTransform comToT8;

    /* Checks every second if it has retrieved the TF. If after 4 seconds it
       still hasn't, print an error message. NOTE. If any non-fixed joints (e.g.
       a revolute joint) gets added, this loop needs to be implemented again for
       them.
       */
    while(!listener.canTransform("COM", "t8", ros::Time(0)) && ros::ok())
        {
        ros::Duration(1).sleep();
        if(listener.canTransform("COM", "t8", ros::Time(0)))
        break;
        ros::Duration(1).sleep();
        if(listener.canTransform("COM", "t8", ros::Time(0)))
        break;
        ros::Duration(1).sleep();
        if(listener.canTransform("COM", "t8", ros::Time(0)))
        break;
        ros::Duration(1).sleep();
        ROS_INFO("Cannot retrieve thruster TF's! Ensure that the TF publisher\
                  node is running");
        }

        // Once the transforms are available, store them
        listener.lookupTransform("COM", "t1", ros::Time(0), comToT1);
        listener.lookupTransform("COM", "t2", ros::Time(0), comToT2);
        listener.lookupTransform("COM", "t3", ros::Time(0), comToT3);
        listener.lookupTransform("COM", "t4", ros::Time(0), comToT4);
        listener.lookupTransform("COM", "t5", ros::Time(0), comToT5);
        listener.lookupTransform("COM", "t6", ros::Time(0), comToT6);
        listener.lookupTransform("COM", "t7", ros::Time(0), comToT7);
        listener.lookupTransform("COM", "t8", ros::Time(0), comToT8);



        // Extract quaternion from rotation
        comToT1RotMat = convQuatToRot(comToT1);
        comToT2RotMat = convQuatToRot(comToT2);
        comToT3RotMat = convQuatToRot(comToT3);
        comToT4RotMat = convQuatToRot(comToT4);
        comToT5RotMat = convQuatToRot(comToT5);
        comToT6RotMat = convQuatToRot(comToT6);
        comToT7RotMat = convQuatToRot(comToT7);
        comToT8RotMat = convQuatToRot(comToT8);



        // Calculate the distance between the origins of the 2 frames
        comToT1PosVec = calcRelativePosition(comToT1);
        comToT2PosVec = calcRelativePosition(comToT2);
        comToT3PosVec = calcRelativePosition(comToT3);
        comToT4PosVec = calcRelativePosition(comToT4);
        comToT5PosVec = calcRelativePosition(comToT5);
        comToT6PosVec = calcRelativePosition(comToT6);
        comToT7PosVec = calcRelativePosition(comToT7);
        comToT8PosVec = calcRelativePosition(comToT8);

        for(int i=0; i<3; ++i)
        {
            for(int j=0;j<3;++j)
            {
                if(std::abs(comToT1RotMat(i,j)) < 0.01)
                    comToT1RotMat(i,j) = 0;
                if(std::abs(comToT2RotMat(i,j)) < 0.01)
                    comToT2RotMat(i,j) = 0;
                if(std::abs(comToT3RotMat(i,j)) < 0.01)
                    comToT3RotMat(i,j) = 0;
                if(std::abs(comToT4RotMat(i,j)) < 0.01)
                    comToT4RotMat(i,j) = 0;
                if(std::abs(comToT5RotMat(i,j)) < 0.01)
                    comToT5RotMat(i,j) = 0;
                if(std::abs(comToT6RotMat(i,j)) < 0.01)
                    comToT6RotMat(i,j) = 0;
                if(std::abs(comToT7RotMat(i,j)) < 0.01)
                    comToT7RotMat(i,j) = 0;
                if(std::abs(comToT8RotMat(i,j)) < 0.01)
                    comToT8RotMat(i,j) = 0;
            }
        }
}




Eigen::Matrix<double,3,3> CalcCoeffMatrix::convQuatToRot(tf::StampedTransform quatIn)
{
    Eigen::Matrix<double,3,3> rotMat;

    rotMat(0,0) = 1 - 2 * pow(quatIn.getRotation().y(), 2) - 2
                  * pow(quatIn.getRotation().z(), 2);
    rotMat(0,1) = 2 * quatIn.getRotation().x() * quatIn.getRotation().y()
                  - 2 * quatIn.getRotation().z() * quatIn.getRotation().w();
    rotMat(0,2) = 2 * quatIn.getRotation().x() * quatIn.getRotation().z() + 2
                  * quatIn.getRotation().y() * quatIn.getRotation().w();
    rotMat(1,0) = 2 * quatIn.getRotation().x() * quatIn.getRotation().y() + 2
                  * quatIn.getRotation().z() * quatIn.getRotation().w();
    rotMat(1,1) = 1 - 2 * pow(quatIn.getRotation().x(), 2) - 2
                  * pow(quatIn.getRotation().z(), 2);
    rotMat(1,2) = 2 * quatIn.getRotation().y() * quatIn.getRotation().z()
                  - 2 * quatIn.getRotation().x() * quatIn.getRotation().w();
    rotMat(2,0) = 2 * quatIn.getRotation().x() * quatIn.getRotation().z()
                  - 2 * quatIn.getRotation().y() * quatIn.getRotation().w();
    rotMat(2,1) = 2 * quatIn.getRotation().y() * quatIn.getRotation().z()
                  + 2 * quatIn.getRotation().x() * quatIn.getRotation().w();
    rotMat(2,2) = 1 - 2 * pow(quatIn.getRotation().x(), 2) - 2
                  * pow(quatIn.getRotation().y(), 2);

    return rotMat;
}



// Extract the distance between the origins of the relative frames from the URDF TF's
Eigen::Matrix<double,3,1> CalcCoeffMatrix::calcRelativePosition(
                                            tf::StampedTransform transformIn)
{
    Eigen::Matrix<double,3,1> relativePos;
    relativePos(0,0) = transformIn.getOrigin().x();
    relativePos(1,0) = transformIn.getOrigin().y();
    relativePos(2,0) = transformIn.getOrigin().z();

    return relativePos;
}

Eigen::Matrix<double,6,8> CalcCoeffMatrix::getCoeffMat()
{
    return coeffMat;

}
