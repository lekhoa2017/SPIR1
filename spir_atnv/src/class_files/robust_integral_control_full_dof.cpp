#include "spir_atnv/robust_integral_control_full_dof.h"
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/lexical_cast.hpp>
#include <nav_msgs/Odometry.h>
#include "Eigen/Geometry"

RobIntControlFullDof::RobIntControlFullDof ( ros::NodeHandle *n, ros::NodeHandle *nPrivate )
{
    ROS_INFO ( "\n" );
    n_ = *n;
    nh = *nPrivate;
    control_pub = n_.advertise<spir_atnv::ThrustStamped> ( "thrust_command_atnv", 2 );
    robot_status_pub = n_.advertise<nav_msgs::Odometry> ( "pile_state_kf",2 );

    /* Calculate and store the coefficient matrix used to convert forces at the
       COM to individual thruster forces
       */
    CalcCoeffMatrix myObj;
    coeffMat = myObj.getCoeffMat();

    // Retrieve the rotation matrix from the URDF and the parameters from the ROS paramater server
    retrieveRotMat();
    //ROS_INFO_STREAM ( rotMat_imu );

    retrieveParams(); // Change this to constantly check param server?

    //integral initializaton
    integralTranslate << 0, 0, 0;
    integralRotate << 0, 0, 0;

    //kalman filter initialization
    kfx = initializeKf ( "x" );
    kfy = initializeKf ( "y" );
    kfz = initializeKf ( "z" );
    initializeKfState ( kfx, 0.0 );
    initializeKfState ( kfy, 0.0 );
    initializeKfState ( kfz, 0.0 );
    status = 4;

}



void RobIntControlFullDof::callBack_imu ( const sensor_msgs::Imu::ConstPtr& msg )
{
    // Store the IMU data message in a queue for use in calculations
    storeMsgs ( *msg,imuMsgQueue,initialImuMsg );

    // update yawGoal since new message about orientation is availble
    setYawGoal();

    /* If the node has received more than just the first IMU reading, calculate
       control inputs and publish data.
       */
    if ( imuMsgQueue.size() == 2 ) {
        sensor_msgs::Imu previousMsg = imuMsgQueue.front();   // message(k)
        sensor_msgs::Imu currentMsg = imuMsgQueue.back();   // message(k+1)
        double dt = calcTimeStep ( previousMsg,currentMsg );

        //calculate generalized forces
        Eigen::Matrix<double,6,1> controlOutput;
        controlOutput = calcControlOutput();

        //publish
        spir_atnv::ThrustStamped thrustFrocesOutput;
        generalizedForces2ThrustersForces ( controlOutput,thrustFrocesOutput,std::string ( "atnvimu" ) );
        control_pub.publish ( thrustFrocesOutput );
    }

    /* If the queue size is 1, this indicates the node just received the first IMU reading. Calculate e(0)
        and then wait for next message to perform control calculations.
    */
    else if ( imuMsgQueue.size() == 1 ) {
        //initialE = calcE(initialMsg);

    }

    /* If the queue size is not 1 or 2, this indicates that the node is acting outside of the intended
       functionality
       */
    else {
        throw std::invalid_argument ( "Incorrect queue size IMU" );
    }
}

void RobIntControlFullDof::callBack_xy ( const geometry_msgs::PointStamped::ConstPtr& msg )
{
    // convert lidar data to COM frame
    tf::Vector3 msgInLidar;
    msgInLidar[0] = msg->point.x;
    msgInLidar[1] = msg->point.y;
    msgInLidar[2] = msg->point.z;
    tf::Vector3 msgInCOM;
    msgInCOM = comToLidar*msgInLidar;

    geometry_msgs::PointStamped msgUpdated;
    msgUpdated = *msg;
    msgUpdated.point.x = msgInCOM [0];
    msgUpdated.point.y = msgInCOM [1];
    msgUpdated.point.z = msgInCOM [2];

    // Store the lidar message in a queue for use in calculations
    storeMsgs ( msgUpdated,xyMsgQueue,initialXyMsg );

    // update yawGoal since new message about lidar is availble
    setYawGoal();

    /* If the node has received more than just the first IMU reading, calculate
    control inputs and publish data.
    */
    if ( xyMsgQueue.size() == 2 ) {
        geometry_msgs::PointStamped previousMsg = xyMsgQueue.front();   // message(k)
        geometry_msgs::PointStamped currentMsg = xyMsgQueue.back();   // message(k+1)
        double dt = calcTimeStep ( previousMsg,currentMsg );

        // update estimation of kalman filter
        Eigen::VectorXd x ( 1 );
        x << msgUpdated.point.x;
        kfx.update ( x,dt );

        // update estimation of kalman filter
        Eigen::VectorXd y ( 1 );
        y <<msgUpdated.point.y;
        kfy.update ( y,dt );

        //calculate generalized forces
        Eigen::Matrix<double,6,1> controlOutput;
        controlOutput = calcControlOutput();

        //publish control topic
        spir_atnv::ThrustStamped thrustFrocesOutput;
        generalizedForces2ThrustersForces ( controlOutput,thrustFrocesOutput,std::string ( "atnvxy" ) );
        control_pub.publish ( thrustFrocesOutput );


    }

    /* If the queue size is 1, this indicates the node just received the first IMU reading. Calculate e(0)
        and then wait for next message to perform control calculations.
    */
    else if ( xyMsgQueue.size() == 1 ) {

        // use the first lidar messge to initialize kalman filters
        initializeKfState ( kfx,msgUpdated.point.x );
        initializeKfState ( kfy,msgUpdated.point.y );
    }

    /* If the queue size is not 1 or 2, this indicates that the node is acting outside of the intended
       functionality
       */
    else {
        throw std::invalid_argument ( "Incorrect queue size xy" );
    }
}

void RobIntControlFullDof::callBack_xyGoal ( const geometry_msgs::PointStamped::ConstPtr& msg )
{
    if ( xyGoalQueue.size() ==0 ) {
        xyGoalQueue.push ( *msg );
    } else {
        xyGoalQueue.pop();
        xyGoalQueue.push ( *msg );

    }
}

void RobIntControlFullDof::callBack_z ( const std_msgs::Float32MultiArray::ConstPtr& msg )
{
    // Store the depth data message in a queue for use in calculations

      tf::Vector3 vect;
    vect = comToDS.getOrigin();
    
    ROS_INFO_STREAM("vect: "<<" "<<vect[0]<<" "<< vect[1]<<" "<< vect[2]);
    
    tf::Quaternion com2world_quat = getCOMOrientation();
    tf::Transform comToWorld(com2world_quat);
    tf::Vector3 dz = comToWorld.inverse()*vect;

    ROS_INFO_STREAM("dz: "<<dz[2]);
    
    geometry_msgs::PointStamped zMsg;
    zMsg.header.frame_id = "inertial";
    zMsg.header.stamp = ros::Time::now();
    zMsg.point.x = 0.0;
    zMsg.point.y = 0.0;
    zMsg.point.z = msg->data[2]+dz[2];
    
    storeMsgs (zMsg,zMsgQueue,initialZMsg );

    /* If the node has received more than just the first IMU reading, calculate
       control inputs and publish data.
       */
    if ( zMsgQueue.size() == 2 ) {
        geometry_msgs::PointStamped previousMsg = zMsgQueue.front();   // message(k)
        geometry_msgs::PointStamped currentMsg = zMsgQueue.back();   // message(k+1)
        double dt = calcTimeStep ( previousMsg,currentMsg );

        //update kalman filter
        Eigen::VectorXd z ( 1 );
        z << zMsg.point.z;
        kfz.update ( z,dt );
	
	ROS_INFO_STREAM("z: "<<kfz.state()(0));

        //calculate generalized forces
        Eigen::Matrix<double,6,1> controlOutput;
        controlOutput = calcControlOutput();
        spir_atnv::ThrustStamped thrustFrocesOutput;

        //publish topics
        generalizedForces2ThrustersForces ( controlOutput,thrustFrocesOutput,std::string ( "atnvz" ) );
        control_pub.publish ( thrustFrocesOutput );
    }

    /* If the queue size is 1, this indicates the node just received the first IMU reading. Calculate e(0)
        and then wait for next message to perform control calculations.
    */
    else if ( zMsgQueue.size() == 1 ) {
        //use the first depth sensor message to initialize filter
        Eigen::VectorXd z0 ( 2 );
        z0 ( 0 ) = zMsg.point.z ;
        z0 ( 1 ) = 0;
        kfz.init ( z0 );
    }

    /* If the queue size is not 1 or 2, this indicates that the node is acting outside of the intended
       functionality
       */
    else {
        throw std::invalid_argument ( "Incorrect queue size z" );
    }
}

void RobIntControlFullDof::callBack_full ( const spir_atnv::PileStateStamped::ConstPtr& msg )
{
    // convert lidar data to COM frame

    storeMsgs ( *msg,fullMsgQueue,initialFullMsg );

    status = msg->status;

    ROS_INFO_STREAM ( "status : "<<status );
    if ( status == 1 && imuMsgQueue.size()>0) {
        // convert lidar data to COM frame
        tf::Vector3 msgInLidar;
        msgInLidar[0] = msg->projection.x;
        msgInLidar[1] = msg->projection.y;
        msgInLidar[2] = msg->projection.z;


        msgInLidar = tf::Transform ( getCOMOrientation() ) *msgInLidar;

        geometry_msgs::PointStamped msgUpdated;
        msgUpdated.header = msg->header;
        msgUpdated.header.frame_id = "COM";
        msgUpdated.point.x = msgInLidar [0];
        msgUpdated.point.y = msgInLidar [1];
        msgUpdated.point.z = msgInLidar [2];

        // Store the lidar message in a queue for use in calculations
        storeMsgs ( msgUpdated,xyMsgQueue,initialXyMsg );

        // update yawGoal since new message about lidar is availble
        setYawGoal();

        /* If the node has received more than just the first IMU reading, calculate
        control inputs and publish data.
        */
        if ( xyMsgQueue.size() == 2 ) {
            geometry_msgs::PointStamped previousMsg = xyMsgQueue.front();   // message(k)
            geometry_msgs::PointStamped currentMsg = xyMsgQueue.back();   // message(k+1)
            double dt = calcTimeStep ( previousMsg,currentMsg );

            // update estimation of kalman filter
            Eigen::VectorXd x ( 1 );
            x << msgUpdated.point.x;
            kfx.update ( x,dt );

            // update estimation of kalman filter
            Eigen::VectorXd y ( 1 );
            y <<msgUpdated.point.y;
            kfy.update ( y,dt );

            //calculate generalized forces
            Eigen::Matrix<double,6,1> controlOutput;
            controlOutput = calcControlOutput();


            //publish control topic
            spir_atnv::ThrustStamped thrustFrocesOutput;
            generalizedForces2ThrustersForces ( controlOutput,thrustFrocesOutput,std::string ( "atnvfull" ) );
            control_pub.publish ( thrustFrocesOutput );
        }
        /* If the queue size is 1, this indicates the node just received the first IMU reading. Calculate e(0)
            and then wait for next message to perform control calculations.
        */
        else if ( xyMsgQueue.size() == 1 ) {

            // use the first lidar messge to initialize kalman filters
            initializeKfState ( kfx,msgUpdated.point.x );
            initializeKfState ( kfy,msgUpdated.point.y );
        }

        /* If the queue size is not 1 or 2, this indicates that the node is acting outside of the intended
           functionality
           */
        else {
            throw std::invalid_argument ( "Incorrect queue size full" );
        }
    } else {
        //calculate generalized forces
        Eigen::Matrix<double,6,1> controlOutput;
        controlOutput = calcControlOutput();

        //publish control topic
        spir_atnv::ThrustStamped thrustFrocesOutput;
        generalizedForces2ThrustersForces ( controlOutput,thrustFrocesOutput,std::string ( "atnvxy" ) );
        control_pub.publish ( thrustFrocesOutput );
    }

}


Eigen::Matrix<double,6,1> RobIntControlFullDof::calcControlOutput()
/*  Calculate control inputs required to keep IMU/robot level in the x and y directions. See
"robust_integral_control.h" for full mathematical explanation:

u2(t) = k * [e(t) - e(0)] + integral{k * e(T) + B * sign[e(t)]} * dT + f1

e(t) = a * W - R12 * omega

W is calcualted based on the rotation between [0 0 -1]^T gravity vector and
[0 0 -1]^T vector in IMU frame * the rotation of yaw in imu frame to align the heading with pile
*/
{

    Eigen::Matrix<double,6,1> controlOutput;

//	ROS_INFO_STREAM("  :  "<<imuMsgQueue.size()<<"  :  "<<zMsgQueue.size()<<"  :  "<<xyMsgQueue.size())
    if ( imuMsgQueue.size() <2 || zMsgQueue.size() <0 || fullMsgQueue.size() <1 ) {
        controlOutput<<0,0,0,0,0,0;
        return controlOutput;
    }

    // Calculate e(t) for rotation and translation
    Eigen::Matrix<double,3,1> rotateE;
    Eigen::Matrix<double,3,1> translateE;
    rotateE = calcRotateE();
    translateE = calcTranslateE();

    // Calculate time step
    double timeStep = calcTimeStepIntegral();
    //ROS_INFO_STREAM("time step for integration : " << timeStep);
    // Calculate integral term
    if (auto_mode)
    {
    integralTranslate += timeStep * ( k_t * translateE + b_t * sign ( translateE ) );
    integralRotate += timeStep * ( k_r * rotateE + b_r * sign ( rotateE ) );
    
      if (integralRotate(2,0)>30.0)
	integralRotate(2,0)=30.0;
      if (integralRotate(2,0)<-30.0)
	integralRotate(2,0)=-30.0;
      
    }
    //ROS_INFO_STREAM("Translate integration : " << integralTranslate);
    //ROS_INFO_STREAM("Rotation error : " << rotateE);
    //     integral += timeStep * (k * e + b * tanh(e * 10));

    // Calculate the control ouput
    Eigen::Matrix<double,3,1> rotateControlOutput;
    Eigen::Matrix<double,3,1> translateControlOutput;
    Eigen::Matrix<double,3,3> c_w;
    /*
    if (rotateE(2)>0)
      integralRotate(2) = fabs(integralRotate(2));
    
    if (rotateE(2)<0)
      integralRotate(2) = -fabs(integralRotate(2));
    */
    c_w<<1,0,0,0,1,0,0,0,0.2;
    
    if (rotateE(2)>0)
      integralRotate(2) = fabs(integralRotate(2));
    
    if (rotateE(2)<1.0 && rotateE(2)>-1.0)
    c_w<<1,0,0,0,1,0,0,0,0;
    
    
    Eigen::Matrix<double,3,3> k_w;
    k_w<<1,0,0,0,1,0,0,0,2;
    rotateControlOutput = k_r * k_w*( rotateE ) + c_r*c_w*integralRotate ;
    
    Eigen::Matrix<double,3,1> translateESign = sign(translateE);
    //integralTranslate(0) = translateESign(0)*fabs(integralTranslate(0));
    //integralTranslate(1) = translateESign(1)*fabs(integralTranslate(1));
    if (integralTranslate(0)>integral_x_max)
      integralTranslate(0)=integral_x_max;
    if (integralTranslate(0)<-integral_x_max)
      integralTranslate(0)=-integral_x_max;
    if (integralTranslate(1)>integral_y_max)
      integralTranslate(1)=integral_y_max;
    if (integralTranslate(1)<-integral_y_max)
      integralTranslate(1)=-integral_y_max;
    
    if (integralTranslate(2)>integral_z_max)
      integralTranslate(2)=integral_z_max;
    if (integralTranslate(2)<-integral_z_max)
      integralTranslate(2)=-integral_z_max;
    if (integralRotate(2)>integral_s_max)
      integralRotate(2)=integral_s_max;
    if (integralRotate(2)<-integral_s_max)
      integralRotate(2)=-integral_s_max;   
    
    translateControlOutput = k_t* ( translateE ) + c_t*integralTranslate ;

    //rotateControlOutput = k_r * ( rotateE );
    //translateControlOutput = k_t* ( translateE );


    for ( unsigned int i = 0 ; i<3; i++ ) {
        controlOutput ( i,0 ) = translateControlOutput ( i,0 );
        controlOutput ( i+3,0 ) = rotateControlOutput ( i,0 );
    }

     controlOutput ( 2,0 ) = 100;
    /*
    if ( status != 1 ) {
        for ( unsigned int i = 0 ; i<3; i++ ) {
            controlOutput ( i,0 ) = 0.0;
        }
    }
    */
    //ROS_INFO_STREAM ( "rotational integral output: " << integralRotate ( 0,0 ) <<" "<< integralRotate ( 1,0 ) <<" "<< integralRotate ( 2,0 ) );
  
    ROS_INFO_STREAM ( "translational control output: " << controlOutput ( 0,0 ) <<" "<< controlOutput ( 1,0 ) <<" "<< controlOutput ( 2,0 ) );
    ROS_INFO_STREAM ( "rotational control output: " << controlOutput ( 3,0 ) <<" "<< controlOutput ( 4,0 ) <<" "<< controlOutput ( 5,0 ) );
    robot_state_publish();
    return controlOutput;
}



Eigen::Matrix<double,3,1> RobIntControlFullDof::calcRotateW()
/* Calculate W to be used in the control equation. The user inputs a sensor msg i.e initialMsg,
   currentMsg or previousMsg and the function will return W(t).
   */
{
    if ( imuMsgQueue.size() >0 ) {

        tf::Vector3 vertVector;
        vertVector[0] =0;
        vertVector[1] =0;
        vertVector[2] =1;

        // transformation from inertial to IMU
        tf::Quaternion quat = getCOMOrientation();
        quat.normalize();
        tf::StampedTransform orientationQuat;
        orientationQuat.setRotation ( quat );

        // transformation from imu to com
        /*
        tf::Matrix3x3 basis ( rotMat_imu ( 0,0 ),rotMat_imu ( 0,1 ),rotMat_imu ( 0,2 ),
                              rotMat_imu ( 1,0 ),rotMat_imu ( 1,1 ),rotMat_imu ( 1,2 ),
                              rotMat_imu ( 2,0 ),rotMat_imu ( 2,1 ),rotMat_imu ( 2,2 ) );

        tf::Matrix3x3 basis ( 1.0,0.0,0.0,
                              0.0,1.0,0.0,
                              0.0,0.0,1.0 );

        tf::StampedTransform imu2com;
        imu2com.setBasis ( basis );
        */
        tf::StampedTransform com2imu = comToIMU;
        com2imu.setOrigin ( tf::Vector3 ( 0,0,0 ) );

	//tf::Vector3 vertVector1 = com2imu*orientationQuat * vertVector;
	tf::Vector3 vertVector1 = orientationQuat * vertVector;
	ROS_INFO_STREAM("vector 1 = " << vertVector1[0]<<" "<<vertVector1[1]<<" "<<vertVector1[2]);

	vertVector1.normalize();
        // get optimal roational axis
        tf::Vector3 W;
        W = tf::tfCross ( vertVector, vertVector1);

        // angle between two vectors
        double theta;
        theta =  tf::tfDot ( vertVector,vertVector1 ) ;
        /*
        if ( theta>1.0 ) {
                   theta = 1.0;
               }
               if ( theta<-1.0 ) {
                   theta = -1.0;
               }
        */
        theta = acos ( theta );
	if (fabs(theta)>0.1)
	  leved= false;
	
	ROS_INFO_STREAM("theta: step 1:  "<< theta);
        // desired rotation in roll and pitch
        //W = ( W ) / sqrt ( pow ( W[1],2 ) + pow ( W[2],2 ) + pow ( W[0],2 ));
        //normalizeVector ( W );

        //ROS_INFO_STREAM ( "W : "<< W[0]<<"  " << W[1]<<"  "<<W[2]<<" angle : "<<" "<<theta );
        W = ( W ) / sqrt ( pow ( W[1],2 ) + pow ( W[2],2 ) + pow ( W[0],2 ) );
        /*
        if ( fabs ( theta ) <0.001 ) {
                   W[0] = 0.0;
                   W[1] = 0.0;
                   W[2] = 1.0;
               }
        */
        tf::Quaternion quatTFrp ( W,theta );
        quatTFrp = quatTFrp.normalize();



        // adjust yawGoal
        if ( status == 2 ) {
            spir_atnv::PileStateStamped msg = fullMsgQueue.back();
            double yawInertial = atan2 ( msg.projection.y,msg.projection.x );
            sensor_msgs::Imu imu_msg = imuMsgQueue.back();
            tf::Quaternion quat = getCOMOrientation();
            tf::StampedTransform orientationQuat;
            orientationQuat.setRotation ( quat );


            tf::StampedTransform com2imu = comToIMU;
            com2imu.setOrigin ( tf::Vector3 ( 0,0,0 ) );

            // get optimal roational axis
            tf::Transform transform = orientationQuat;
            yawGoal = yawInertial - tf::getYaw ( transform.getRotation().inverse() );
	    double dx = cos(yawGoal);
	    double dy = sin(yawGoal);
	    yawGoal = atan2(dy,dx);
            //ROS_INFO_STREAM ( " desired delta yaw : "<<yawGoal );

        } else if ( status == 3 || status == 4 || status ==5 ) {
            yawGoal == 0.0;
        }

        // ROS_INFO_STREAM ( "yaw goal : "<< yawGoal );
        // desired rotation in yaw
        tf::Quaternion quatTFy;
	double yaw_tmp;
	yaw_tmp = 10.0*yawGoal/4;
	
	if (yaw_tmp>3.0)
	  yaw_tmp =3.0;
	else if(yaw_tmp<-3.0)
	  yaw_tmp = -3.0;
        quatTFy = tf::createQuaternionFromYaw ( yaw_tmp );
        quatTFy.normalize();
        ROS_INFO_STREAM("status : "<<status<<"  yawGoal : "<< yaw_tmp);
        // desired rotation in roll and pitch and yaw
        tf::Quaternion quatTFgoal;

        //quatTFgoal = quatTFrp*quatTFy;
        quatTFgoal = quatTFy;

        quatTFgoal.normalize();
        // output
        tf::Vector3 axis = quatTFgoal.getAxis();
        double angle = quatTFgoal.getAngle();
        //ROS_INFO_STREAM("rotational error : "<< axis[0]<<"  " << axis[1]<<"  "<<axis[2] <<" "<<angle);
        Eigen::Matrix<double,3,1> w;
        w << axis[0]*angle, axis[1]*angle, axis[2]*angle;
        //ROS_INFO_STREAM("rotational error : "<< w(0)<<"  " << w(1)<<"  "<<w(2) );
        return w;
    } else {
        // if no message is available set output to zeros
        Eigen::Matrix<double,3,1> w;
        w << 0,0,0;
        return w;
    }
}


Eigen::Matrix<double,3,1> RobIntControlFullDof::calcTranslateW()
/* Calculate W to be used in the control equation. The user inputs a sensor msg i.e initialMsg,
   currentMsg or previousMsg and the function will return W(t).
   */
{
    geometry_msgs::PointStamped xyGoal;
    xyGoal = xyGoalQueue.back();
    xyGoal.point.z = z_goal;

    geometry_msgs::PointStamped xyCurrent;
    xyCurrent.point.x = kfx.state() ( 0 );
    xyCurrent.point.y = kfy.state() ( 0 );
    xyCurrent.point.z = kfz.state() ( 0 );

    geometry_msgs::PointStamped dxyz;
    double dx = xyGoal.point.x - xyCurrent.point.x;
    double dy = xyGoal.point.y - xyCurrent.point.y;
    double dz = xyGoal.point.z - xyCurrent.point.z;

    // Calculate W
    Eigen::Matrix<double,3,1> w;

    w << dx, dy, dz;

    return w;
}


Eigen::Matrix<double,3,1> RobIntControlFullDof::calcTranslateE()
/* Calculate e(t) to be used in the control equation. The user inputs a sensor msg i.e initialMsg,
   currentMsg or previousMsg and the function will return e(t).
   */
{
    //get angular velocity in COM
    sensor_msgs::Imu msg = imuMsgQueue.back();
    tf::Vector3 angVel_tf ( -msg.angular_velocity.x, msg.angular_velocity.y,
                            -msg.angular_velocity.z );
    //tf::Vector3 angVel_tf_1 = tf::Transform ( getCOMOrientation() ) *angVel_tf;
    Eigen::Matrix<double,3,1> angVel;
    
    tf::Matrix3x3 basis ( rotMat_imu ( 0,0 ),rotMat_imu ( 0,1 ),rotMat_imu ( 0,2 ),
                          rotMat_imu ( 1,0 ),rotMat_imu ( 1,1 ),rotMat_imu ( 1,2 ),
                          rotMat_imu ( 2,0 ),rotMat_imu ( 2,1 ),rotMat_imu ( 2,2 ) );

    tf::StampedTransform imu2com;
    imu2com.setBasis ( basis );
    

    angVel_tf = imu2com* angVel_tf;
    angVel << angVel_tf[0],angVel_tf[1],angVel_tf[2];
    ROS_INFO_STREAM("angVel: x: "<<  angVel_tf[0] <<" y:" << angVel_tf[1]<<" z:" <<angVel_tf[2]);

    // get estimated velocity from kalman fitlers
    Eigen::Matrix<double,3,1> vel;
    vel << kfx.state() ( 1 ), kfy.state() ( 1 ), 0.0;

    // get estimated position from kalman filters
    Eigen::Matrix<double,3,1> X;
    X << kfx.state() ( 0 ), kfy.state() ( 0 ), 0.0;

    // get error in COM of x y directions
    Eigen::Matrix<double,3,1> e;
    Eigen::Matrix<double,3,1> w;
    w = calcTranslateW();

    Eigen::Matrix<double,3,1> w_xy;
    w_xy = w ;
    
    if (w_xy(0)>0.5)
      w_xy(0)=0.5;
    if (w_xy(0)<-0.5)
      w_xy(0)=-0.5;
    
    if (w_xy(1)>0.5)
      w_xy(1)=0.5;
    if (w_xy(1)<-0.5)
      w_xy(1)=-0.5;
    
    w_xy ( 2 ) = 0;
    //ROS_INFO_STREAM("w_xy: "<<w_xy);
    //e = -a_t * w_xy + vel;
    //e = -a_t * w_xy + ( vel + skewMatrix ( angVel ) *X );
    e = -a_t * w_xy + ( vel + skewMatrix ( angVel ) *X );
    //e = -a_t * w_xy;
    //e ( 2 ) = 0;
    /*  */
    ROS_INFO_STREAM ( "xy velocity : "<< vel(0) <<"  "<< vel(1) <<"\n"
    		<<"xy distance : " << w_xy(0) << " "<< w_xy(1)<<" "
		<<"\n"<<"e:  "<<e(0)<<"  "<<e(1)
    		<<"\n angular velocity : "<<angVel(0)<<" "<<angVel(1));
  
    if ( status == 2 || status == 3 || status == 4 || status == 5 ) {
        e = 0.0 * e;
    }
    // get error in z directions

    if ( zMsgQueue.size() >0 ) {
        Eigen::Matrix<double,3,1> w_z;
        w_z = w ;
        w_z ( 0 ) = 0;
        w_z ( 1 ) = 0;
        vel << 0.0, 0.0, kfz.state() ( 1 );

        tf::Vector3 zeVector;
        zeVector[0] =0;
        zeVector[1] =0;
        zeVector[2] =-z_t*w_z ( 2 )+vel ( 2 );

	//if (fabs(zeVector[2])>0.1)
	//  leved= false;
	  
        //ROS_INFO_STREAM ( "w_z : "<< w_z ( 2 ) <<" vel_z : " << vel ( 2 ) );
        // transformation from inertial to IMU
        tf::Quaternion quat = getCOMOrientation();
        tf::StampedTransform orientationQuat;
        orientationQuat.setRotation ( quat );


        // get optimal roational axis
        tf::Vector3 ze_tf;
        ze_tf = orientationQuat * zeVector ;

        Eigen::Matrix<double,3,1> e_z;
        e_z ( 0 ) = ze_tf[0];
        e_z ( 1 ) = ze_tf[1];
        e_z ( 2 ) = ze_tf[2];
        //ROS_INFO_STREAM("e : "<< e);
        //ROS_INFO_STREAM("e_z : "<< e_z);
        e =  e + e_z;
        //ROS_INFO_STREAM("e + e_z: "<< e);

    }
    if ( !std::isfinite ( e ( 0 ) ) ) {
        e ( 0 ) =0.0;
    }

    if ( !std::isfinite ( e ( 1 ) ) ) {
        e ( 1 ) =0.0;
    }

    if ( !std::isfinite ( e ( 2 ) ) ) {
        e ( 2 ) =0.0;
    }

    return e;
}

Eigen::Matrix<double,3,1> RobIntControlFullDof::calcRotateE()
/* Calculate e(t) to be used in the control equation. The user inputs a sensor msg i.e initialMsg,
   currentMsg or previousMsg and the function will return e(t).
   */
{
    Eigen::Matrix<double,3,1> e;
    Eigen::Matrix<double,3,1> w;

    if ( imuMsgQueue.size() ==0 ) {
        e<<0,0,0;
        return e;
    }

    //get angular velocity in COM
    sensor_msgs::Imu msg = imuMsgQueue.back();
    
    tf::Vector3 angVel_tf ( -msg.angular_velocity.x, msg.angular_velocity.y,
                            -msg.angular_velocity.z );
    //tf::Vector3 angVel_tf_1 = tf::Transform ( getCOMOrientation() ) *angVel_tf;
    Eigen::Matrix<double,3,1> angVel;
    
    tf::Matrix3x3 basis ( rotMat_imu ( 0,0 ),rotMat_imu ( 0,1 ),rotMat_imu ( 0,2 ),
                          rotMat_imu ( 1,0 ),rotMat_imu ( 1,1 ),rotMat_imu ( 1,2 ),
                          rotMat_imu ( 2,0 ),rotMat_imu ( 2,1 ),rotMat_imu ( 2,2 ) );

    tf::StampedTransform imu2com;
    imu2com.setBasis ( basis );
    angVel_tf = imu2com* angVel_tf;
    angVel << angVel_tf[0],angVel_tf[1],angVel_tf[2];
    
    // get error in COM

    w = calcRotateW();
    //ROS_INFO_STREAM ( "angular error: " << w );

    e =  a_r*w -  angVel;
    //ROS_INFO_STREAM ( "angular velocity: " << angVel );

    if ( !std::isfinite ( e ( 0 ) ) ) {
        e ( 0 ) =0.0;
    }

    if ( !std::isfinite ( e ( 1 ) ) ) {
        e ( 1 ) =0.0;
    }

    if ( !std::isfinite ( e ( 2 ) ) ) {
        e ( 2 ) =0.0;
    }


    return e;
}

template <typename MSGTYPE>
double RobIntControlFullDof::calcTimeStep ( const MSGTYPE& previousMsg, const MSGTYPE& currentMsg )
/* Calculate time step between current and previous imu message.
   */
{
    int timeStepS;
    int timeStepNs;

    /* Get time step information from message header. The time information comes as one int in seconds
       and another int in nanoseconds
       */
    timeStepS = currentMsg.header.stamp.sec - previousMsg.header.stamp.sec;
    timeStepNs = currentMsg.header.stamp.nsec - previousMsg.header.stamp.nsec;

    // Convert the time step in nanoseconds to seconds and add to the time step in seconds
    double timestep = timeStepS + timeStepNs * pow ( 10 ,-9 );

    return timestep;
}


template <typename MSGTYPE>
void RobIntControlFullDof::storeMsgs ( const MSGTYPE& msgIn, std::queue <MSGTYPE>& msgQueue,MSGTYPE& initialMsg )
/*
   Stores the initial data message, and the latest 2 messages. I.e if 8 messages have been sent
   from the IMU, it will store the 1st message, the 7th message and the 8th message.

   The latest 2 messages are used to determine the timestep for the integration term in the control
   algorithm and to calculate e(t). The initial message is used to determine the values of e(0).
*/
{
    // If the queue is empty, this is the first imu reading. Store the message as the initial message
    // for use in e(0) and keep filling the queue.
    if ( msgQueue.empty() ) {
        initialMsg = msgIn;
        msgQueue.push ( msgIn );
    }

    else if ( msgQueue.size() == 1 ) {
        // Push the new message into the queue so that the queue now has 2 elements. Then store
        // the 2 messages in class variables to be used in calculations.
        msgQueue.push ( msgIn );
    }

    else {
        // Push the new message vector into the queue and pop the last one out. Then store
        // the 2 messages in class variables to be used in calculations.
        msgQueue.pop();
        msgQueue.push ( msgIn );

    }
}



void RobIntControlFullDof::retrieveParams()
/*  Retrieve parameters from the ROS parameter server. If the parameters haven't been set, use
    dafult values
    */
{
    // Get gains values
    nh.param ( "k_r", k_r, 30.0 );
    nh.param ( "b_r", b_r, 10.0 );
    nh.param ( "a_r", a_r, 0.4 );
    nh.param ( "c_r",c_r,0.1 );

    nh.param ( "k_t", k_t, 30.0 );
    nh.param ( "b_t", b_t, 10.0 );
    nh.param ( "a_t", a_t, 1.0 );
    nh.param ( "c_t",c_t,0.1 );
    nh.param ( "z_t", z_t, 0.06 );

    nh.param ( "g", g, 0.3 );

    
    nh.param ( "integral_x_max", integral_x_max, 10.0 );
    nh.param ( "integral_y_max", integral_y_max, 10.0 );
    nh.param ( "integral_z_max", integral_z_max, 200.0 );
    nh.param ( "integral_s_max", integral_s_max, 35.0 );
    // prepare goal of pile view in lidar frame
    geometry_msgs::PointStamped goal;
    goal.header.frame_id = "LIDAR";
    goal.header.stamp = ros::Time::now();
    goal.point.x = g;
    goal.point.y = 0.0;
    goal.point.z = 0.0;
    xyGoalQueue.push ( goal );

    // Get f1 values from parameter server and convert to Eigen Matrix
    double f1Array [3];
    if ( nh.getParam ( "f1", f1Array[3] ) ) { // CHANGE THIS TO n_.param to set default value)
        for ( int i=0; i<3; ++i ) {
            f1 ( i,0 ) = f1Array[i];
        }
    } else {
        for ( int i=0; i<3; ++i ) {
            f1 ( i,0 ) = 0;
        }
    }


}


Eigen::Matrix<double,3,3> RobIntControlFullDof::skewMatrix ( Eigen::Matrix<double,3,1> inputVector )
// Outputs a 3x3 skew matrix based on a 3x1 input vector
{
    Eigen::Matrix<double,3,3> outputVector;

    outputVector ( 0,0 ) = 0;
    outputVector ( 0,1 ) = -inputVector ( 2,0 );
    outputVector ( 0,2 ) = inputVector ( 1,0 );
    outputVector ( 1,0 ) = inputVector ( 2,0 );
    outputVector ( 1,1 ) = 0;
    outputVector ( 1,2 ) = -inputVector ( 0,0 );
    outputVector ( 2,0 ) = -inputVector ( 1,0 );
    outputVector ( 2,1 ) = inputVector ( 0,0 );
    outputVector ( 2,2 ) = 0;

    return outputVector;
}

void RobIntControlFullDof::retrieveRotMat()
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

    rotMat_imu = convQuatToRot ( comToIMU );



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

    rotMat_lidar = convQuatToRot ( comToLidar );
    

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

    listener.lookupTransform ( "COM", "DS", ros::Time ( 0 ), comToDS );
    

}



Eigen::Matrix<double,3,3> RobIntControlFullDof::convQuatToRot ( tf::StampedTransform quatIn )
{
    Eigen::Matrix<double,3,3> rotMatOut;

    rotMatOut ( 0,0 ) = 1 - 2 * pow ( quatIn.getRotation().y(), 2 ) - 2
                        * pow ( quatIn.getRotation().z(), 2 );
    rotMatOut ( 0,1 ) = 2 * quatIn.getRotation().x() * quatIn.getRotation().y()
                        - 2 * quatIn.getRotation().z() * quatIn.getRotation().w();
    rotMatOut ( 0,2 ) = 2 * quatIn.getRotation().x() * quatIn.getRotation().z() + 2
                        * quatIn.getRotation().y() * quatIn.getRotation().w();
    rotMatOut ( 1,0 ) = 2 * quatIn.getRotation().x() * quatIn.getRotation().y() + 2
                        * quatIn.getRotation().z() * quatIn.getRotation().w();
    rotMatOut ( 1,1 ) = 1 - 2 * pow ( quatIn.getRotation().x(), 2 ) - 2
                        * pow ( quatIn.getRotation().z(), 2 );
    rotMatOut ( 1,2 ) = 2 * quatIn.getRotation().y() * quatIn.getRotation().z()
                        - 2 * quatIn.getRotation().x() * quatIn.getRotation().w();
    rotMatOut ( 2,0 ) = 2 * quatIn.getRotation().x() * quatIn.getRotation().z()
                        - 2 * quatIn.getRotation().y() * quatIn.getRotation().w();
    rotMatOut ( 2,1 ) = 2 * quatIn.getRotation().y() * quatIn.getRotation().z()
                        + 2 * quatIn.getRotation().x() * quatIn.getRotation().w();
    rotMatOut ( 2,2 ) = 1 - 2 * pow ( quatIn.getRotation().x(), 2 ) - 2
                        * pow ( quatIn.getRotation().y(), 2 );

    return rotMatOut;
}


Eigen::Matrix<double,3,1> RobIntControlFullDof::sign ( Eigen::Matrix<double,3,1> inputMat )
/* Calculate the sign of the matrix. For each element x in the matrix:
       if x > 0, return 1
       if x == 0, return 0  ==> in this case we don't need to change anything
       if x < 0, return -1
       */
{
    for ( int i=0; i<3; ++i ) {
        if ( inputMat ( i,0 ) > 0 ) {
            inputMat ( i,0 ) = 1;
        } else if ( inputMat ( i,0 ) < 0 ) {
            inputMat ( i,0 ) = -1;
        }
    }

    return inputMat;
}


void RobIntControlFullDof::generalizedForces2ThrustersForces ( const Eigen::Matrix<double,6,1>& gf,spir_atnv::ThrustStamped& thrust,std::string suffix )
/*  convert gererlzied forces to thruster forces
    */
{
    // Apply matrix to compute required thruster forces
    // 8x1 matrix of thruster forces
    Eigen::MatrixXd u ( 8,1 );
    u = ( coeffMat.transpose() * ( coeffMat * coeffMat.transpose() ).inverse() ) * gf;

    // Publish thruster values on topic
    thrust.header.stamp = ros::Time::now();
    thrust.name.resize ( 8 );
    thrust.name[0] = std::string ( "t1_" ) +suffix;
    thrust.name[1] = std::string ( "t2_" ) +suffix;
    thrust.name[2] = std::string ( "t3_" ) +suffix;
    thrust.name[3] = std::string ( "t4_" ) +suffix;
    thrust.name[4] = std::string ( "t5_" ) +suffix;
    thrust.name[5] = std::string ( "t6_" ) +suffix;
    thrust.name[6] = std::string ( "t7_" ) +suffix;
    thrust.name[7] = std::string ( "t8_" ) +suffix;

    thrust.thrust.resize ( 8 );
    thrust.thrust[0] = u ( 0,0 );
    thrust.thrust[1] = u ( 1,0 );
    thrust.thrust[2] = u ( 2,0 );
    thrust.thrust[3] = u ( 3,0 );
    thrust.thrust[4] = u ( 4,0 )/2.0;
    thrust.thrust[5] = u ( 5,0 )/2.0;
    thrust.thrust[6] = u ( 6,0 )/2.0;
    thrust.thrust[7] = u ( 7,0 )/2.0;
}


KalmanFilter RobIntControlFullDof::initializeKf ( std::string suffix )
{
    int n = 2; // Number of states
    int m = 1; // Number of measurements

    double dt = 0.5; // Time step

    Eigen::MatrixXd A ( n, n ); // System dynamics matrix
    Eigen::MatrixXd C ( m, n ); // Output matrix
    Eigen::MatrixXd Q ( n, n ); // Process noise covariance
    Eigen::MatrixXd R ( m, m ); // Measurement noise covariance
    Eigen::MatrixXd P ( n, n ); // Estimate error covariance

    // Discrete LTI projectile motion, measuring position only
    A << 1, dt, 0, 1;
    C << 1, 0;

    // Reasonable covariance matrices
    std::string strQ;
    std::string strR;
    std::string strP;
    nh.param ( std::string ( "Q" ) +suffix, strQ, std::string ( "0,0,0,100" ) );
    nh.param ( std::string ( "R" ) +suffix, strR, std::string ( "0.00001" ) );
    nh.param ( std::string ( "P" ) +suffix, strP, std::string ( "1,0,0,1" ) );

    string2Matrix ( Q,strQ,std::string ( "," ) );
    string2Matrix ( R,strR,std::string ( "," ) );
    string2Matrix ( P,strP,std::string ( "," ) );
    return KalmanFilter ( A, C, Q, R, P );
}


void RobIntControlFullDof::initializeKfState ( KalmanFilter& kf, double state )
{
    Eigen::VectorXd state0 ( 2 );
    state0 ( 0 ) = state;
    state0 ( 1 ) = 0.0;
    kf.init ( state0 );
}

void RobIntControlFullDof::string2Matrix ( Eigen::MatrixXd& matrixOut, std::string str, std::string delim )
// convert string to a set of parameters
{
    std::vector<std::string> strvec;

    boost::algorithm::trim_if ( str, boost::algorithm::is_any_of ( delim ) );
    boost::algorithm::split ( strvec,str,boost::algorithm::is_any_of ( delim ), boost::algorithm::token_compress_on );
    for ( unsigned int i=0; i<strvec.size(); i++ ) {
        boost::algorithm::trim_if ( strvec[i], boost::algorithm::is_any_of ( delim ) );
        matrixOut ( i ) = boost::lexical_cast<double> ( strvec[i] );

    }
    //std::cout<<matrixOut<<std::endl;
}



void RobIntControlFullDof::setYawGoal()
{
    if ( (status==1 || status==2) &&fullMsgQueue.size() >0 && imuMsgQueue.size() >0 ) {
        spir_atnv::PileStateStamped currentFull = fullMsgQueue.back();
        double yawInertial = atan2 ( currentFull.projection.y,currentFull.projection.x );

        tf::Quaternion quat = getCOMOrientation();
        tf::StampedTransform orientationQuat;
        orientationQuat.setRotation ( quat );

        yawGoal = yawInertial - tf::getYaw ( orientationQuat.getRotation().inverse() );
	yawGoal = atan2(sin(yawGoal),cos(yawGoal));

    }
    else
      yawGoal=0.0;
}

tf::Quaternion RobIntControlFullDof::getCOMOrientation()
{
    tf::Quaternion quatOut;

    if ( imuMsgQueue.size() >0 ) {
        sensor_msgs::Imu imuMsg = imuMsgQueue.back();

        tf::Quaternion quatOut;

        tf::Quaternion quat;
        quat.setX ( imuMsg.orientation.y );
        quat.setY ( imuMsg.orientation.z );
        quat.setZ ( imuMsg.orientation.w );
        quat.setW ( imuMsg.orientation.x );


        tf::Quaternion quat_inv = quat.inverse();
        tf::Vector3 axis = quat_inv.getAxis();
        double angle = quat_inv.getAngle();
        axis[0] = - axis[0];
        axis[2] = - axis[2];

        tf::Quaternion quat_right_inv ( axis,angle );

	tf::Transform tf_right_inv ( quat_right_inv.normalize() );

	tf::StampedTransform com2imu = comToIMU;
        com2imu.setOrigin ( tf::Vector3 ( 0,0,0 ) );
	
        quatOut = (com2imu * tf_right_inv).getRotation();
        quatOut.normalize();

        return quatOut;
    } else {
        throw std::invalid_argument ( "Incorrect size of imuMsgQueue" );
    }

    return quatOut;
}


double RobIntControlFullDof::calcTimeStepIntegral()
{
    double dt = 0;
    ros::Time currentLoopTime = ros::Time::now();

    if ( previousLoopTimeQueue.size() >0 ) {
        ros::Time previousLoopTime = previousLoopTimeQueue.front();
        previousLoopTimeQueue.pop();
        ros::Duration rosdt = currentLoopTime - previousLoopTime;
        dt = rosdt.toSec();
    }

    previousLoopTimeQueue.push ( currentLoopTime );

    return dt;
}

void RobIntControlFullDof::normalizeVector ( tf::Vector3& W )
{
    if ( fabs ( W[0] ) < 1e-300 && fabs ( W[1] ) < 1e-300 && fabs ( W[2] ) < 1e-300 ) {
        W[0]=0.0;
        W[1]=0.0;
        W[2]=1.0;
    }
}

void RobIntControlFullDof::robot_state_publish ( )
{
    nav_msgs::Odometry msg;
    msg.header.frame_id = "COM";
    msg.header.stamp = ros::Time::now();
    msg.pose.pose.position.x = kfx.state() ( 0 );
    msg.pose.pose.position.y = kfy.state() ( 0 );
    msg.pose.pose.position.z = kfz.state() ( 0 );

    Eigen::VectorXd a ( 3 );
    Eigen::VectorXd b ( 3 );

    a ( 0 ) = 1;
    a ( 1 ) = 0;
    a ( 2 ) = 0;

    b ( 0 ) = kfx.state() ( 1 );
    b ( 1 ) = kfy.state() ( 1 );
    b ( 2 ) = kfz.state() ( 1 );

    Eigen::Quaternion<double> quat;
    quat.setFromTwoVectors ( a, b );

    msg.pose.pose.orientation.w = quat.w();
    msg.pose.pose.orientation.x = quat.x();
    msg.pose.pose.orientation.y = quat.y();
    msg.pose.pose.orientation.z = quat.z();

    msg.twist.twist.linear.x = kfx.state() ( 1 );
    msg.twist.twist.linear.y = kfy.state() ( 1 );
    msg.twist.twist.linear.z = kfz.state() ( 1 );

    robot_status_pub.publish ( msg );
}

void RobIntControlFullDof::callBack_atnv_trigger ( const std_msgs::Bool::ConstPtr& msg )
{
    auto_mode=msg->data;
}

void RobIntControlFullDof::callBack_z_d ( const std_msgs::Float32MultiArray::ConstPtr& msg )
{
    z_goal = 0.0;//msg->data[0];
    pitch_goal = msg->data[1];
    roll_goal = msg->data[2];
}
