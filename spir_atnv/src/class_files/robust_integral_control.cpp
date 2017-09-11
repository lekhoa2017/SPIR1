#include "spir_atnv/robust_integral_control.h"
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/lexical_cast.hpp>
#include "geometry_msgs/PoseStamped.h"
#include "visualization_msgs/Marker.h"
RobIntControl::RobIntControl ( ros::NodeHandle *n, ros::NodeHandle *nPrivate )
{
    ROS_INFO ( "\n" );
    n_ = *n;
    nh = *nPrivate;
    imu_control_pub = n_.advertise<spir_atnv::ThrustStamped> ( "thrust_command", 2 );
    yaw_desired_pub = n_.advertise<visualization_msgs::Marker>("yaw_desired",2);
    /* Calculate and store the coefficient matrix used to convert forces at the
       COM to individual thruster forces
       */
    CalcCoeffMatrix myObj;
    coeffMat = myObj.getCoeffMat();

    // Retrieve the rotation matrix from the URDF and the parameters from the ROS paramater server
    retrieveRotMat();
    ROS_INFO_STREAM ( "rotMat = " <<rotMat );

    retrieveParams(); // CHange this to constantly check param server?

    zMsg.data.resize ( 4 );
    zMsg.data[2] = 0.0;

    integral << 0, 0, 0;
    e_z = 0.0;
    integral_z = 0.0;
    auto_mode = false;
    overLimitFlag=false;
    kfz = initializeKf ( "z" );
    initializeKfState ( kfz, 0.0 );

    pitch_goal=0;
    roll_goal=0;
    current_yaw=0;
    yawLock=false;

}



void RobIntControl::callBack_imu ( const sensor_msgs::Imu::ConstPtr& msg )
{
    // Store the IMU data message in a queue for use in calculations
    storeMsgs ( *msg );

    /* If the node has received more than just the first IMU reading, calculate
       control inputs and publish data.
       */
    if ( msgQueue.size() == 2 ) {
        Eigen::Matrix<double,3,1> controlOutput;
        controlOutput = calcControlOutput();
//ROS_INFO_STREAM("control_desired = " << controlOutput);
        // Construct eigen vector of desired motion at COM
        Eigen::Matrix<double,6,1> x;
        x ( 0,0 ) = 0;
        x ( 1,0 ) = 0;
        x ( 2,0 ) = 0;
        if ( zMsgQueue.size() >0 ) {
            geometry_msgs::PointStamped previousMsg = zMsgQueue.back();   // message(k)
            ros::Time currentTime = ros::Time::now();
            int timeStepS;
            int timeStepNs;

            /* Get time step information from message header. The time information comes as one int in seconds
               and another int in nanoseconds
               */
            timeStepS = -previousMsg.header.stamp.sec + currentTime.sec;
            timeStepNs = -previousMsg.header.stamp.nsec + currentTime.nsec;

            // Convert the time step in nanoseconds to seconds and add to the time step in seconds
            double dt = static_cast<double> ( timeStepS ) + static_cast<double> ( timeStepNs ) * pow ( 10 ,-9 );

            if ( dt>0.5 || previousMsg.point.z>5.0 ) {
                x ( 2,0 ) = 0.0;
            } else {
                x ( 2,0 ) = z_k*e_z + z_c*integral_z;
            }
        }
        /*
        ROS_INFO_STREAM ( " z control = " << x ( 2,0 ) << "  e_z: "<<e_z<<" integral_z "<<integral_z
                          <<"\n" <<" z: "<< kfz.state() ( 0 ) <<" z_v: "<< kfz.state() ( 1 ) );
*/
        tf::Vector3 force;
        force[0]= 0;
        force[1]= 0;
        force[2]= x ( 2,0 );

        sensor_msgs::Imu imu_msg = msgQueue.back();

        tf::Quaternion quatRot;
        quatRot.setX ( imu_msg.orientation.y );
        quatRot.setY ( imu_msg.orientation.z );
        quatRot.setZ ( imu_msg.orientation.w );
        quatRot.setW ( imu_msg.orientation.x );

        tf::Quaternion  quat_inv = quatRot.inverse();

        tf::Vector3 axis = quat_inv.getAxis();
        double angle = quat_inv.getAngle();
        axis[0] = - axis[0];
        axis[2] = - axis[2];

        tf::Quaternion quat_right_inv ( axis,angle );

        tf::Transform tf_right_inv ( quat_right_inv.normalize() );

        tf::StampedTransform com2imu = comToIMU;
        com2imu.setOrigin ( tf::Vector3 ( 0,0,0 ) );
        force = ( com2imu*tf_right_inv ) * force;

        //ROS_INFO_STREAM ( "body force: "<< force[0]<<"  "<<force[1]<<"  "<<force[2] );
        double forceNorm = force.length();
	double torqueNorm = sqrt(pow(controlOutput(0,0),2.0)+pow(controlOutput(1,0),2.0)+pow(controlOutput(2,0),2.0));
	
	x ( 0,0 ) = force[0];
        x ( 1,0 ) = force[1];
        x ( 2,0 ) = force[2];
        x ( 3,0 ) = 0;
        x ( 4,0 ) = 0;
        x ( 5,0 ) = 0;

	/*
	// add yaw
	double dangle = yaw_goal-current_yaw;
	double dy = sin(dangle);
	double dx = cos(dangle);
	dangle = atan2(dy,dx);
	double yaw_t = 80.0*(dangle);
	  
	tf::Vector3 xx;
	xx [0] = 0;
	xx [1] = 0;
	xx [2] = yaw_t;
	
	xx = comToWorld.getBasis() * xx;
	x(0,0) += xx[0];
	x(1,0) += xx[1];
	x(2,0) += xx[2];

	*/  
	
	// ROS_INFO_STREAM(" current yaw "<<current_yaw);
        // 8x1 matrix of thruster forces
        Eigen::MatrixXd u_f = ( coeffMat.transpose() * ( coeffMat * coeffMat.transpose() ).inverse() ) * x;

	x ( 0,0 ) = 0;
        x ( 1,0 ) = 0;
        x ( 2,0 ) = 0;
        x ( 3,0 ) = controlOutput ( 0,0 );
        x ( 4,0 ) = controlOutput ( 1,0 );
        x ( 5,0 ) = controlOutput ( 2,0 );
	Eigen::MatrixXd u_t = ( coeffMat.transpose() * ( coeffMat * coeffMat.transpose() ).inverse() ) * x;
        
	
	
	// Apply matrix to compute required thruster forces
        Eigen::MatrixXd u ( 8,1 );
	u = u_t+u_f;

	/*
        //check u and reshape u
	unsigned int indx=-1;
	double thrust_force_max=0.0;	ROS_INFO_STREAM("\n yaw force: "<< yaw_t<<"y force: "<<y_force);
	for (unsigned int i=0;i<8;i++)
	{
	  if (fabs(u(i,0))>thrust_force_max)
	  {
	    thrust_force_max= fabs(u(i,0));
	    indx =i;
	  }
	}
	
	if (thrust_force_max>max_thruster_val)
	{
	  if ( (u_f(indx,0)>max_torque_depth) && (u_t(indx,0)>max_force_stabilization))	    
	  {
	    u_f = u_f*(max_torque_depth/u_f(indx,0));
	    u_t = u_t*(max_force_stabilization/u_t(indx,0));
	  }

	  if ( (u_f(indx,0)>max_torque_depth) && !(u_t(indx,0)>max_force_stabilization))	    
	  {
	    u_f = u_f*((max_thruster_val-max_force_stabilization)/u_f(indx,0));
	  }
	  
	if ( !(u_f(indx,0)>max_torque_depth) && (u_t(indx,0)>max_force_stabilization))	    
	  {
	    u_t = u_t*((max_thruster_val-max_torque_depth)/u_t(indx,0));
	  }
	  u = u_t+u_f;
	}
	*/
	//	ROS_INFO_STREAM("gf = " <<  x(3,0) <<" "<< x(4,0) <<" "<< x(5,0));
        // Publish thruster values on topic
        spir_atnv::ThrustStamped thrust;
        thrust.header.stamp = msg->header.stamp;
        thrust.name.resize ( 8 );
        thrust.name[0] = "t1_imu";
        thrust.name[1] = "t2_imu";
        thrust.name[2] = "t3_imu";
        thrust.name[3] = "t4_imu";
        thrust.name[4] = "t5_imu";
        thrust.name[5] = "t6_imu";
        thrust.name[6] = "t7_imu";
        thrust.name[7] = "t8_imu";

        thrust.thrust.resize ( 8 );
        thrust.thrust[0] = u ( 0,0 );
        thrust.thrust[1] = u ( 1,0 );
        thrust.thrust[2] = u ( 2,0 );
        thrust.thrust[3] = u ( 3,0 );
        thrust.thrust[4] = u ( 4,0 );
        thrust.thrust[5] = u ( 5,0 );
        thrust.thrust[6] = u ( 6,0 );
        thrust.thrust[7] = u ( 7,0 );

        for ( unsigned i =0; i<8; i++ ) {
            if ( fabs ( thrust.thrust[i] ) >max_thruster_val ) {
                overLimitFlag=true;
                break;
            }
        }
        // if the thrust is over limit, then un-integrate the last bit
        double timeStep = calcTimeStep();

        // Calculate integral term when it is triggered
	/*
        if ( !auto_mode && overLimitFlag ) {
            Eigen::Matrix<double,3,1> e;
            e = calcE ( currentMsg );
            integral -= timeStep * ( k * e + b * sign ( e ) );

        }
*/
        imu_control_pub.publish ( thrust );
    }

    /* If the queue size is 1, this indicates the node just received the first IMU reading. Calculate e(0)
        and then wait for next message to perform control calculations.
    */
    else if ( msgQueue.size() == 1 ) {
        initialE = calcE ( initialMsg );
	calcW ( initialMsg );
	double r, p,y;
	getRPY( r, p, y);
    
	pitch_goal = p;
	roll_goal = r;
	yaw_goal = y;
    }
    /* If the queue size is not 1 or 2, this indicates that the node is acting outside of the intended
       functionality
       */
    else {
        throw std::invalid_argument ( "Incorrect queue size" );
    }
}





Eigen::Matrix<double,3,1> RobIntControl::calcControlOutput()
/*  Calculate control inputs required to keep IMU/robot level in the x and y directions. See
    "robust_integral_control.h" for full mathematical explanation:

    u2(t) = k * [e(t) - e(0)] + integral{k * e(T) + B * sign[e(t)]} * dT + f1

    e(t) = a * W - R12 * omega

    W = skew([0; 0; 1]) * R12 * C

    C = [-sin(pitch); sin(roll) * cos(pitch); cos(roll) * cos(pitch)]
*/
{
    // Calculate e(t)
    Eigen::Matrix<double,3,1> e;
    e = calcE ( currentMsg );

    // Calculate time step
    double timeStep = calcTimeStep();

    // Calculate integral term when it is triggered
    if ( !auto_mode && !joy_mode) {
        
	 Eigen::Matrix<double,3,1> integral_s_tmp = integral + timeStep * ( k * e + b * sign ( e ) );
	    if (fabs(integral_s_tmp(0))<max_integral_s
	      		&& fabs(integral_s_tmp(1))<max_integral_s
	        && fabs(integral_s_tmp(2))<max_integral_s
	        && integral_s_tmp(0)<max_integral_s/3.0 
	        && integral_s_tmp(1)<max_integral_s/3.0
	        && integral_s_tmp(2)<max_integral_s/3.0
	    )
	      integral=integral_s_tmp;
    }

//     integral += timeStep * (k * e + b * tanh(e * 10));

    // Calculate the control ouput u2(t)
    Eigen::Matrix<double,3,1> controlOutput;

    //controlOutput = k * (e - initialE) + c*integral + f1;

    controlOutput = k * ( e ) + c*integral ;

    //controlOutput = k * ( e );
    //ROS_INFO_STREAM ("IMU control integral" << integral);
    //ROS_INFO_STREAM ( "IMU control output: "<< controlOutput );

    return controlOutput;
}


Eigen::Matrix<double,3,1> RobIntControl::calcW ( sensor_msgs::Imu msgIn )
/* Calculate W to be used in the control equation. The user inputs a sensor msg i.e initialMsg,
   currentMsg or previousMsg and the function will return W(t).
   */
{
    tf::Vector3 vertVector;
    vertVector[0] = 0;
    vertVector[1] = 0;
    vertVector[2] = 1;
    // Calculate C in terms of roll and pitch angles
    Eigen::Matrix<double,3,1> C;

    /* Get the orientation quaternion from the IMU. Note, the quaternion from
       the IMU is sent as a (w;x;y;z) quaternion. However, the ros msg used
       sends data in an (x;y;z;w) quaternion. Therefore, the actual values are:
            actual_quaternion(x) = msg_quaternion(y)
            actual_quaternion(y) = msg_quaternion(z)
            actual_quaternion(z) = msg_quaternion(w)
            actual_quaternion(w) = msg_quaternion(x)
        */



    
    tf::Transform tf_pr;
    /*
    tf::Quaternion q_pr;
    q_pr.setRPY ( roll_goal,pitch_goal,yaw_goal );
    q_pr.normalize();
    tf_pr.setRotation ( q_pr );
    */
    tf::Matrix3x3 R_d;    
    R_d.setEulerYPR(yaw_goal,pitch_goal,roll_goal);
    tf_pr.setBasis(R_d);
    tf::Vector3 vertVector_tmp;
    // ROS_INFO_STREAM ( "vector  before = " << vertVector[0]<<" "<<vertVector[1]<<" "<<vertVector[2] );

    //vertVector_tmp = tf_pr*vertVector;
    //vertVector = vertVector_tmp;
    //vertVector.normalize();
    
    //ROS_INFO_STREAM ( "vector  after = " << vertVector[0]<<" "<<vertVector[1]<<" "<<vertVector[2] );

    tf::Quaternion quatRot;
    quatRot.setX ( msgIn.orientation.y );
    quatRot.setY ( msgIn.orientation.z );
    quatRot.setZ ( msgIn.orientation.w );
    quatRot.setW ( msgIn.orientation.x );

    tf::Quaternion  quat_inv = quatRot.inverse();

//    C = convQuatToRot(orientationQuat).transpose() * vertVector;
    // Calculate W
    tf::Vector3 W;

    tf::Matrix3x3 basis ( rotMat ( 0,0 ),rotMat ( 0,1 ),rotMat ( 0,2 ),
                          rotMat ( 1,0 ),rotMat ( 1,1 ),rotMat ( 1,2 ),
                          rotMat ( 2,0 ),rotMat ( 2,1 ),rotMat ( 2,2 ) );

    tf::StampedTransform imu2com;
    imu2com.setBasis ( basis );

    tf::Vector3 axis = quat_inv.getAxis();
    double angle = quat_inv.getAngle();
    axis[0] = - axis[0];
    axis[2] = - axis[2];

    tf::Quaternion quat_right_inv ( axis,angle );
    
    tf::Transform tf_right_inv ( quat_right_inv.normalize() );

    tf::StampedTransform com2imu = comToIMU;
    com2imu.setOrigin ( tf::Vector3 ( 0,0,0 ) );
    tf::Vector3 vertVector1 = ( com2imu*tf_right_inv ) * vertVector;
    comToWorld = com2imu*tf_right_inv ; // R from world to com

    current_yaw = tf::getYaw ( comToWorld.getRotation().inverse() );
    //ROS_INFO_STREAM("vector 1 = " << vertVector1[0]<<" "<<vertVector1[1]<<" "<<vertVector1[2]);

    vertVector1.normalize();
    tf::Vector3 vertVector2 = tf_pr*vertVector;
    vertVector2.normalize();
    W = tf::tfCross ( vertVector2,vertVector1 );


    //ROS_INFO_STREAM("vector 1 = " << vertVector1[0]<<" "<<vertVector1[1]<<" "<<vertVector1[2]);

    double theta = tf::tfDot ( vertVector2,vertVector1 );

    // ROS_INFO_STREAM ( "cos(theta) = " << theta );
    /*
    if (theta>1.0)
      theta = 1.0;
    else if (theta<-1.0)
      theta = -1.0;
    */


    theta = acos ( theta );
    // ROS_INFO_STREAM ( "theta = " << theta );

    //ROS_INFO_STREAM("Theta = " << theta * 180 / 3.14);

    //ROS_INFO_STREAM("rotMat = " << rotMat);
//    double sintheta;
//    sintheta = sqrt(pow(W(0,0),2) + pow(W(1,0),2) + pow(W(2,0),2));

    //ROS_INFO_STREAM("W before = " << W[0] <<" "<< W[1] <<" "<< W[2]);

    W = ( W * theta ) / sqrt ( pow ( W[1],2.0 ) + pow ( W[2],2.0 ) + pow ( W[0],2.0 ) );
    //normalizeVector(W);

    //C << W[0]*theta,W[1]*theta,W[2]*theta;
   
    tf::Matrix3x3 R = comToWorld.getBasis(); // in inerial frame
    tf::Matrix3x3 dR = R_d.inverse() * R;
    tf::Matrix3x3 dR_inv = dR.transpose();
    
    tf::Vector3 W_I;
    W_I[0] = dR[2][1]-dR_inv[2][1];
    W_I[1] = -dR[2][0]+dR_inv[2][0];
    W_I[2] = dR[1][0]-dR_inv[1][0];

    C<<W_I[0],W_I[1],W_I[2];
    // ROS_INFO_STREAM("W before:  " << C[0] <<" "<< C[1] <<" "<< C[2]);

    
    W_I = R*W_I;
    C<<W_I[0],W_I[1],W_I[2];

    // ROS_INFO_STREAM("W after:  " << C[0] <<" "<< C[1] <<" "<< C[2]);
    //C<<W[0],W[1],W[2];
    return C;
}


Eigen::Matrix<double,3,1> RobIntControl::calcE ( sensor_msgs::Imu msgIn )
/* Calculate e(t) to be used in the control equation. The user inputs a sensor msg i.e initialMsg,
   currentMsg or previousMsg and the function will return e(t).
   */
{
    // Store the angular velocity values in a vector
    tf::Vector3 angVel_tf;
    Eigen::Matrix<double,3,1> angVel;
    angVel << msgIn.angular_velocity.x, msgIn.angular_velocity.y,
           msgIn.angular_velocity.z;
    angVel_tf[0] = -angVel[0];
    angVel_tf[1] = angVel[1];
    angVel_tf[2] = -angVel[2];

    Eigen::Matrix<double,3,1> e;
    Eigen::Matrix<double,3,1> W;

    W = calcW ( msgIn );

    tf::Quaternion quatRot;
    quatRot.setX ( msgIn.orientation.y );
    quatRot.setY ( msgIn.orientation.z );
    quatRot.setZ ( msgIn.orientation.w );
    quatRot.setW ( msgIn.orientation.x );

    tf::Quaternion  quat_inv = quatRot.inverse();

    tf::Matrix3x3 basis ( rotMat ( 0,0 ),rotMat ( 0,1 ),rotMat ( 0,2 ),
                          rotMat ( 1,0 ),rotMat ( 1,1 ),rotMat ( 1,2 ),
                          rotMat ( 2,0 ),rotMat ( 2,1 ),rotMat ( 2,2 ) );

    tf::StampedTransform imu2com;
    imu2com.setBasis ( basis );

    tf::Vector3 axis = quat_inv.getAxis();
    double angle = quat_inv.getAngle();
    axis[0] = - axis[0];
    axis[2] = - axis[2];

    tf::Quaternion quat_right_inv ( axis,angle );

    tf::Transform tf_right_inv ( quat_right_inv );

    //angVel_tf = imu2com*tf_right_inv * angVel_tf;
    angVel_tf = imu2com* angVel_tf;
    angVel[0] = angVel_tf[0];
    angVel[1] = angVel_tf[1];
    angVel[2] = angVel_tf[2];

    //ROS_INFO_STREAM ( "angle velocity = " << angVel[0] <<" "<< angVel[1] <<" "<< angVel[2] );

    Eigen::Matrix<double,3,1> angVel_desired;
    tf::Vector3 angVel_z;
    angVel_z[0] = 0;
    angVel_z[1] = 0;
    angVel_z[2] = yaw_vel_goal;
    angVel_z =  comToWorld* angVel_z;

    angVel_desired[0] = roll_vel_goal + angVel_z[0];
    angVel_desired[1] = pitch_vel_goal + angVel_z[1];
    angVel_desired[2] = angVel_z[2];
    
    Eigen::Matrix<double,3,3> aa;
    aa(0,0) = a;
    aa(1,1) = a;
    aa(2,2) = a;

    Eigen::Matrix<double,3,1> angVel_desired_tuned;
    angVel_desired_tuned(0) = angVel_desired(0);
    angVel_desired_tuned(1) = angVel_desired(1);
    angVel_desired_tuned(2) = angVel_desired(2)*0.1;
    
    e = a * W - angVel + angVel_desired;

    ROS_INFO_STREAM ( " rpy = 		" << roll_goal <<" "<< pitch_goal <<" "<< yaw_goal );
    double r,p,y;
    comToWorld.getBasis ().getRPY(r,p,y);
    ROS_INFO_STREAM ( " rpy desired  =  " << r <<" "<< p <<" "<< y );
    ROS_INFO_STREAM ( " W = " << W[0] <<" "<< W[1] <<" "<< W[2] );
    ROS_INFO_STREAM ( " e = " << e[0] <<" "<< e[1] <<" "<< e[2] );

    return e;
}


double RobIntControl::calcTimeStep()
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



void RobIntControl::storeMsgs ( sensor_msgs::Imu msgIn )
/*
   Stores the initial IMU data message, and the latest 2 messages. I.e if 8 messages have been sent
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
        previousMsg = msgQueue.front();   // message(t-1)
        currentMsg = msgQueue.back();   // message(k)
    }

    else {
        // Push the new message vector into the queue and pop the last one out. Then store
        // the 2 messages in class variables to be used in calculations.
        msgQueue.pop();
        msgQueue.push ( msgIn );

        previousMsg = msgQueue.front();   // message(k)
        currentMsg = msgQueue.back();   // message(k+1)
    }
}



void RobIntControl::retrieveParams()
/*  Retrieve parameters from the ROS parameter server. If the parameters haven't been set, use
    dafult values
    */
{
    // Get gains values
    nh.param ( "k", k, 40.0 );
    nh.param ( "b", b, 10.0 );
    nh.param ( "a", a, 1.0 );
    nh.param ( "c", c, 0.1 );

    nh.param ( "z_c", z_c, 10.0 );
    nh.param ( "z_k", z_k, 40.0 );
    nh.param ( "z_b", z_b, 10.0 );
    nh.param ( "z_a", z_a, 1.0 );

    nh.param ( "max_thruster_val", max_thruster_val, 17.0 );
    
    nh.param ( "max_integral_z", max_integral_z, 200.0 );
    nh.param ( "max_integral_s", max_integral_s, 40.0 );
    nh.param ( "max_torque_depth", max_torque_depth, 10.0 );
    nh.param ( "max_force_stabilization", max_force_stabilization, 10.0 );
    
    nh.param ( "depth", z_goal, 0.4 );

    // Get f1 values from parameter server and convert to Eigen Matrix
    double f1Array [3];
    nh.getParam ( "f1", f1Array[3] ); // CHANGE THIS TO n_.param to set default value

    for ( int i=0; i<3; ++i ) {
        f1 ( i,0 ) = f1Array[i];
    }
}


Eigen::Matrix<double,3,3> RobIntControl::skewMatrix ( Eigen::Matrix<double,3,1> inputVector )
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

void RobIntControl::retrieveRotMat()
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
    
    rotMat = convQuatToRot ( comToIMU );
}



Eigen::Matrix<double,3,3> RobIntControl::convQuatToRot ( tf::StampedTransform quatIn )
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


Eigen::Matrix<double,3,1> RobIntControl::sign ( Eigen::Matrix<double,3,1> inputMat )
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

void RobIntControl::normalizeVector ( tf::Vector3& W )
{
    if ( fabs ( W[0] ) < 1e-5 && fabs ( W[1] ) < 1e-5 && fabs ( W[2] ) < 1e-5 ) {
        W[0]=0.0;
        W[1]=0.0;
        W[2]=1.0;
    }
}


void RobIntControl::callBack_z ( const std_msgs::Float32MultiArray::ConstPtr& msg )
{
    //zMsg = *msg;

    tf::Vector3 vect;
    vect = comToDS.getOrigin();
    
    ROS_INFO_STREAM("vect: "<<" "<<vect[0]<<" "<< vect[1]<<" "<< vect[2]);
    
    tf::Vector3 dz = comToWorld.inverse()*vect;

    ROS_INFO_STREAM("dz: "<<dz[2]);
    
    geometry_msgs::PointStamped zMsg;
    zMsg.header.frame_id = "inertial";
    zMsg.header.stamp = ros::Time::now();
    zMsg.point.x = 0.0;
    zMsg.point.y = 0.0;
    zMsg.point.z = msg->data[2]-0.4;//+dz[2];

    
    if ( zMsgQueue.size() <2 ) {
        zMsgQueue.push ( zMsg );
    } else {
        zMsgQueue.pop();
        zMsgQueue.push ( zMsg );
    }

    if ( zMsgQueue.size() ==2 ) {
        geometry_msgs::PointStamped previousMsg = zMsgQueue.front();   // message(k)
        geometry_msgs::PointStamped currentMsg = zMsgQueue.back();   // message(k+1)
        int timeStepS;
        int timeStepNs;

        /* Get time step information from message header. The time information comes as one int in seconds
           and another int in nanoseconds
           */
        timeStepS = currentMsg.header.stamp.sec - previousMsg.header.stamp.sec;
        timeStepNs = currentMsg.header.stamp.nsec - previousMsg.header.stamp.nsec;

        // Convert the time step in nanoseconds to seconds and add to the time step in seconds
        double dt = static_cast<double> ( timeStepS ) + static_cast<double> ( timeStepNs ) * pow ( 10 ,-9 );

        Eigen::VectorXd z ( 1 );
        z << currentMsg.point.z;
        kfz.update ( z,dt );
        e_z =z_goal-kfz.state() ( 0 );
        e_z = -e_z*z_a+ kfz.state() ( 1 )-z_vel_goal;
        double sign_e_z;
        if ( e_z<0 ) {
            sign_e_z=-1.0;
        }
        if ( e_z>0 ) {
            sign_e_z=1.0;
        }

        if ( !auto_mode && !joy_mode) {
	    double integral_z_tmp = integral_z + dt * ( z_k*e_z+z_b* sign_e_z );
	    if (fabs(integral_z_tmp)<max_integral_z )
	      integral_z=integral_z_tmp;
        }

    }
}

void RobIntControl::callBack_z_d ( const std_msgs::Float32MultiArray::ConstPtr& msg )
{
    z_vel_goal = msg->data[0];
    roll_vel_goal = msg->data[1];
    pitch_vel_goal = msg->data[2];
    yaw_vel_goal = msg->data[3];
 
    if (isInBound(z_vel_goal,0.01))
    {
      z_vel_goal = 0;
    }else
    {
      z_goal = kfz.state() ( 0 ); // set current depth as desired depth
    }
    
    double r, p,y;
    getRPY( r, p, y);

    if (isInBound(pitch_vel_goal,0.01))
    {
      pitch_vel_goal = 0;
    }else
    {
      pitch_goal = p;
    }
    
    if (isInBound(roll_vel_goal,0.01))
    {
      roll_vel_goal = 0;
    }
    else
    {
      roll_goal = r;
    }
    
    if (isInBound(yaw_vel_goal,0.01))
    {
      yaw_vel_goal = 0;
    }else
    {
      yaw_goal = y;
    }
    
    
    visualization_msgs::Marker marker;
    marker.header.frame_id="world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "yaw_desired";
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    marker.scale.x = 0.030; // Change this to set arrow diameter
    marker.scale.y = 0.060; // Arrow head diameter
    marker.scale.z = 0; // Arrow head length (0=default)
    marker.color.a = 0.75;
    marker.color.r = 255/255;
    marker.color.g =   0/255;
    marker.color.b = 255/255;

    
    tf::Quaternion quat;
    quat.setRPY(0,0,roll_goal+yaw0);

      
    geometry_msgs::Point pt0, pt1;
    pt0.x = 0;
    pt0.y = 0;
    pt0.z = 0;
    pt1.x = cos(roll_goal+yaw0);
    pt1.y = sin(roll_goal+yaw0);
    pt1.z = 0;

    marker.points.clear();
    marker.points.push_back ( pt0 );
    marker.points.push_back ( pt1 );
      
    yaw_desired_pub.publish(marker);
    
    
}

void RobIntControl::callBack_atnv_trigger ( const std_msgs::Bool::ConstPtr& msg )
{
    auto_mode=msg->data;
}

KalmanFilter RobIntControl::initializeKf ( std::string suffix )
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


void RobIntControl::initializeKfState ( KalmanFilter& kf, double state )
{
    Eigen::VectorXd state0 ( 2 );
    state0 ( 0 ) = state;
    state0 ( 1 ) = 0.0;
    kf.init ( state0 );
}

void RobIntControl::string2Matrix ( Eigen::MatrixXd& matrixOut, std::string str, std::string delim )
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


void RobIntControl::callBack_joy(const sensor_msgs::Joy::ConstPtr& msg)
{
  double bar=0.1;
  if(fabs(msg->axes[0])>bar||fabs(msg->axes[1])>bar
    ||fabs(msg->axes[3])>bar||fabs(msg->axes[4])>bar)
  joy_mode = true;
  else
    joy_mode = false;
}

bool RobIntControl::isInBound(double value, double bound)
{
  bound = fabs(bound);
  value = fabs(value);
  if (value>bound)
  return false;
  else
    return true;  
}

void RobIntControl::getRPY(double& r, double& p, double& y) 
// get r p y in inetial frame
{
    
    comToWorld.getBasis().getRPY(r,p,y);
    
    //ROS_INFO_STREAM("rpy:				"<<": "<<r<<" "<<p<<" "<<y);
    
    /*
    tf::Matrix3x3 R_d;
    R_d.setEulerYPR(0.2,0.3,0);
    tf::Matrix3x3 R = comToWorld.getBasis();
    tf::Matrix3x3 dR = R_d.inverse() * R;
    tf::Matrix3x3 dR_inv = dR.transpose();
    Eigen::Matrix<double,3,1> C;
    C<<dR[2][1]-dR_inv[2][1],-dR[2][0]+dR_inv[2][0],dR[1][0]-dR_inv[1][0];
    ROS_INFO_STREAM("W:  " << C[0] <<" "<< C[1] <<" "<< C[2]);
    */
}

