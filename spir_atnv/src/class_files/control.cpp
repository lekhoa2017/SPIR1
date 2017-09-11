#include "spir_atnv/control.h"
#include <qpOASES/QProblem.hpp>
Control:: Control ( ros::NodeHandle *n, ros::NodeHandle *nPrivate )
{
    ROS_INFO ( "\n" );
    n_ = *n;
    nh = *nPrivate;
    control_pub = n_.advertise<spir_atnv::ThrustStamped> ( "thrust_command", 2 );

    /* Calculate and store the coefficient matrix used to convert forces at the
       COM to individual thruster forces
       */
    CalcCoeffMatrix myObj;
    coeffMat = myObj.getCoeffMat();

    // Retrieve the rotation matrix from the URDF and the parameters from the ROS paramater server
    retrieveRotMat();
    retrieveParams(); // CHange this to constantly check param server?

    initializeParameters();

}

void Control::initializeParameters()
{

    tau.setZero();
    time_stamp = ros::Time::now();
    // UB << 50,50,50,50,17,17,17,17;
    // LB << 5,5,5,5,2,2,2,2;
    
    
        this->eta1_d.setZero();
        this->eta2_d.setZero();
        this->velB_d.setZero();
        this->omegaB_d.setZero();
        this->accB_d.setZero();
        this->alphaB_d.setZero();

        this->eta1.setZero();
        this->eta2.setZero();
        this->velB.setZero();
        this->omegaB.setZero();
	this->thruster_tau.setZero();
	


    UB << 50,50,50,50,17, 17, 17, 17;
    LB << -50,-50,-50,-50,-17,-17,-17,-17;
    UB << 60,60,60,60,20, 20, 20, 20;
    LB << -60,-60,-60,-60,-20,-20,-20,-20;   
    UB_DB << 3,3,3,3,0,0,0,0;
    LB_DB << -3,-3,-3,-3,0,0,0,0;
    //UB_DB.setZero();
    //LB_DB.setZero();
}

void Control::generalizedForces2ThrustersForces ( const Eigen::Matrix< double, 6 , 1  >& gf, spir_atnv::ThrustStamped& thrust, std::string suffix )
/*  convert gererlzied forces to thruster forces
    */
{
    // Apply matrix to compute required thruster forces
    // 8x1 matrix of thruster forces
    Eigen::Matrix<double,8,1> u;
    u = solveQP ( gf );
    this->thruster_tau = u;
    ROS_INFO_STREAM ( "tau" << gf );
    // check result
    Eigen::Matrix<double,6,1> tau_err;
    tau_err = this->coeffMat*u - gf;
    ROS_INFO_STREAM ( "tau_err"<<tau_err );
    ROS_INFO_STREAM ( "u"<<u );

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
    thrust.thrust[4] = u ( 4,0 );
    thrust.thrust[5] = u ( 5,0 );
    thrust.thrust[6] = u ( 6,0 );
    thrust.thrust[7] = u ( 7,0 );
}

Eigen::Matrix<double,8,1> Control::solveQP ( const Eigen::Matrix< double, 6 , 1  >& gf )
/*  solve QP problem
    */
{
    /*
    *	This file is part of qpOASES.
    *
    *	qpOASES -- An Implementation of the Online Active Set Strategy.
    *	Copyright (C) 2007-2008 by Hans Joachim Ferreau et al. All rights reserved.
    *
    *	qpOASES is free software; you can redistribute it and/or
    *	modify it under the terms of the GNU Lesser General Public
    *	License as published by the Free Software Foundation; either
    *	version 2.1 of the License, or (at your option) any later version.
    *
    *	qpOASES is distributed in the hope that it will be useful,
    *	but WITHOUT ANY WARRANTY; without even the implied warranty of
    *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    *	Lesser General Public License for more details.
    *
    *	You should have received a copy of the GNU Lesser General Public
    *	License along with qpOASES; if not, write to the Free Software
    *	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
    *
    */


    /**
     *	\file EXAMPLES/example1.cpp
     *	\author Hans Joachim Ferreau
     *	\version 1.3embedded
     *	\date 2007-2008
     *
     *	Very simple example for testing qpOASES (using QProblem class).
     */
    /* Setup data of first QP. */
    qpOASES::real_t H[8*8] = { 1.0 + this->gw, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 1.0+ this->gw, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 1.0+ this->gw, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 1.0+ this->gw, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 1.0+ this->gw, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 1.0+ this->gw, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0+ this->gw, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0+ this->gw
                             };
    qpOASES::real_t A[6*8];
    // need to check raw first or column first
    for ( int i=0; i<6; i++ )
    {
        for ( int j=0; j<8; j++ )
            A[i*8+j] = this->coeffMat ( i,j );
    }


    qpOASES::real_t g[8] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    
    for (int i=0;i<8;i++)
    {
	    g[i] = - 2*this->gw*this->thruster_tau(i);
    }

    qpOASES::real_t lb[8];
    qpOASES::real_t ub[8];

    double mix_integer[2];
    mix_integer[0] = -1;
    mix_integer[1] = 1;

    qpOASES::real_t lbA[6];
    qpOASES::real_t ubA[6];
    for ( int i=0; i<6; i++ )
    {
        lbA[i] = gf ( i );
        ubA[i] = gf ( i );
    }

    double V = 1e6;
    qpOASES::real_t opt[8];
    for ( int ia = 0; ia<2; ia++ )
    {
        for ( int ib = 0; ib<2; ib++ )
        {
            for ( int ic = 0; ic<2; ic++ )
            {
                for ( int id = 0; id<2; id++ )
                {
                    Eigen::Matrix<double,16,1> bound = getBoundMixedInteger ( mix_integer[ia],
                                                       mix_integer[ib],
                                                       mix_integer[ic],
                                                       mix_integer[id] );
		    // ROS_INFO_STREAM("bound: "<< bound);
                    for ( int i=0; i<8; i++ )
                    {
                        lb[i] = bound ( 8+i );
                        ub[i] = bound ( i );

                    }

                    // ROS_INFO_STREAM("gf: "<< gf);
                    /* Setting up QProblem object. */
                    qpOASES::QProblem example ( 8,6 );

                    /* Solve first QP. */
                    int nWSR = 20;

                    qpOASES::returnValue result = example.init ( H,g,A,lb,ub,lbA,ubA, nWSR,0 );
                    if ( result == qpOASES::SUCCESSFUL_RETURN )
                    {
		      //ROS_INFO_STREAM("SUCCESSFULLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL");
                        double V_new = example.getObjVal();
                        if ( V_new<=V )
                        {
                            V = V_new;
                            example.getPrimalSolution ( opt );

                        }
                    }



                }
            }
        }
    }

    Eigen::Matrix<double,8,1> u ( 8,1 );
    for ( int i=0; i<8; i++ )
    {
        u ( i ) = opt[i];
    }

    // to solve it peroperly, use MIQP,  but here the number of mixed integer is 4, we solve it by enumerating all 16 cases


    return u;
}




void Control::updateThrusterControl()
/*
*/
{
    Eigen::Matrix<double,6,1> gf = getControlGF();
    //gf<<3,0,1,0,1,0;
    spir_atnv::ThrustStamped thrust;
    std::string suffix = "atnv";
    generalizedForces2ThrustersForces ( gf,thrust, suffix );
    this->control_pub.publish ( thrust );
}




double Control::calcTimeStep()
/* Calculate time step between current and previous robot state message.
   */
{
    ros::Time current_time = ros::Time::now();
    ros::Duration dTime = current_time - time_stamp;

    return dTime.toSec();
}


void Control::retrieveParams()
/*  Retrieve parameters from the ROS parameter server. If the parameters haven't been set, use
    dafult values
    */
{
    // Get gains values
    nh.param ( "max_thruster_val", max_thruster_val, 30.0 );
    nh.param ( "smooth_thruster_w", gw,1.0 );
    std::vector<double> w;
    w.resize ( 6 );
    nh.getParam ( "max_gf",w );

    for ( int i=0; i<6; i++ )
    {
        max_gf ( i ) = w[i];
    }
}

void Control::retrieveRotMat()
// Eventually will get rotation matrix from urdf
{
    tf::TransformListener listener;


    while ( !listener.canTransform ( "COM", "IMU", ros::Time ( 0 ) ) && ros::ok() )
    {
        ros::Duration ( 1 ).sleep();
        if ( listener.canTransform ( "COM", "IMU", ros::Time ( 0 ) ) )
        {
            break;
        }
        ros::Duration ( 1 ).sleep();
        if ( listener.canTransform ( "COM", "IMU", ros::Time ( 0 ) ) )
        {
            break;
        }
        ros::Duration ( 1 ).sleep();
        if ( listener.canTransform ( "COM", "IMU", ros::Time ( 0 ) ) )
        {
            break;
        }
        ros::Duration ( 1 ).sleep();
        ROS_INFO ( "Cannot retrieve TF's! Ensure that the TF publisher\
                  node is running" );
    }
}

void Control::callBackStateDesired ( const std_msgs::Float32MultiArray::ConstPtr& msg )
/*
 * varaibles end with "_d" denote the desired state
 */
{
    for ( int i=0; i<3; i++ )
    {
        this->eta1_d ( i ) = msg->data[i];
        this->eta2_d ( i ) = msg->data[i+3];
        this->velB_d ( i ) = msg->data[i+6];
        this->omegaB_d ( i ) = msg->data[i+9];
        this->accB_d ( i ) = msg->data[i+12]*0;
        this->alphaB_d ( i ) = msg->data[i+15]*0;
    }

    // update thruster control (published thruter control toptic inside)
    updateThrusterControl();
}

void Control::callBackState ( const std_msgs::Float32MultiArray::ConstPtr& msg )
/*
 * get current state of the robot
 */
{
    for ( int i=0; i<3; i++ )
    {
        this->eta1 ( i ) = msg->data[i];
        this->eta2 ( i ) = msg->data[i+3];
        this->velB ( i ) = msg->data[i+6];
        this->omegaB ( i ) = msg->data[i+9];
    }

    this->velB ( 0 ) = 0;
    this->velB ( 1 ) = 0;
    // update estimation of augmented disturbance
    updateControlParam();

    // update thruster control (published thruter control toptic inside)
    updateThrusterControl();
}

Eigen::Matrix<double,6,1> Control::imposeSaturation ( Eigen::Matrix<double,6,1> gf )
// the saturation threshould is given by max_gf
// treat force and torque seperately
{
    Eigen::Matrix<double,6,1> output;
    output = gf;
    
    // torque
    double v = 0;
    for ( int i =3; i<6; i++ )
    {
        double s = std::abs ( gf ( i ) ) /max_gf ( i );
        if ( s>v ) v =s;
    }

    if ( v>1 )
    {
        for ( int i=3; i<6; i++ )
        {
            output ( i ) /= v;
        }
    }
    
    //forces
    v = 0;
    for ( int i =0; i<2; i++ )
    {
        double s = std::abs ( gf ( i ) ) /max_gf ( i );
        if ( s>v ) v =s;
    }

    if ( v>1 )
    {
        for ( int i=0; i<2; i++ )
        {
            output ( i ) /= v;
        }
    }
    
    
    v = 0;
    for ( int i =2; i<3; i++ )
    {
        double s = std::abs ( gf ( i ) ) /max_gf ( i );
        if ( s>v ) v =s;
    }

    if ( v>1 )
    {
        for ( int i=2; i<3; i++ )
        {
            output ( i ) /= v;
        }
    }
    
    
    return output;
    
}


Eigen::Matrix<double,12,1> Control::getError()
/*get desired acceleration in linear and angualr velocity
* variables start with "d" denote error or difference
*/
{
    Eigen::Matrix<double,3,3> R = rpy2rot ( this->eta2 );
    Eigen::Matrix<double,3,3> R_d = rpy2rot ( this->eta2_d );
    Eigen::Matrix<double,3,3> dR = R_d.transpose() * R;
    Eigen::Matrix<double,3,3> dR_dRT = dR - dR.transpose();
    Eigen::Matrix<double,3,1> dEta2;
    dEta2<< -dR_dRT ( 2,1 ),dR_dRT ( 2,0 ),-dR_dRT ( 1,0 );

    Eigen::Matrix<double,3,1> dEta1 = this->eta1_d - this->eta1;

    Eigen::Matrix<double,6,1> dEta;
    dEta.block ( 0,0,3,1 ) = R.transpose() * ( dEta1 );
    dEta.block ( 3,0,3,1 ) = dEta2;

    Eigen::Matrix<double,6,1> dVel;
    dVel.block ( 0,0,3,1 ) = R.transpose() * ( this->velB_d - this->velB );
    dVel.block ( 3,0,3,1 ) = this->omegaB_d - this->omegaB;

    Eigen::Matrix<double,12,1> error;

    error.block ( 0,0,6,1 ) = dEta;
    error.block ( 6,0,6,1 ) = dVel;

    return error;
}

Eigen::Matrix<double,16,1> Control::getBoundMixedInteger ( double a,double b,double c,double d )
// first half is upper bound, second half is lower bound
{
    Eigen::Matrix<double,4,1> mixed;
    mixed<< a,b,c,d;
    Eigen::Matrix<double,16,1> bound;
    for ( int i =4; i<8; i++ )
    {
        bound ( i ) = UB ( i );
        bound ( i+8 ) = LB ( i );
    }

    for ( int i =0; i<4; i++ )
    {
        if ( mixed ( i ) >0 )
        {
            bound ( i ) = UB ( i );
            bound ( i+8 ) = UB_DB ( i );
        }
        else
        {
            bound ( i ) = LB_DB ( i );
            bound ( i+8 ) = LB ( i );
        }
    }
    return bound;
}
