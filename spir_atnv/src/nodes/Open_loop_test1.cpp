/*
 * Open_loop_test1.cpp
 *
 *  Created on: Sep 7, 2017
 *      Author: khoa
 */

#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<sensor_msgs/Joy.h>
#include<std_msgs/Float32MultiArray.h>
#include<spir_atnv/ThrustStamped.h>
#include<iostream>
#include<Eigen/Dense>
#include "std_msgs/String.h"

using namespace std;


class TeleopJoy{
	public:
	TeleopJoy();
	ros::NodeHandle n;
	ros::NodeHandle nh;
	ros::Timer timer1;
	void cb_timer(const ros::TimerEvent& event);
	ros::Publisher pub_command;
	private:
	// Callback functions
	double deadband(double in, double H, double L);
	double saturation(double in, double H, double L);

	// Receive commands from joystick
	void cb_joy(const sensor_msgs::Joy::ConstPtr& joy);


	//Step time calculation

//	double calcTimeStep();

	// Thruster allocation convert 6x1 force/torque into 8x1 thruster commands
	Eigen::Matrix<double,8,1> Thruster_Allocation ( const Eigen::Matrix< double, 6 , 1  >& gf );
	void generalizedForces2ThrustersForces ( const Eigen::Matrix< double, 6 , 1  >& gf, spir_atnv::ThrustStamped& thrust, std::string suffix );


	ros::Publisher control_pub; // publish command to thrusters
	ros::Subscriber sub_joy; // subscribe joystick
//	ros::Publisher pub;

	//Gain from the joystick control
	Eigen::Matrix<double,6,1> G; // Open loop: Surge, Sway; closed loop: Heave, Yaw




	// Control signal

	Eigen::Matrix<double,6,1> c_state; // Commands that are proportional to the input of joystick by the gain G (surge sway heave roll pitch yaw)


	// Control output
	Eigen::Matrix<double,6,1> M_force; // 6 forces and torques for 6DOF motion (Surge Sway Heave Roll Pitch Yaw)
	Eigen::Matrix<double,8,1> thruster; // 8 control signals for thrusters



	Eigen::Matrix<double,8,1> U_thruster; // Limit for each thruster
	Eigen::Matrix<double,8,1> L_thruster;

	ros::Time time_stamp;
	double dt; // Control sampling time


};

TeleopJoy::TeleopJoy() // Constructors method
{
	// Set control gains
	std::vector<double> Gw;
	Gw.resize(6);

	G << 20,20,15,5,5,15;  // Surge, Sway, heave, Roll, Pitch, Yaw


	// Retrieve control gain
	nh.getParam("Gain",Gw);

    for (int i=0;i<6;i++)
    {
	    G(i) = Gw[i];

    }
	// Initialize control signal

	this->c_state.setZero();



	// Initialize control output
	this->M_force.setZero();
	this->thruster.setZero();


	U_thruster << 17,17,17,17,50,50,50,50;
	L_thruster << -17,-17,-17,-17,-50,-50,-50,-50;

	// Initial time values
	time_stamp=ros::Time::now();
	dt=0.1;



	pub_command = n.advertise<std_msgs::Float32MultiArray>("command", 2);
	control_pub = n.advertise<spir_atnv::ThrustStamped> ( "thrust_command_out", 2 );
	sub_joy = n.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopJoy::cb_joy,this);



}

double TeleopJoy::deadband(double in, double H, double L)
{
	double temp;
	if ((in<=H) && (in>=L))
	{
		temp=0;
	}
	else
	{
		temp=in;
	}
	return temp;
}

double TeleopJoy::saturation(double in, double H, double L)
{
	double temp;
	if (in>=H)
	{
		temp=H;
	}
	else if (in<=L)
	{
		temp=L;
	}
	else
	{
		temp=in;
	}
	return temp;
}

Eigen::Matrix<double,8,1> TeleopJoy::Thruster_Allocation ( const Eigen::Matrix< double,6,1 >& gf )
{

	Eigen::Matrix<double,8,1> u ( 8,1 );
	Eigen::Matrix<double,6,8> M ;
	Eigen::Matrix<double,6,6> temp;

	// Distance between thrusters
	double a=0.5;
	double b=0.5;
	double  D= std::sqrt(std::pow(a,2)+std::pow(b,2));
	const double a1=1/std::sqrt(2);
	M <<  a1,   a1,   a1,    a1, 0,   0,     0,   0,
		 -a1,   a1,  -a1,    a1, 0,   0,     0,   0,
		   0,   0,    0,      0, 1,   1,     1,   1,
		   0,   0,    0,      0, b/2, b/2,  -b/2, -b/2,
		   0,   0,    0,      0, -a/2, a/2,  a/2, -a/2,
		  -D,  -D,    D,      D,  0,    0,    0,    0;
	temp=M*M.transpose();
	u=M.transpose()*temp.inverse()*gf; // pseudo inverse matrix

	//Saturation
	for (int i=0;i<8;i++)
	{
		u(i)=this->saturation(u(i),U_thruster(i),L_thruster(i));
	}
	return u;
}

void TeleopJoy::generalizedForces2ThrustersForces ( const Eigen::Matrix< double,6,1 >& gf, spir_atnv::ThrustStamped& thrust, std::string suffix )
{
    Eigen::Matrix<double,8,1> u;
    u = Thruster_Allocation ( gf );
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
    for (int i=0;i<8;i++)
    {
        thrust.thrust[i] = u ( i,0 );
    }
}


void TeleopJoy::cb_joy(const sensor_msgs::Joy::ConstPtr& joy)
{
	// Receive command
	c_state(0) = this->deadband(joy->axes[3],0.1,-0.1);  	// Surge command
	c_state(1) = this->deadband(joy->axes[0],0.1,-0.1);     // Sway command
	c_state(2) = this->deadband(joy->axes[1],0.1,-0.1);     // Heave command
	c_state(3) = -1*this->deadband(joy->axes[4],0.1,-0.1);  // Roll command
	c_state(4) = this->deadband(joy->axes[5],0.1,-0.1);    // Pitch command
	c_state(5) = this->deadband(joy->axes[2],0.1,-0.1);     // Yaw command


}

void TeleopJoy::cb_timer(const ros::TimerEvent& event)
{

	std_msgs::Float32MultiArray command;


	for (int i=0;i<6;i++)
	{
		M_force(i)=G(i)*c_state(i);
	}

	 command.data.resize(6);
	    for (int i=0; i<6; i++)
	    {
	    	command.data[i]=M_force(i);
	    }

	ROS_INFO_STREAM("Command to SPIR: " << command);
	pub_command.publish(command);

	// Output control signal for each thruster
    spir_atnv::ThrustStamped thrust;
    std::string suffix = "atnv";
    generalizedForces2ThrustersForces ( M_force,thrust, suffix );
    ROS_INFO_STREAM("Command to thrusters: " << thrust);
    this->control_pub.publish ( thrust ); // Publish  commands to thruster controller


}


/*
double TeleopJoy::calcTimeStep()
{
	 ros::Time current_time = ros::Time::now();
	 ros::Duration dTime = current_time - time_stamp;
	 time_stamp=current_time;
	 return dTime.toSec();
}
*/


int main(int argc, char** argv)
{
	ros::init(argc, argv, "Open_loop_control");
	TeleopJoy teleop_SPIR;
	teleop_SPIR.timer1 = teleop_SPIR.n.createTimer(ros::Duration(0.1), &TeleopJoy::cb_timer, &teleop_SPIR);
	ros::spin();
}


