/*
 * PID_control1.cpp
 * SPIR control using PID antiwindup algorithm
 * Open loop control for surge and sway
 * Closed loop control for heave and yaw
 * Maintaining stabilisation for roll and pitch
 *  Created on: Sep 3, 2017
 *  Author: khoa
 */


#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<sensor_msgs/Joy.h>
#include<std_msgs/Float32MultiArray.h>
#include<spir_atnv/ThrustStamped.h>
#include<iostream>
#include<Eigen/Dense>

using namespace std;


class TeleopJoy{
	public:
	TeleopJoy();
	private:
	// Callback functions
	double deadband(double in, double H, double L);
	double saturation(double in, double H, double L);

	// Receive commands from joystick
	void cb_joy(const sensor_msgs::Joy::ConstPtr& joy);

	// Receive data from sensor and calculate control signal
	void cb_body_state(const std_msgs::Float32MultiArray::ConstPtr& msg );

	//Step time calculation
	double calcTimeStep();

	// Thruster allocation convert 6x1 force/torque into 8x1 thruster commands
	Eigen::Matrix<double,8,1> Thruster_Allocation ( const Eigen::Matrix< double, 6 , 1  >& gf );

	//
	void generalizedForces2ThrustersForces ( const Eigen::Matrix< double, 6 , 1  >& gf, spir_atnv::ThrustStamped& thrust, std::string suffix );

	ros::NodeHandle n;
	ros::Publisher control_pub; // publish command to thrusters
	ros::Subscriber sub_joy; // subscribe joystick
	ros::Subscriber sub_body_state; // subscribe sensors system

	//Gain from the joystick control
	Eigen::Matrix<double,4,1> G; // Open loop: Surge, Sway; closed loop: Heave, Yaw

	// Control parameters of Heave, Yaw control and Roll, Pitch stabilisation
	// Proportional gains
	Eigen::Matrix<double,4,1> Kp;

	// Integral gains
	Eigen::Matrix<double,4,1> Ki;

	// Derivative gains
	Eigen::Matrix<double,4,1> Kd;

	// Antiwindup gains
	Eigen::Matrix<double,4,1> Kw;


	// Error
	Eigen::Matrix<double,4,1> E;   // Error
	Eigen::Matrix<double,4,1> E_1; // Error of the previous iteration
	Eigen::Matrix<double,4,1> I_E; // Integral error
	Eigen::Matrix<double,4,1> D_E; // Derivative error

	// Control signal
	Eigen::Matrix<double,4,1> U; // Control signal
	Eigen::Matrix<double,4,1> U_1; // Control of the previous iteration

	// State of SPIR
	Eigen::Matrix<double,4,1> state; //depth, Yaw, roll and pitch feedback
	Eigen::Matrix<double,4,1> d_state; // Desired states
	Eigen::Matrix<double,4,1> c_state; // Commands that are proportional to the input of joystick by the gain G
	Eigen::Matrix<double,4,1> c_state_1; // Previous commands
	// Control output
	Eigen::Matrix<double,6,1> M_force; // 6 forces and torques for 6DOF motion (Surge Sway Heave Roll Pitch Yaw
	Eigen::Matrix<double,8,1> thruster; // 8 control signals for thrusters

	// Saturation values
	Eigen::Matrix<double,4,1> U_force; // Limit force for 4 controlling motions (Heave, Yaw, Roll, Pitch)
	Eigen::Matrix<double,4,1> L_force;

	Eigen::Matrix<double,8,1> U_thruster; // Limit for each thruster
	Eigen::Matrix<double,8,1> L_thruster;

	ros::Time time_stamp;
	double dt; // Control sampling time


};

TeleopJoy::TeleopJoy() // Constructors method
{
	// Set control gains
/*	n.param<Eigen::Matrix<double,4,1>("Control Gain",G,G);
	n.param("Proportional gains",Kp,Kp);
	n.param("Integral gains",Ki,Ki);
	n.param("Derivative gains",Kd,Kd);
	n.param("Antiwindup gains",Kw,Kw);*/

	// Get control parameters from launch file
	 std::vector<double> Gw;
	 Gw.resize(5);
	 n.getParam("c",Gw);

	 std::vector<double> k1w;
	 k1w.resize(6);
	 n.getParam("k1",k1w);

	 std::vector<double> k2w;
	 k2w.resize(6);
	 n.getParam("k2",k2w);

	G <<20,20,20,20;
	Kp <<5,5,5,5;
	Ki <<0.2,0.2,0.2,0.2;
	Kd <<0,0,0,0;
	Kw <<1,1,1,1;

	// Initialize error
	this->E.setZero();
	this->E_1.setZero();
	this->I_E.setZero();
	this->D_E.setZero();

	// Initialize control signal
	this->U.setZero();
	this->U_1.setZero();

	// Initialize state of SPIR
	this->state.setZero();
	this->d_state.setZero();
	this->c_state.setZero();
	this->c_state_1.setZero();

	// Initialize control output
	this->M_force.setZero();
	this->thruster.setZero();

	// Saturation setup
	U_force << 60, 20, 20, 40;
	L_force <<-60, 20, 20, 40;
	U_thruster <<20,20,20,20,50,50,50,50;
	L_thruster <<-20,-20,-20,-20,-50,-50,-50,-50;

	// Initial time values
	time_stamp=ros::Time::now();
	dt=0.1;


	control_pub = n.advertise<spir_atnv::ThrustStamped> ( "thrust_command_out", 2 );
	sub_joy = n.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopJoy::cb_joy,this);
	sub_body_state=n.subscribe<std_msgs::Float32MultiArray>("robot_state_body", 1, &TeleopJoy::cb_body_state,this);


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
	for (int i=0;i<4; i++)
	{
		c_state_1(i)=c_state(i);
		c_state(i)= G(i) * this->deadband(joy->axes[i],0.1,-0.1); // Check the order of Joystick commands
	}

}

void TeleopJoy::cb_body_state(const std_msgs::Float32MultiArray::ConstPtr& msg )
{

	dt=this->calcTimeStep(); // Sampling time calculation

	// Update actual states
	state(0)=msg->data[2]; // Depth feedback
	state(1)=msg->data[5]; // Heading feedback
	state(2)=msg->data[3]; // Roll feedback
	state(3)=msg->data[4]; // Pitch feedback


	for (int i=0;i<4; i++)   // Heave, yaw control and roll pitch stabilisation
		{



			if (i<2) // Update Desired state for  heave and yaw control
			{
				if ((c_state(i+2)==0) && (c_state_1(i+2)!=0))
				{
					d_state(i)=state(i);	// Release the joystick then take the final state as desired state
				}
				else
				{
					d_state(i) += dt*c_state(i+2); // Update desired state following the joystick commands
				}
			}
			else // Desired states of roll and pitch stabilisation
			{
				d_state(i) = 0 ;
			}


			// Error calculation

			E(i)   = d_state(i) - state(i); // Error

			D_E(i) = (E(i) - E_1(i))/dt;	// Differential of Error

			E_1(i) = E(i); 				// Previous error

			//Integral of error with antiwindup
			I_E(i) += (Ki(i)*E(i) + Kw(i) * (this->saturation(U(i),U_force(i),L_force(i))-U(i)))*dt;

			// PID Control output signal
			U(i)= Kp(i)*E(i) + I_E(i) + Kd(i)*D_E(i);


		}

	// Output force for 6DOF motion
	M_force(0)= c_state(0); // Surge force
	M_force(1)= c_state(1); // Sway force
	M_force(2)= this->saturation(U(0),U_force(0),L_force(0));	    // Heave force
	M_force(3)= this->saturation(U(2),U_force(2),L_force(2));		// Roll torque
	M_force(4)= this->saturation(U(3),U_force(3),L_force(3));		// Pitch torque
	M_force(5)= this->saturation(U(1),U_force(1),L_force(1));		// Yaw torque

	// Output control signal for each thruster
    spir_atnv::ThrustStamped thrust;
    std::string suffix = "atnv";
    generalizedForces2ThrustersForces ( M_force,thrust, suffix );
    this->control_pub.publish ( thrust ); // Publish  commands to thruster controller

}



double TeleopJoy::calcTimeStep()
{
	 ros::Time current_time = ros::Time::now();
	 ros::Duration dTime = current_time - time_stamp;
	 time_stamp=current_time;
	 return dTime.toSec();
}



int main(int argc, char** argv)
{
	ros::init(argc, argv, "PID_control");
	TeleopJoy teleop_SPIR;
	ros::spin();
}
