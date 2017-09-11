#include "spir_atnv/rise_pid.h"
#include <qpOASES/QProblem.hpp>
RisePid:: RisePid( ros::NodeHandle *n, ros::NodeHandle *nPrivate ):Control(n,nPrivate)
{
    initialize();
    retrieveParams();
}

void RisePid::initialize()
{
	integral.setZero();
        tau.setZero();
	retrieveParams();
}



void RisePid::retrieveParams()
/*  Retrieve parameters from the ROS parameter server. If the parameters haven't been set, use
    dafult values
    */
{
    // Get gains values
    
    std::vector<double> k_tmp;
    k_tmp.resize(6);
    nh.getParam("k",k_tmp);

    std::vector<double> b_tmp;
    b_tmp.resize(6);
    nh.getParam("b",b_tmp);

    std::vector<double> a_tmp;
    a_tmp.resize(6);
    nh.getParam("a",a_tmp);
    
    std::vector<double> c_tmp;
    c_tmp.resize(6);
    nh.getParam("c",c_tmp);
    /*
     * <rosparam param="max_gf">[30,30,30,30,30,30]</rosparam> 	
     */
    k.setZero();
    b.setZero();
    a.setZero();
    c.setZero();
    for (int i=0;i<6;i++)
    {
	    k(i,i) = k_tmp[i];
	    b(i,i) = b_tmp[i];
	    a(i,i) = a_tmp[i];
	    c(i,i) = c_tmp[i];  
    }
}



Eigen::Matrix<double,6,1> RisePid::getS()
/*get desired acceleration in linear and angualr velocity
* variables start with "d" denote error or difference
*/
{   
    Eigen::Matrix<double,12,1> err = getError();
   
    Eigen::Matrix<double,6,1> dEta;
    dEta.block(0,0,3,1) = err.block(0,0,3,1);
    dEta.block(3,0,3,1) = err.block(3,0,3,1);

    Eigen::Matrix<double,6,1> dVel;
    dVel.block(0,0,3,1) = err.block(6,0,3,1);
    dVel.block(3,0,3,1) = err.block(9,0,3,1);
    
    Eigen::Matrix<double,6,1> s;
    s = dVel +  this->a* dEta;
    //ROS_INFO_STREAM("k1"<< k1);
    //ROS_INFO_STREAM("k2"<< k2);
    //ROS_INFO_STREAM("dEta"<< dEta);
    //ROS_INFO_STREAM("dVel"<< dVel);
    return s;    
}

Eigen::Matrix<double,6,1> RisePid::getControlGF()
// get control in generalized forces
// u2(t) = k * [e(t) - e(0)] + integral{k * e(T) + B * sign[e(t)]} * dT + f1
{
	Eigen::Matrix<double,6,1> s = getS();
	
	ROS_INFO_STREAM("s"<< s) ;
	
	this->tau = this->k*s + this->c * this->integral;

	this->tau = imposeSaturation(this->tau);
	
	return this->tau;
}

void RisePid::updateControlParam()
// get time derivative of augmented_disturbance
{
	double dt = calcTimeStep();
	Eigen::Matrix<double,6,1> s = getS();
	this->integral +=  (this->k * s + this->b * sign (s))* dt;
}
  
Eigen::Matrix<double,6,1> RisePid::sign(Eigen::Matrix<double,6,1> s)
{
  Eigen::Matrix<double,6,1> output;
  for (int i=0;i<6;i++)
  {
    if ( s(i)> 0.0001 )
    {
      output(i) = 1;
    }
    else if ( s(i)< -0.0001 )
    {
      output(i) = -1;
    }else
    {
      output(i) = 0;
    }
  }
  
  return output;
}