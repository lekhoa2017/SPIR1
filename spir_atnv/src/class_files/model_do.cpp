#include "spir_atnv/model_do.h"
#include <qpOASES/QProblem.hpp>
ModelDO:: ModelDO( ros::NodeHandle *n, ros::NodeHandle *nPrivate ):Control(n,nPrivate)
{
    initialize();
    retrieveParams();
}

void ModelDO::initialize()
{
	augmented_disturbance.setZero();
	G.setZero();
        tau.setZero();
	M.setZero();
	M(0,0) = 70;
	M(1,1) = 70;
	M(2,2) = 70;
	M(3,3) = 5;
	M(4,4) = 5;
	M(5,5) = 5;
	this->DGB = 100;
	this->BuoF = 10;
	this->eta2_ini<<0.11961865425109863, 0.038111258298158646, -2.1480822563171387;
	Eigen::Matrix<double,3,1> z;
	z<<0,0,1;
	Eigen::Matrix<double,3,3> R_ini = rpy2rot(this->eta2_ini);
	this->zGB = R_ini.transpose()*z;
	this->retrieveParams();
}



void ModelDO::retrieveParams()
/*  Retrieve parameters from the ROS parameter server. If the parameters haven't been set, use
    dafult values
    */
{
    // Get gains values

    nh.param ( "DGB", this->DGB, 100.0 );
    nh.param ( "BuoF", this->BuoF, 10.0 );
    std::vector<double> cw;
    cw.resize(6);
    nh.getParam("c",cw);

    std::vector<double> k1w;
    k1w.resize(6);
    nh.getParam("k1",k1w);

    std::vector<double> k2w;
    k2w.resize(6);
    nh.getParam("k2",k2w);
    
    /*
     * <rosparam param="max_gf">[30,30,30,30,30,30]</rosparam> 	
     */
    c.setZero();
    k1.setZero();
    k2.setZero();
    for (int i=0;i<6;i++)
    {
	    c(i,i) = cw[i];
	    k1(i,i) = k1w[i];
	    k2(i,i) = k2w[i];	    
    }
}


Eigen::Matrix<double,6,1> ModelDO::P()
// augment function to formulate augmented_disturbance
{
	Eigen::Matrix<double,6,1> output;
	output.block(0,0,3,1) = this->velB;
	output.block(3,0,3,1) = this->omegaB;
	
	return this->c * output;
}


Eigen::Matrix<double,6,6> ModelDO::L()
// gain function for update disturbance estimation
{
	return this->c * this->M.inverse();
}

Eigen::Matrix<double,6,1> ModelDO::getDerivativeAugmentedDisturbance()
// get time derivative of augmented_disturbance
{ 
	Eigen::Matrix<double,6,6> l;
	l = L();
	
	Eigen::Matrix<double,6,1> p;
	p = P();	
	
	return -l*augmented_disturbance + l*(G-tau-p);
}

Eigen::Matrix<double,6,1> ModelDO::getDesiaredAcc()
/*get desired acceleration in linear and angualr velocity
* variables start with "d" denote error or difference
*/
{   
    Eigen::Matrix<double,12,1> err = getError();
   
    Eigen::Matrix<double,6,1> dEta;
    dEta.block(0,0,3,1) = err.block(0,0,3,1);
    dEta.block(3,0,3,1) = err.block(3,0,3,1);;

    Eigen::Matrix<double,6,1> dVel;
    dVel.block(0,0,3,1) = err.block(6,0,3,1);;
    dVel.block(3,0,3,1) = err.block(9,0,3,1);;

    Eigen::Matrix<double,6,1> a_d;
    a_d.block(0,0,3,1) = this->accB_d;
    a_d.block(3,0,3,1) = this->alphaB_d;
    
    Eigen::Matrix<double,6,1> a;
    
    a = a_d + this->k1 * dVel + this->k2 * dEta;
    //ROS_INFO_STREAM("k1"<< k1);
    //ROS_INFO_STREAM("k2"<< k2);
    //ROS_INFO_STREAM("dEta"<< dEta);
    //ROS_INFO_STREAM("dVel"<< dVel);
    return a;    
}

Eigen::Matrix<double,6,1> ModelDO::getControlGF()
// get control in generalized forces
{
	Eigen::Matrix<double,6,1> a_r = getDesiaredAcc();

	
	// this->tau = M*a_r + G + 0*(- this->augmented_disturbance + P());
	this->tau = M*a_r + G - this->augmented_disturbance + P();
	ROS_INFO_STREAM("disturbane: "<< this->augmented_disturbance);
	ROS_INFO_STREAM("a_r: "<< a_r);
	ROS_INFO_STREAM("G: "<< G);
	this->tau = imposeSaturation(this->tau);
	
	return this->tau;
}

void ModelDO::updateControlParam()
// get time derivative of augmented_disturbance
{
	double dt = calcTimeStep();
	
	Eigen::Matrix<double,3,1> z;
	z<<0,0,1;
	Eigen::Matrix<double,3,3> R = rpy2rot(this->eta2_d);
	this->G.block(3,0,3,1) = rpy2rot(this->eta2).transpose()*this->DGB * (R*zGB).cross(z);
	this->G.block(0,0,3,1) = rpy2rot(this->eta2).transpose()* additionalBuo();
	this->augmented_disturbance += getDerivativeAugmentedDisturbance()*dt;
	
	 
}
  
Eigen::Matrix<double,3,1> ModelDO::additionalBuo()
{
    double depth;
    depth = this->eta1_d(2);
    
    if (depth<-0.8)
    {
      depth = -0.8;
    }
    if (depth>-0.5)
    {  depth = -0.5;
    }
    
    Eigen::Matrix<double,3,1> output;
    output<<0,0,this->BuoF * (depth+0.4);
    
    
    return output;
}