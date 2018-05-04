#include "ScaraInverseKinematic.hpp"
#include <math.h>
#include <iostream>

using namespace scara;
using namespace eeros::control;
using namespace eeros::math;

ScaraInverseKinematic::ScaraInverseKinematic(double l1, double l2) : l1(l1), l2(l2) {
    r_min = 2*this->l1*sin(alpha/2);
    R_max = 2*this->l1*cos(alpha/2);
	
	cartesianCoords.zero();
}

ScaraInverseKinematic::~ScaraInverseKinematic() { 
	// nothing to do...
}

Output<Vector4>& ScaraInverseKinematic::getOutCart(){
	return outCart;
}

void ScaraInverseKinematic::run(){
	Vector4 jointCoords;
	
	double R2 = in.getSignal().getValue()(0)*in.getSignal().getValue()(0)+in.getSignal().getValue()(1)*in.getSignal().getValue()(1);
	double R = sqrt(R2);
	
	// 1. Check x,y coordinates
	if(R2<r_min*r_min)
	{
	  // invalid input (small)
	  cartesianCoords[0] = in.getSignal().getValue()(0)*r_min/R; 
	  cartesianCoords[1] = in.getSignal().getValue()(1)*r_min/R;
// 	  std::cout << "R2<rmin²" << in.getSignal().getValue()(0) << "; " << in.getSignal().getValue()(1) << "; " << R2 << "; " << r_min*r_min <<std::endl;
	}
	else if(R2>R_max*R_max)
	{
	  // invalid input (big)
	  cartesianCoords[0] = in.getSignal().getValue()(0)*R_max/R;
	  cartesianCoords[1] = in.getSignal().getValue()(1)*R_max/R;
// 	  std::cout << "R>Rmax²" << in.getSignal().getValue()(0) << "; " << in.getSignal().getValue()(1) << "; " <<  R2 << "; " << R_max*R_max <<std::endl;
	}
	else
	{
	  cartesianCoords[0] = in.getSignal().getValue()(0);
	  cartesianCoords[1] = in.getSignal().getValue()(1);
	}
	
	// 2. Check z coordinate
	if(in.getSignal().getValue()[2]<z_min)
	{
		cartesianCoords[2] = z_min;
// 		std::cout << "z_min" << in.getSignal().getValue()[2] << "; " << cartesianCoords[2] << std::endl;
	}
	else if(in.getSignal().getValue()[2]>z_max)
	{
		cartesianCoords[2] = z_max;
// 		std::cout << "z_max" << in.getSignal().getValue()[2] << "; " << cartesianCoords[2] << std::endl;
	}
	else
	  cartesianCoords[2] = in.getSignal().getValue()(2);
	
	// 3. Check alpha coordinate
	if(in.getSignal().getValue()[3]<alpha_min)
	{
		cartesianCoords[3] = alpha_min;
// 		std::cout << "alpha_min" << in.getSignal().getValue()[3] << "; " << cartesianCoords[3] << std::endl;
	}
	else if(in.getSignal().getValue()[3]>alpha_max)
	{
		cartesianCoords[3] = alpha_max; 
// 		std::cout << "alpha_max " << in.getSignal().getValue()[3] << "; " << cartesianCoords[3] << std::endl;
	}
	else
	{
		cartesianCoords[3] = in.getSignal().getValue()(3);
	}
	
// 	cartesianCoords = in.getSignal().getValue();
	
	// 4. Calculate inverse kinematic
	jointCoords[1] = acos((-l1*l1-l2*l2+cartesianCoords[0]*cartesianCoords[0]+cartesianCoords[1]*cartesianCoords[1])/(2*l1*l2));
 
	if (jointCoords[1] >= 0)
		jointCoords[0] = atan2(cartesianCoords[1],cartesianCoords[0]) - acos((cartesianCoords[0]*cartesianCoords[0]+cartesianCoords[1]*cartesianCoords[1]+l1*l1-l2*l2)/(2*l1*sqrt(cartesianCoords[0]*cartesianCoords[0]+cartesianCoords[1]*cartesianCoords[1])));
	else
		jointCoords[0] = atan2(cartesianCoords[1],cartesianCoords[0]) + acos((cartesianCoords[0]*cartesianCoords[0]+cartesianCoords[1]*cartesianCoords[1]+l1*l1-l2*l2)/(2*l1*sqrt(cartesianCoords[0]*cartesianCoords[0]+cartesianCoords[1]*cartesianCoords[1])));
    
	jointCoords[0] = -jointCoords[0];
	jointCoords[1] = -jointCoords[1];
	jointCoords[2] = -cartesianCoords[2];
	jointCoords[3] = -jointCoords[0]-jointCoords[1]-cartesianCoords[3];
	
	out.getSignal().setValue(jointCoords);
	out.getSignal().setTimestamp(this->in.getSignal().getTimestamp());
	
	outCart.getSignal().setValue(cartesianCoords);
	outCart.getSignal().setTimestamp(this->in.getSignal().getTimestamp());
}