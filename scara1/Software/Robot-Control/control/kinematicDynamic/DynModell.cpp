#include "DynModell.hpp"
#include <math.h>
#include <iostream>

using namespace scara;
using namespace eeros::control;
using namespace eeros::math;

DynModell::DynModell(double m,double c) : m(m), c(c) {
	
}

DynModell::~DynModell() { 
	// nothing to do...
} 

void DynModell::run(){
	Vector4 cartpdd;
	Vector4 inForce;
	Vector4 inCartPos;
	Vector4 inCartVel;
	
// 	
	inCartPos = getInCartPos().getSignal().getValue();
	inCartVel = getInCartVel().getSignal().getValue();
	inForce = getInForce().getSignal().getValue();

	// 5. Calculate transpose jacobi
	double x = 1,y = 1,z = 1,mz = 1;
	double xx = 1, yy = 1, zz = 1, mzz = 1;
	
	if(abs(inCartVel[0]) <= 0.1){
		xx = 0;
	}
	if(abs(inCartVel[1]) <= 0.1){
		yy = 0;
	}
	if(abs(inCartVel[2]) <= 0.1){
		zz = 0;
	}
	if(abs(inCartVel[3]) <= 0.1){
		mz = 0;
	}
	if(abs(inForce[0]) <=1 ){
		x = 0;
	}
	if(abs(inForce[1]) <=1 ){
		y = 0;
	}
	if(abs(inForce[2]) <=1 ){
		z = 0;
	}
	if(abs(inForce[3]) <=1 ){
		mz = 0;
	}
// 	static int i;
// 	if(i >= 2000){
// 		std::cout << " Force  = " << getInForce().getSignal().getValue() << std::endl;
// 		std::cout << " " <<  std::endl;
// 		std::cout << " Velocity = " << getInCartVel().getSignal().getValue() << std::endl;
// 		std::cout << " " <<  std::endl;
// 		std::cout << " out = " << cartpdd << std::endl;
// 		std::cout << " " <<  std::endl;
// 		std::cout << " xx = " << xx <<  std::endl;
// 		std::cout << " yy = " << yy <<  std::endl;
// 		std::cout << " zz = " << zz <<  std::endl;
// 		std::cout << " mzz = " << mzz <<  std::endl;
// 		std::cout << " " <<  std::endl;
// 		std::cout << " x = " << x<<  std::endl;
// 		std::cout << " y = " << y<<  std::endl;
// 		std::cout << " z = " << z<<  std::endl;
// 		std::cout << " mz = " << mz<<  std::endl;
// 		std::cout << " " <<  std::endl;
// 		i = 0;
// 	}
// 	i++;
	
	cartpdd[0] =+x*inForce[0]/m-inCartVel[0]*c;
	cartpdd[1] =-y*inForce[1]/m-inCartVel[1]*c;
	cartpdd[2] =+z*inForce[2]/m-inCartVel[2]*c;
	cartpdd[3] =+mz*inForce[3]/m-inCartVel[3]*c;

	out.getSignal().setValue(cartpdd);
	out.getSignal().setTimestamp(this->inForce.getSignal().getTimestamp());
}