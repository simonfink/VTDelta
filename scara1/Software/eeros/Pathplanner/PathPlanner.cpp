#include <iostream>
#include <stdlib.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <eeros/control/Output.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/D.hpp>
#include <eeros/core/System.hpp>
#include "PathPlanner.hpp"

using namespace scara;
using namespace eeros;

PathPlanner::PathPlanner(Vector4 velMax, Vector4 accMax, Vector4 decMax, double dt, uint32_t minSafetyLevel, TrajectoryType trajType) : 
					velMax(velMax), accMax(accMax), decMax(decMax), dt(dt), minSafetyLevel(minSafetyLevel), trajType(trajType) {
	this->indexAddPos = 0;
	this->indexReadPos = 0;
	this->isNewValue = false;
	setTrajCoeff(trajType);
}
 
PathPlanner::PathPlanner(Vector4 velMax, Vector4 accMax, double dt, uint32_t minSafetyLevel,TrajectoryType trajType) : 
					velMax(velMax), accMax(accMax), decMax(accMax), dt(dt), minSafetyLevel(minSafetyLevel), trajType(trajType) {  
      this->indexAddPos = 0;
      this->indexReadPos = 0;
	  this->isNewValue = false;
	  setTrajCoeff(trajType);
 }

// *** public *** //
PathPlanner::~PathPlanner() { 
	    // nothing to do...
    }
void PathPlanner::reset() {
   first = true;
   disable();
 }
void PathPlanner::enable(){
	 time = System::getTime();
	 tOffset = time;
	 enabled = true;
}
void PathPlanner::disable(){
   enabled = false;
   trajParamSet = false;
   // reset vectors isNewValue
   isNewValue = false;
   // reset index Read and Add
   indexReadPos = 0;
   indexAddPos = 0;
   pathEnded = false;
}

bool PathPlanner::goPoint(Vector4 posFinal){
	disable();
	addPosition(posFinal);
	enable();
	calculateTrajectoryParameters();
	while(!pathEnded){
		std::cout << pos.getSignal().getValue()(0) << std::endl;
		std::cout << pos.getSignal().getValue()(1) << std::endl;
		std::cout << pos.getSignal().getValue()(2) << std::endl;
		std::cout << pos.getSignal().getValue()(3) << std::endl;
	};
	disable();
	return 1;
}
 
void PathPlanner::setVelMax(Vector4 velMax) { 
	 mutex.lock();
	 this->velMax = velMax; 
	 mutex.unlock();
    }
void PathPlanner::setAccMax(Vector4 accMax) {
	 mutex.lock();
	 this->accMax = accMax;
	 mutex.unlock();
    }
void PathPlanner::setDecMax(Vector4 decMax) {
	 mutex.lock();
	 this->decMax = decMax; 
	 mutex.unlock();
    }
    
void PathPlanner::addPosition(Vector4 posFinal) {
   mutex.lock();
   if (isNewValue[indexAddPos] == false){
     posFinalBuffer[indexAddPos] = posFinal;
     isNewValue[indexAddPos] = true; 
     // update index for adding new positions
     indexAddPos = indexAddPos + 1;
   }
   else
     // TODO throw exception --> do not increment indexAddPos and do not save new point
     std::cout << "Exceeding max biffer size" << std::endl;
   mutex.unlock();
}   
void PathPlanner::setInitialPosition(Vector4 posInit) {
	mutex.lock();
	this->posFinalPrev = posInit;
	mutex.unlock();
}

Output<Vector4>& PathPlanner::getOutPos() { 
	return pos; 
}
Output<Vector4>& PathPlanner::getOutVel() { 
	return vel;
}
Output<Vector4>& PathPlanner::getOutAcc() { 
	return acc;
}

// *** private *** // 
 void PathPlanner::readPosition() { 
   mutex.lock();
   if(isNewValue[indexReadPos] == true){
       this->isNewValue[indexReadPos] = false; 
    }
    mutex.unlock();
} 
 void PathPlanner::updateIndexReadPos() { 
   mutex.lock();
     if (isNewValue[indexReadPos+1] == false)
       indexReadPos = indexReadPos;
     else
       indexReadPos = indexReadPos + 1; 
   mutex.unlock();
} 

void PathPlanner::setTrajCoeff(TrajectoryType trajType) {
	switch(trajType) {
		case linearVelocity:
			trajCoeff = 1;
			break;
		case limitedJerk:
			trajCoeff = 2.0/3.0;
			break;
		case limitedJerkSquare:
			trajCoeff = 0.5;
			break;
		case limitedSnap:
			trajCoeff = 0.625;
			break;
		case trigonometric:
			trajCoeff = 0.636619772369;
			break;
		default:
			trajCoeff = 1;
			break;
	}
}
void PathPlanner::calculateTrajectoryParameters(){

   Vector4 calcVelNorm, calcAccNorm, calcDecNorm;
   double squareNormVel; 
   
  if(trajParamSet == false) {
    
    // read new positions
    readPosition();
	// calculate vel norm and acc norm for each axis
	distance = posFinalBuffer[indexReadPos] - posFinalPrev;
	
	for(sigdim_t i = 0; i < velMax.getNofRows(); i++) { 
		calcVelNorm(i) = fabs((velMax(i)/distance(i))*trajCoeff);
		calcAccNorm(i) = fabs((accMax(i)/distance(i))*trajCoeff);
		calcDecNorm(i) = fabs((decMax(i)/distance(i))*trajCoeff);
	}
	// init velNorm, accNorm, decNorm
	velNorm = calcVelNorm(0);
	accNorm = calcAccNorm(0);
	decNorm = calcDecNorm(0);
	for(sigdim_t i = 0; i < calcVelNorm.getNofRows(); i++) {
		// find min value for vel norm and acc norm
		if(calcVelNorm(i)<velNorm)
			velNorm = calcVelNorm(i);
		if(calcAccNorm(i)<accNorm)
			accNorm = calcAccNorm(i);
		if(calcDecNorm(i)<decNorm)
			decNorm = calcDecNorm(i);
	}
	// minimize velocity
	squareNormVel = sqrt(2*(accNorm*decNorm)/(accNorm + decNorm));
	if(velNorm > squareNormVel)
		velNorm = squareNormVel; 
	   // calculate time intervals    
	   dT1 = velNorm/accNorm;
	   dT3 = velNorm/decNorm;
	   dT2 = 1/velNorm - (dT1 + dT3)*0.5;
	   if (dT2<0)
		   dT2 = 0;
	   // adaptation to timestamps
	   dT1 = ceil(dT1/dt)*dt;
	   dT2 = ceil(dT2/dt)*dt;
	   dT3 = ceil(dT3/dt)*dt; 
	   // adaptation of speed to new timestamps
	   velNorm = 1/((dT2 + (dT1 + dT3)*0.5)*dt);

	   trajParamSet = true;     
   }
}

double PathPlanner::setPosGain(double k, double dK){   
	double gainP, u, k2; 
	switch(trajType) {
		case linearVelocity:
			u = k/dK; 
			gainP = 0.5*dt*k*u;
			break;
		case limitedJerk:
			u = k/dK; 			
			gainP = k*dt*u*u*(1-u*0.5);
			break;
		case limitedJerkSquare:
			if(2*k<dK){
				u = k/dK;
				gainP = dt*(2.0/3.0)*k*u*u;
			}
			else{
				k2=dK-k;
				u = k2/dK;
				gainP = dt*(dK*0.5-k2+(2.0/3.0)*k2*u*u); 
			}
			break;
		case limitedSnap:
			u = k/dK;
			gainP = k*dt*0.5*u*u*u*(2.0*u*u - 6.0*u + 5.0);
			break;
		case trigonometric:
			u = k/dK;
			gainP = dt*0.5*(k - dK/M_PI*sin(u*M_PI));
			break;
		default:
			u = k/dK; 
			gainP = 0.5*dt*k*u;
			break;
	}
	return gainP;
} 
double PathPlanner::setVelGain(double k, double dK){
	double gainV, u; 
	switch(trajType) {
		case linearVelocity:
			u = k/dK; 
			gainV = u;
			break;
		case limitedJerk:
			u = k/dK; 
			gainV = u*u*(3.0-2.0*u);
			break;
		case limitedJerkSquare:
			if(2*k<dK){
				u = k/dK;
				gainV = 2.0*u*u;
			}
			else{
				u = (dK-k)/dK;
				gainV = 1.0 - 2.0*u*u;
			}
			break;
		case limitedSnap:
			u = k/dK;
			gainV = u*u*u*(6.0*u*u-15.0*u+10.0);
			break;
		case trigonometric:
			u = k/dK;
			gainV = 0.5*(1-cos(u*M_PI));
			break;
		default:
			u = k/dK; 
			gainV = u;
			break;
	}
	return gainV;
}
double PathPlanner::setAccGain(double k, double dK){
	double gainA, u; 
	switch(trajType) {
		case linearVelocity:
			gainA = 1/dK/dt;
			break;
		case limitedJerk:
			u = k/dK; 
			gainA = 1/dK/dt*u*6*(1-u);
			break;
		case limitedJerkSquare:
			if(2*k<dK){
				u = k/dK;
				gainA = 4.0*u/dK/dt;
			}
			else{
				u = (dK-k)/dK;
				gainA = 4.0*u/dK/dt;
			}
			break;
		case limitedSnap:
			u = k/dK;
			gainA = 1.0/dK/dt*30.0*u*u*(u*u-2.0*u+1.0);
			break;
		case trigonometric:
			u = k/dK;
			gainA = 0.5*sin(u*M_PI)*M_PI/dK/dt;
			break;
		default:
			gainA = 1/dK/dt;
			break;
	}
	return gainA;
}
 
void PathPlanner::checkPath(){
	if(isNewValue[indexReadPos+1] == false)
      pathEnded = true; 
    else
      pathEnded = false;  
}

 // *** run *** //
 void PathPlanner::run() {
	     
  time = System::getTime();
  timeScaled = time - tOffset; 
  
  if(first) { 
	acc.getSignal().setValue(0);
	vel.getSignal().setValue(0);
	pos.getSignal().setValue(posFinalPrev);
	acc.getSignal().setTimestamp(System::getTimeNs());
	vel.getSignal().setTimestamp(System::getTimeNs());
	pos.getSignal().setTimestamp(System::getTimeNs());
	first = false;   
  }
  else{
// 	if(safetySys.getCurrentLevel().getId()>=minSafetyLevel){
		if(enabled){
			if(timeScaled < 0) {
				acc.getSignal().setValue(0.0*distance*dt*dt);
				vel.getSignal().setValue(0.0*distance*dt);
				pos.getSignal().setValue(posFinalPrev);
			}
			else if(timeScaled < dT1) {
				acc.getSignal().setValue((velNorm*setAccGain(timeScaled, dT1))*distance*dt*dt);
				vel.getSignal().setValue((velNorm*setVelGain(timeScaled, dT1))*distance*dt);
				pos.getSignal().setValue(posFinalPrev + (velNorm*setPosGain(timeScaled, dT1))*distance);
			}
			else if(timeScaled < dT1 + dT2) { 
				acc.getSignal().setValue((0.0)*distance*dt*dt);
				vel.getSignal().setValue((velNorm)*distance*dt);
				pos.getSignal().setValue(posFinalPrev + (velNorm*((time-tOffset)-0.5*dT1)*dt)*distance); 
			}
			else if (timeScaled < dT1 + dT2 + dT3) {
				acc.getSignal().setValue((-velNorm*setAccGain((dT1+dT2+dT3)-(timeScaled), dT3))*distance*dt*dt);
				vel.getSignal().setValue((velNorm*setVelGain((dT1+dT2+dT3)-(timeScaled), dT3))*distance*dt);		
				pos.getSignal().setValue(posFinalPrev + (1-velNorm*setPosGain((dT1+dT2+dT3)-(timeScaled), dT3))*distance);
			}
			else { 
				acc.getSignal().setValue(0.0);
				vel.getSignal().setValue(0.0);
				pos.getSignal().setValue(posPrev);
				
				tOffset = time; 	  
				trajParamSet = false; 
				posFinalPrev = pos.getSignal().getValue();
				checkPath();
				// update reading index
				updateIndexReadPos();
			}
			acc.getSignal().setTimestamp(System::getTimeNs());
			vel.getSignal().setTimestamp(System::getTimeNs());
			pos.getSignal().setTimestamp(System::getTimeNs());
			
			posPrev = pos.getSignal().getValue();
		}
		else{ // disabled    
				acc.getSignal().setValue(0.0);
				vel.getSignal().setValue(0.0);
				pos.getSignal().setValue(posFinalPrev);
				acc.getSignal().setTimestamp(System::getTimeNs());
				vel.getSignal().setTimestamp(System::getTimeNs());
				pos.getSignal().setTimestamp(System::getTimeNs());
		}
// 	}
// 	else{
// 		acc.getSignal().setValue(0.0);
// 		vel.getSignal().setValue(0.0);
// 		pos.getSignal().setValue(posPrev);
// 		acc.getSignal().setTimestamp(System::getTimeNs());
// 		vel.getSignal().setTimestamp(System::getTimeNs());
// 		pos.getSignal().setTimestamp(System::getTimeNs());
// 	}
  }
  timePrev = time;
 }
