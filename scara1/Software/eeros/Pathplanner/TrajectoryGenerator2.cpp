#include <iostream>
#include <stdlib.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include "TrajectoryGenerator2.hpp"
#include <eeros/control/RealSignalOutput.hpp>
#include <eeros/control/RealSignalInput.hpp>
#include <eeros/control/D.hpp>
#include <eeros/core/System.hpp>

// definition of possible trajectories
#define LINEARVELOCITY 		0
#define LIMITEDJERK 		1
#define LIMITEDJERKSQUARE 	2
#define LIMITEDSNAP 		3
#define TRIGONOMETRIC 		4	

TrajectoryGenerator2::TrajectoryGenerator2(const double velMax[], const double accMax[], const double decMax[], double dt, 
					 sigdim_t dim, int trajType) : pos(dim), vel(dim), acc(dim), posFinalPrev(dim), 
					 posPrev(dim), posHelp(dim), posA(2), posB(2), cCircle(2), velMax(dim), 
					 accMax(dim), decMax(dim), isNewValue(dim), isHelpValue(dim), distance(dim), 
					 distance1(dim), distance2(dim) {  
      this->dim = dim;
      this->dt = dt; 
      this->trajType = trajType;
      this->tOffset = 4.0;
      this->velNorm = 1000;
      this->velNorm1 = 1000;
      this->velNorm2 = 1000;
      first = true;
      trajParamSet = false;
      smoothTrajectory = false; 
      helpParamError = false;
      for(sigdim_t i = 0; i < dim; i++) {	
	this->velMax[i] = velMax[i];
	this->accMax[i] = accMax[i];
	this->decMax[i] = decMax[i];	
      }      
      this->indexAddPos = 0;
      this->indexReadPos = 0;
      posFinal.resize(dimBuffer);
      for (sigdim_t i = 0; i<dimBuffer; i++) {
	posFinal[i].resize(dim);
      }     
 }
 
TrajectoryGenerator2::TrajectoryGenerator2(const double velMax[], const double accMax[], double dt, sigdim_t dim, 
					 int trajType) : pos(dim), vel(dim), acc(dim), posFinalPrev(dim), posPrev(dim), 
					 posHelp(dim), posA(2), posB(2), cCircle(2), velMax(dim), accMax(dim), 
					 decMax(dim), isNewValue(dim), isHelpValue(dim), distance(dim), 
					 distance1(dim), distance2(dim) {  
      this->dim = dim;
      this->dt = dt; 
      this->trajType = trajType;
      this->tOffset = 4.0;
      this->velNorm = 1000;
      this->velNorm1 = 1000;
      this->velNorm2 = 1000;
      first = true;
      trajParamSet = false;   
      smoothTrajectory = false; 
      helpParamError = false;
      for(sigdim_t i = 0; i < dim; i++) {	
	this->velMax[i] = velMax[i];
	this->accMax[i] = accMax[i];
	this->decMax[i] = accMax[i];	
      }     
      this->indexAddPos = 0;
      this->indexReadPos = 0;
      posFinal.resize(dimBuffer);
      for (sigdim_t i = 0; i<dimBuffer; i++) {
	posFinal[i].resize(dim);
      }     
 }
  
 
 TrajectoryGenerator2::~TrajectoryGenerator2() { 
	    // nothing to do...
    }
 
 void TrajectoryGenerator2::reset() { 
	    first = true;
	    trajParamSet = false;
    }
    
 void TrajectoryGenerator2::setVelMax(const double velMax[]) { 
	    for(sigdim_t i = 0; i < dim; i++) {	    
	      this->velMax[i] = velMax[i];
	      }
    }
  
 void TrajectoryGenerator2::setAccMax(const double accMax[]) {
   	    for(sigdim_t i = 0; i < dim; i++) {	    
	      this->accMax[i] = accMax[i];
	      }
    }

 void TrajectoryGenerator2::setDecMax(const double decMax[]) {
   	    for(sigdim_t i = 0; i < dim; i++) {	    
	      this->decMax[i] = decMax[i];
	      }
    }
    
 void TrajectoryGenerator2::setPosition(const double posFinal[], bool isHelp) {  	
      if (isNewValue[indexAddPos] == 0){
	for(sigdim_t i = 0; i < dim; i++) {
	  this->posFinal[indexAddPos][i] = posFinal[i];
	  }
	this->isNewValue[indexAddPos] = 1;
	if(isHelp == 1){				// added  this if cyclus
	  this->isHelpValue[indexAddPos] = 1;
	}						// until here
      }
      else
	std::cout << "Buffer size is: " << dimBuffer << ": cannot load all positions" << std::endl;
 
    if (indexAddPos % dimBuffer == (dimBuffer-1)) 
       indexAddPos = 0;
     else 
       indexAddPos = indexAddPos + 1;    
}   

void TrajectoryGenerator2::setLambda(int trajType){
  if (trajType == LINEARVELOCITY)
    this->lambda = 1;
  else if (trajType == LIMITEDJERK)
    this->lambda = 0.666666666666;
  else if (trajType == LIMITEDJERKSQUARE)
    this->lambda = 0.5;
  else if (trajType == LIMITEDSNAP)
    this->lambda = 0.625;
  else if (trajType == TRIGONOMETRIC)
    this->lambda = 0.636619772369;
}
 
 RealSignalOutput& TrajectoryGenerator2::getOut(int outputIndex) { 
      if (outputIndex == 0)
	return pos; 
      else if (outputIndex == 1)
	return vel;
      else if (outputIndex == 2)
	return acc;
    }

void TrajectoryGenerator2::setSmoothCurvesParameters(){

  double m1, q1, a1, b1, c1, m2, q2, a2, b2, c2;
  double m3, q3, a3, b3, c3, m4, q4, a4, b4, c4;
  double alpha1, alpha2, alpha3, alpha4;

  // find coordinates of beginning and end of circular trajectory
  m1 = (posHelp[1] - posFinalPrev[1])/(posHelp[0] - posFinalPrev[0]);
  q1 = -posFinalPrev[0]*m1 + posFinalPrev[1];
  a1 = m1; b1 = -1; c1 = q1;
  
  m2 = (posFinal[indexReadPos][1] - posHelp[1])/(posFinal[indexReadPos][0] - posHelp[0]);
  q2 = -posFinal[indexReadPos][0]*m2 + posFinal[indexReadPos][1];
  a2 = m2; b2 = -1; c2 = q2;
  
  alpha1 = atan(m1);
  alpha2 = atan(m2);
  
  if(posHelp[0]>posFinalPrev[0]){
      posA[0] = posHelp[0] - dCircle*cos(alpha1);
      posA[1] = posHelp[1] - dCircle*sin(alpha1);
  }
  else{
      posA[0] = posHelp[0] + dCircle*cos(alpha1);
      posA[1] = posHelp[1] + dCircle*sin(alpha1);
  }
  
  if(posHelp[0]<posFinal[indexReadPos][0]){
      posB[0] = posHelp[0] + dCircle*cos(alpha2);
      posB[1] = posHelp[1] + dCircle*sin(alpha2);
  }
  else{
      posB[0] = posHelp[0] - dCircle*cos(alpha2);
      posB[1] = posHelp[1] - dCircle*sin(alpha2);
  }
	  	  
if ( (posA[0]<posHelp[0] && posA[0]<posFinalPrev[0]) || (posA[0]>posHelp[0] && posA[0]>posFinalPrev[0]) 
  || (posB[0]<posHelp[0] && posB[0]<posFinal[indexReadPos][0]) || (posB[0]>posHelp[0] && posB[0]>posFinal[indexReadPos][0]) 
  || (posA[1]<posHelp[1] && posA[1]<posFinalPrev[1]) || (posA[1]>posHelp[1] && posA[1]>posFinalPrev[1]) 
  || (posB[1]<posHelp[1] && posB[1]<posFinal[indexReadPos][1]) || (posB[1]>posHelp[1] && posB[1]>posFinal[indexReadPos][1])) {
  std::cout << "Cannot use this help point, trajectory will reach the following point" << std::endl;
  helpParamError = 1;
  }
	  
  // find circle centre
  m3 = -1/m1;
  q3 = -posA[0]*m3 + posA[1];
  a3 = m3; b3 = -1; c3 = q3;

  m4 = -1/m2;
  q4 = -posB[0]*m4 + posB[1];
  a4 = m4; b4 = -1; c4 = q4;
  
  cCircle[0] = (q4-q3)/(m3-m4);
  cCircle[1] = m3*cCircle[0] + q3;
  rCircle = sqrt((cCircle[1]-posA[1])*(cCircle[1]-posA[1]) + (cCircle[0]-posA[0])*(cCircle[0]-posA[0]));
  
  // find angle of circumference arc
  alpha3 = atan(m3);
  alpha4 = atan(m4);
   
if(posFinalPrev[0]<=posHelp[0] && posFinal[indexReadPos][0]>=posHelp[0]){
    if((posFinalPrev[1]<=posHelp[1] && posFinal[indexReadPos][1]>=posHelp[1])||(posFinal[indexReadPos][1]<=posHelp[1] && posFinalPrev[1]>=posHelp[1])){
        thetaCircleInit = M_PI+alpha3;
        thetaCircleEnd = M_PI+alpha4;
    }
    else if((posFinalPrev[1]<=posFinal[indexReadPos][1] && posFinal[indexReadPos][1]<=posHelp[1])||(posFinal[indexReadPos][1]<=posFinalPrev[1] && posFinalPrev[1]<=posHelp[1])){
        thetaCircleInit = M_PI+alpha3;
        thetaCircleEnd = alpha4;
    }
    else if((posHelp[1]<=posFinalPrev[1] && posFinalPrev[1]<=posFinal[indexReadPos][1])||(posHelp[1]<=posFinal[indexReadPos][1] && posFinal[indexReadPos][1]<=posFinalPrev[1])){
        thetaCircleInit = M_PI+alpha3;
        thetaCircleEnd = 2*M_PI+alpha4;
    }
}
else if(posFinal[indexReadPos][0]<=posHelp[0] && posFinalPrev[0]>=posHelp[0]){
    if((posFinalPrev[1]<=posHelp[1] && posFinal[indexReadPos][1]>=posHelp[1])||(posFinal[indexReadPos][1]<=posHelp[1] && posFinalPrev[1]>=posHelp[1])){
        thetaCircleInit = M_PI+alpha3;
        thetaCircleEnd = M_PI+alpha4;
    }
    else if((posFinalPrev[1]<=posFinal[indexReadPos][1] && posFinal[indexReadPos][1]<=posHelp[1])||(posFinal[indexReadPos][1]<=posFinalPrev[1] && posFinalPrev[1]<=posHelp[1])){
        thetaCircleInit = alpha3;
        thetaCircleEnd = M_PI+alpha4;
    }
    else if((posHelp[1]<=posFinalPrev[1] && posFinalPrev[1]<=posFinal[indexReadPos][1])||(posHelp[1]<=posFinal[indexReadPos][1] && posFinal[indexReadPos][1]<=posFinalPrev[1])){
        thetaCircleInit = 2*M_PI+alpha3;
        thetaCircleEnd = M_PI+alpha4;
    }
}
else if(posFinalPrev[0]<=posFinal[indexReadPos][0] && posFinal[indexReadPos][0]<=posHelp[0]){
    if((posFinalPrev[1]<=posHelp[1] && posFinal[indexReadPos][1]>=posHelp[1])||(posFinal[indexReadPos][1]<=posHelp[1] && posFinalPrev[1]>=posHelp[1])){
        thetaCircleInit = alpha3;
        thetaCircleEnd = alpha4;
    }
    else if((posFinalPrev[1]<=posFinal[indexReadPos][1] && posFinal[indexReadPos][1]<=posHelp[1])||(posFinal[indexReadPos][1]<=posFinalPrev[1] && posFinalPrev[1]<=posHelp[1])){
        thetaCircleInit = M_PI+alpha3;
        thetaCircleEnd = alpha4;
    }
    else if((posHelp[1]<=posFinalPrev[1] && posFinalPrev[1]<=posFinal[indexReadPos][1])||(posHelp[1]<=posFinal[indexReadPos][1] && posFinal[indexReadPos][1]<=posFinalPrev[1])){
        thetaCircleInit = M_PI+alpha3;
        thetaCircleEnd = 2*M_PI+alpha4;
    }
}
else if(posFinal[indexReadPos][0]<=posFinalPrev[0] && posFinalPrev[0]<=posHelp[0]) {
    if((posFinalPrev[1]=posHelp[1] && posFinal[indexReadPos][1]>=posHelp[1])||(posFinal[indexReadPos][1]<=posHelp[1] && posFinalPrev[1]>=posHelp[1])){
        thetaCircleInit = alpha3;
        thetaCircleEnd = alpha4;
    }
    else if((posFinalPrev[1]<=posFinal[indexReadPos][1] && posFinal[indexReadPos][1]<=posHelp[1])||(posFinal[indexReadPos][1]<=posFinalPrev[1] && posFinalPrev[1]<=posHelp[1])){
        thetaCircleInit = alpha3;
        thetaCircleEnd = M_PI+alpha4;
    }
    else if((posHelp[1]<=posFinalPrev[1] && posFinalPrev[1]<=posFinal[indexReadPos][1])||(posHelp[1]<=posFinal[indexReadPos][1] && posFinal[indexReadPos][1]<=posFinalPrev[1])){
        thetaCircleInit = alpha3;
        thetaCircleEnd = -alpha4-M_PI/2;
    }
}
else if(posHelp[0]<=posFinalPrev[0] && posFinalPrev[0]<=posFinal[indexReadPos][0]){
    if((posFinalPrev[1]<=posHelp[1] && posFinal[indexReadPos][1]>=posHelp[1])||(posFinal[indexReadPos][1]<=posHelp[1] && posFinalPrev[1]>=posHelp[1])){
        thetaCircleInit = M_PI+alpha3;
        thetaCircleEnd = M_PI+alpha4;
    }
    else if(posFinalPrev[1]<=posFinal[indexReadPos][1] && posFinal[indexReadPos][1]<=posHelp[1]){
        thetaCircleInit = M_PI+alpha3;
        thetaCircleEnd = alpha4;
    }
    else if(posFinal[indexReadPos][1]<=posFinalPrev[1] && posFinalPrev[1]<=posHelp[1]){
        thetaCircleInit = alpha3;
        thetaCircleEnd = M_PI+alpha4;
    }
    else if((posHelp[1]<=posFinalPrev[1] && posFinalPrev[1]<=posFinal[indexReadPos][1])||(posHelp[1]<=posFinal[indexReadPos][1] && posFinal[indexReadPos][1]<=posFinalPrev[1])){
        thetaCircleInit = M_PI+alpha3;
        thetaCircleEnd = 2*M_PI+alpha4;
    }
}
else if(posHelp[0]<=posFinal[indexReadPos][0] && posFinal[indexReadPos][0]<=posFinalPrev[0]) { 
    if((posFinalPrev[1]<=posHelp[1] && posFinal[indexReadPos][1]>=posHelp[1])||(posFinal[indexReadPos][1]<=posHelp[1] && posFinalPrev[1]>=posHelp[1])) {
        thetaCircleInit = M_PI+alpha3;
        thetaCircleEnd = M_PI+alpha4;
    }
    else if(posFinalPrev[1]<=posFinal[indexReadPos][1] && posFinal[indexReadPos][1]<=posHelp[1]){
        thetaCircleInit = M_PI+alpha3;
        thetaCircleEnd = alpha4;
     }
    else if(posFinal[indexReadPos][1]<=posFinalPrev[1] && posFinalPrev[1]<=posHelp[1]) {
        thetaCircleInit = alpha3;
        thetaCircleEnd = M_PI+alpha4;
    }
    else if((posHelp[1]<=posFinalPrev[1] && posFinalPrev[1]<=posFinal[indexReadPos][1])||(posHelp[1]<=posFinal[indexReadPos][1] && posFinal[indexReadPos][1]<=posFinalPrev[1])){
        thetaCircleInit = alpha3;
        thetaCircleEnd = -alpha4-M_PI-M_PI/4;
    }
}
 
thetaPrev = thetaCircleInit; 
// std::cout << "thetaCircleInit: " << thetaCircleInit << std::endl;
// std::cout << "thetaCircleEnd: " << thetaCircleEnd << std::endl;
  
if(fabs(thetaCircleEnd-thetaCircleInit)<0.001){
   cCircle[0] = posHelp[0];
   cCircle[1] = posHelp[1];
   rCircle = 0.0;
   std::cout << "Can't avoid help point, trajectory will pass through it" << std::endl;
}
}        
 
 void TrajectoryGenerator2::setTrajectoryParameters(){

   double calcVelNorm[dim], calcAccNorm[dim], calcDecNorm[dim];
   double calcVelNorm1[dim], calcAccNorm1[dim], calcDecNorm1[dim];
   double calcVelNorm2[dim], calcAccNorm2[dim], calcDecNorm2[dim];
   double squareNormVel, squareNormVel1, squareNormVel2;
   double accNorm = 1000, accNorm1 = 1000, accNorm2 = 1000; 
   double decNorm = 1000, decNorm1 = 1000, decNorm2 = 1000;
   
  if(trajParamSet == 0) {
    
    // calculation of the distance between the two points
    if(isNewValue[indexReadPos] == 1){
      if (isHelpValue[indexReadPos] == 1){
	for(sigdim_t i = 0; i < dim; i++)
	  this->posHelp[i] = this->posFinal[indexReadPos][i];
	
	  this->isNewValue[indexReadPos] = 0; 
	  this->isHelpValue[indexReadPos] = 0;
	  indexReadPos = indexReadPos + 1; 
	  this->isNewValue[indexReadPos] = 0; 
	  setSmoothCurvesParameters();
	  if(helpParamError){
	    smoothTrajectory = 0;
	    helpParamError = 0;
	  }
	  else
	     smoothTrajectory = 1;
	}
	else{
	  smoothTrajectory == 0; 
	  this->isNewValue[indexReadPos] = 0;  
	}
    }
    else {
	smoothTrajectory == 0; 
	for(sigdim_t i = 0; i < dim; i++)
	  distance[i] = 0.0;
    }
    // set lambda
    setLambda(trajType);
    
    if (smoothTrajectory == 1) { 
	// Path-distances calculations
	  for(sigdim_t i = 0; i < 2; i++) {            // xy axis
	    distance1[i] = posA[i] - posFinalPrev[i]; 
	    distance2[i] = posFinal[indexReadPos][i] - posB[i];
	    }
	  for(sigdim_t i = 2; i < dim; i++) {          // z axis
	    distance1[i] = 0.0; 
	    distance2[i] = 0.0;
	    }
	
	// calculate vel norm and acc norm for each axis
	for(sigdim_t i = 0; i < 2; i++) {
	  //path 1
	  calcVelNorm1[i] = fabs(velMax[i]/distance1[i]*lambda);
	  calcAccNorm1[i] = fabs(accMax[i]/distance1[i]*lambda);
	  calcDecNorm1[i] = fabs(decMax[i]/distance1[i]*lambda);
	  // path 2
	  calcVelNorm2[i] = fabs(velMax[i]/distance2[i]*lambda); 
	  calcAccNorm2[i] = fabs(accMax[i]/distance2[i]*lambda);
	  calcDecNorm2[i] = fabs(decMax[i]/distance2[i]*lambda);
	  }  
	  
	  // find min value for vel norm and acc norm 1-2
	  for(sigdim_t i = 0; i < 2; i++) {  
	    //path 1
	    if(calcVelNorm1[i]<velNorm1){
	      velNorm1 = calcVelNorm1[i];
	    }
	    if(calcAccNorm1[i]<accNorm1) {
	      accNorm1 = calcAccNorm1[i];
	    }
	    if(calcDecNorm1[i]<decNorm1) {
	      decNorm1 = calcDecNorm1[i];
	    }
	    // path 2
	    if(calcVelNorm2[i]<velNorm2){
	      velNorm2 = calcVelNorm2[i];
	    }
	    if(calcAccNorm2[i]<accNorm2) {
	      accNorm2 = calcAccNorm2[i];
	    }
	    if(calcDecNorm2[i]<decNorm2) {
	      decNorm2 = calcDecNorm2[i];
	    }
	  }
	  
	  // minimize velocity
	  squareNormVel1 = sqrt(fabs(2*(accNorm1*decNorm1)/(accNorm1 + decNorm1)));
	  squareNormVel2 = sqrt(fabs(2*(accNorm2*decNorm2)/(accNorm2 + decNorm2)));
	    
	  if(velNorm1 > squareNormVel1)
	    velNorm1 = squareNormVel1;
	  if(velNorm2 > squareNormVel2)
	    velNorm2 = squareNormVel2;
	  
	  dT1 = velNorm1/accNorm1; 
	  dT2 = 1/fabs(velNorm1)-(dT1)*0.5;
	  dT4 = velNorm2/decNorm2;
	  dT3 = 1/fabs(velNorm2)-(dT4)*0.5;
	  dTC = ceil(fabs(thetaCircleEnd - thetaCircleInit)/fabs(velNorm1)/dt)*dt;
	 
	  if (dT2<0)
	    dT2 = 0;
	  
          if (dTC<0.0011)
	    dTC = 0;
	  
	  dT1 = ceil(dT1/dt)*dt;
	  dT2 = ceil(dT2/dt)*dt;
	  dT3 = ceil(dT3/dt)*dt;
	  dT4 = ceil(dT4/dt)*dt;
	  dTC = ceil(dTC/dt)*dt;

	  velNorm1 = 1/((dT2+0.5*dT1)*dt);
	  velNorm2 = 1/((dT3+0.5*dT4)*dt);
      }
      else {  	
	
	// calculate vel norm and acc norm for each axis
	for(sigdim_t i = 0; i < dim; i++) {
	  distance[i] = posFinal[indexReadPos][i] - posFinalPrev[i];
	  calcVelNorm[i] = velMax[i]/distance[i]*lambda;
	  calcAccNorm[i] = accMax[i]/distance[i]*lambda;
	  calcDecNorm[i] = decMax[i]/distance[i]*lambda;
	  } 
	  
	  // find min value for vel norm and acc norm
	  for(sigdim_t i = 0; i < dim; i++) {
	    if(calcVelNorm[i]<velNorm) {
	      velNorm = calcVelNorm[i];
	      }
	    if(calcAccNorm[i]<accNorm) {
	      accNorm = calcAccNorm[i];
	      }
	    if(calcDecNorm[i]<decNorm) {
	      decNorm = calcDecNorm[i];
	      }
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
      }
        
      if (isNewValue[indexReadPos+1] == 0)
	  indexReadPos = indexReadPos;
      else{
	if (indexReadPos % dimBuffer == (dimBuffer-1)) 
	  indexReadPos = 0;
	else
	  indexReadPos = indexReadPos + 1; 
      }
      trajParamSet = true;     
   }
} 

 double TrajectoryGenerator2::setPosGain(double k, double dK){
   
  double gainP; 
   
  if (trajType == LINEARVELOCITY){
    double u = k/dK; 
    gainP = 0.5*dt*k*u;
  }
  else if (trajType == LIMITEDJERK){
    double u = k/dK; 
    gainP = k*dt*u*u*(1-u*0.5);
  }
  else if (trajType == LIMITEDJERKSQUARE){
    if(2*k<dK){
      double u = k/dK;
      gainP = dt*0.666666666666*k*u*u;
    }
    else{
      double k2=dK-k;
      double u = k2/dK;
      gainP = dt*(dK*0.5-k2+0.666666666666*k2*u*u); 
    }
  }
  else if (trajType == LIMITEDSNAP){
    double u = k/dK;
    gainP = k*dt*0.5*u*u*u*(2.0*u*u - 6.0*u + 5.0);
  }
  else if (trajType == TRIGONOMETRIC){
    double u = k/dK;
    gainP = dt*0.5*(k - dK/M_PI*sin(u*M_PI));
  }
  return gainP;
}
  
 double TrajectoryGenerator2::setVelGain(double k, double dK){
   
  double gainV; 
   
  if (trajType == LINEARVELOCITY){
    double u = k/dK; 
    gainV = u;
   }
  else if (trajType == LIMITEDJERK){
    double u = k/dK; 
    gainV = u*u*(3-2*u);
   }
  else if (trajType == LIMITEDJERKSQUARE){
    if(2*k<dK){
      double u = k/dK;
      gainV = 2.0*u*u;
    }
    else{
      double u = (dK-k)/dK;
      gainV = 1.0 - 2.0*u*u;
    }
  }
  else if (trajType == LIMITEDSNAP){
    double u = k/dK;
    gainV = u*u*u*(6.0*u*u-15.0*u+10.0);
  }
  else if (trajType == TRIGONOMETRIC){
    double u = k/dK;
    gainV = 0.5*(1-cos(u*M_PI));
  }
  return gainV;
}
   
 double TrajectoryGenerator2::setAccGain(double k, double dK){
   
  double gainA; 
   
  if (trajType == LINEARVELOCITY){
    gainA = 1/dK/dt;
  }
  else if (trajType == LIMITEDJERK){
    double u = k/dK; 
    gainA = 1/dK/dt*u*6*(1-u);
  }
  else if (trajType == LIMITEDJERKSQUARE){   
    if(2*k<dK){
      double u = k/dK;
      gainA = 4.0*u/dK/dt;
    }
    else{
      double u = (dK-k)/dK;
      gainA = 4.0*u/dK/dt;
	}
  }
  else if (trajType == LIMITEDSNAP){
    double u = k/dK;
    gainA = 1.0/dK/dt*30.0*u*u*(u*u-2.0*u+1.0);
  }
  else if (trajType == TRIGONOMETRIC){
    double u = k/dK;
    gainA = 0.5*sin(u*M_PI)*M_PI/dK/dt;
  }
  return gainA;
}
 
 void TrajectoryGenerator2::run() {
      
  double theta;
  time = System::getTime();
  timeScaled = time - tOffset; 
  
  setTrajectoryParameters();

      if (trajParamSet == 1){  	
	if(first) { 
	  for(sigdim_t i = 0; i < dim; i++) {	    
	    acc.setValue(0.0, i);
	    vel.setValue(0.0, i);
	    pos.setValue(posFinalPrev[i], i);
		    
	    acc.setTimeStamp(System::getTimeNs(),i);
	    vel.setTimeStamp(System::getTimeNs(),i);
	    pos.setTimeStamp(System::getTimeNs(),i);
		
	    first = false;   
	 } 
	}
	else {	
	  if(smoothTrajectory == 1){
	    
	     for(sigdim_t i = 0; i < dim; i++){
	      	  if(timeScaled < 0) { 
		    acc.setValue(0.0*distance1[i]*dt*dt, i);
		    vel.setValue(0.0*distance1[i]*dt, i);
		    pos.setValue(posFinalPrev[i] + 0.0*distance1[i], i);
		    }
		  else if(timeScaled < dT1) {
		    acc.setValue((velNorm1*setAccGain(timeScaled, dT1))*distance1[i]*dt*dt, i);
		    vel.setValue((velNorm1*setVelGain(timeScaled, dT1))*distance1[i]*dt, i);
		    pos.setValue(posFinalPrev[i] + (velNorm1*setPosGain(timeScaled, dT1))*distance1[i], i);
		  }
		  else if(timeScaled < dT1 + dT2) { 
		    acc.setValue((0.0)*distance1[i]*dt*dt, i); 		
		    vel.setValue((velNorm1)*distance1[i]*dt, i);			
		    pos.setValue(posFinalPrev[i] + (velNorm1*((timeScaled)-0.5*dT1)*dt)*distance1[i], i); 
		  }
		  else if(timeScaled < dT1 + dT2 + dTC){
		    if (i == 0){
			theta = ((thetaCircleEnd-thetaCircleInit)/dTC)*dt+thetaPrev;
			pos.setValue(cCircle[0] + rCircle*cos(theta), i); 
			vel.setValue(rCircle*((thetaCircleEnd-thetaCircleInit)/dTC*distance1[i])*(-sin(theta)), i); 
			acc.setValue(rCircle*((thetaCircleEnd-thetaCircleInit)/dTC*distance1[i])*((thetaCircleEnd-thetaCircleInit)/dTC)*(-cos(theta)), i); 
		      }
		      else if (i == 1){
			pos.setValue(cCircle[1] + rCircle*sin(theta), i);
			vel.setValue(rCircle*((thetaCircleEnd-thetaCircleInit)/dTC*distance1[i])*(cos(theta)), i);
			acc.setValue(rCircle*((thetaCircleEnd-thetaCircleInit)/dTC*distance1[i])*((thetaCircleEnd-thetaCircleInit)/dTC)*(-sin(theta)), i);
		      }
		      else{
			acc.setValue(0.0, i);
			vel.setValue(0.0, i);
			pos.setValue(posFinalPrev[i], i);
			thetaPrev = theta;
		      }
		  }
		  else if(timeScaled < dT1 + dT2 + dTC + dT3){
		    acc.setValue((0.0)*distance2[i]*dt*dt, i);		
		    vel.setValue((velNorm2)*distance2[i]*dt, i);
		    if(i==0 || i==1)
		      pos.setValue(posB[i] + (velNorm2*((timeScaled)-(dT1+dTC+dT2))*dt)*distance2[i], i); 
		    else
		      pos.setValue(posFinalPrev[i], i);
		  }
		  else if (timeScaled < dT1 + dT2 + dTC + dT3 + dT4) {
		    acc.setValue((-velNorm2*setAccGain((dT1+dT2+dTC+dT3+dT4)-(timeScaled), dT4))*distance2[i]*dt*dt, i);
		    vel.setValue((velNorm2*setVelGain((dT1+dT2+dTC+dT3+dT4)-(timeScaled), dT4))*distance2[i]*dt, i);		
		    if(i==0 || i==1)
		      pos.setValue(posB[i] + (1-velNorm2*setPosGain((dT1+dT2+dTC+dT3+dT4)-(timeScaled), dT4))*distance2[i], i);
		    else
		      pos.setValue(posFinalPrev[i], i);
		  }
		  else { 
		      acc.setValue(0.0, i);
		      vel.setValue(0.0, i);
		      if(i==0 || i==1)
			pos.setValue(posFinal[indexReadPos][i], i);
		      else
			pos.setValue(posFinalPrev[i], i);

		      posFinalPrev[i] = pos.getValue(i);
		      
		      std::cout << "pos[" << i <<  "]: " << pos.getValue(i) << std::endl;
		      std::cout << "posPrev[" << i <<  "]: " << posFinalPrev[i] << std::endl;

		      if(i==dim-1){ 		// calculate new trajectory
			tOffset = time; 	  
			trajParamSet = false; 
			smoothTrajectory = false;
		      }
		}
		    acc.setTimeStamp(System::getTimeNs(),i);
		    vel.setTimeStamp(System::getTimeNs(),i);
		    pos.setTimeStamp(System::getTimeNs(),i);
		    
		    posPrev[i] = pos.getValue(i);
	    }	
	  }
	  else {	
	    for(sigdim_t i = 0; i < dim; i++){
	      	 if(timeScaled < 0) { 
		    acc.setValue(0.0*distance[i]*dt*dt, i);
		    vel.setValue(0.0*distance[i]*dt, i);
		    pos.setValue(posFinalPrev[i], i);
		    }
		  else if(timeScaled < dT1) {
		    acc.setValue((velNorm*setAccGain(timeScaled, dT1))*distance[i]*dt*dt, i);
		    vel.setValue((velNorm*setVelGain(timeScaled, dT1))*distance[i]*dt, i);
		    pos.setValue(posFinalPrev[i] + (velNorm*setPosGain(timeScaled, dT1))*distance[i], i);
		    }
		  else if(timeScaled < dT1 + dT2) { 
		    acc.setValue((0.0)*distance[i]*dt*dt, i);		
		    vel.setValue((velNorm)*distance[i]*dt, i);			
		    pos.setValue(posFinalPrev[i] + (velNorm*((time-tOffset)-0.5*dT1)*dt)*distance[i], i); 			
		    }
		  else if (timeScaled < dT1 + dT2 + dT3) {
		    acc.setValue((-velNorm*setAccGain((dT1+dT2+dT3)-(timeScaled), dT3))*distance[i]*dt*dt, i);
		    vel.setValue((velNorm*setVelGain((dT1+dT2+dT3)-(timeScaled), dT3))*distance[i]*dt, i);		
		    pos.setValue(posFinalPrev[i] + (1-velNorm*setPosGain((dT1+dT2+dT3)-(timeScaled), dT3))*distance[i], i);
		  }
		  else { 
		      acc.setValue(0.0, i);
		      vel.setValue(0.0, i);
		      pos.setValue(posPrev[i], i);
	      		      
		      if(i==dim-1){ 		// calculate new trajectory
			tOffset = time; 	  
			trajParamSet = false; 
			for(sigdim_t i = 0; i<dim; i++)
			  posFinalPrev[i] = pos.getValue(i);
		      }
		  }
		    acc.setTimeStamp(System::getTimeNs(),i);
		    vel.setTimeStamp(System::getTimeNs(),i);
		    pos.setTimeStamp(System::getTimeNs(),i);
		    
		    posPrev[i] = pos.getValue(i);
	    }
	  }
	}
      }
    timePrev = time;
 }
