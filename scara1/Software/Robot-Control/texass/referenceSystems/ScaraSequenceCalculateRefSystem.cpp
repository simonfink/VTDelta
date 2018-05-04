#include "ScaraSequenceCalculateRefSystem.hpp"
#include "../../ScaraControlSystem.hpp"
#include "../../ScaraSafetyProperties.hpp"
#include <eeros/safety/SafetySystem.hpp>
#include <unistd.h>
#include <iostream>
#include <cmath>
#include "../../constants.hpp"
#include <fstream>

using namespace scara;
using namespace eeros::control;
using namespace eeros::sequencer;
using namespace eeros::safety;
using namespace eeros::math;

ScaraSequenceCalculateRefSystem::ScaraSequenceCalculateRefSystem(Sequencer* sequencer, ScaraControlSystem* controlSys, SafetySystem* safetySys) : 
											Sequence<void, eeros::math::Frame*>("main", sequencer), controlSys(controlSys), safetySys(safetySys) {
	// nothing to do
}

void ScaraSequenceCalculateRefSystem::init() {
	std::bind(&ScaraSequenceCalculateRefSystem::init, *this);
}

bool ScaraSequenceCalculateRefSystem::checkPreCondition() {
	return safetySys->getCurrentLevel().getId() == teaching;
}

void ScaraSequenceCalculateRefSystem::run(eeros::math::Frame* frame_in) {
	log.info() << "[ Calculate Ref System ] started";
	
	this->frame = frame_in;
	
	AxisVector x_actual = controlSys->dirKin.getOut().getSignal().getValue();
	controlSys->pathPlannerCS.setInitPos(x_actual);
	controlSys->posIntegral.setInitCondition(x_actual);
	controlSys->xbox.setInitPos(x_actual);
	controlSys->pathPlannerPosSwitch.switchToInput(1);	// cartesian path planner
	controlSys->autoToManualSwitch.switchToInput(1);	// manual mode
	
	controlSys->posIntegral.enable(); 

	char m = 0; int i = 0;
	
	log.info() << "To calculate a ref. system, you first have to save 6 POINTS";
	log.info() << "Press 's' + ENTER to save a point";
	log.info() << "Press 'v' + ENTER to change the joystick speed (scaling factor, default = 1.0)";

	while(i<6) {
		std::cin >> m;
		switch(m) {
			case 's':
				// define offset
				if(this->frame == &controlSys->KM){ 
					offset << controlSys->cameraX, controlSys->cameraY, controlSys->cameraZ;
				}
				else{
					offset << 0, 0, tasterZ;
				}
				// save points with offset
				for(int j = 0; j<3; j++){
						savePoints(i,j) = controlSys->dirKin.getOut().getSignal().getValue()(j)+offset(j);
					}
				// go to next point
				i++;
				break;
			case 'v':
				log.warn() << "Write a positive scaling factor for the speed (default = 1.0) and then press ENTER";
				std::cin >> inputScale; 
				controlSys->xbox.setSpeedScaleFactor(inputScale);
				log.info() << "Velocity scale set to: " << inputScale;
				break;
			default:
					// nothing to do
					break;
			}
		}
		// write input data in A, B, ...
		for(int j = 0; j<3; j++){
			A(j) = savePoints(0,j);	B(j) = savePoints(1,j);	C(j) = savePoints(2,j);
			D(j) = savePoints(3,j);	E(j) = savePoints(4,j);	F(j) = savePoints(5,j);
		}
		// calculate unity vectors
		AB = B - A;
		AC = C - A;
		zABC = Matrix<3,1>::crossProduct(AB, AC);
		modzABC = sqrt(zABC(0) * zABC(0) + zABC(1) * zABC(1) + zABC(2) * zABC(2)); 
		e3 = zABC/modzABC;
		DE = E - D;
		k = (DE(0)*e3(0)+DE(1)*e3(1)+DE(2)*e3(2))*e3;
		l =  DE - k;
		modl = sqrt(l(0)*l(0)+l(1)*l(1)+l(2)*l(2));
		e2 = (l/modl);
		e1 = Matrix<3,1>::crossProduct(e2, e3);

		// calculate plane coefficients
		aABCmatrix << B(1)-A(1), B(2)-A(2), C(1)-A(1), C(2)-A(2);
		bABCmatrix << B(0)-A(0), B(2)-A(2), C(0)-A(0), C(2)-A(2);
		cABCmatrix << B(0)-A(0), B(1)-A(1), C(0)-A(0), C(1)-A(1);
		aABC =  aABCmatrix.det();
		bABC = -bABCmatrix.det();
		cABC =  cABCmatrix.det();   
		dABC =  aABC*(-A(0)) + bABC*(-A(1)) + cABC*(-A(2));
		
		// find projections of points (D, E, F) on the plane ABC
		k_D = -(D(0)*aABC+D(1)*bABC+D(2)*cABC+dABC)/(e3(0)*aABC+e3(1)*bABC+e3(2)*cABC);
		k_E = -(E(0)*aABC+E(1)*bABC+E(2)*cABC+dABC)/(e3(0)*aABC+e3(1)*bABC+e3(2)*cABC);
		k_F = -(F(0)*aABC+F(1)*bABC+F(2)*cABC+dABC)/(e3(0)*aABC+e3(1)*bABC+e3(2)*cABC);
		D_1 << D(0)+k_D*e3(0), D(1)+k_D*e3(1), D(2)+k_D*e3(2);
		E_1 << E(0)+k_E*e3(0), E(1)+k_E*e3(1), E(2)+k_E*e3(2);
		F_1 << F(0)+k_F*e3(0), F(1)+k_F*e3(1), F(2)+k_F*e3(2);

		// find origin
		aMatrix << e1(0), -e2(0), e1(1), -e2(1), e1(2), -e2(2);
		bMatrix << D_1(0)-F_1(0), D_1(1)-F_1(1), D_1(2)-F_1(2);
		aMatrixT = aMatrix.transpose(); 
		lambda = !(aMatrixT*aMatrix) * (aMatrixT*bMatrix); 
		O << F_1(0)+lambda(0)*e1(0), F_1(1)+lambda(0)*e1(1), F_1(2)+lambda(0)*e1(2);
		
		// find transformation matrix for frame1 
		for(int i = 0; i<16; i++){
			if(i<3) 				Tr(i) = e1(i);
			else if(i>=4  && i<7) 	Tr(i) = e2(i-4);
			else if(i>=8  && i<11)	Tr(i) = e3(i-8);
			else if(i>=12 && i<15)	Tr(i) = O(i-12);
			else if(i == 15)		Tr(i) = 1;
			else					Tr(i) = 0;
		} 
// 		frame->set(Tr*toolMatrix);
		frame->set(Tr);

		std::fstream file;
		file.open("referenceSystems.txt", std::fstream::out | std::fstream::app);
		if(!file.is_open()) throw EEROSException("File for saving ref. systems is not open!");
		
		file << frame->getFromCoordinateSystem();
		file << "\t";
		file << frame->getToCoordinateSystem();
		file << "\t";
		for(int j = 0; j<4; j++){
			for(int i = 0; i<4; i++){
				file << frame->get()(i,j);
				file << "\t";
			}
		}		
		file << std::endl;
		file.close();
		log.info() << "Frame saved to 'referenceSystems.txt' ";

	controlSys->posIntegral.disable(); 
}

bool ScaraSequenceCalculateRefSystem::checkPostCondition() {
	return safetySys->getCurrentLevel().getId() == systemOn;
}
void ScaraSequenceCalculateRefSystem::exit() {
	log.info() << "[ Calculate Ref System ] exit done";
}

// void ScaraSequenceCalculateRefSystem::setToolOffset() {
// 	if(this->frame == &controlSys->KM){ 
// 		offset << controlSys->cameraX, controlSys->cameraY, controlSys->cameraZ;
// 	}
// 	else{
// 		offset << 0, 0, 0; //controlSys.tasterZ;
// 	}
// }
