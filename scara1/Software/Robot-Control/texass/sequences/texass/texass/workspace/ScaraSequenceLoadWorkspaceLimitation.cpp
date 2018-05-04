#include "ScaraSequenceLoadWorkspaceLimitation.hpp"
#include "../../ScaraControlSystem.hpp"
#include "../../ScaraSafetyProperties.hpp"
#include <eeros/safety/SafetySystem.hpp>
#include <unistd.h>
#include <iostream>
#include <cmath>
#include <fstream>

using namespace scara;
using namespace eeros::control;
using namespace eeros::sequencer;
using namespace eeros::safety;
using namespace eeros::math;

ScaraSequenceLoadWorkspaceLimitation::ScaraSequenceLoadWorkspaceLimitation(Sequencer* sequencer, ScaraControlSystem* controlSys, SafetySystem* safetySys) : 
													Sequence<>("main", sequencer), controlSys(controlSys), safetySys(safetySys) {
	// nothing to do
}

void ScaraSequenceLoadWorkspaceLimitation::init() {
	std::bind(&ScaraSequenceLoadWorkspaceLimitation::init, *this);
}

void ScaraSequenceLoadWorkspaceLimitation::run() {
	log.info() << "[ Load Workspace Limitation ] " << "started";

	int line = 0;
	
	std::fstream file;
	file.open("cartesianWorkspaceLimitationTable.txt", std::fstream::in);
	if(!file.is_open()) throw EEROSException("File for loading workspace limitations is not open!");
	
	int j = 0; 
	while (!file.eof() && j < 2) {
		// Save limit points
		for(int i = 0; i<9; i++)
			file >> limitPoints(i);
		
		// Transform to robot coordinate
		A << limitPoints(0), limitPoints(1), limitPoints(8), 1;
		B << limitPoints(2), limitPoints(3), limitPoints(8), 1;
		C << limitPoints(4), limitPoints(5), limitPoints(8), 1;
		D << limitPoints(6), limitPoints(7), limitPoints(8), 1;
		
		if(j == 0) {
			A_r = controlSys->toBasisCoordinate(A, 'n', controlSys->spannrahmen);
			B_r = controlSys->toBasisCoordinate(B, 'n', controlSys->spannrahmen);
			C_r = controlSys->toBasisCoordinate(C, 'n', controlSys->spannrahmen);
			D_r = controlSys->toBasisCoordinate(D, 'n', controlSys->spannrahmen);
		}
		else  if (j == 1) {
			A_r = controlSys->toBasisCoordinate(A, 'n', controlSys->bauteile);
			B_r = controlSys->toBasisCoordinate(B, 'n', controlSys->bauteile);
			C_r = controlSys->toBasisCoordinate(C, 'n', controlSys->bauteile);
			D_r = controlSys->toBasisCoordinate(D, 'n', controlSys->bauteile);
		}
		else
			throw EEROSException("Exceeded file length!");
		
		log.info() << "A_r: " << A_r(0) << ", " << A_r(1) << ", " << A_r(2) << ", " << A_r(3);
		log.info() << "B_r: " << B_r(0) << ", " << B_r(1) << ", " << B_r(2) << ", " << B_r(3);
		log.info() << "C_r: " << C_r(0) << ", " << C_r(1) << ", " << C_r(2) << ", " << C_r(3);
		log.info() << "D_r: " << D_r(0) << ", " << D_r(1) << ", " << D_r(2) << ", " << D_r(3);
		
		// Caluculate parameters
		if(fabs(B_r(0) - A_r(0)) < 0.001) {
			a_ab = 1;
			b_ab = 0;
			c_ab = -A_r(0);
		}
		else {
			a_ab = -(B_r(1) - A_r(1)) / (B_r(0) - A_r(0));
			b_ab = 1;
			c_ab = -(a_ab * A_r(0) + A_r(1));
		}
		
		if(fabs(C_r(0) - B_r(0)) < 0.001) {
			a_bc = 1;
			b_bc = 0;
			c_bc = -B_r(0);
		}
		else {
			a_bc = -(C_r(1) - B_r(1)) / (C_r(0) - B_r(0));
			b_bc = 1;
			c_bc = -(a_bc * B_r(0) + B_r(1));
		}
		
		if(fabs(D_r(0) - C_r(0)) < 0.001) {
			a_cd = 1;
			b_cd = 0;
			c_cd = -C_r(0);
		}
		else {
			a_cd = -(D_r(1) - C_r(1)) / (D_r(0) - C_r(0));
			b_cd = 1;
			c_cd = -(a_cd * C_r(0) + C_r(1));
		}
	
		if(fabs(A_r(0) - D_r(0)) < 0.001) {
			a_da = 1;
			b_da = 0;
			c_da = -A_r(0);
		}
		else {
			a_da = -(A_r(1) - D_r(1)) / (A_r(0) - D_r(0));
			b_da = 1;
			c_da = -(a_da * D_r(0) + D_r(1));
		}
		
		// Save parameters on control system
		if (j == 0) {
			controlSys->mesh_cartesianLimitation_parameters << a_ab, b_ab, c_ab, a_bc, b_bc, c_bc, a_cd, b_cd, c_cd, a_da, b_da, c_da, A_r(2);
		}
		else if (j == 1) {
			controlSys->parts_cartesianLimitation_parameters << a_ab, b_ab, c_ab, a_bc, b_bc, c_bc, a_cd, b_cd, c_cd, a_da, b_da, c_da, A_r(2);
		}
		else
			throw EEROSException("Exceeded file length!");
		
		j++;
	}
	eeros::math::Matrix<13, 1, double> out1 = controlSys->mesh_cartesianLimitation_parameters;
	eeros::math::Matrix<13, 1, double> out2 = controlSys->parts_cartesianLimitation_parameters;
	
	file.close();
	
}

void ScaraSequenceLoadWorkspaceLimitation::exit() {
	log.info() << "[ Load Workspace Limitation ] exit done";
}
