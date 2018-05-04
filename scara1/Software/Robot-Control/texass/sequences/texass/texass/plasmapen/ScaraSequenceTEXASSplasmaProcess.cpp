#include "ScaraSequenceTEXASSplasmaProcess.hpp"
#include "../../../ScaraControlSystem.hpp"
#include "../../../ScaraSafetyProperties.hpp"
#include <eeros/safety/SafetySystem.hpp>
#include <unistd.h>
#include <iostream>
#include "../../../constants.hpp"

#include <eeros/hal/HAL.hpp>

using namespace scara;
using namespace eeros::control;
using namespace eeros::sequencer;
using namespace eeros::safety;
using namespace eeros::hal;
using namespace eeros::math;

// enum {
// 	compr_air,
// 	valve
// };

ScaraSequenceTEXASSplasmaProcess::ScaraSequenceTEXASSplasmaProcess(Sequencer* sequencer, ScaraControlSystem* controlSys, SafetySystem* safetySys) : 
													Sequence<void, Vector2>("main", sequencer), controlSys(controlSys), safetySys(safetySys), 
													singlePlasmaLine(sequencer, controlSys, safetySys), singlePlasmaPoint(sequencer, controlSys, safetySys) {
	// nothing to do
}

void ScaraSequenceTEXASSplasmaProcess::init() {
	std::bind(&ScaraSequenceTEXASSplasmaProcess::init, *this);
}

bool ScaraSequenceTEXASSplasmaProcess::checkPreCondition() {
	return safetySys->getCurrentLevel().getId() >= ready;
}

void ScaraSequenceTEXASSplasmaProcess::run(Vector2 shift_in) {
	log.info() << "Process started";
	shift << shift_in(0), shift_in(1), 0, 0;
	
	HAL& hal = HAL::instance();
	
	// 1. Init path planner parameters & start motion
	AxisVector x_actual = controlSys->dirKin.getOut().getSignal().getValue();
	controlSys->pathPlannerCS.setInitPos(x_actual);
	controlSys->posIntegral.setInitCondition(x_actual);
	controlSys->pathPlannerPosSwitch.switchToInput(1);	// cartesian path planner
	controlSys->autoToManualSwitch.switchToInput(0);	// automatic mode
	usleep(100000); 
	
	controlSys->posIntegral.enable();
	safetySys->triggerEvent(doStartingMotion); 
	while(safetySys->getCurrentLevel().getId() < moving); 

	// Plasma warm up 
	hal.getLogicPeripheralOutput("plasmaON")->set(false);
	sleep(1);
	hal.getLogicPeripheralOutput("ventil")->set(true);
	sleep(1);
	
	// First point HIGH
	p << 0.0, 0.0, z_high, 1; p = p + shift;
	p = controlSys->toCalibratedValue(p, controlSys->meshCalibrationLut);
	p = controlSys->toBasisCoordinate(p, 'p', controlSys->mesh);
	controlSys->pathPlannerCS.gotoPoint(p);
	while (!controlSys->pathPlannerCS.posReached()) {usleep(100000);}
	
	// Set workspace limitation parameters / enable workspace limitation
	AxisVector offset; offset << controlSys->plasmaX, controlSys->plasmaY, controlSys->plasmaZ, controlSys->plasmaAlpha;
	controlSys->cartesianWorkspaceLimit.setParameters(controlSys->mesh_cartesianLimitation_parameters, offset);
	controlSys->cartesianWorkspaceLimit.enable();
	
	hal.getLogicPeripheralOutput("ventil")->set(false); 
	sleep(2);
	
	//p << 0.0, 0.0, z_low, 1; p = p + shift;
	//singlePlasmaPoint(p); 				//  Test Point 0,0
	
	AxisVector kf;
	
	//***********************************************************************************************
	
	//TestPoints
	
	hal.getLogicPeripheralOutput("ventil")->set(true); 
	sleep(2);
	
	
	
	kf << 1.5 * kfx, 2 * kfy, 0, 0;
	p1 << 0.001905, 0.00224, z_low, 1; p1 = p1 + shift + kf; // R1 1.1
	kf << 3.5 * kfx, 2 * kfy, 0, 0;
	p2 << 0.004445, 0.00224, z_low, 1; p2 = p2 + shift + kf; // R1 1.2
	singlePlasmaLine(p1, p2);
	
	kf << 5.5 * kfx, 2 * kfy, 0, 0;
	p1 << 0.006985, 0.00239, z_low, 1; p1 = p1 + shift + kf; // R1 2.1
	kf << 7.5 * kfx, 2 * kfy, 0, 0;
	p2 << 0.009525, 0.00239, z_low, 1; p2 = p2 + shift + kf; // R1 2.2
	singlePlasmaLine(p1, p2);
	
	kf << 9.5 * kfx, 2 * kfy, 0, 0;
	p1 << 0.012065, 0.00254, z_low, 1; p1 = p1 + shift + kf; // R1 3.1
	kf << 11.5 * kfx, 2 * kfy, 0, 0;
	p2 << 0.014605, 0.00254, z_low, 1; p2 = p2 + shift + kf; // R1 3.2
	singlePlasmaLine(p1, p2);
	
	kf << 13.5 * kfx, 2 * kfy, 0, 0;
	p1 << 0.017145, 0.00269, z_low, 1; p1 = p1 + shift + kf; // R1 4.1
	kf << 15.5 * kfx, 2 * kfy, 0, 0;
	p2 << 0.019685, 0.00269, z_low, 1; p2 = p2 + shift + kf; // R1 4.2
	singlePlasmaLine(p1, p2);
	
	kf << 17.5 * kfx, 2 * kfy, 0, 0;
	p1 << 0.022225, 0.00284, z_low, 1; p1 = p1 + shift + kf; // R1 5.1
	kf << 19.5 * kfx, 2 * kfy, 0, 0;
	p2 << 0.024765, 0.00284, z_low, 1; p2 = p2 + shift + kf; // R1 5.2
	singlePlasmaLine(p1, p2);
	
	char a = 0; 
	log.info() << "Press 's' to start the process";
	while (a != 's'){
		std::cin >> a;
	}
	
	kf << 1.5 * kfx, 5 * kfy, 0, 0;
	p1 << 0.001905, 0.00605, z_low, 1; p1 = p1 + shift + kf; // R2 1.1
	kf << 3.5 * kfx, 5 * kfy, 0, 0;
	p2 << 0.004445, 0.00605, z_low, 1; p2 = p2 + shift + kf; // R2 1.2
	singlePlasmaLine(p1, p2);
	
	kf << 5.5 * kfx, 5 * kfy, 0, 0;
	p1 << 0.006985, 0.00620, z_low, 1; p1 = p1 + shift + kf; // R2 2.1
	kf << 7.5 * kfx, 5 * kfy, 0, 0;
	p2 << 0.009525, 0.00620, z_low, 1; p2 = p2 + shift + kf; // R2 2.2
	singlePlasmaLine(p1, p2);
	
	kf << 9.5 * kfx, 5 * kfy, 0, 0;
	p1 << 0.012065, 0.00635, z_low, 1; p1 = p1 + shift + kf; // R2 3.1
	kf << 11.5 * kfx, 5 * kfy, 0, 0;
	p2 << 0.014605, 0.00635, z_low, 1; p2 = p2 + shift + kf; // R2 3.2
	singlePlasmaLine(p1, p2);
	
	kf << 13.5 * kfx, 5 * kfy, 0, 0;
	p1 << 0.017145, 0.00650, z_low, 1; p1 = p1 + shift + kf; // R2 4.1
	kf << 15.5 * kfx, 5 * kfy, 0, 0;
	p2 << 0.019685, 0.00650, z_low, 1; p2 = p2 + shift + kf; // R2 4.2
	singlePlasmaLine(p1, p2);
	
	kf << 17.5 * kfx, 5 * kfy, 0, 0;
	p1 << 0.022225, 0.00665, z_low, 1; p1 = p1 + shift + kf; // R2 5.1
	kf << 19.5 * kfx, 5 * kfy, 0, 0;
	p2 << 0.024765, 0.00665, z_low, 1; p2 = p2 + shift + kf; // R2 5.2
	singlePlasmaLine(p1, p2);
	
	a = 0; 
	log.info() << "Press 's' to start the process";
	while (a != 's'){
		std::cin >> a;
	}
	
	kf << 1.5 * kfx, 8 * kfy, 0, 0;
	p1 << 0.001905, 0.00986, z_low, 1; p1 = p1 + shift + kf; // R3 1.1
	kf << 3.5 * kfx, 8 * kfy, 0, 0;
	p2 << 0.004445, 0.00986, z_low, 1; p2 = p2 + shift + kf; // R3 1.2
	singlePlasmaLine(p1, p2);
	
	kf << 5.5 * kfx, 8 * kfy, 0, 0;
	p1 << 0.006985, 0.01001, z_low, 1; p1 = p1 + shift + kf; // R3 2.1
	kf << 7.5 * kfx, 8 * kfy, 0, 0;
	p2 << 0.009525, 0.01001, z_low, 1; p2 = p2 + shift + kf; // R3 2.2
	singlePlasmaLine(p1, p2);
	
	kf << 9.5 * kfx, 8 * kfy, 0, 0;
	p1 << 0.012065, 0.01016, z_low, 1; p1 = p1 + shift + kf; // R3 3.1
	kf << 11.5 * kfx, 8 * kfy, 0, 0;
	p2 << 0.014605, 0.01016, z_low, 1; p2 = p2 + shift + kf; // R3 3.2
	singlePlasmaLine(p1, p2);
	
	kf << 13.5 * kfx, 8 * kfy, 0, 0;
	p1 << 0.017145, 0.01031, z_low, 1; p1 = p1 + shift + kf; // R3 4.1
	kf << 15.5 * kfx, 8 * kfy, 0, 0;
	p2 << 0.019685, 0.01031, z_low, 1; p2 = p2 + shift + kf; // R3 4.2
	singlePlasmaLine(p1, p2);
	
	kf << 17.5 * kfx, 8 * kfy, 0, 0;
	p1 << 0.022225, 0.01046, z_low, 1; p1 = p1 + shift + kf; // R3 5.1
	kf << 19.5 * kfx, 8 * kfy, 0, 0;
	p2 << 0.024765, 0.01046, z_low, 1; p2 = p2 + shift + kf; // R3 5.2
	singlePlasmaLine(p1, p2);
	
	a = 0; 
	log.info() << "Press 's' to start the process";
	while (a != 's'){
		std::cin >> a;
	}
	
	kf << 1.5 * kfx, 11 * kfy, 0, 0;
	p1 << 0.001905, 0.01367, z_low, 1; p1 = p1 + shift + kf; // R4 1.1
	kf << 3.5 * kfx, 11 * kfy, 0, 0;
	p2 << 0.004445, 0.01367, z_low, 1; p2 = p2 + shift + kf; // R4 1.2
	singlePlasmaLine(p1, p2);
	
	kf << 5.5 * kfx, 11 * kfy, 0, 0;
	p1 << 0.006985, 0.01382, z_low, 1; p1 = p1 + shift + kf; // R4 2.1
	kf << 7.5 * kfx, 11 * kfy, 0, 0;
	p2 << 0.009525, 0.01382, z_low, 1; p2 = p2 + shift + kf; // R4 2.2
	singlePlasmaLine(p1, p2);
	
	kf << 9.5 * kfx, 11 * kfy, 0, 0;
	p1 << 0.012065, 0.01397, z_low, 1; p1 = p1 + shift + kf; // R4 3.1
	kf << 11.5 * kfx, 11 * kfy, 0, 0;
	p2 << 0.014605, 0.01397, z_low, 1; p2 = p2 + shift + kf; // R4 3.2
	singlePlasmaLine(p1, p2);
	
	kf << 13.5 * kfx, 11 * kfy, 0, 0;
	p1 << 0.017145, 0.01412, z_low, 1; p1 = p1 + shift + kf; // R4 4.1
	kf << 15.5 * kfx, 11 * kfy, 0, 0;
	p2 << 0.019685, 0.01412, z_low, 1; p2 = p2 + shift + kf; // R4 4.2
	singlePlasmaLine(p1, p2);
	
	kf << 17.5 * kfx, 11 * kfy, 0, 0;
	p1 << 0.022225, 0.01427, z_low, 1; p1 = p1 + shift + kf; // R4 5.1
	kf << 19.5 * kfx, 11 * kfy, 0, 0;
	p2 << 0.024765, 0.01427, z_low, 1; p2 = p2 + shift + kf; // R4 5.2
	singlePlasmaLine(p1, p2);
	
	a = 0; 
	log.info() << "Press 's' to start the process";
	while (a != 's'){
		std::cin >> a;
	}
	
	kf << 1.5 * kfx, 14 * kfy, 0, 0;
	p1 << 0.001905, 0.01748, z_low, 1; p1 = p1 + shift + kf; // R5 1.1
	kf << 3.5 * kfx, 14 * kfy, 0, 0;
	p2 << 0.004445, 0.01748, z_low, 1; p2 = p2 + shift + kf; // R5 1.2
	singlePlasmaLine(p1, p2);
	
	kf << 5.5 * kfx, 14 * kfy, 0, 0;
	p1 << 0.006985, 0.01763, z_low, 1; p1 = p1 + shift + kf; // R5 2.1
	kf << 7.5 * kfx, 14 * kfy, 0, 0;
	p2 << 0.009525, 0.01763, z_low, 1; p2 = p2 + shift + kf; // R5 2.2
	singlePlasmaLine(p1, p2);
	
	kf << 9.5 * kfx, 14 * kfy, 0, 0;
	p1 << 0.012065, 0.01778, z_low, 1; p1 = p1 + shift + kf; // R5 3.1
	kf << 11.5 * kfx, 14 * kfy, 0, 0;
	p2 << 0.014605, 0.01778, z_low, 1; p2 = p2 + shift + kf; // R5 3.2
	singlePlasmaLine(p1, p2);
	
	kf << 13.5 * kfx, 14 * kfy, 0, 0;
	p1 << 0.017145, 0.01793, z_low, 1; p1 = p1 + shift + kf; // R5 4.1
	kf << 15.5 * kfx, 14 * kfy, 0, 0;
	p2 << 0.019685, 0.01793, z_low, 1; p2 = p2 + shift + kf; // R5 4.2
	singlePlasmaLine(p1, p2);
	
	kf << 17.5 * kfx, 14 * kfy, 0, 0;
	p1 << 0.022225, 0.01808, z_low, 1; p1 = p1 + shift + kf; // R5 5.1
	kf << 19.5 * kfx, 14 * kfy, 0, 0;
	p2 << 0.024765, 0.01808, z_low, 1; p2 = p2 + shift + kf; // R5 5.2
	singlePlasmaLine(p1, p2);
	
	a = 0; 
	log.info() << "Press 's' to start the process";
	while (a != 's'){
		std::cin >> a;
	}
	
	kf << 1.5 * kfx, 17 * kfy, 0, 0;
	p1 << 0.001905, 0.02129, z_low, 1; p1 = p1 + shift + kf; // R6 1.1
	kf << 3.5 * kfx, 17 * kfy, 0, 0;
	p2 << 0.004445, 0.02129, z_low, 1; p2 = p2 + shift + kf; // R6 1.2
	singlePlasmaLine(p1, p2);
	
	kf << 5.5 * kfx, 17 * kfy, 0, 0;
	p1 << 0.006985, 0.02144, z_low, 1; p1 = p1 + shift + kf; // R6 2.1
	kf << 7.5 * kfx, 17 * kfy, 0, 0;
	p2 << 0.009525, 0.02144, z_low, 1; p2 = p2 + shift + kf; // R6 2.2
	singlePlasmaLine(p1, p2);
	
	kf << 9.5 * kfx, 17 * kfy, 0, 0;
	p1 << 0.012065, 0.02159, z_low, 1; p1 = p1 + shift + kf; // R6 3.1
	kf << 11.5 * kfx, 17 * kfy, 0, 0;
	p2 << 0.014605, 0.02159, z_low, 1; p2 = p2 + shift + kf; // R6 3.2
	singlePlasmaLine(p1, p2);
	
	kf << 13.5 * kfx, 17 * kfy, 0, 0;
	p1 << 0.017145, 0.02174, z_low, 1; p1 = p1 + shift + kf; // R6 4.1
	kf << 15.5 * kfx, 17 * kfy, 0, 0;
	p2 << 0.019685, 0.02174, z_low, 1; p2 = p2 + shift + kf; // R6 4.2
	singlePlasmaLine(p1, p2);
	
	kf << 17.5 * kfx, 17 * kfy, 0, 0;
	p1 << 0.022225, 0.02189, z_low, 1; p1 = p1 + shift + kf; // R6 5.1
	kf << 19.5 * kfx, 17 * kfy, 0, 0;
	p2 << 0.024765, 0.02189, z_low, 1; p2 = p2 + shift + kf; // R6 5.2
	singlePlasmaLine(p1, p2);
	
	//***********************************************************************************
	
	
/*	
	//Demonstrator TEXASS 
	
	kf << 2 * kfx, 1 * kfy, 0, 0;
	p1 << 0.00254, 0.00127, z_low, 1; p1 = p1 + shift + kf;
	kf << 3 * kfx, 1 * kfy, 0, 0;
	p2 << 0.00381, 0.00127, z_low, 1; p2 = p2 + shift + kf;
	singlePlasmaLine(p1, p2);			// 1.1
	
	
	kf << 2 * kfx, 3 * kfy, 0, 0;
	p1 << 0.00254, 0.00381, z_low, 1; p1 = p1 + shift + kf;
	kf << 3 * kfx, 3 * kfy, 0, 0;
	p2 << 0.00381, 0.00381, z_low, 1; p2 = p2 + shift + kf;
	singlePlasmaLine(p1, p2);			// 1.2
	
	kf << 2 * kfx, 17 * kfy, 0, 0;
	p1 << 0.00254, 0.02159, z_low, 1; p1 = p1 + shift + kf;
	kf << 3 * kfx, 17 * kfy, 0, 0;
	p2 << 0.00381, 0.02159, z_low, 1; p2 = p2 + shift + kf;
	singlePlasmaLine(p1, p2);			// 2.1
	
	kf << 2 * kfx, 19 * kfy, 0, 0;
	p1 << 0.00254, 0.02413, z_low, 1; p1 = p1 + shift + kf;
	kf << 3 * kfx, 19 * kfy, 0, 0;
	p2 << 0.00381, 0.02413, z_low, 1; p2 = p2 + shift + kf;
	singlePlasmaLine(p1, p2);			// 2.2
	
	kf << 5 * kfx, 1 * kfy, 0, 0;
	p1 << 0.00635, 0.000635, z_low, 1; p1 = p1 + shift + kf;
	kf << 5 * kfx, 2 * kfy, 0, 0;
	p2 << 0.00635, 0.001905, z_low, 1; p2 = p2 + shift + kf;
	singlePlasmaLine(p1, p2);			// 3.1
	
	kf << 7 * kfx, 1 * kfy, 0, 0;
	p1 << 0.00889, 0.000635, z_low, 1; p1 = p1 + shift + kf;
	kf << 7 * kfx, 2 * kfy, 0, 0;
	p2 << 0.00889, 0.001905, z_low, 1; p2 = p2 + shift + kf;
	singlePlasmaLine(p1, p2);			// 3.2
	
	kf << 13 * kfx, 1 * kfy, 0, 0;
	p1 << 0.01651, 0.000635, z_low, 1; p1 = p1 + shift + kf;
	kf << 13 * kfx, 2 * kfy, 0, 0;
	p2 << 0.01651, 0.001905, z_low, 1; p2 = p2 + shift + kf;
	singlePlasmaLine(p1, p2);			// 4.1
	
	kf << 15 * kfx, 1 * kfy, 0, 0;
	p1 << 0.01905, 0.000635, z_low, 1; p1 = p1 + shift + kf;
	kf << 15 * kfx, 2 * kfy, 0, 0;
	p2 << 0.01905, 0.001905, z_low, 1; p2 = p2 + shift + kf;
	singlePlasmaLine(p1, p2);			// 4.2
	
	kf << 17 * kfx, 1 * kfy, 0, 0;
	p1 << 0.02159, 0.00127, z_low, 1; p1 = p1 + shift + kf;
	kf << 18 * kfx, 1 * kfy, 0, 0;
	p2 << 0.02286, 0.00127, z_low, 1; p2 = p2 + shift + kf;
	singlePlasmaLine(p1, p2); 			// 5.1
	
	kf << 17 * kfx, 3 * kfy, 0, 0;
	p1 << 0.02159, 0.00381, z_low, 1; p1 = p1 + shift + kf;
	kf << 18 * kfx, 3 * kfy, 0, 0;
	p2 << 0.02286, 0.00381, z_low, 1; p2 = p2 + shift + kf;
	singlePlasmaLine(p1, p2); 			// 5.2
	
	kf << 17 * kfx, 17 * kfy, 0, 0;
	p1 << 0.02159, 0.02159, z_low, 1; p1 = p1 + shift + kf;
	kf << 18 * kfx, 17 * kfy, 0, 0;
	p2 << 0.02286, 0.02159, z_low, 1; p2 = p2 + shift + kf;
	singlePlasmaLine(p1, p2); 			// 6.1
	
	kf << 17 * kfx, 19 * kfy, 0, 0;
	p1 << 0.02159, 0.02413, z_low, 1; p1 = p1 + shift + kf;
	kf << 18 * kfx, 19 * kfy, 0, 0;
	p2 << 0.02286, 0.02413, z_low, 1; p2 = p2 + shift + kf;
	singlePlasmaLine(p1, p2); 			// 6.2
	
	kf << 9 * kfx, 9* kfy, 0, 0;
	p1 << 0.010795, 0.01143, z_low, 1; p1 = p1 + shift + kf;
	kf << 11 * kfx, 9 * kfy, 0, 0;
	p2 << 0.013335, 0.01143, z_low, 1; p2 = p2 + shift + kf;
	singlePlasmaLine(p1, p2); 			// 7.1
	
	kf << 9 * kfx, 11 * kfy, 0, 0;
	p1 << 0.010795, 0.01397, z_low, 1; p1 = p1 + shift + kf;
	kf << 11 * kfx, 11 * kfy, 0, 0;
	p2 << 0.013335,  0.01397, z_low, 1; p2 = p2 + shift + kf;
	singlePlasmaLine(p1, p2); 			// 7.2
	
	kf << 9 * kfx, 16 * kfy, 0, 0;
	p1 << 0.010795, 0.02032, z_low, 1; p1 = p1 + shift + kf;
	kf << 10 * kfx, 16 * kfy, 0, 0;
	p2 << 0.012065, 0.02032, z_low, 1; p2 = p2 + shift + kf;
	singlePlasmaLine(p1, p2); 			// 8.1
	
	kf << 9 * kfx, 18 * kfy, 0, 0;
	p1 << 0.010795, 0.02286, z_low, 1; p1 = p1 + shift + kf;
	kf << 10 * kfx, 18 * kfy, 0, 0;
	p2 << 0.012065, 0.02286, z_low, 1; p2 = p2 + shift + kf;
	singlePlasmaLine(p1, p2);			// 8.2
	
	kf << 14 * kfx, 14 * kfy, 0, 0;
	p << 0.01778, 0.01778, z_low, 1; p = p + shift +kf;
	singlePlasmaPoint(p); 				// 9
	
	kf << 14.5 * kfx, 12 * kfy, 0, 0;
	p << 0.01888, 0.01524, z_low, 1; p = p + shift + kf;
	singlePlasmaPoint(p); 				// 10
	
	kf << 13.5 * kfx, 12 * kfy, 0, 0;
	p << 0.01698, 0.01524, z_low, 1; p = p + shift + kf;
	singlePlasmaPoint(p); 				// 11

	//** open connections ** //
	kf << 15 * kfx, 9 * kfy, 0, 0;
	p << 0.01905, 0.01143, z_low, 1; p = p + shift +kf;
	singlePlasmaPoint(p); 				// 210
	
	kf << 15 * kfx, 12 * kfy, 0, 0;
	p << 0.01905, 0.01524, z_low, 1; p = p + shift + kf;
	singlePlasmaPoint(p); 				// 211
	
	kf << 9 * kfx, 12 * kfy, 0, 0;
	p << 0.01143, 0.01524, z_low, 1; p = p + shift + kf;
	singlePlasmaPoint(p); 				// 212
	
	kf << 7 * kfx, 14 * kfy, 0, 0;
	p << 0.00889, 0.01778, z_low, 1; p = p + shift + kf;
	singlePlasmaPoint(p); 				// 213
	
	kf << 9 * kfx, 19 * kfy, 0, 0;
	p << 0.01143, 0.02413, z_low, 1; p = p + shift + kf;
	singlePlasmaPoint(p); 				// 214
	
	kf << 20 * kfx, 19 * kfy, 0, 0;
	p << 0.02540, 0.02413, z_low, 1; p = p + shift + kf;
	singlePlasmaPoint(p); 				// connector point 1
	
	kf << 21 * kfx, 9 * kfy, 0, 0;
	p << 0.02667, 0.01143, z_low, 1; p = p + shift + kf;
	singlePlasmaPoint(p); 				// connector point 2

	kf << 0 * kfx, 19 * kfy, 0, 0;
	p << 0.0, 0.02413, z_low, 1; p = p + shift + kf;
	singlePlasmaPoint(p); 				// connector point for all 4 
	
	kf << 0 * kfx, 9 * kfy, 0, 0;
	p << 0.0, 0.01143, z_low, 1; p = p + shift + kf;
	singlePlasmaPoint(p); 				// connector point for all 4
	
	hal.getLogicPeripheralOutput("ventil")->set(false); 
	sleep(2);
	*/
/*	
	*** cutting-lines *** //
	
	p1 << 0.00214, 0.00294, z_low, 1; p1 = p1 + shift;
	p2 << 0.00294, 0.00294, z_low, 1; p2 = p2 + shift;
	singlePlasmaLine(p1, p2);			// a
	
	p1 << 0.00341, 0.00214, z_low, 1; p1 = p1 + shift;
	p2 << 0.00421, 0.00214, z_low, 1; p2 = p2 + shift;
	singlePlasmaLine(p1, p2);			// b
	
	p1 << 0.00722, 0.00294, z_low, 1; p1 = p1 + shift;
	p2 << 0.00722, 0.00214, z_low, 1; p2 = p2 + shift;
	singlePlasmaLine(p1, p2);			// c
	
	p1 << 0.00802, 0.00167, z_low, 1; p1 = p1 + shift;
	p2 << 0.00802, 0.00087, z_low, 1; p2 = p2 + shift;
	singlePlasmaLine(p1, p2);			// d
	
	p1 << 0.00802, 0.00167, z_low, 1; p1 = p1 + shift;
	p2 << 0.00802, 0.00087, z_low, 1; p2 = p2 + shift;
	singlePlasmaLine(p1, p2);			// d
	
	p1 << 0.01738, 0.00294, z_low, 1; p1 = p1 + shift;
	p2 << 0.01738, 0.00214, z_low, 1; p2 = p2 + shift;
	singlePlasmaLine(p1, p2);			// e
	
	p1 << 0.01818, 0.00167, z_low, 1; p1 = p1 + shift;
	p2 << 0.01818, 0.00087, z_low, 1; p2 = p2 + shift;
	singlePlasmaLine(p1, p2);			// f
	
	p1 << 0.01855, 0.006985, z_low, 1; p1 = p1 + shift;
	p2 << 0.02005, 0.006985, z_low, 1; p2 = p2 + shift;
	singlePlasmaLine(p1, p2);			// g
	
	p1 << 0.019685, 0.00421, z_low, 1; p1 = p1 + shift;
	p2 << 0.019685, 0.00341, z_low, 1; p2 = p2 + shift;
	singlePlasmaLine(p1, p2);			// h
	
	p1 << 0.02119, 0.00294, z_low, 1; p1 = p1 + shift;
	p2 << 0.02199, 0.00294, z_low, 1; p2 = p2 + shift;
	singlePlasmaLine(p1, p2);			// i
	
	p1 << 0.02246, 0.00214, z_low, 1; p1 = p1 + shift;
	p2 << 0.02326, 0.00214, z_low, 1; p2 = p2 + shift;
	singlePlasmaLine(p1, p2);			// j
	
	p1 << 0.01103, 0.01310, z_low, 1; p1 = p1 + shift;
	p2 << 0.01183, 0.01310, z_low, 1; p2 = p2 + shift;
	singlePlasmaLine(p1, p2);			// k
	
	p1 << 0.01357, 0.01310, z_low, 1; p1 = p1 + shift;
	p2 << 0.01437, 0.01310, z_low, 1; p2 = p2 + shift;
	singlePlasmaLine(p1, p2);			// l
	
	p1 << 0.01230, 0.01230, z_low, 1; p1 = p1 + shift;
	p2 << 0.01310, 0.01230, z_low, 1; p2 = p2 + shift;
	singlePlasmaLine(p1, p2);			// m
	
	p1 << 0.01768, 0.01524, z_low, 1; p1 = p1 + shift;
	p2 << 0.01788, 0.01524, z_low, 1; p2 = p2 + shift;
	singlePlasmaLine(p1, p2);			// n1
	
	p1 << 0.01778, 0.01534, z_low, 1; p1 = p1 + shift;
	p2 << 0.01778, 0.01514, z_low, 1; p2 = p2 + shift;
	singlePlasmaLine(p1, p2);			// n2
	
	p1 << 0.01778, 0.01534, z_low, 1; p1 = p1 + shift;
	p2 << 0.01778, 0.01514, z_low, 1; p2 = p2 + shift;
	singlePlasmaLine(p1, p2);			// n2
	
	p1 << 0.019685, 0.02199, z_low, 1; p1 = p1 + shift;
	p2 << 0.019685, 0.02119, z_low, 1; p2 = p2 + shift;
	singlePlasmaLine(p1, p2);			// o
	
	p1 << 0.02119, 0.02326, z_low, 1; p1 = p1 + shift;
	p2 << 0.02199, 0.02326, z_low, 1; p2 = p2 + shift;
	singlePlasmaLine(p1, p2);			// p
	
	p1 << 0.02246, 0.02246, z_low, 1; p1 = p1 + shift;
	p2 << 0.02326, 0.02246, z_low, 1; p2 = p2 + shift;
	singlePlasmaLine(p1, p2);			// q
	
	p1 << 0.01103, 0.02199, z_low, 1; p1 = p1 + shift;
	p2 << 0.01183, 0.02199, z_low, 1; p2 = p2 + shift;
	singlePlasmaLine(p1, p2);			// r
	
	p1 << 0.01230, 0.02119, z_low, 1; p1 = p1 + shift;
	p2 << 0.01310, 0.02119, z_low, 1; p2 = p2 + shift;
	singlePlasmaLine(p1, p2);			// s
	
	p1 << 0.00214, 0.02326, z_low, 1; p1 = p1 + shift;
	p2 << 0.00294, 0.02326, z_low, 1; p2 = p2 + shift;
	singlePlasmaLine(p1, p2);			// t
	
	p1 << 0.00314, 0.02246, z_low, 1; p1 = p1 + shift;
	p2 << 0.00421, 0.02246, z_low, 1; p2 = p2 + shift;
	singlePlasmaLine(p1, p2);			// v*/
	
	
	/*
	// ** cutting ** //
	p << 0.00254, 0.00254, z_low, 1; p = p + shift;
	singlePlasmaPoint(p); 					// 15
	
	p << 0.00381, 0.00254, z_low, 1; p = p + shift;
	singlePlasmaPoint(p); 					// 16
	
	p << 0.00762, 0.00127, z_low, 1; p = p + shift;
	singlePlasmaPoint(p); 					// 17
	
	p << 0.00762, 0.00254, z_low, 1; p = p + shift;
	singlePlasmaPoint(p); 					// 18
	
	p << 0.01778, 0.00127, z_low, 1; p = p + shift;
	singlePlasmaPoint(p); 					// 19
	
	p << 0.01778, 0.00254, z_low, 1; p = p + shift;
	singlePlasmaPoint(p); 					// 20
	
	p << 0.02159, 0.00254, z_low, 1; p = p + shift;
	singlePlasmaPoint(p); 					// 21
	
	p << 0.02286, 0.00254, z_low, 1; p = p + shift;
	singlePlasmaPoint(p); 					// 22
	
	p << 0.019685, 0.00381, z_low, 1; p = p + shift;
	singlePlasmaPoint(p); 					// 23
	
	p << 0.01905, 0.00635, z_low, 1; p = p + shift;
	singlePlasmaPoint(p); 					// 24
	
	p << 0.01143, 0.01270, z_low, 1; p = p + shift;
	singlePlasmaPoint(p); 					// 25
	
	p << 0.01270, 0.01270, z_low, 1; p = p + shift;
	singlePlasmaPoint(p); 					// 26
	
	p << 0.01397, 0.01270, z_low, 1; p = p + shift;
	singlePlasmaPoint(p); 					// 27
	
	p << 0.01778, 0.01524, z_low, 1; p = p + shift;
	singlePlasmaPoint(p); 					// 28
	
	p << 0.00254, 0.02286, z_low, 1; p = p + shift;
	singlePlasmaPoint(p); 					// 29
	
	p << 0.00381, 0.02286, z_low, 1; p = p + shift;
	singlePlasmaPoint(p); 					// 30
	
	p << 0.01143, 0.02159, z_low, 1; p = p + shift;
	singlePlasmaPoint(p); 					// 31
	
	p << 0.01270, 0.02159, z_low, 1; p = p + shift;
	singlePlasmaPoint(p); 					// 32
	
	p << 0.019685, 0.02159, z_low, 1; p = p + shift;
	singlePlasmaPoint(p); 					// 33
	
	p << 0.02159, 0.02286, z_low, 1; p = p + shift;
	singlePlasmaPoint(p); 					// 34 
	
	p << 0.02286, 0.02286, z_low, 1; p = p + shift;
	singlePlasmaPoint(p); 					// 35*/
	
	
	// Set workspace limitation parameters / enable workspace limitation
	controlSys->cartesianWorkspaceLimit.disable();
		
	// Last point HIGH
	kf << 19.5 * kfx, 17 * kfy, 0, 0;
	p2 << 0.024765, 0.02189, z_low, 1; p2 = p2 + shift + kf;
	
	//p << 0.0, 0.0, z_low, 1; p = p + shift;
	p = controlSys->toCalibratedValue(p, controlSys->meshCalibrationLut);
	p = controlSys->toBasisCoordinate(p, 'p', controlSys->mesh);
	controlSys->pathPlannerCS.setVelMax(vel);
	controlSys->pathPlannerCS.gotoPoint(p);
	while (!controlSys->pathPlannerCS.posReached()) {usleep(100000);}

	// 3. Stop motion
	controlSys->posIntegral.disable();
	safetySys->triggerEvent(doMotionStopping);
	while(!(safetySys->getCurrentLevel().getId() == ready));
}

bool ScaraSequenceTEXASSplasmaProcess::checkPostCondition() {
	return safetySys->getCurrentLevel().getId() == ready;
}

void ScaraSequenceTEXASSplasmaProcess::exit() {
// 	log.info() << "[ Plasmapen ] exit done";
}


 
