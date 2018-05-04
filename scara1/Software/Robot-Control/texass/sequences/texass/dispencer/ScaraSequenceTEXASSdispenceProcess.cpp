#include "ScaraSequenceTEXASSdispenceProcess.hpp"
#include "../../../ScaraControlSystem.hpp"
#include "../../../ScaraSafetyProperties.hpp"
#include <eeros/safety/SafetySystem.hpp>
#include <unistd.h>
#include <iostream>

#include <eeros/hal/HAL.hpp>

using namespace scara;
using namespace eeros::control;
using namespace eeros::sequencer;
using namespace eeros::safety;
using namespace eeros::hal;
using namespace eeros::math;

ScaraSequenceTEXASSdispenceProcess::ScaraSequenceTEXASSdispenceProcess(Sequencer* sequencer, ScaraControlSystem* controlSys, SafetySystem* safetySys) : 
													Sequence<void, Vector2>("main", sequencer), controlSys(controlSys), safetySys(safetySys), 
													singleDispence(sequencer, controlSys, safetySys) {
	// nothing to do
}

void ScaraSequenceTEXASSdispenceProcess::init() {
	std::bind(&ScaraSequenceTEXASSdispenceProcess::init, *this);
}

bool ScaraSequenceTEXASSdispenceProcess::checkPreCondition() {
	return safetySys->getCurrentLevel().getId() >= ready;
}

void ScaraSequenceTEXASSdispenceProcess::run(Vector2 shift_in) {
	HAL& hal = HAL::instance();
	shift << shift_in(0), shift_in(1), 0, 0;

	// 3. Init path planner parameters & start motion
	AxisVector x_actual = controlSys->dirKin.getOut().getSignal().getValue();
	controlSys->pathPlannerCS.setInitPos(x_actual);
	controlSys->posIntegral.setInitCondition(x_actual);
	controlSys->pathPlannerPosSwitch.switchToInput(1);	// cartesian path planner
	controlSys->autoToManualSwitch.switchToInput(0);	// automatic mode
	usleep(100000); 
	
	controlSys->posIntegral.enable();
	safetySys->triggerEvent(doStartingMotion); 
	while(safetySys->getCurrentLevel().getId() < moving); 

	// 4. Dispencing process
	
	// First point HIGH
	p <<  0.00254, 0.00127, z_high, 1; p = p + shift;
	p = controlSys->toCalibratedValue(p, controlSys->meshCalibrationLut);
	p = controlSys->toBasisCoordinate(p, 'd', controlSys->mesh);
	controlSys->pathPlannerCS.gotoPoint(p);
	while (!controlSys->pathPlannerCS.posReached()) {usleep(100000);}
	
	// Set workspace limitation parameters / enable workspace limitation
	AxisVector offset; offset << controlSys->dispencerX, controlSys->dispencerY, controlSys->dispencerZ, controlSys->dispencerAlpha;
	controlSys->cartesianWorkspaceLimit.setParameters(controlSys->mesh_cartesianLimitation_parameters, offset);
	controlSys->cartesianWorkspaceLimit.enable();
	/*
	p << 0.00381,  0.00381, z_low, 1; p = p + shift;
	singleDispence(p, vel);
	*/
	

	// Open
	p << 0.00254,  0.00127, z_low, 1; p = p + shift;	// Punkt 101
	singleDispence(p, vel);
	
	p << 0.00381,  0.00127, z_low, 1; p = p + shift;	// Punkt 101.2
	singleDispence(p, vel);
	
	p << 0.00254,  0.00381, z_low, 1; p = p + shift;	// Punkt 102
	singleDispence(p, vel);
	
	p << 0.00381,  0.00381, z_low, 1; p = p + shift;	// Punkt 102.2
	singleDispence(p, vel);
	
	p << 0.00635,  0.001905, z_low, 1; p = p + shift;	// Punkt 103
	singleDispence(p, vel);
	
	p << 0.00889,  0.001905, z_low, 1; p = p + shift;	// Punkt 104
	singleDispence(p, vel);
	
	p << 0.01651,  0.001905, z_low, 1; p = p + shift;	// Punkt 105
	singleDispence(p, vel);
	
	p << 0.01905,  0.001905, z_low, 1; p = p + shift;	// Punkt 106
	singleDispence(p, vel);
	
	p << 0.02159,  0.00127, z_low, 1; p = p + shift;	// Punkt 107
	singleDispence(p, vel);
	
	p << 0.02286,  0.00127, z_low, 1; p = p + shift;	// Punkt 107.2
	singleDispence(p, vel);
	
	p << 0.02159,  0.00381, z_low, 1; p = p + shift;	// Punkt 108
	singleDispence(p, vel);
	
	p << 0.02286,  0.00381, z_low, 1; p = p + shift;	// Punkt 108.2
	singleDispence(p, vel);
	
	p << 0.012065, 0.01143, z_low, 1; p = p + shift;	// Punkt 109
	singleDispence(p, vel);
	
	p << 0.013335, 0.01143, z_low, 1; p = p + shift;	// Punkt 109.2
	singleDispence(p, vel);
	
	p << 0.012065, 0.01397, z_low, 1; p = p + shift;	// Punkt 110
	singleDispence(p, vel);
	
	p << 0.013335, 0.013970, z_low, 1; p = p + shift;	// Punkt 110.2
	singleDispence(p, vel);
	
	p << 0.01651,  0.01524, z_low, 1; p = p + shift;	// Punkt 111
	singleDispence(p, vel);
	
	p << 0.01905,  0.01524, z_low, 1; p = p + shift;	// Punkt 112
	singleDispence(p, vel);
	
	p << 0.01778,  0.01778, z_low, 1; p = p + shift;	// Punkt 113
	singleDispence(p, vel);
	
	p << 0.00254,  0.02159, z_low, 1; p = p + shift;	// Punkt 114
	singleDispence(p, vel);
	
	p << 0.00381,  0.02159, z_low, 1; p = p + shift;	// Punkt 114.2
	singleDispence(p, vel);
	
	p << 0.00254,  0.02413, z_low, 1; p = p + shift;	// Punkt 115
	singleDispence(p, vel);
	
	p << 0.00381,  0.024130, z_low, 1; p = p + shift;	// Punkt 115.2
	singleDispence(p, vel);
	
	p << 0.012065, 0.02032, z_low, 1; p = p + shift;	// Punkt 116
	singleDispence(p, vel);
	
	p << 0.012065, 0.02286, z_low, 1; p = p + shift;	// Punkt 117
	singleDispence(p, vel);
	
	p << 0.02159,  0.02159, z_low, 1; p = p + shift;	// Punkt 118
	singleDispence(p, vel);
	
	p << 0.022860, 0.02159, z_low, 1; p = p + shift;	// Punkt 118.2
	singleDispence(p, vel);
	
	p << 0.02159,  0.02413, z_low, 1; p = p + shift;	// Punkt 119
	singleDispence(p, vel);
	
	p << 0.022860, 0.02413, z_low, 1; p = p + shift;	// Punkt 119.2
	singleDispence(p, vel);
	
	//**apply connections**
	p << 0.01905, 0.01143, z_low, 1; p = p + shift;		// Punkt 210
	singleDispence(p, vel);
	
	p << 0.01905, 0.01524, z_low, 1; p = p + shift;		// Punkt 211
	singleDispence(p, vel);
	
	p << 0.01143, 0.01524, z_low, 1; p = p + shift;		// Punkt 212
	singleDispence(p, vel);
	
	p << 0.00889, 0.01778, z_low, 1; p = p + shift;		// Punkt 213
	singleDispence(p, vel);
	
	p <<0.01143, 0.02413 , z_low, 1; p = p + shift;		// Punkt 214
	singleDispence(p, vel);

	// Set workspace limitation parameters / enable workspace limitation	
	controlSys->cartesianWorkspaceLimit.disable();
	
	// Last point HIGH
	p <<  0.01143, 0.02413, z_high, 1; p = p + shift;
	p = controlSys->toCalibratedValue(p, controlSys->meshCalibrationLut);
	p = controlSys->toBasisCoordinate(p, 'd', controlSys->mesh);
	controlSys->pathPlannerCS.setVelMax(vel);
	controlSys->pathPlannerCS.gotoPoint(p);
	while (!controlSys->pathPlannerCS.posReached()) {usleep(100000);}

	// 5. Stop motion
	controlSys->posIntegral.disable();
	safetySys->triggerEvent(doMotionStopping);
	while(!(safetySys->getCurrentLevel().getId() == ready));
}

bool ScaraSequenceTEXASSdispenceProcess::checkPostCondition() {
	return safetySys->getCurrentLevel().getId() == ready;
}

void ScaraSequenceTEXASSdispenceProcess::exit() {
// 	log.info() << "[ Dispencer ] exit done";
}