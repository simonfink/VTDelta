#include "ScaraSequenceTEXASSsinglePlasmaPoint.hpp"
#include "../../../ScaraControlSystem.hpp"
#include "../../../ScaraSafetyProperties.hpp"
#include <eeros/safety/SafetySystem.hpp>
#include <unistd.h>
#include <iostream>
#include "../../../types.hpp"
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

ScaraSequenceTEXASSsinglePlasmaPoint::ScaraSequenceTEXASSsinglePlasmaPoint(Sequencer* sequencer, ScaraControlSystem* controlSys, SafetySystem* safetySys) : 
													Sequence<void, AxisVector> //, int>
													("main", sequencer), controlSys(controlSys), safetySys(safetySys) {
	// nothing to do
}

void ScaraSequenceTEXASSsinglePlasmaPoint::init() {
	std::bind(&ScaraSequenceTEXASSsinglePlasmaPoint::init, *this);
}

bool ScaraSequenceTEXASSsinglePlasmaPoint::checkPreCondition() {
	return safetySys->getCurrentLevel().getId() == moving;
}

void ScaraSequenceTEXASSsinglePlasmaPoint::run(AxisVector p_in) { //, int mode) {
	HAL& hal = HAL::instance();
	
	p = p_in;
	p = controlSys->toCalibratedValue(p, controlSys->meshCalibrationLut);
	p = controlSys->toBasisCoordinate(p, 'p', controlSys->mesh);
	controlSys->pathPlannerCS.setVelMax(vel_high);
	controlSys->pathPlannerCS.gotoPoint(p);
	while (!controlSys->pathPlannerCS.posReached()) {usleep(100000);}
	
// 	if(mode == valve){
// 		hal.getLogicPeripheralOutput("ventil")->set(true); 
// 		sleep(2);
// 	}
	
	//plasma ON
	hal.getLogicPeripheralOutput("plasmaON")->set(true);
	sleep(4);
	hal.getLogicPeripheralOutput("plasmaON")->set(false);
	sleep(1);
	
// 	if(mode == valve){
// 		hal.getLogicPeripheralOutput("ventil")->set(false); 
// 		sleep(2);
// 	}
	
}

bool ScaraSequenceTEXASSsinglePlasmaPoint::checkPostCondition() {
	return true;
}

void ScaraSequenceTEXASSsinglePlasmaPoint::exit() {
// 	log.info() << "[ Single Plasma Point ] exit done";
}