#include "ScaraSequenceTEXASSsinglePlasmaLine.hpp"
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

ScaraSequenceTEXASSsinglePlasmaLine::ScaraSequenceTEXASSsinglePlasmaLine(Sequencer* sequencer, ScaraControlSystem* controlSys, SafetySystem* safetySys) : 
													Sequence<void, AxisVector, AxisVector> //, int>
													("main", sequencer), controlSys(controlSys), safetySys(safetySys) {
	// nothing to do
}

void ScaraSequenceTEXASSsinglePlasmaLine::init() {
	std::bind(&ScaraSequenceTEXASSsinglePlasmaLine::init, *this);
}

bool ScaraSequenceTEXASSsinglePlasmaLine::checkPreCondition() {
	return safetySys->getCurrentLevel().getId() == moving;
}

void ScaraSequenceTEXASSsinglePlasmaLine::run(AxisVector p1_in, AxisVector p2_in) { //, int mode) {
// 	log.info() << "[ Single Plasma Line ] started";
	HAL& hal = HAL::instance();
	
	p = p1_in;
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
	sleep(1);
	
	p = p2_in;
	p = controlSys->toCalibratedValue(p, controlSys->meshCalibrationLut);
	p = controlSys->toBasisCoordinate(p, 'p', controlSys->mesh);
	controlSys->pathPlannerCS.setVelMax(vel_low);
	controlSys->pathPlannerCS.gotoPoint(p);
	while (!controlSys->pathPlannerCS.posReached()) {usleep(100000);}
	
	hal.getLogicPeripheralOutput("plasmaON")->set(false);
	sleep(1);
	
// 	if(mode == valve){
// 		hal.getLogicPeripheralOutput("ventil")->set(false); 
// 		sleep(2);
// 	}
}

bool ScaraSequenceTEXASSsinglePlasmaLine::checkPostCondition() {
	return true;
}

void ScaraSequenceTEXASSsinglePlasmaLine::exit() {
// 	log.info() << "[ Single Plasma Line ] exit done";
}