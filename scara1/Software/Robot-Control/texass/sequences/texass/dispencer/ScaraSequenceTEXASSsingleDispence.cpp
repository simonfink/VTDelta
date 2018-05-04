#include "ScaraSequenceTEXASSsingleDispence.hpp"
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

ScaraSequenceTEXASSsingleDispence::ScaraSequenceTEXASSsingleDispence(Sequencer* sequencer, ScaraControlSystem* controlSys, SafetySystem* safetySys) : 
													Sequence<void, AxisVector, AxisVector>
													("main", sequencer), controlSys(controlSys), safetySys(safetySys) {
	// nothing to do
}

void ScaraSequenceTEXASSsingleDispence::init() {
	std::bind(&ScaraSequenceTEXASSsingleDispence::init, *this);
}

bool ScaraSequenceTEXASSsingleDispence::checkPreCondition() {
	return safetySys->getCurrentLevel().getId() == moving;
}

void ScaraSequenceTEXASSsingleDispence::run(AxisVector p_in, AxisVector vel) {
	HAL& hal = HAL::instance();
	
	// z_low
	p = p_in;
	p = controlSys->toCalibratedValue(p, controlSys->meshCalibrationLut);
	p = controlSys->toBasisCoordinate(p, 'd', controlSys->mesh);
	controlSys->pathPlannerCS.setVelMax(vel);
	controlSys->pathPlannerCS.gotoPoint(p);
	while (!controlSys->pathPlannerCS.posReached()) {usleep(100000);}
	
	// switch on
	hal.getLogicPeripheralOutput("ventil_on_negate")->set(false);
	hal.getLogicPeripheralOutput("ventil_on")->set(true);
	sleep(1);
	
	// switch off 
	hal.getLogicPeripheralOutput("ventil_on_negate")->set(true);
	hal.getLogicPeripheralOutput("ventil_on")->set(false);
	sleep(1);
}

bool ScaraSequenceTEXASSsingleDispence::checkPostCondition() {
	return true;
}

void ScaraSequenceTEXASSsingleDispence::exit() {
// 	log.info() << "[ Single Dispence ] exit done";
}