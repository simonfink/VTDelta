#include "ScaraSequenceTEXASS.hpp"
#include "../../ScaraControlSystem.hpp"
#include "../../ScaraSafetyProperties.hpp"
#include "../tools/ScaraSequenceCheckTool.hpp"
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

ScaraSequenceTEXASS::ScaraSequenceTEXASS(Sequencer* sequencer, ScaraControlSystem* controlSys, SafetySystem* safetySys) : 
													Sequence<>("main", sequencer), controlSys(controlSys), safetySys(safetySys), 
													goToTool(sequencer, controlSys, safetySys), goToReady(sequencer, controlSys, safetySys),
													checkTool(sequencer, controlSys, safetySys), plasmapen(sequencer, controlSys, safetySys), 
													dispencer(sequencer, controlSys, safetySys), grabber(sequencer, controlSys, safetySys) {
	// nothing to do
}

void ScaraSequenceTEXASS::init() {
	std::bind(&ScaraSequenceTEXASS::init, *this);
}

bool ScaraSequenceTEXASS::checkPreCondition() {
	return safetySys->getCurrentLevel().getId() >= ready;
}

void ScaraSequenceTEXASS::run() {
	log.info() << "[" << name << "] " << "started";
	HAL& hal = HAL::instance();
	
	// Plasmapen Sequence
	plasmapen();
	
	// Dispencer Sequence
	dispencer();

	// Grabber Sequence
	grabber();
	
	// Remove Tool
	goToTool();
	goToReady();
}

bool ScaraSequenceTEXASS::checkPostCondition() {
	return safetySys->getCurrentLevel().getId() == ready;
}

void ScaraSequenceTEXASS::exit() {
	AxisVector x_actual = controlSys->dirKin.getOut().getSignal().getValue();
	controlSys->pathPlannerCS.setInitPos(x_actual);
	controlSys->pathPlannerPosSwitch.switchToInput(1);	// cartesian path planner
	controlSys->autoToManualSwitch.switchToInput(0);	// auto mode
	usleep(100000);
	log.info() << "[ Plasmapen ] exit done";
}

