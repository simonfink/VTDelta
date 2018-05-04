#include "ScaraSequenceTEXASSdispencer.hpp"
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

ScaraSequenceTEXASSdispencer::ScaraSequenceTEXASSdispencer(Sequencer* sequencer, ScaraControlSystem* controlSys, SafetySystem* safetySys) : 
													Sequence<>("main", sequencer), controlSys(controlSys), safetySys(safetySys), 
													dispenceProcess(sequencer, controlSys, safetySys), goToReady(sequencer, controlSys, safetySys), 
													goToTool(sequencer, controlSys, safetySys), checkTool(sequencer, controlSys, safetySys) {
	// nothing to do
}

void ScaraSequenceTEXASSdispencer::init() {
	std::bind(&ScaraSequenceTEXASSdispencer::init, *this);
}

bool ScaraSequenceTEXASSdispencer::checkPreCondition() {
	return safetySys->getCurrentLevel().getId() >= ready;
}

void ScaraSequenceTEXASSdispencer::run() {
	log.info() << "[ Dispencer ] started";
	HAL& hal = HAL::instance();

	// 1. Mount dispencer
	goToTool();
	goToReady();
	
	// 2. Calibrate dispencer
	safetySys->triggerEvent(doTeaching);
	checkTool('d', controlSys->toBasisCoordinate(controlSys->tipRefPoint, 'n', controlSys->bauteile));
	goToReady();
	usleep(100000); 
	
	
	// 3. Dispence process


//	dispenceProcess({0.0002,0.0002});
//	dispenceProcess({0.0510,0.0002});
//	dispenceProcess({0.0510,0.0510});
//	dispenceProcess({0.0256,0.0762});
//	dispenceProcess({0.0766,0.0762});
	dispenceProcess({0.0762,0.0256});
	
	// 4. Go To Ready
	goToReady();
}

bool ScaraSequenceTEXASSdispencer::checkPostCondition() {
	return safetySys->getCurrentLevel().getId() == ready;
}

void ScaraSequenceTEXASSdispencer::exit() {
	log.info() << "[ Dispencer ] exit done";
}