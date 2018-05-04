#include "ScaraSequenceTEXASSgrabber.hpp"
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

ScaraSequenceTEXASSgrabber::ScaraSequenceTEXASSgrabber(Sequencer* sequencer, ScaraControlSystem* controlSys, SafetySystem* safetySys) : 
													Sequence<>("main", sequencer), controlSys(controlSys), safetySys(safetySys), 
													goToReady(sequencer, controlSys, safetySys), goToTool(sequencer, controlSys, safetySys), 
													checkTool(sequencer, controlSys, safetySys), grabProcess(sequencer, controlSys, safetySys) {
	// nothing to do
}

void ScaraSequenceTEXASSgrabber::init() {
	std::bind(&ScaraSequenceTEXASSgrabber::init, *this);
}

bool ScaraSequenceTEXASSgrabber::checkPreCondition() {
	return safetySys->getCurrentLevel().getId() >= ready;
}

void ScaraSequenceTEXASSgrabber::run() {
	log.info() << "[ Grabber ] started";
	HAL& hal = HAL::instance();

	// 1. Mount tool
	goToTool();
	goToReady();
	
	// 2. Check Tool
	safetySys->triggerEvent(doTeaching);
	checkTool('g', controlSys->toBasisCoordinate(controlSys->tipRefPoint, 'n', controlSys->bauteile));
	goToReady();
	usleep(100000); 
	
	// 3. Grab process
	grabProcess({0.0  , 0.0}, {0.0,0.0});
//	grabProcess({0.005  , 0.0}, {0.0256,0.0762});
//	grabProcess({0.01, 0.0}, {0.0766,0.0762});
	
//	grabProcess({0.010, 0.0}, {0.02540, 0.02540});
//	grabProcess({0.015, 0.0}, {0.0762,0.02540});
	
	
	
	
	// 4. Go to ready
	goToReady();
}

bool ScaraSequenceTEXASSgrabber::checkPostCondition() {
	return safetySys->getCurrentLevel().getId() == ready;
}

void ScaraSequenceTEXASSgrabber::exit() {
	log.info() << "[ Grabber ] exit done";
}
