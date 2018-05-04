#include "ScaraSequenceHomeToReady.hpp"
#include "../../ScaraControlSystem.hpp"
#include "../../ScaraSafetyProperties.hpp"
#include <eeros/safety/SafetySystem.hpp>
#include <unistd.h>
#include <iostream>

using namespace scara;
using namespace eeros::control;
using namespace eeros::sequencer;
using namespace eeros::safety;

ScaraSequenceHomeToReady::ScaraSequenceHomeToReady(Sequencer* sequencer, ScaraControlSystem* controlSys, SafetySystem* safetySys) : 
													Sequence<>("main", sequencer), controlSys(controlSys), safetySys(safetySys) {
	// nothing to do
}

void ScaraSequenceHomeToReady::init() {
	std::bind(&ScaraSequenceHomeToReady::init, *this);
}

bool ScaraSequenceHomeToReady::checkPreCondition() {
	return safetySys->getCurrentLevel().getId() == systemOn;
}

void ScaraSequenceHomeToReady::run() {
	log.info() << "[ Home To Ready ] started";
	
	// Set path planner init position = actual position
	controlSys->pathPlannerJS.setInitPos(controlSys->muxEncPos.getOut().getSignal().getValue());
	usleep(100000); 
	
	// Check that the previous cmd was executed (needs at least one cycle)
	AxisVector enc = controlSys->muxEncPos.getOut().getSignal().getValue();
	AxisVector pp = controlSys->pathPlannerJS.getPosOut().getSignal().getValue();
	AxisVector diff = (enc - pp);
	double d = 0;
	for (int i = 0; i < 4; i++) {
		if (diff[i] < 0)
			d += -diff[i];
		else
			d += diff[i];
	}
	if (d > 0.001)
		throw eeros::EEROSException("initialization failed (" + std::to_string(d) + ")");
	
	// Powering up
	safetySys->triggerEvent(doPoweringUp);
	while(safetySys->getCurrentLevel().getId() < powerOn); 
	safetySys->triggerEvent(goToReady); 
	
	// Go To Ready Position
	posReady1 << 0.78,  2.00, 0.105, 0.78; 
	posReady2 << 0.78, -1.57, 0.105, 1.57; 
	
	controlSys->pathPlannerJS.gotoPoint(posReady1);
	while (!controlSys->pathPlannerJS.posReached()) {
		usleep(100000);
	}
	controlSys->pathPlannerJS.gotoPoint(posReady2); 
	while (!controlSys->pathPlannerJS.posReached()) {
		usleep(100000);
	}
	
	safetySys->triggerEvent(isReady);
}

bool ScaraSequenceHomeToReady::checkPostCondition() {
	return safetySys->getCurrentLevel().getId() == ready;
// 	return true;
}

void ScaraSequenceHomeToReady::exit() {
	log.info() << "[ Home To Ready] exit done";
}
