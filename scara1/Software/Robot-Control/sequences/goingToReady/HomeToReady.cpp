#include "HomeToReady.hpp"
#include "../../ScaraControlSystem.hpp"
#include "../../ScaraSafetyProperties.hpp"
#include <eeros/safety/SafetySystem.hpp>
#include <unistd.h>
#include <iostream>

using namespace scara;
using namespace eeros::control;
using namespace eeros::sequencer;
using namespace eeros::safety;

HomeToReady::HomeToReady(Sequencer* sequencer, ScaraControlSystem* controlSys, SafetySystem* safetySys, ScaraSafetyProperties* safetyProp) : 
						 Sequence<>("main", sequencer), controlSys(controlSys), safetySys(safetySys), safetyProp(safetyProp) {
	// nothing to do
}

void HomeToReady::init() {
	std::bind(&HomeToReady::init, *this);
}

bool HomeToReady::checkPreCondition() {
	return safetySys->getCurrentLevel() == safetyProp->goingToReady;
}

void HomeToReady::run() {
	log.info() << "[ Home To Ready ] started";
	
	posReady1 << 0.78,  2.00, 0.105, 0.78; 
	posReady2 << 0.78, -1.57, 0.105, 1.57; 
	
	controlSys->pathPlannerJS.move(posReady1);
	while (!controlSys->pathPlannerJS.posReached()) 
	{
		usleep(100000);
		if(isTerminating()) return;
		if(isEmergency()) return;
	}
	
	if(isTerminating()) return;
	if(isEmergency()) return;
		
	controlSys->pathPlannerJS.move(posReady2); 
	while (!controlSys->pathPlannerJS.posReached()) 
	{
		usleep(100000);
		if(isTerminating()) return;
		if(isEmergency()) return;
	}
	
	if(isTerminating()) return;
	if(isEmergency()) return;
	
	safetySys->triggerEvent(safetyProp->isReady);
	while (safetySys->getCurrentLevel() < safetyProp->ready) 
	{
		usleep(100000);
		if(isTerminating()) return;
		if(isEmergency()) return;
	}
	
	if(isTerminating()) return;
	if(isEmergency()) return;
	
}

bool HomeToReady::checkPostCondition() {
	return safetySys->getCurrentLevel() == safetyProp->ready;
}

void HomeToReady::exit() {
	log.info() << "[ Home To Ready] exit done";
}

bool HomeToReady::isTerminating() {
	return sequencer->getState() == state::terminating;
}

bool HomeToReady::isEmergency() {
	return (safetySys->getCurrentLevel() == safetyProp->emergency);
}
