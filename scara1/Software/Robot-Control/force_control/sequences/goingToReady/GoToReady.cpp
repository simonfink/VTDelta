#include "GoToReady.hpp"
#include "../../ScaraControlSystem_force.hpp"
#include "../../ScaraSafetyProperties_force.hpp"
#include <eeros/safety/SafetySystem.hpp>
#include <unistd.h>
#include <iostream>

using namespace scara;
using namespace eeros::control;
using namespace eeros::sequencer;
using namespace eeros::safety;

GoToReady::GoToReady(Sequencer* sequencer, ScaraControlSystem_force* controlSys, SafetySystem* safetySys, ScaraSafetyProperties_force* safetyProp) : 
					 Sequence<>("main", sequencer), controlSys(controlSys), safetySys(safetySys), safetyProp(safetyProp) {
	// nothing to do
}

void GoToReady::init() {
	std::bind(&GoToReady::init, *this);
}

bool GoToReady::checkPreCondition() {
	return (safetySys->getCurrentLevel() == safetyProp->goingToReady || safetySys->getCurrentLevel() == safetyProp->moving);
}

void GoToReady::run() {
	log.info() << "[ Go To Ready ] started";
	
	// Go back to ready position
	AxisVector actPos_xyz = controlSys->dirKin.getOut().getSignal().getValue();
	AxisVector posUp; posUp << actPos_xyz(0), actPos_xyz(1), -0.105,  actPos_xyz;
	controlSys->pathPlannerCS.move(posUp);
	while (!controlSys->pathPlannerCS.posReached()) 
	{
		usleep(100000);
		if(isTerminating()) return;
		if(isEmergency()) return;
	}
	
	if(isTerminating()) return;
	if(isEmergency()) return;
	
	AxisVector posReady = readyPos_cartes;
	controlSys->pathPlannerCS.move(posReady);
	while (!controlSys->pathPlannerCS.posReached()) 
	{
		usleep(100000);
		if(isTerminating()) return;
		if(isEmergency()) return;
	}

	if(isTerminating()) return;
	if(isEmergency()) return;
	
	if(safetySys->getCurrentLevel() == safetyProp->moving)
		safetySys->triggerEvent(safetyProp->doStopMoving);
	else if(safetySys->getCurrentLevel() == safetyProp->goingToReady)
		safetySys->triggerEvent(safetyProp->isReady);
	
	while(!(safetySys->getCurrentLevel() != safetyProp->ready))
	{
		usleep(100000);
		if(isTerminating()) return;
		if(isEmergency()) return;
	}
	
	if(isTerminating()) return;
	if(isEmergency()) return;
}

bool GoToReady::checkPostCondition() {
	return safetySys->getCurrentLevel() == safetyProp->ready;
}

void GoToReady::exit() {
	log.info() << "[ Go To Ready ] exit done";
}

bool GoToReady::isTerminating() {
	return sequencer->getState() == state::terminating;
}

bool GoToReady::isEmergency() {
	return (safetySys->getCurrentLevel() == safetyProp->emergency);
}
