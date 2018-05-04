#include "Move.hpp"
#include "../../ScaraControlSystem.hpp"
#include "../../ScaraSafetyProperties.hpp"
#include <eeros/safety/SafetySystem.hpp>
#include <unistd.h>
#include <iostream>

using namespace scara;
using namespace eeros::control;
using namespace eeros::sequencer;
using namespace eeros::safety;

Move::Move(Sequencer* sequencer, ScaraControlSystem* controlSys, SafetySystem* safetySys, ScaraSafetyProperties* safetyProp) : 
						 Sequence<>("main", sequencer), controlSys(controlSys), safetySys(safetySys), safetyProp(safetyProp) {
	// nothing to do
}

void Move::init() {
	std::bind(&Move::init, *this);
}

bool Move::checkPreCondition() {
	return safetySys->getCurrentLevel() == safetyProp->moving;
}

void Move::run() {
	log.info() << "[ Move ] started";
	
	AxisVector pos1; pos1 << 0.35, -0.15, -0.105, -0.78;
	AxisVector pos2; pos2 << 0.35,  0.15, -0.105, -0.78;
	
	controlSys->pathPlannerCS.move(pos1);
	while (!controlSys->pathPlannerCS.posReached()) 
	{
		usleep(100000);
		if(isTerminating()) return;
		if(isEmergency()) return;
	}
	
	if(isTerminating()) return;
	if(isEmergency()) return;
	
	controlSys->pathPlannerCS.move(pos2); 
	while (!controlSys->pathPlannerCS.posReached()) 
	{
		usleep(100000);
		if(isTerminating()) return;
		if(isEmergency()) return;
	}

	if(isTerminating()) return;
	if(isEmergency()) return;
	
	safetySys->triggerEvent(safetyProp->doStopMoving);
	while(safetySys->getCurrentLevel() != safetyProp->ready)
	{
		usleep(100000);
		if(isTerminating()) return;
		if(isEmergency()) return;
	}
	
	if(isTerminating()) return;
	if(isEmergency()) return;
	
}

bool Move::checkPostCondition() {
	return safetySys->getCurrentLevel() == safetyProp->ready;
}

void Move::exit() {
	log.info() << "[ Move ] exit done";
}

bool Move::isTerminating() {
	return sequencer->getState() == state::terminating;
}

bool Move::isEmergency() {
	return (safetySys->getCurrentLevel() == safetyProp->emergency);
}
