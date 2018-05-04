#include "Balancing.hpp"
#include "../control/ParallelScaraControlSystem.hpp"
#include "../safety/ParallelScaraSafetyProperties.hpp"
#include <eeros/safety/SafetySystem.hpp>
#include <unistd.h>
#include <iostream>
#include <fstream>

using namespace parallelscara;
using namespace eeros::control;
using namespace eeros::sequencer;
using namespace eeros::safety;

Balancing::Balancing(Sequencer* sequencer, ParallelScaraControlSystem* controlSys, SafetySystem* safetySys, ParallelScaraSafetyProperties* safetyProp) : 
		Sequence<>("main", sequencer), controlSys(controlSys), safetySys(safetySys), safetyProp(safetyProp) {
	// nothing to do
}

void Balancing::init() {
	std::bind(&Balancing::init, *this);
}

bool Balancing::checkPreCondition() {
	return safetySys->getCurrentLevel() == safetyProp->balancing;
}

void Balancing::run() {
	log.info() << "[ Balancing ] started";

	controlSys->pathPlanner.setMaxSpeed(0.7);
	controlSys->pathPlanner.setMaxAcc(0.7);

	while(safetySys->getCurrentLevel() == safetyProp->balancing){
// 		AxisVector pos; pos = {0.25, 0.30};
// 		controlSys->pathPlanner.move(pos);
// 		while (!controlSys->pathPlanner.posReached()) {
// 			usleep(100000);
// 			yield();
// 			if (isTerminating()) return;
// 			if (isStopping()) break;
// 		}
// 		sleep(2.0);
// 		yield();
// 		if (isStopping()) break;
// 		pos = {0.00, 0.35};
// 		controlSys->pathPlanner.move(pos); 
// 		while (!controlSys->pathPlanner.posReached()) {
// 			usleep(100000);
// 			yield();
// 			if (isTerminating()) return;
// 			if (isStopping()) break;
// 		}
// 		sleep(2.0);
// 		yield();
// 		if (isStopping()) break;
// 		pos = {-0.25, 0.30};
// 		controlSys->pathPlanner.move(pos);
// 		while (!controlSys->pathPlanner.posReached()) {
// 			usleep(100000);
// 			yield();
// 			if (isTerminating()) return;
// 			if (isStopping()) break;
// 		}
// 		sleep(2.0);
// 		yield();
// 		if (isStopping()) break;
// 		pos = {0.00, 0.35};
// 		controlSys->pathPlanner.move(pos); 
// 		while (!controlSys->pathPlanner.posReached()) {
// 			usleep(100000);
// 			yield();
// 			if (isTerminating()) return;
// 			if (isStopping()) break;
// 		}
		sleep(2.0);
		yield();
		if (isStopping()) break;
		usleep(100000);
	}

	usleep(100000);
	
	// Handle eventual emergency & go out of sequence
	while(safetySys->getCurrentLevel() < safetyProp->powerOn) usleep(100000);
	
	if(safetySys->getCurrentLevel() == safetyProp->balancing){
		safetySys->triggerEvent(safetyProp->stopMoving);
	}
	
	if(safetySys->getCurrentLevel() == safetyProp->powerOn){
		safetySys->triggerEvent(safetyProp->doWaitReady);
	}
	
	while (!(safetySys->getCurrentLevel() == safetyProp->homedWaitReady)) usleep(100000); 
	
}

bool Balancing::checkPostCondition() {
	return safetySys->getCurrentLevel() == safetyProp->homedWaitReady;
}

void Balancing::exit() {	
	log.info() << "[ Balancing ] exit done";
}

inline bool Balancing::isTerminating() {
	return sequencer->getState() == state::terminating;
}

inline bool Balancing::isStopping() {
	return (!(safetySys->getCurrentLevel() == safetyProp->balancing) || !controlSys->hallDataRead.isBarOn());
}
