#include "HighSpeed.hpp"
#include "../control/ParallelScaraControlSystem.hpp"
#include "../safety/ParallelScaraSafetyProperties.hpp"
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/core/Fault.hpp>

#include <unistd.h>
#include <fstream>
#include "../constants.hpp"

#include <chrono>

using namespace parallelscara;
using namespace eeros;
using namespace eeros::control;
using namespace eeros::sequencer;
using namespace eeros::safety;

HighSpeed::HighSpeed(Sequencer* sequencer, ParallelScaraControlSystem* controlSys, SafetySystem* safetySys, ParallelScaraSafetyProperties* safetyProp) : 
		Sequence<>("main", sequencer), controlSys(controlSys), safetySys(safetySys), safetyProp(safetyProp) {
	// nothing to do
}

void HighSpeed::init() {
	std::bind(&HighSpeed::init, *this);
}

bool HighSpeed::checkPreCondition() {
	return safetySys->getCurrentLevel() == safetyProp->highSpeed;
}

void HighSpeed::run() {
	log.trace() << "sequence [ High Speed ] started";
	bool done = false; 
	
	while(!done && safetySys->getCurrentLevel() == safetyProp->highSpeed){
		
		// Run high speed 4 points
		controlSys->pathPlanner.setMaxSpeed(2.0);
		controlSys->pathPlanner.setMaxAcc(3.0);		
		
		int count = 0;
		AxisVector pos; 
		while (count < 4) {
			pos = {0.20, 0.25};
			controlSys->pathPlanner.move(pos);
			while (!controlSys->pathPlanner.posReached()) {
				usleep(100000);
				if (isTerminating()) return;
				if (isStopping()) break;
			}			
			if (isStopping()) break;
			pos = {0.20, 0.35};
			controlSys->pathPlanner.move(pos);
			auto start = std::chrono::high_resolution_clock::now();
			while (!controlSys->pathPlanner.posReached()) {
				usleep(100000);
				if (isTerminating()) return;
				if (isStopping()) break;
			}
			auto stop = std::chrono::high_resolution_clock::now();
			auto d = stop - start;
			double duration = std::chrono::duration_cast<std::chrono::duration<double>>(d).count();
			log.info() << "duration: " << duration;
			
			if (isStopping()) break;
			pos = {-0.20, 0.35};
			controlSys->pathPlanner.move(pos);
			while (!controlSys->pathPlanner.posReached()) {
				usleep(100000);
				if (isTerminating()) return;
				if (isStopping()) break;
			}
			if (isStopping()) break;
			pos = {-0.20, 0.25};
			controlSys->pathPlanner.move(pos);
			while (!controlSys->pathPlanner.posReached()) {
				usleep(100000);
				if (isTerminating()) return;
				if (isStopping()) break;
			}
			if (isStopping()) break;
			controlSys->pathPlanner.move(pos);
			while (!controlSys->pathPlanner.posReached()) {
				usleep(100000);
				if (isTerminating()) return;
				if (isStopping()) break;
			}
			if (isStopping()) break;
			count ++;
		}		
		if (isStopping()) break;
		
	}
	
	// set default speed and acceleration
	controlSys->pathPlanner.setMaxSpeed(velMax); 
	controlSys->pathPlanner.setMaxAcc(accMax); 
	
	// Handle eventual emergency & go out of sequence
	while(safetySys->getCurrentLevel() < safetyProp->powerOn) usleep(100000);
		
	if(safetySys->getCurrentLevel() == safetyProp->highSpeed) 
		safetySys->triggerEvent(safetyProp->stopMoving);
		
	if(safetySys->getCurrentLevel() == safetyProp->powerOn) 
		safetySys->triggerEvent(safetyProp->doWaitReady);
		
	while (!(safetySys->getCurrentLevel() == safetyProp->homedWaitReady)) usleep(100000); 
}

bool HighSpeed::checkPostCondition() {
	return safetySys->getCurrentLevel() == safetyProp->homedWaitReady;
}

void HighSpeed::exit() {
	log.trace() << "sequence [ High Speed ] done";
}

inline bool HighSpeed::isTerminating() {
	return sequencer->getState() == state::terminating;
}

inline bool HighSpeed::isStopping() {
	return !(safetySys->getCurrentLevel() == safetyProp->highSpeed);
}