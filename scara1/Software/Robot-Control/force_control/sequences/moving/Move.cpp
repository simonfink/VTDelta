#include "Move.hpp"
#include "../../ScaraControlSystem_force.hpp"
#include "../../ScaraSafetyProperties_force.hpp"
#include <eeros/safety/SafetySystem.hpp>
#include <unistd.h>
#include <iostream>
#include <fstream>


using namespace scara;
using namespace eeros::control;
using namespace eeros::sequencer;
using namespace eeros::safety;

Move::Move(Sequencer* sequencer, ScaraControlSystem_force* controlSys, SafetySystem* safetySys, ScaraSafetyProperties_force* safetyProp) : 
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
	
	AxisVector pos1; pos1 << 0.25, -0.05, -0.105, -0.78; //0.2, 0, -0.105, 0;
	AxisVector pos2; pos2 << 0.35, 0.25, -0.205, 0; //0.45,  0, -0.105, 0;
	
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
	std::ofstream file;
	file.open("traceData.txt", std::ios::trunc);
	timestamp_t* timeStampBuf = controlSys->trace0.getTimestampTrace();
	AxisVector* buf1 = controlSys->trace0.getTrace();
	AxisVector* buf2 = controlSys->trace1.getTrace();
	log.info() << "Test finished...";
	for (int i = 0; i < 5000; i++) file << timeStampBuf[i] << " " << buf1[i] << " " << buf2[i] << std::endl;
	file.close();
	log.info() << "file written";
	
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
