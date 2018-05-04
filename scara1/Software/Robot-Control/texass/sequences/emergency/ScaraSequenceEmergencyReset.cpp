#include "ScaraSequenceEmergencyReset.hpp"
#include "../../ScaraControlSystem.hpp"
#include "../../ScaraSafetyProperties.hpp"
#include <eeros/safety/SafetySystem.hpp>
#include <unistd.h>
#include <iostream>

using namespace scara;
using namespace eeros::control;
using namespace eeros::sequencer;
using namespace eeros::safety;

ScaraSequenceEmergencyReset::ScaraSequenceEmergencyReset(std::string name, SafetySystem& safetySys, ScaraControlSystem& controlSys) : 
												ScaraSequence(name, safetySys, controlSys) {
}


void ScaraSequenceEmergencyReset::init() {
	log.info() << "[" << name << "] " << "started";
	
	addStep([&]() {
		log.info() << "hello hello"; 
	});
	
// 	if(SL NOT INIT)
// 		....
// 		else // SL INIT
// 			....
}

void ScaraSequenceEmergencyReset::exit() {
	log.info() << "[" << name << "] " << "exit done";
}

bool ScaraSequenceEmergencyReset::checkPreCondition() {
	return safetySys.getCurrentLevel().getId() >= off;
}

bool ScaraSequenceEmergencyReset::checkPostCondition() {
	return true;
}