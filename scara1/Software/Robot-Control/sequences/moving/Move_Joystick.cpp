#include "Move_Joystick.hpp"
#include "../../ScaraSafetyProperties.hpp"
#include "../../ScaraControlSystem.hpp"
#include <eeros/safety/SafetySystem.hpp>
#include <iostream>
#include <string.h>
#include <unistd.h>

using namespace scara;
using namespace eeros::sequencer;
using namespace eeros::safety;
using namespace eeros::math;
using namespace eeros::control;

Move_Joystick::Move_Joystick(Sequencer* sequencer, ScaraControlSystem* controlSys, SafetySystem* safetySys, ScaraSafetyProperties* safetyProp) : 
					     	 Sequence<>("main", sequencer), controlSys(controlSys), safetySys(safetySys), safetyProp(safetyProp) {
	// nothing to do
}

void Move_Joystick::init() {
	std::bind(&Move_Joystick::init, *this);
}

bool Move_Joystick::checkPreCondition() {
	return safetySys->getCurrentLevel() == safetyProp->moving_joystick;
}

void Move_Joystick::run() {

	char a = 0; double inputScale;
	while(a != 'b') {
		log.warn() << "'v' = change joystick speed (scaling factor, default = 1.0)";
		log.warn() << "'b' = go back to the main menu";
	
		std::cin >> a;
		switch(a) {
			case 'v':
				log.warn() << "write a positive scaling factor for the speed (default = 1.0) and then press ENTER";
				std::cin >> inputScale; 
				controlSys->xbox.setSpeedScaleFactor(inputScale);
				log.info() << "Velocity scale set to: " << inputScale;
				break;
			default:
				// nothing to do
				break;
		}
		
		usleep(100000);
		if(isTerminating()) return;
		if(isEmergency()) return;
	}
	
	if(isTerminating()) return;
	if(isEmergency()) return;
	
	// Go back to ready position
	safetySys->triggerEvent(safetyProp->doStopMoving_joystick);
	while(safetySys->getCurrentLevel() != safetyProp->ready)
	{
		usleep(100000);
		if(isTerminating()) return;
	}
}

bool Move_Joystick::checkPostCondition() {
	return safetySys->getCurrentLevel() == safetyProp->ready;
}

void Move_Joystick::exit() {
}

bool Move_Joystick::isTerminating() {
	return sequencer->getState() == state::terminating;
}

bool Move_Joystick::isEmergency() {
	return (safetySys->getCurrentLevel() == safetyProp->emergency);
}
