#include <eeros/safety/SafetySystem.hpp>
#include "ScaraSequenceMain.hpp"
#include "ScaraSafetyProperties.hpp"
#include "ScaraControlSystem.hpp"
#include <unistd.h>
#include <queue>

using namespace scara;
using namespace eeros::sequencer;
using namespace eeros::safety;
using namespace eeros::math;

ScaraSequenceMain::ScaraSequenceMain(Sequencer* sequencer, ScaraControlSystem* controlSys, SafetySystem* safetySys, ScaraSafetyProperties* safetyProp) : 
									Sequence<void>("main", sequencer), controlSys(controlSys), safetySys(safetySys), safetyProp(safetyProp),
									homeToReady(sequencer, controlSys, safetySys, safetyProp),
									goToReady(sequencer, controlSys, safetySys, safetyProp),
									moveSequence(sequencer, controlSys, safetySys, safetyProp),
									moveSequenceJoystick(sequencer, controlSys, safetySys, safetyProp)
									{
	// nothing to do
}

bool ScaraSequenceMain::checkPreCondition() {
	return safetySys->getCurrentLevel() >= safetyProp->off;
}

void ScaraSequenceMain::run() {
	log.trace() << "Sequencer '" << name << "': started.";
	
	// Wait that the robot is in going to ready state
	while(safetySys->getCurrentLevel() < safetyProp->goingToReady){
		usleep(100000);
		if(isTerminating()) return;
	}
	
	// Robot goingToReady -> start homeToReady sequence
	homeToReady();
	
	// Robot ready -> choose what to do 
	char m = 0;
	while(m != 'k') {
		
		if(isEmergency())
			log.info() << "Robot is in EMERGENCY state! Press 'r' to reset emergency";
		else {
			log.info() << "p = moving with pathplanner";
// 			log.info() << "j = moving with joystick";
			log.info() << "e = exit";
		}
		
		std::cin >> m;
		switch(m) {
			case 'p':
				std::cout << "Path planner" << std::endl;
				safetySys->triggerEvent(safetyProp->doSetMoving);
				while(safetySys->getCurrentLevel() != safetyProp->moving)
				{
					usleep(100000);
					if(isTerminating()) return;
					if(isEmergency()) return;
				}
				moveSequence();
				if(isEmergency()) break;
				
				safetySys->triggerEvent(safetyProp->doSetMoving);
				while(safetySys->getCurrentLevel() != safetyProp->moving)
				{
					usleep(100000);
					if(isTerminating()) return;
					if(isEmergency()) return;
				}
				goToReady();
				
				break;
				
			case 'j':
				std::cout << "Joystick" << std::endl;
				safetySys->triggerEvent(safetyProp->doSetMoving_joystick);
				while(safetySys->getCurrentLevel() != safetyProp->moving_joystick)
				{
					usleep(100000);
					if(isTerminating()) return;
					if(isEmergency()) return;
				}
				moveSequenceJoystick();
				if(isEmergency()) break;
				
				safetySys->triggerEvent(safetyProp->doSetMoving);
				while(safetySys->getCurrentLevel() != safetyProp->moving)
				{
					usleep(100000);
					if(isTerminating()) return;
					if(isEmergency()) return;
				}
				goToReady();
				
				break;
			
			case 'e':	// Exit from loop
				std::cout << "Exit loop" << std::endl;
				m = 'k';
				break;
				
			case 'r':  // Reset Emergency
				safetySys->triggerEvent(safetyProp->doResetEmergency);
				
				while(safetySys->getCurrentLevel() != safetyProp->goingToReady)
				{
					usleep(100000);
					if(isTerminating()) return;
					if(isEmergency()) return;
				}
				goToReady();
				
			default:
				// nothing to do
				break;
		}
	}

	// Safety level must be 'ready' here
	// Autopark
	safetySys->triggerEvent(safetyProp->doAutoParking_shutdown);
	
	// Wait safety system shuts down
	while(!(safetySys->getCurrentLevel() == safetyProp->off));

	log.trace() << "Sequencer '" << name << "': finished";
}

void ScaraSequenceMain::exit() {
	log.info() << "[ Exit Main Sequence ]";
}

bool ScaraSequenceMain::isTerminating() {
	return sequencer->getState() == state::terminating;
}

bool ScaraSequenceMain::isEmergency() {
	return (safetySys->getCurrentLevel() == safetyProp->emergency);
}