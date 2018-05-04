 #include <eeros/safety/SafetySystem.hpp>
#include "ScaraSequenceMain_force.hpp"
#include "ScaraSafetyProperties_force.hpp"
#include "ScaraControlSystem_force.hpp"
#include <unistd.h>
#include <queue>

using namespace scara;
using namespace eeros::sequencer;
using namespace eeros::safety;
using namespace eeros::math;

ScaraSequenceMain_force::ScaraSequenceMain_force(Sequencer* sequencer, ScaraControlSystem_force* controlSys, SafetySystem* safetySys, ScaraSafetyProperties_force* safetyProp) : 
									Sequence<void>("main", sequencer), controlSys(controlSys), safetySys(safetySys), safetyProp(safetyProp),
									homeToReady(sequencer, controlSys, safetySys, safetyProp),
									goToReady(sequencer, controlSys, safetySys, safetyProp),
									moveSequence(sequencer, controlSys, safetySys, safetyProp),
									moveSequenceJoystick(sequencer, controlSys, safetySys, safetyProp),
									forceControlSequence(sequencer, controlSys, safetySys, safetyProp)
									{
	// nothing to do
}

bool ScaraSequenceMain_force::checkPreCondition() {
	return safetySys->getCurrentLevel() >= safetyProp->off;
}

void ScaraSequenceMain_force::run() {
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
			log.info() << "f = moving with forcControl";
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
				
			case 'f':
				std::cout << "Foce Control" << std::endl;
				safetySys->triggerEvent(safetyProp->doSetForce_control);
				while(safetySys->getCurrentLevel() != safetyProp->force_control)
				{
					usleep(100000);
					if(isTerminating()) return;
					if(isEmergency()) return;
				}
				forceControlSequence();
				if(isEmergency()) break;

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

void ScaraSequenceMain_force::exit() {
	log.info() << "[ Exit Main Sequence ]";
}

bool ScaraSequenceMain_force::isTerminating() {
	return sequencer->getState() == state::terminating;
}

bool ScaraSequenceMain_force::isEmergency() {
	return (safetySys->getCurrentLevel() == safetyProp->emergency);
}