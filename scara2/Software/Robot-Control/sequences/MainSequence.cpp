#include "MainSequence.hpp"
#include <eeros/safety/SafetySystem.hpp>
#include "../safety/ParallelScaraSafetyProperties.hpp"
#include "../control/ParallelScaraControlSystem.hpp"
#include <eeros/core/Fault.hpp>
#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/Sequence.hpp>

#include <unistd.h>
#include <queue>
#include <fstream>

using namespace eeros;
using namespace parallelscara;
using namespace eeros::sequencer;
using namespace eeros::safety;
using namespace eeros::math;

MainSequence::MainSequence(Sequencer& seq, ParallelScaraControlSystem* controlSys, SafetySystem* safetySys, ParallelScaraSafetyProperties* safetyProp) : 
							Sequence("main sequence", seq), 
							controlSys(controlSys), safetySys(safetySys), safetyProp(safetyProp)
							/*, homing_s(sequencer, controlSys, safetySys, safetyProp), goToReady_s(sequencer, controlSys, safetySys, safetyProp), 
							highSpeed_s(sequencer, controlSys, safetySys, safetyProp), balancing_s(sequencer, controlSys, safetySys, safetyProp)*/ 
							{
	setNonBlocking();
}

// bool MainSequence::checkPreCondition() {
// 	return safetySys->getCurrentLevel() >= safetyProp->off;
// }

int MainSequence::action() {
	log.trace() << "Sequencer '" << name << "': started.";
	
	while(safetySys->getCurrentLevel() < safetyProp->powerOn) {     // start up
		usleep(100000); /*if (isTerminating())*/ return 0; }
		
	safetySys->triggerEvent(safetyProp->startWaitHoming);                      // wait for homing
	while(safetySys->getCurrentLevel() < safetyProp->waitForHoming) {
		usleep(100000); /*if (isTerminating())*/ return 0; }
		
	safetySys->triggerEvent(safetyProp->startHoming);                          // do homing
	while(safetySys->getCurrentLevel() < safetyProp->homing) {
		usleep(100000); /*if (isTerminating())*/ return 0; }
	log.trace() << "main seq homing started";	
// 	homing_s();                                                 // homing
	log.trace() << "main seq homing done";	
	
// 	while( sequencer->getState() != state::terminating){     // !(safetySys->getCurrentLevel().getId() == off) &&
		
		if(safetySys->getCurrentLevel() == safetyProp->homedWaitReady) {     // go to ready
			safetySys->triggerEvent(safetyProp->doReady);
			while(safetySys->getCurrentLevel() < safetyProp->goingToSystemReady) {
				usleep(100000); /*if (isTerminating())*/ return 0; }
// 			goToReady_s();
		}
		else if(safetySys->getCurrentLevel() == safetyProp->powerOn) {       // restore after emergency
			safetySys->triggerEvent(safetyProp->doWaitReady);
		}
		else if(safetySys->getCurrentLevel() == safetyProp->highSpeed) {     // high speed sequence
// 			highSpeed_s();
		}
		else if(safetySys->getCurrentLevel() == safetyProp->balancing) {     // balancing sequence
// 			balancing_s();
		}
		else {
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
// 	}
// 	log.trace() << "Sequencer '" << name << "': finished";
}

// void MainSequence::exit() {
// 	log.info() << "[ Exit Main Sequence ]";
// }

// inline bool MainSequence::isTerminating() {
// // 	return sequencer->getState() == state::terminating;
// }