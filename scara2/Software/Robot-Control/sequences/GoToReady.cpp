#include "GoToReady.hpp"
#include <eeros/safety/SafetySystem.hpp>
#include <unistd.h>
#include "../control/ParallelScaraControlSystem.hpp"
#include "../safety/ParallelScaraSafetyProperties.hpp"
#include "../constants.hpp"

using namespace parallelscara;
using namespace eeros::control;
using namespace eeros::sequencer;
using namespace eeros::safety;

GoToReady::GoToReady(Sequencer* sequencer, ParallelScaraControlSystem* controlSys, SafetySystem* safetySys, ParallelScaraSafetyProperties* safetyProp) : 
		Sequence<>("main", sequencer), controlSys(controlSys), safetySys(safetySys), safetyProp(safetyProp) {
	// nothing to do
}

void GoToReady::init() {
	std::bind(&GoToReady::init, *this);
}

bool GoToReady::checkPreCondition() {
	return safetySys->getCurrentLevel() == safetyProp->goingToSystemReady;
}

void GoToReady::run() {
	log.trace() << "sequence [ Go To Ready ] started";
	
	sleep(1);
	
	controlSys->pathPlanner.setMaxSpeed(1.0);
	controlSys->pathPlanner.setMaxAcc(0.5);
	
	controlSys->pathPlanner.move(ready_pos);
	while (!controlSys->pathPlanner.posReached() && safetySys->getCurrentLevel() == safetyProp->goingToSystemReady) {
			usleep(10000);
			if (isTerminating()) return;
			if (!(safetySys->getCurrentLevel() == safetyProp->goingToSystemReady)) break;
		}
	
	// Handle eventual emergency & go out of sequence
	while(safetySys->getCurrentLevel() < safetyProp->powerOn) usleep(100000);
			
	if(safetySys->getCurrentLevel() == safetyProp->goingToSystemReady) {
		safetySys->triggerEvent(safetyProp->readyDone);
		AxisVector actPos = controlSys->robotController.getEncPosAct().getSignal().getValue();
		while(!(safetySys->getCurrentLevel() == safetyProp->systemReady)) usleep(100000); 
	}
	
	if(safetySys->getCurrentLevel() == safetyProp->powerOn) {
		safetySys->triggerEvent(safetyProp->doWaitReady);
		while (!(safetySys->getCurrentLevel() == safetyProp->homedWaitReady)) usleep(100000); 
	}
}

bool GoToReady::checkPostCondition() {
	return (safetySys->getCurrentLevel() == safetyProp->systemReady || safetySys->getCurrentLevel() == safetyProp->homedWaitReady);
}

void GoToReady::exit() {
	log.trace() << "sequence [ Go To Ready ] done";
}

inline bool GoToReady::isTerminating() {
	return sequencer->getState() == state::terminating;
}

// inline bool GoToReady::isStopping() {
// 	return !(safetySys->getCurrentLevel() == safetyProp->goingToSystemReady);
// }