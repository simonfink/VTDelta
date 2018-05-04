#include "ScaraSequenceGoToReady.hpp"
#include "../../ScaraControlSystem.hpp"
#include "../../ScaraSafetyProperties.hpp"
#include <eeros/safety/SafetySystem.hpp>
#include <unistd.h>
#include <iostream>

using namespace scara;
using namespace eeros::control;
using namespace eeros::sequencer;
using namespace eeros::safety;

ScaraSequenceGoToReady::ScaraSequenceGoToReady(Sequencer* sequencer, ScaraControlSystem* controlSys, SafetySystem* safetySys) : 
													Sequence<>("main", sequencer), controlSys(controlSys), safetySys(safetySys) {
	// nothing to do
}

void ScaraSequenceGoToReady::init() {
	std::bind(&ScaraSequenceGoToReady::init, *this);
}

bool ScaraSequenceGoToReady::checkPreCondition() {
	return safetySys->getCurrentLevel().getId() >= ready;
}

void ScaraSequenceGoToReady::run() {
	log.info() << "[ Go To Ready ] started";

	AxisVector x_actual = controlSys->dirKin.getOut().getSignal().getValue();
	controlSys->pathPlannerCS.setInitPos(x_actual);
	controlSys->posIntegral.setInitCondition(x_actual);
	controlSys->pathPlannerPosSwitch.switchToInput(1);		// cartesian path planner
	controlSys->autoToManualSwitch.switchToInput(0);		// automatic mode
	usleep(100000);
	
	controlSys->posIntegral.enable(); 
	safetySys->triggerEvent(doStartingMotion); 
	while(safetySys->getCurrentLevel().getId() < moving); 

	posUp << controlSys->dirKin.getOut().getSignal().getValue()(0), controlSys->dirKin.getOut().getSignal().getValue()(1), -0.105,  controlSys->dirKin.getOut().getSignal().getValue()(3);
	controlSys->pathPlannerCS.gotoPoint(posUp);
	while (!controlSys->pathPlannerCS.posReached()) {
		usleep(100000);
	}
	posReady = controlSys->readyPositionCartesian;
	controlSys->pathPlannerCS.gotoPoint(posReady);
	while (!controlSys->pathPlannerCS.posReached()) {
		usleep(100000);
	}

	controlSys->posIntegral.disable(); 
	safetySys->triggerEvent(doMotionStopping);	
	while(!(safetySys->getCurrentLevel().getId() == ready));
}

bool ScaraSequenceGoToReady::checkPostCondition() {
	return safetySys->getCurrentLevel().getId() == ready;
}

void ScaraSequenceGoToReady::exit() {
	log.info() << "[ Go To Ready ] exit done";
}