#include "ScaraSequenceGoToToolChange.hpp"
#include "../../ScaraControlSystem.hpp"
#include "../../ScaraSafetyProperties.hpp"
#include <eeros/safety/SafetySystem.hpp>
#include <unistd.h>
#include <iostream>

using namespace scara;
using namespace eeros::control;
using namespace eeros::sequencer;
using namespace eeros::safety;

ScaraSequenceGoToToolChange::ScaraSequenceGoToToolChange(Sequencer* sequencer, ScaraControlSystem* controlSys, SafetySystem* safetySys) : 
										Sequence<>("main", sequencer), controlSys(controlSys), safetySys(safetySys) {
	// nothing to do
}

void ScaraSequenceGoToToolChange::init() {
	std::bind(&ScaraSequenceGoToToolChange::init, *this);
}

bool ScaraSequenceGoToToolChange::checkPreCondition() {
	return safetySys->getCurrentLevel().getId() >= ready;
}

void ScaraSequenceGoToToolChange::run() {
	log.info() << "[ Tool Change ] started";

	AxisVector x_actual = controlSys->dirKin.getOut().getSignal().getValue();
	controlSys->pathPlannerCS.setInitPos(x_actual);
	controlSys->posIntegral.setInitCondition(x_actual);
	controlSys->pathPlannerPosSwitch.switchToInput(1);	// cartesian path planner
	controlSys->autoToManualSwitch.switchToInput(0);	// automatic mode
	usleep(100000);
	
	controlSys->posIntegral.enable(); 
	safetySys->triggerEvent(doStartingMotion); 
	while(safetySys->getCurrentLevel().getId() < moving);
	
	xTool << 0.25, -0.35,  -0.105, controlSys->dirKin.getOut().getSignal().getValue()(3);
	controlSys->pathPlannerCS.gotoPoint(xTool);
	while (!controlSys->pathPlannerCS.posReached()) {
		usleep(100000);
	}

	controlSys->posIntegral.disable();
	safetySys->triggerEvent(doMotionStopping);
	while(!(safetySys->getCurrentLevel().getId() == ready));

	log.info() << "Mount the tool and press 'y' to go back to ready position";
	char a = 0;
	while(a != 'y') {
		std::cin >> a;
	}
}

bool ScaraSequenceGoToToolChange::checkPostCondition() {
	return safetySys->getCurrentLevel().getId() == ready;
}

void ScaraSequenceGoToToolChange::exit() {
	log.info() << "[ Tool Change ] Exit Done";
}
