#include "ScaraSequenceManual.hpp"
#include "../referenceSystems/ScaraSequenceCalculateRefSystem.hpp"
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

ScaraSequenceManual::ScaraSequenceManual(Sequencer* sequencer, ScaraControlSystem* controlSys, SafetySystem* safetySys) : 
										Sequence<>("main", sequencer), controlSys(controlSys), safetySys(safetySys), 
										goToReady(sequencer, controlSys, safetySys) {
	// nothing to do
}

void ScaraSequenceManual::init() {
	std::bind(&ScaraSequenceManual::init, *this);
}

bool ScaraSequenceManual::checkPreCondition() {
	return safetySys->getCurrentLevel().getId() == teaching;
}

void ScaraSequenceManual::run() {
	log.info() << "[ Manual ] started";
	AxisVector x_actual = controlSys->dirKin.getOut().getSignal().getValue();
	controlSys->pathPlannerCS.setInitPos(x_actual);
	controlSys->posIntegral.setInitCondition(x_actual);
	controlSys->xbox.setInitPos(x_actual);
	controlSys->pathPlannerPosSwitch.switchToInput(1);	// cartesian path planner
	controlSys->autoToManualSwitch.switchToInput(1);	// manual mode
	usleep(100000);
	
	controlSys->posIntegral.enable();
	
	char a = 0; double inputScale;
	while(a != 'b') {
		log.warn() << "'v' = change joystick speed (scaling factor, default = 1.0)";
		log.warn() << "'b' = go back to the main menu";
	
		std::cin >> a;
		switch(a) {
			case 'a':
				log.info() << controlSys->autoToManualSwitch.getOut().getSignal().getValue()(0) << "; " << controlSys->autoToManualSwitch.getOut().getSignal().getValue()(1) << "; " << controlSys->autoToManualSwitch.getOut().getSignal().getValue()(2) << "; " << controlSys->autoToManualSwitch.getOut().getSignal().getValue()(3);
				log.info() << "in ws?: " << controlSys->cartesianWorkspaceLimit.in_workspace;
				break;
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
	}

	controlSys->posIntegral.disable(); 
	safetySys->triggerEvent(doMotionStopping);
	while(!(safetySys->getCurrentLevel().getId() == ready));
	
	goToReady();
}

bool ScaraSequenceManual::checkPostCondition() {
	return safetySys->getCurrentLevel().getId() == ready;
}

void ScaraSequenceManual::exit() {
	AxisVector x_actual = controlSys->dirKin.getOut().getSignal().getValue();
	controlSys->pathPlannerCS.setInitPos(x_actual);
	controlSys->autoToManualSwitch.switchToInput(0);
	log.info() << "[ Manual ] exit done";
}