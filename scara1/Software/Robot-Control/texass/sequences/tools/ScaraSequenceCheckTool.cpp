#include "ScaraSequenceCheckTool.hpp"
#include "../../ScaraControlSystem.hpp"
#include "../../ScaraSafetyProperties.hpp"
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/safety/inputActions.hpp>
#include <unistd.h>
#include <iostream>
#include <cmath>
#include <iostream>
#include <fstream>

using namespace scara;
using namespace eeros::control;
using namespace eeros::sequencer;
using namespace eeros::safety;
using namespace eeros::math;
using namespace eeros::hal;

ScaraSequenceCheckTool::ScaraSequenceCheckTool(Sequencer* sequencer, ScaraControlSystem* controlSys, SafetySystem* safetySys) : 
										Sequence<void,char,AxisVector>("main", sequencer), controlSys(controlSys), safetySys(safetySys) {
	// nothing to do
}

void ScaraSequenceCheckTool::init() {
	std::bind(&ScaraSequenceCheckTool::init, *this);
}

bool ScaraSequenceCheckTool::checkPreCondition() {
	return safetySys->getCurrentLevel().getId() == teaching;
}

void ScaraSequenceCheckTool::run(char tool_in,AxisVector checkPoint_in) {
	log.info() << "[ Check Tool ] started";
	controlSys->setPlasmaPen.enable();
	
	// Define toolc and check point used
	this->checkPoint = checkPoint_in;
	if(tool_in != 'p' && tool_in!= 'd' && tool_in != 'g' && tool_in != 'c'){
		throw EEROSException("Wrong tool id selected");
	}
	else
		this->tool = tool_in;
	
	// Initialize path planners and controllers
	AxisVector x_actual = controlSys->dirKin.getOut().getSignal().getValue();
	controlSys->pathPlannerCS.setInitPos(x_actual);
	controlSys->posIntegral.setInitCondition(x_actual);
	controlSys->xbox.setInitPos(x_actual);
	controlSys->pathPlannerPosSwitch.switchToInput(1);	// cartesian path planner
	controlSys->autoToManualSwitch.switchToInput(1);	// manual mode
	
	controlSys->posIntegral.enable(); 
	
	char m = 0; int i = 0;
	AxisVector offset;
	while(m!='b') {
		log.info() << "Press 's' + ENTER to save the right position";
		log.info() << "Press 'v' + ENTER to change the joystick speed (scaling factor, default = 1.0)";
		
		std::cin >> m;
		switch(m) {
			case 's':
				offset = controlSys->dirKin.getOut().getSignal().getValue() - checkPoint;
				log.info() << "offset: " << offset(0) << ", " << offset(1) << ", " << offset(2) << ", " << offset(3);
				
				// p = plasmapen; g = greifer; d = dispencer; c = camera; n = no tool
				if (tool == 'p'){
					controlSys->plasmaX = offset(0); 
					controlSys->plasmaY = offset(1);
					controlSys->plasmaZ = offset(2);
					controlSys->plasmaAlpha = controlSys->dirKin.getOut().getSignal().getValue()(3);
				}
				else if (tool == 'g'){
					controlSys->greiferX = offset(0);
					controlSys->greiferY = offset(1);
					controlSys->greiferZ = offset(2);
					controlSys->greiferAlpha = controlSys->dirKin.getOut().getSignal().getValue()(3);
				}
				else if (tool == 'd'){
					controlSys->dispencerX = offset(0);
					controlSys->dispencerY = offset(1);
					controlSys->dispencerZ = offset(2);
					controlSys->dispencerAlpha = controlSys->dirKin.getOut().getSignal().getValue()(3);
				}
				else if (tool == 'c'){
					controlSys->cameraX = - offset(0);
					controlSys->cameraY = - offset(1);
					controlSys->cameraZ = - offset(2);
					controlSys->cameraAlpha = controlSys->dirKin.getOut().getSignal().getValue()(3);
				}
				else if (tool == 'k'){
					controlSys->kameraX = offset(0);
					controlSys->kameraY = offset(1);
					controlSys->kameraZ = offset(2);
					controlSys->kameraAlpha = controlSys->dirKin.getOut().getSignal().getValue()(3);
				}
				// set new ready position (same orientation angle)
				controlSys->readyPositionCartesian(3) =  controlSys->dirKin.getOut().getSignal().getValue()(3);
				// end process					
				m = 'b'; 
				break;
			case 'v':
				log.info() << "Write a positive scaling factor for the speed (default = 1.0) and then press ENTER";
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
	controlSys->setPlasmaPen.disable();
	
	safetySys->triggerEvent(doMotionStopping);
	while(!(safetySys->getCurrentLevel().getId() == ready));	
}

bool ScaraSequenceCheckTool::checkPostCondition() {
	return safetySys->getCurrentLevel().getId() == ready;
}

void ScaraSequenceCheckTool::exit() {
	AxisVector x_actual = controlSys->dirKin.getOut().getSignal().getValue();
	controlSys->pathPlannerCS.setInitPos(x_actual);
	controlSys->autoToManualSwitch.switchToInput(0);
	controlSys->xbox.setSpeedScaleFactor(1.0);
	log.info() << "[ Check Tool ] exit done";
}