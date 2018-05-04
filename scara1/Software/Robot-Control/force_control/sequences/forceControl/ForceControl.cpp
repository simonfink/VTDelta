#include "ForceControl.hpp"
#include "../../ScaraControlSystem_force.hpp"
#include "../../ScaraSafetyProperties_force.hpp"
#include <eeros/safety/SafetySystem.hpp>
#include <unistd.h>
#include <iostream>
#include <stdio.h>

using namespace scara;
using namespace eeros::control;
using namespace eeros::sequencer;
using namespace eeros::safety;

ForceControl::ForceControl(Sequencer* sequencer, ScaraControlSystem_force* controlSys, SafetySystem* safetySys, ScaraSafetyProperties_force* safetyProp) : 
						 Sequence<>("main", sequencer), controlSys(controlSys), safetySys(safetySys), safetyProp(safetyProp) {
	// nothing to do
}

void ForceControl::init() {
	std::bind(&ForceControl::init, *this);
}

bool ForceControl::checkPreCondition() {
	return safetySys->getCurrentLevel() == safetyProp->force_control;
}

void ForceControl::run() {
	log.info() << "[ ForceControl ] started";
	
	AxisVector pos1; pos1 << 0.35, -0.05, -0.105, 0; //0.2, 0, -0.105, 0;
	AxisVector pos2; pos2 << 0.35, -0.01, -0.125, 0; //0.45,  0, -0.105, 0;
	
	controlSys->pathPlannerCS.move(pos1);
	while (!controlSys->pathPlannerCS.posReached()) 
	{
		usleep(100000);
		if(isTerminating()) return;
		if(isEmergency()) return;
	}
	
	if(isTerminating()) return;
	if(isEmergency()) return;
	
	controlSys->pathPlannerCS.move(pos2); 
	while (!controlSys->pathPlannerCS.posReached()) 
	{
		usleep(100000);
		if(isTerminating()) return;
		if(isEmergency()) return;
	}

	if(isTerminating()) return;
	if(isEmergency()) return;
	char m = 0;
		

		log.info() << "a = direct force control";
		log.info() << "b = force control with inner velocity loop";
		log.info() << "c = forcecontrol with inner position loop";
		log.info() << "d = forcecontrol modell based";
		AxisVector init_value;
		init_value.zero();
		
		std::cin >> m;
		while(m != 'k') {
			switch(m) {
				case 'a':
					controlSys->I_pt1.setInitCondition(init_value);
					controlSys->positionOrForceControlSwitch.switchToInput(1);
					m = 'k';
					break;
				
				case 'b':
					controlSys->I_pt1.setInitCondition(init_value);
					controlSys->positionOrForceControlSwitch.switchToInput(2);
					m = 'k';
					break;
			
				case 'c':	// Exit from loop
					controlSys->I_pt1.setInitCondition(init_value);
					controlSys->positionOrForceControlSwitch.switchToInput(3);
					m = 'k';
					break;
				case 'd':	// Exit from loop
					 	controlSys->I_pt1.setInitCondition(init_value);
						controlSys->I_vel_to_pos.setInitCondition(pos2);
						controlSys->I_acc_to_vel.setInitCondition(init_value);
						controlSys->positionOrForceControlSwitch.switchToInput(0);
						controlSys->cartesRefSwitch.switchToInput(2);
						controlSys->jointsRefSwitch.switchToInput(1);
						controlSys->initSwitch.switchToInput(1);
					m = 'k';
					break;
				
				default:
					// nothing to do
					break;
			} 
		}

    char c;
	std::cin >> c;
	while((c !='e'))
		{
			if(controlSys->muxEncPos.getIn(1).getSignal().getValue() <= -2.5){
				log.info() << "q2 out of range";
				std::cout << " 1 = " << controlSys->muxEncPos.getIn(1).getSignal().getValue() << std::endl;
				c = 'e';	
			}
			if(controlSys->muxEncPos.getIn(1).getSignal().getValue() >= -.70){
				std::cout << " 1 = " << controlSys->muxEncPos.getIn(1).getSignal().getValue() << std::endl;
				log.info() << "q2 out of range";
				c = 'e';
			}
		}
	controlSys->positionOrForceControlSwitch.switchToInput(0);
	
	safetySys->triggerEvent(safetyProp->doStopForce_control);
	while(safetySys->getCurrentLevel() != safetyProp->ready)
	{
		usleep(100000);
		if(isTerminating()) return;
		if(isEmergency()) return;
	}
	
	if(isTerminating()) return;
	if(isEmergency()) return;
	//controlSys->positionOrForceControlSwitch.switchToInput(0);
	
}

bool ForceControl::checkPostCondition() {
	return safetySys->getCurrentLevel() == safetyProp->ready;
}

void ForceControl::exit() {
	log.info() << "[ ForceControl ] exit done";
}

bool ForceControl::isTerminating() {
	return sequencer->getState() == state::terminating;
}

bool ForceControl::isEmergency() {
	return (safetySys->getCurrentLevel() == safetyProp->emergency);
}
