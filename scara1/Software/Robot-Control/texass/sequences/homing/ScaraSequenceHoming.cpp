#include "ScaraSequenceHoming.hpp"
#include "../../ScaraControlSystem.hpp"
#include "../../ScaraSafetyProperties.hpp"
#include <eeros/safety/SafetySystem.hpp>
#include <unistd.h>
#include <iostream>

using namespace scara;
using namespace eeros::control;
using namespace eeros::sequencer;
using namespace eeros::safety;

ScaraSequenceHoming::ScaraSequenceHoming(Sequencer* sequencer, ScaraControlSystem* controlSys, SafetySystem* safetySys) : 
										Sequence<>("main", sequencer), controlSys(controlSys), safetySys(safetySys) {
	// nothing to do
}

void ScaraSequenceHoming::init() {
	std::bind(&ScaraSequenceHoming::init, *this);
}

bool ScaraSequenceHoming::checkPreCondition() {
	return safetySys->getCurrentLevel().getId() == robotParked;
}

void ScaraSequenceHoming::run() {
	log.info() << "[ Homing Sequence Started ]";
	
	// Initialize Path Planner and switches
	AxisVector phi_actual = controlSys->muxEncPos.getOut().getSignal().getValue();
	AxisVector x_actual = controlSys->dirKin.getOut().getSignal().getValue();
	controlSys->pathPlannerJS.setInitPos(phi_actual);
	controlSys->pathPlannerCS.setInitPos(x_actual);
	controlSys->pathPlannerPosSwitch.switchToInput(0);		// joint path planner
	controlSys->autoToManualSwitch.switchToInput(0);		// automatic mode

	usleep(100000);
	while(safetySys->getCurrentLevel().getId() < homing3);

// 	log.info() << "[ home 3 ]";
	qHome; qHome = controlSys->muxEncPos.getOut().getSignal().getValue();
	controlSys->pathPlannerJS.setInitPos(qHome);
	qHome(3) = controlSys->muxEncPos.getOut().getSignal().getValue()(3) + 0.4;
	controlSys->pathPlannerJS.gotoPoint(qHome); 
	while(safetySys->getCurrentLevel().getId() == homing3);
	
// 	log.info() << "[ home 2 ]";
	qHome = controlSys->muxEncPos.getOut().getSignal().getValue();
	controlSys->pathPlannerJS.setInitPos(qHome);
	qHome(2) = controlSys->muxEncPos.getOut().getSignal().getValue()(2) + 0.09;
	controlSys->pathPlannerJS.gotoPoint(qHome);
	while(safetySys->getCurrentLevel().getId() == homing2);
	
// 	log.info() << "[ home 1 ]";
	qHome = controlSys->muxEncPos.getOut().getSignal().getValue();
	controlSys->pathPlannerJS.setInitPos(qHome);
	qHome(1) = controlSys->muxEncPos.getOut().getSignal().getValue()(1) - 0.03;
	controlSys->pathPlannerJS.gotoPoint(qHome);
	while(safetySys->getCurrentLevel().getId() == homing1);
	
// 	log.info() << "[ home 0 ]";
	qHome = controlSys->muxEncPos.getOut().getSignal().getValue();
	controlSys->pathPlannerJS.setInitPos(qHome);
	qHome(0) = controlSys->muxEncPos.getOut().getSignal().getValue()(0) - 0.03;
	controlSys->pathPlannerJS.gotoPoint(qHome);
	while(safetySys->getCurrentLevel().getId() == homing0);
	
	while(safetySys->getCurrentLevel().getId() < systemOn) {usleep(100000);}
}

bool ScaraSequenceHoming::checkPostCondition() {
	return safetySys->getCurrentLevel().getId() == systemOn;
}

void ScaraSequenceHoming::exit() {
	log.info() << "[ Homing Sequence Exit Done ]";
}