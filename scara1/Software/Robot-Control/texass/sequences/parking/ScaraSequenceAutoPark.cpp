#include "ScaraSequenceAutoPark.hpp"
#include "../../ScaraControlSystem.hpp"
#include "../../ScaraSafetyProperties.hpp"
#include <eeros/safety/SafetySystem.hpp>
#include <unistd.h>
#include <iostream>

using namespace scara;
using namespace eeros::control;
using namespace eeros::sequencer;
using namespace eeros::safety;

ScaraSequenceAutoPark::ScaraSequenceAutoPark(Sequencer* sequencer, ScaraControlSystem* controlSys, SafetySystem* safetySys) : 
											Sequence<>("main", sequencer), controlSys(controlSys), safetySys(safetySys) {
	// nothing to do
}

void ScaraSequenceAutoPark::init() {
	std::bind(&ScaraSequenceAutoPark::init, *this);
}

bool ScaraSequenceAutoPark::checkPreCondition() {
	return safetySys->getCurrentLevel().getId() == ready;
}

void ScaraSequenceAutoPark::run() {
	log.info() << "[ Auto Park ] started";

	AxisVector phi_actual = controlSys->muxEncPos.getOut().getSignal().getValue();
	controlSys->pathPlannerJS.setInitPos(phi_actual);
	controlSys->pathPlannerPosSwitch.switchToInput(0);		// joint path planner
	controlSys->autoToManualSwitch.switchToInput(0);		// automatic mode
	usleep(100000);

	log.info() << "DID YOU REMOVE THE TOOL FROM TCP? Press 'y' to continue";
	char a = 0; 
	while(a != 'y') {
		std::cin >> a;
	}
	
	safetySys->triggerEvent(doAutoParkingBeforeShutdown); 
	
// 	log.info() << "[ park 3 ]";
	qPark = controlSys->muxEncPos.getOut().getSignal().getValue();
	controlSys->pathPlannerJS.setInitPos(qPark); 
	qPark(3) = -0.07;
	controlSys->pathPlannerJS.gotoPoint(qPark); 
	while(safetySys->getCurrentLevel().getId() == autoParkingBeforeShutdown3);
	
// 	log.info() << "[ park 2 ]";
	qPark = controlSys->muxEncPos.getOut().getSignal().getValue();
	controlSys->pathPlannerJS.setInitPos(qPark);
	qPark(2) = 0.098;
	controlSys->pathPlannerJS.gotoPoint(qPark); 
	while(safetySys->getCurrentLevel().getId() == autoParkingBeforeShutdown2);
	
// 	log.info() << "[ park 1 ]";
	qPark = controlSys->muxEncPos.getOut().getSignal().getValue();
	controlSys->pathPlannerJS.setInitPos(qPark);
	qPark(1) = 2.48;
	controlSys->pathPlannerJS.gotoPoint(qPark); 
	while(safetySys->getCurrentLevel().getId() == autoParkingBeforeShutdown1);
	
// 	log.info() << "[ park 0 ]";
	qPark = controlSys->muxEncPos.getOut().getSignal().getValue();
	controlSys->pathPlannerJS.setInitPos(qPark);
	qPark(0) = 2.70;
	controlSys->pathPlannerJS.gotoPoint(qPark); 
	while(safetySys->getCurrentLevel().getId() == autoParkingBeforeShutdown0);
		
	while(safetySys->getCurrentLevel().getId() < systemOn) {usleep(100000);}
}

bool ScaraSequenceAutoPark::checkPostCondition() {
	return safetySys->getCurrentLevel().getId() == systemOn;
}

void ScaraSequenceAutoPark::exit() {
	log.info() << "[ Auto Park ] exit done";
}
