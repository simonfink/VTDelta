#include "ScaraSequenceTEXASSplasmapen.hpp"
#include "../../../ScaraControlSystem.hpp"
#include "../../../ScaraSafetyProperties.hpp"
#include <eeros/safety/SafetySystem.hpp>
#include <unistd.h>
#include <iostream>

#include <eeros/hal/HAL.hpp>

using namespace scara;
using namespace eeros::control;
using namespace eeros::sequencer;
using namespace eeros::safety;
using namespace eeros::hal;
using namespace eeros::math;

enum {
	compr_air,
	valve
};

ScaraSequenceTEXASSplasmapen::ScaraSequenceTEXASSplasmapen(Sequencer* sequencer, ScaraControlSystem* controlSys, SafetySystem* safetySys) : 
													Sequence<void>("main", sequencer), controlSys(controlSys), safetySys(safetySys), 
													goToTool(sequencer, controlSys, safetySys), goToReady(sequencer, controlSys, safetySys), 
													checkTool(sequencer, controlSys, safetySys), plasmaProcess(sequencer, controlSys, safetySys) {
	// nothing to do
}

void ScaraSequenceTEXASSplasmapen::init() {
	std::bind(&ScaraSequenceTEXASSplasmapen::init, *this);
}

bool ScaraSequenceTEXASSplasmapen::checkPreCondition() {
	return safetySys->getCurrentLevel().getId() >= ready;
}

void ScaraSequenceTEXASSplasmapen::run() {
	log.info() << "[ Plasmapen ] started";
	
	HAL& hal = HAL::instance();
	
	// Mount Plasmapen check tool
	goToTool();
	goToReady();
	
	// Check Tool
	safetySys->triggerEvent(doTeaching);
	checkTool('p', controlSys->toBasisCoordinate(controlSys->tipRefPoint, 'n', controlSys->bauteile));
	goToReady();
	
	// Mount Plasmapen tool
	goToTool();
	
	// Warm up
	log.info() << "Plasmapen warm up";
	hal.getLogicPeripheralOutput("plasmaON")->set(true); 
	sleep(25);
	hal.getLogicPeripheralOutput("plasmaON")->set(false);
	sleep(1);
	hal.getLogicPeripheralOutput("ventil")->set(true);
	sleep(1);
	
	// Go to ready
	goToReady();
	
	char a = 0; 
	log.info() << "Press 's' to start the process";
	while (a != 's'){
		std::cin >> a;
	}
	usleep(100000); 
	
	// Run plasma process
	plasmaProcess({0.0514,-0.00351});
	//plasmaProcess({0.0512,0.02204});
	//plasmaProcess({0.0512,0.04719});
	//plasmaProcess({0.0513,0.07259});
	goToReady();
}

bool ScaraSequenceTEXASSplasmapen::checkPostCondition() {
	return safetySys->getCurrentLevel().getId() == ready;
}

void ScaraSequenceTEXASSplasmapen::exit() {
	log.info() << "[ Plasmapen ] exit done";
}


