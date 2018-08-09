#include <iostream>
#include <ostream>
#include <fstream>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>

#include <eeros/hal/HAL.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/Monitor.hpp>

#include "control/DeltaControlSystem.hpp"
#include "safety/DeltaSafetyProperties.hpp"

#include "sequence/CalibrateSequence.hpp"
#include "sequence/ShuffleSequence.hpp"
#include "sequence/SortSequence.hpp"
#include "sequence/MouseSequence.hpp"
#include "sequence/MainSequence.hpp"
#include "sequence/ExceptionSequence.hpp"

//#include "conditions/MissingBlock.hpp"
#include "conditions/MoveMouse.hpp"

using namespace eeros;
using namespace eeros::hal;
using namespace eeros::control;
using namespace eeros::safety;
using namespace eeros::logger;
using namespace eeros::sequencer;

using namespace eeduro::delta;

void signalHandler(int signum){
	SafetySystem::exitHandler();
	Executor::stop();
	Sequencer::instance().abort();
}

int main(int argc, char **argv) {
	signal(SIGINT, signalHandler);
	
	StreamLogWriter w(std::cout);
	Logger::setDefaultWriter(&w);
	Logger log;
	w.show();
	
	log.info() << "delta test started...";
	
	log.info() << "Initializing Hardware...";
	HAL& hal = HAL::instance();
	hal.readConfigFromFile(&argc, argv);
	
	// Create the control system
	DeltaControlSystem controlSys(dt);
	
	// Create and initialize a safety system
	DeltaSafetyProperties properties(controlSys, dt);
	SafetySystem safetySys(properties, dt);
	controlSys.timedomain.registerSafetyEvent(safetySys, properties.doEmergency);
	
	Calibration calibration;
	calibration.loadDefaults();
	if (!calibration.load()) {
		log.warn() << "could not load calibration";
	}
	
	auto& sequencer = Sequencer::instance();

  	MainSequence mainSequence("Main Sequence", sequencer, controlSys, safetySys, properties, calibration);
// 	MouseSequence mouseSequence("Mouse Sequence", sequencer, controlSys, safetySys, calibration);
// 	SortSequence sortSequence("Sort Sequence", sequencer, controlSys, safetySys,calibration);
// 	ShuffleSequence shuffleSequence("Shuffle Sequence",sequencer, controlSys, safetySys, calibration);
//  	CalibrateSequence calibSequence("Calibration Sequence", sequencer, controlSys, safetySys, calibration);
// 	ExceptionSequence exSeq("Exception Sequence", sequencer, controlSys, safetySys, properties);
	
 	sequencer.addSequence(mainSequence);
// 	sequencer.addSequence(mouseSequence);
// 	sequencer.addSequence(sortSequence);
// 	sequencer.addSequence(shuffleSequence);
//  	sequencer.addSequence(calibSequence);
	
//   	calibSequence.start();
//  	mainSequence.start();
	
	
	auto &executor = Executor::instance();
	executor.setMainTask(safetySys);
	safetySys.triggerEvent(properties.doSwInit);
	
	executor.run();
	
	sequencer.wait();
	
	safetySys.triggerEvent(properties.doParking);
	safetySys.exitHandler();

	controlSys.stop();
	
	log.info() << "Example finished...";
	return 0;
}