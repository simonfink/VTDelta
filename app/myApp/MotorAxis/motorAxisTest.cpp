#include <iostream>
#include <ostream>
#include <fstream>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>

#include <eeros/control/PeripheralInput.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/control/PeripheralOutput.hpp>
#include <eeros/control/Saturation.hpp>
#include <eeros/task/Lambda.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/hal/HAL.hpp>
#include <eeros/sequencer/Sequencer.hpp>

#include "MotorAxisControlSystem.hpp"
#include "MotorAxisSafetyProperties.hpp"
#include "MotorAxisSequence.hpp"


using namespace eeros;
using namespace eeros::hal;
using namespace eeros::control;
using namespace eeros::safety;
using namespace eeros::logger;
using namespace eeros::sequencer;
using namespace eeros::task;


void signalHandler(int signum){
	SafetySystem::exitHandler();
	Sequencer::instance().abort();
}

const double td = 0.1;

int main(int argc, char **argv) {
	signal(SIGINT, signalHandler);
	
	StreamLogWriter w(std::cout);
	Logger::setDefaultWriter(&w);
	Logger log;
	w.show();
	
	log.info() << "Simple Motor Controller Demo started...";
	
	log.info() << "Initializing Hardware...";
	HAL& hal = HAL::instance();
	hal.readConfigFromFile(&argc, argv);
	
	// Create the control system
	MotorAxisControlSystem controlSys(td);
	
	// Create and initialize a safety system
	MotorAxisSafetyProperties properties(controlSys, td);
	SafetySystem safetySys(properties, td);
	//controlSys.timedomain.registerSafetyEvent(safetySys, properties.seRun);
	
	auto& sequencer = Sequencer::instance();
	MotorAxisSequence mainSequence("Main Sequence", sequencer, safetySys, properties, controlSys, 3.14/10);
	sequencer.addSequence(mainSequence);
	mainSequence.start();
	
	auto &executor = Executor::instance();
	executor.setMainTask(safetySys);
	//safetySys.triggerEvent(properties.seStop);
	executor.add(controlSys.timedomain);
	
	executor.run();
	
	sequencer.join();
	log.info() << "Example finished...";

	return 0;
}