#include <unistd.h>
#include <iostream>
#include <signal.h>

#include <eeros/core/Executor.hpp>
#include <eeros/hal/HAL.hpp>
#include <eeros/hal/ComediDevice.hpp>
#include <eeros/hal/ComediDigIn.hpp>
#include <eeros/hal/ComediDigOut.hpp>
#include <eeros/hal/ComediFqd.hpp>
#include <eeros/hal/ComediDac.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp> 
#include <eeros/logger/SysLogWriter.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/sequencer/Sequencer.hpp>
#include "PPSafetyProperties.hpp"
#include "PPControlSystem.hpp"
#include "../../constants.hpp"

using namespace scara;
using namespace eeros;
using namespace eeros::hal;
using namespace eeros::safety;
using namespace eeros::control;
using namespace eeros::sequencer;
using namespace eeros::logger;

volatile bool running = true;
void signalHandler(int signum) {
	running = false;
}

int main() {
	signal(SIGINT, signalHandler);
	
	StreamLogWriter w(std::cout);
	w.show(); // show(0) to switch off 
	Logger<LogWriter>::setDefaultWriter(&w);
	
	Logger<LogWriter> log('M');
	log.trace() << "SCARA Robot Control Application started";
	log.set(w);
	
	auto &executor = eeros::Executor::instance();
	
	// Get Control System instance
	PPControlSystem controlSystem;
	
	// Get Safety System instance
	PPSafetyProperties safetyProperties(&controlSystem);
	SafetySystem safetySystem(safetyProperties, dt);
	executor.setMainTask(safetySystem);
	
	executor.run();

	return 0;
}


