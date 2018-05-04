#include <unistd.h>
#include <iostream>
#include <signal.h>

#include <eeros/core/Executor.hpp>
#include <eeros/hal/HAL.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp> 
#include <eeros/logger/SysLogWriter.hpp>
#include <eeros/safety/SafetySystem.hpp>
// #include <eeros/sequencer/Sequencer.hpp>
#include "force_control/ScaraSafetyProperties_force.hpp"
#include "force_control/ScaraControlSystem_force.hpp"
#include "force_control/ScaraSequenceMain_force.hpp" 
#include "constants.hpp"

using namespace scara;
using namespace eeros;
using namespace eeros::hal;
using namespace eeros::safety;
using namespace eeros::control;
using namespace eeros::sequencer;
using namespace eeros::logger;
using namespace eeros::safety;

volatile bool running = true;
void signalHandler(int signum) {
	running = false;
}

int main(int argc, char **argv) {
	signal(SIGINT, signalHandler);
	
	StreamLogWriter w(std::cout);
	w.show(); // show(0) to switch off 
	Logger::setDefaultWriter(&w);
	
	Logger log('M');
	log.trace() << "SCARA Robot Control Application started";
	log.set(w);
	
	// Get HAL instance & initialize HW
	HAL& hal = HAL::instance();
	hal.readConfigFromFile(&argc, argv);
	
	hal.getLogicOutput("setDriveCurrentControl0")->set(false);
	hal.getLogicOutput("setDriveCurrentControl1")->set(false);
	hal.getLogicOutput("setDriveCurrentControl2")->set(false);
	hal.getLogicOutput("setDriveCurrentControl3")->set(false);
	hal.getLogicOutput("safeUserLight", false)->set(false);
	
	auto &executor = eeros::Executor::instance();
	
	// Create and initialize control and safety system
	ScaraSafetyProperties_force safetyProperties;
	SafetySystem safetySystem(safetyProperties, dt);
	ScaraControlSystem_force controlSystem(safetySystem, safetyProperties);
	safetyProperties.controlSys = &controlSystem;
	
	executor.setMainTask(safetySystem);
	
// 	TimeDomain td("td1", dt, true);
// 	task::Periodic ss("ss", dt, td);
// //	task::Periodic ss("ss", dt, safetySystem);
// 	executor.setMainTask(ss);
//  	ss.monitors.push_back([](PeriodicCounter &c, Logger<LogWriter> &log){
// 		static int count = 0;
// 		if (++count < 1000) return;
// 		count = 0;
// 		std::cout << "ss period: max = " << c.period.max << " min = " << c.period.min << " mean = " << c.period.mean << std::endl;
// 		c.reset();
// 	});
	
	// Get Sequencer
	Sequencer sequencer;
	ScaraSequenceMain_force mainSequence(&sequencer, &controlSystem, &safetySystem, &safetyProperties);
	sequencer.start(&mainSequence);
	
	executor.run();
	
	// Shut down process
// 	h.enable.set(0);      // disable motors (needed if stopped with Ctrl+C) // TODO
	sequencer.abort();
	
//    while(safetySystem.getCurrentLevel().getId() ==  off) usleep(100000);
	log.trace() << "SCARA Robot Control Application finished";
	
	return 0;
}


