#include <unistd.h>
#include <signal.h>
#include <fstream>

#include <eeros/core/Executor.hpp>
#include <eeros/task/Periodic.hpp>
#include <eeros/task/Lambda.hpp>
#include <eeros/hal/HAL.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp> 
#include <eeros/logger/SysLogWriter.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/sequencer/Sequencer.hpp>

#include "safety/ParallelScaraSafetyProperties.hpp"
#include "control/ParallelScaraControlSystem.hpp"
#include "sequences/Homing.hpp"
#include "sequences/Readying.hpp"

#include "constants.hpp"
#include "types.hpp"

using namespace parallelscara;
using namespace eeros;
using namespace eeros::hal;
using namespace eeros::safety;
using namespace eeros::control;
using namespace eeros::sequencer;
using namespace eeros::logger;
using namespace eeros::math;
using namespace eeros::task;
 
void signalHandler(int signum) {
	SafetySystem::exitHandler();
	Sequencer::instance().abort();
} 

int main(int argc, char *argv[]) {
	signal(SIGINT, signalHandler);
	
	StreamLogWriter w(std::cout);
	w.show(LogLevel::TRACE); // show(0) to switch off 
	Logger::setDefaultWriter(&w);
	
	Logger log('M');
	log.info() << "SCARA Robot Control Application started";
	log.set(w);
	
	HAL& hal = HAL::instance();
	hal.readConfigFromFile(&argc, argv);
	
	ParallelScaraSafetyProperties safetyProperties;
	SafetySystem safetySystem(safetyProperties, dt);
	
	ParallelScaraControlSystem controlSystem(safetySystem, safetyProperties);
	safetyProperties.controlSys = &controlSystem;
	
	auto &executor = eeros::Executor::instance();
	executor.setMainTask(safetySystem);
	safetySystem.triggerEvent(safetyProperties.initSw);

	auto& sequencer = Sequencer::instance();
	Homing homing("Homing Sequence", sequencer, controlSystem, safetySystem, safetyProperties);
	sequencer.addSequence(homing);
	Readying readying("Readying Sequence", sequencer, controlSystem, safetySystem, safetyProperties);
	sequencer.addSequence(readying);

	// create periodic function for logging
	Lambda l1 ([&] () { });
	Periodic per2("per2", 1.0, l1);
	per2.monitors.push_back([&](PeriodicCounter &pc, Logger &log){
// 		log.info() << controlSystem.robotController.speedSwitch.getOut().getSignal();
// 		log.info() << controlSystem.robotController.initPos;
		log.info() << controlSystem.encPosMux.getOut().getSignal();	// log angular position
		log.info() << controlSystem.directKinematic.getOut().getSignal();	// log xy position
	});
	executor.add(per2);

	executor.run();
	log.trace() << "executor stopped";
	
	log.info() << "start writing file";
	std::ofstream file;
	file.open("trace.txt", std::ios::trunc);
	timestamp_t* timeStampBuf = controlSystem.trace1.getTimestampTrace();
	AxisVector* buf1 = controlSystem.trace1.getTrace();
	AxisVector* buf2 = controlSystem.trace2.getTrace();
	AxisVector* buf3 = controlSystem.trace3.getTrace();
	AxisVector* buf4 = controlSystem.trace4.getTrace();
	for (int i = 0; i < controlSystem.trace1.getSize(); i++) file << timeStampBuf[i] << " " << buf1[i] << " " << buf2[i] << " " << buf3[i] << " " << buf4[i] << std::endl;
	file.close();
	log.info() << "file written";

	log.trace() << "SCARA Robot Control Application finished";
	return 0;
}
