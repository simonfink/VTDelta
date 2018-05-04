#include <iostream>
#include <ostream>
#include <fstream>
#include <stdlib.h>
#include <unistd.h>
#include <eeros/core/Executor.hpp>
#include <eeros/core/SharedMemory.hpp>
#include <eeros/control/Step.hpp>
#include <eeros/control/GlobalSignalProvider.hpp>
#include <eeros/control/SignalBufferReader.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/logger/LogWriter.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include "../kinematicDynamic/ScaraDirectKinematic.hpp"
#include "../kinematicDynamic/ScaraInverseKinematic.hpp"

#include "../pathPlanner/PathPlanner.hpp"

#define TIMETOWAIT 100

using namespace eeros;
using namespace scara;
using namespace eeros::control;
using namespace eeros::logger;

int main() {	
	
	StreamLogWriter w(std::cout);
	Logger<LogWriter>::setDefaultWriter(&w);
	Logger<LogWriter> log;
	
	double dt = 0.001;
	Vector4 velMaxPP; velMaxPP << 1.0, 1.0, 1.0, 1.0;
	Vector4 accMaxPP; accMaxPP << 0.5, 0.5, 0.5, 0.5;
	double l1 = 0.25;
	double l2 = 0.25;
	
	Vector4 posInit; posInit << 2.46, 2.46, 0.10, 1.00;
	Vector4 posReady1; posReady1 << 0.78,  2.46, 0.105, 1.57;
	Vector4 posReady2; posReady2 << 0.78, -1.57, 0.105, 1.57;
	Vector4 point1;    point1    << 0.78,  3.14, 0.105, 1.57;
	Vector4 point2;    point2    << 0.78, -1.57, 0.105, 1.57;
	
	std::cout << "Pathplanner Demo started..." << std::endl;
	std::cout << "Creating executors..." << std::endl;
	Executor e1(dt);
	std::cout << "Creating and connecting control system elements..." << std::endl;

	PathPlanner pathPlanner(velMaxPP, accMaxPP, dt, 0);
	
	// scope
	GlobalSignalProvider globalSignalProvider;
	e1.addRunnable(globalSignalProvider);
	e1.addRunnable(pathPlanner);
	
	std::cout << "Starting executors..." << std::endl;
	e1.start();

	pathPlanner.setInitialPosition(posInit);
	
	pathPlanner.goPoint(posReady1);
	pathPlanner.goPoint(posReady2);
	pathPlanner.goPoint(point1);
	pathPlanner.goPoint(point2);
	log.info() << "<<< last position reached <<<";
	
// 	std::cout << "Waiting for " << TIMETOWAIT << " seconds while executors are running" << std::endl;
// 	sleep(TIMETOWAIT);

// 	std::cout << "Stopping executor..." << std::endl;
// 	e1.stop();

// 	std::cout << "Waiting for executors to terminate..." << std::endl;
//  	while(!e1.isTerminated());
        
	std::cout << "Example done..." << std::endl;
}
