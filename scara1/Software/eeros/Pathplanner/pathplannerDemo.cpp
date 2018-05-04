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

#include "PathPlanner.hpp"

#define TIMETOWAIT 100

using namespace eeros;
using namespace eeros::control;
using namespace scara;

int main() {	
	double dt = 0.001;
	int dimVec = 4;
	
	Vector4 velMaxPpJS = 0.6;
	Vector4 accMaxPpJS = 0.3;
	
	Vector4 pos1; pos1 << 0.100, 0.100, 0.015, 0.0;
	Vector4 pos2; pos2 << 0.300, 0.400, 0.200, 0.0;
	
	std::cout << "Pathplanner Demo started..." << std::endl;
	std::cout << "Creating executors..." << std::endl;
	Executor e1(dt);
	std::cout << "Creating and connecting control system elements..." << std::endl;

	PathPlanner pathPlanner(velMaxPpJS, accMaxPpJS, dt, 0);
	
	pathPlanner.goPoint(pos1);
// 	pathPlanner.addPosition({0.200, 0.300, 0.020, 0.0});
// 	pathPlanner.addPosition({0.400, 0.400, 0.030, 0.0});
// 	pathPlanner.addPosition({0.200, 0.700, 0.020, 0.0});
// 	pathPlanner.addPosition({0.200, 0.700, 0.010, 0.0});
// 	pathPlanner.addPosition({0.400, 0.500, 0.010, 0.0});
// 	pathPlanner.addPosition({0.600, 0.700, 0.040, 0.0});
// 	pathPlanner.addPosition({0.600, 0.700, 0.010, 0.0});

// 	// scope
// 	GlobalSignalProvider globalSignalProvider;
// 	
// 	std::cout << "Available signals:" << std::endl;
// 	for(std::list<Signal*>::iterator i = Signal::getSignalList()->begin(); i != Signal::getSignalList()->end(); i++) {
// 		uint32_t length = (*i)->getDimension();
// 		for(uint32_t j = 0; j < length; j++) {
// 			std::cout << "  " << (*i)->getLabel(j) << std::endl;
// 		}
// 	}
// 	e1.addRunnable(globalSignalProvider);
	
	e1.addRunnable(pathPlanner);
	
	std::cout << "Starting executors..." << std::endl;
	e1.start();

	std::cout << "Waiting for " << TIMETOWAIT << " seconds while executors are running" << std::endl;
	sleep(TIMETOWAIT);

	std::cout << "Stopping executor..." << std::endl;
	e1.stop();

	std::cout << "Waiting for executors to terminate..." << std::endl;
 	while(!e1.isTerminated());
        
	std::cout << "Example done..." << std::endl;
}
