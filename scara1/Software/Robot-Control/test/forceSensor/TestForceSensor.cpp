#include <unistd.h>
#include <iostream>
#include <signal.h>

#include <eeros/core/Executor.hpp>
#include <eeros/hal/HAL.hpp>
#include <eeros/hal/ComediDevice.hpp>
#include <eeros/hal/ComediDigIn.hpp>
#include <eeros/hal/ComediAdc.hpp>
#include <eeros/hal/ComediDigOut.hpp>
#include <eeros/hal/ComediFqd.hpp>
#include <eeros/hal/ComediDac.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp> 
#include <eeros/logger/SysLogWriter.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/sequencer/Sequencer.hpp>
#include "FSSafetyProperties.hpp"
#include "FSControlSystem.hpp"
#include "../../constants.hpp"

using namespace scara;
using namespace eeros;
using namespace eeros::hal;
using namespace eeros::safety;
using namespace eeros::control;
using namespace eeros::sequencer;
using namespace eeros::logger;

void initHardware() {
	HAL& hal = HAL::instance();
	
	std::cout << "  Creating device structure..." << std::endl;
	ComediDevice* comedi0 = new ComediDevice("/dev/comedi0");
	ComediDevice* comedi1 = new ComediDevice("/dev/comedi1");
	
	std::cout << "  Registering I/Os in the HAL..." << std::endl;
	
	hal.addPeripheralInput(new ComediFqd("q0" , comedi1, 11, 8, 10, 9, 6.28318530718 / (4 * 4096.0), 0, 0)); // scale/offset/initVal
 	hal.addPeripheralInput(new ComediFqd("q1" , comedi1, 12, 3, 11, 4, 6.28318530718 / (4 * 4096.0), 0, 0));  
	hal.addPeripheralInput(new ComediFqd("q2r", comedi0, 11, 8, 10, 9, 6.28318530718 / (4 * 4096.0), 0, 0));
	hal.addPeripheralInput(new ComediFqd("q3" , comedi0, 12, 3, 11, 4, 6.28318530718 / (4 * 4096.0), 0, 0)); 
	
	hal.addPeripheralInput(new ComediDigIn("approval", comedi0, 2, 4, true));
	hal.addPeripheralInput(new ComediDigIn("limitSwitchQ0p", comedi1, 2, 12, true));
	hal.addPeripheralInput(new ComediDigIn("limitSwitchQ0n", comedi1, 2, 13, true));
	hal.addPeripheralInput(new ComediDigIn("limitSwitchQ1p", comedi1, 2, 14, true));
	hal.addPeripheralInput(new ComediDigIn("limitSwitchQ1n", comedi1, 2, 15, true));
	hal.addPeripheralInput(new ComediDigIn("limitSwitchQ2p", comedi1, 2, 16, true));
	hal.addPeripheralInput(new ComediDigIn("limitSwitchQ2n", comedi1, 2, 17, true));
	hal.addPeripheralInput(new ComediDigIn("limitSwitchQ3p", comedi1, 2, 18, true));
	hal.addPeripheralInput(new ComediDigIn("limitSwitchQ3n", comedi1, 2, 19, true));
	hal.addPeripheralOutput(new ComediDigOut("watchdog", comedi1, 2, 22));
	hal.addPeripheralOutput(new ComediDigOut("enable0", comedi1, 2, 0));
	hal.addPeripheralOutput(new ComediDigOut("enable1", comedi1, 2, 1));
	hal.addPeripheralOutput(new ComediDigOut("enable2", comedi1, 2, 2));
	hal.addPeripheralOutput(new ComediDigOut("enable3", comedi1, 2, 3));	
	hal.addPeripheralOutput(new ComediDigOut("brake0", comedi1, 2, 4, true));
	hal.addPeripheralOutput(new ComediDigOut("brake1", comedi1, 2, 5, true));
	hal.addPeripheralOutput(new ComediDigOut("brake2", comedi1, 2, 6, true));
	hal.addPeripheralOutput(new ComediDigOut("brake3", comedi1, 2, 7, true));
	
	hal.addPeripheralInput(new ComediAdc("force_fx", comedi1, 0, 0, 1, 0)); // id, device, subdeviceNr, channel, scale, offset MODIFICARE QUI PER CALIBRAZIONE
	hal.addPeripheralInput(new ComediAdc("force_fy", comedi1, 0, 1, 1, 0)); 
	hal.addPeripheralInput(new ComediAdc("force_fz", comedi1, 0, 2, 1, 0)); 
	hal.addPeripheralInput(new ComediAdc("force_mx", comedi1, 0, 3, 1, 0)); 
	hal.addPeripheralInput(new ComediAdc("force_my", comedi1, 0, 4, 1, 0)); 
	hal.addPeripheralInput(new ComediAdc("force_mz", comedi1, 0, 5, 1, 0)); 
	
}

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
	log.trace() << "SCARA Robot Control Application started"
	log.set(w);
	
	// Get HAL instance & initialize HW
	HAL& hal = HAL::instance();
	initHardware();
	
	auto &executor = eeros::Executor::instance();
	
	// Get Control System instance
	FSControlSystem controlSystem;
	
	// Get Safety System instance
	FSSafetyProperties safetyProperties(&controlSystem);
	SafetySystem safetySystem(safetyProperties, dt);
	executor.setMainTask(safetySystem);
	
	executor.run();

	return 0;
}


