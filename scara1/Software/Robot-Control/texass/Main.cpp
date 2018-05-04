#include <unistd.h>
#include <iostream>
#include <signal.h>

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
#include "ScaraSafetyProperties.hpp"
#include "ScaraControlSystem.hpp"
#include "ScaraSequenceMain.hpp" 
#include "constants.hpp"

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
	hal.addPeripheralOutput(new ComediDac("setCurrent0", comedi1, 1, 0));
	hal.addPeripheralOutput(new ComediDac("setCurrent1", comedi1, 1, 1));
	hal.addPeripheralOutput(new ComediDac("setCurrent2", comedi1, 1, 2));
	hal.addPeripheralOutput(new ComediDac("setCurrent3", comedi1, 1, 3));
	
	hal.addPeripheralOutput(new ComediDigOut("safeUserLight", comedi0, 2, 0));
	hal.addPeripheralInput(new ComediDigIn("safeUserButton", comedi0, 2, 5, true));

	hal.addPeripheralOutput(new ComediDigOut("setDriveCurrentControl0", comedi1, 2, 28, true));
	hal.addPeripheralOutput(new ComediDigOut("setDriveCurrentControl1", comedi1, 2, 29, true));
	hal.addPeripheralOutput(new ComediDigOut("setDriveCurrentControl2", comedi1, 2, 30, true));
	hal.addPeripheralOutput(new ComediDigOut("setDriveCurrentControl3", comedi1, 2, 31, true));
	
	hal.getLogicPeripheralOutput("setDriveCurrentControl0")->set(false);
	hal.getLogicPeripheralOutput("setDriveCurrentControl1")->set(false);
	hal.getLogicPeripheralOutput("setDriveCurrentControl2")->set(false);
	hal.getLogicPeripheralOutput("setDriveCurrentControl3")->set(false);
	hal.getLogicPeripheralOutput("safeUserLight")->set(false);
	
	hal.addPeripheralOutput(new ComediDigOut("vakuumAusblasen", comedi0, 2, 2));
	hal.addPeripheralOutput(new ComediDigOut("vakuum", comedi0, 2, 3));
	hal.getLogicPeripheralOutput("vakuumAusblasen")->set(false);
	hal.getLogicPeripheralOutput("vakuum")->set(false);					// TRUE = vakuum
	
	hal.addPeripheralOutput(new ComediDigOut("ventil_on_negate", comedi1, 2, 20));
	hal.addPeripheralOutput(new ComediDigOut("ventil_on", comedi1, 2, 21));
	hal.getLogicPeripheralOutput("ventil_on_negate")->set(false);
	hal.getLogicPeripheralOutput("ventil_on")->set(false);	
	
	// test outputs
 	hal.addPeripheralOutput(new ComediDigOut("ringlight", comedi1, 2, 24));
 	hal.getLogicPeripheralOutput("ringlight")->set(true);
	hal.addPeripheralOutput(new ComediDigOut("remotePlasmaPen", comedi1, 2, 25));
	hal.getLogicPeripheralOutput("remotePlasmaPen")->set(true);
	hal.addPeripheralOutput(new ComediDigOut("plasmaON", comedi1, 2, 27));
	hal.getLogicPeripheralOutput("plasmaON")->set(false);
	hal.addPeripheralOutput(new ComediDigOut("ventil", comedi1, 2, 26));
	hal.getLogicPeripheralOutput("ventil")->set(false);
}

volatile bool running = true;
void signalHandler(int signum) {
	running = false;
}

int main() {
	signal(SIGINT, signalHandler);
	
	StreamLogWriter w(std::cout);
	w.show();
	Logger<LogWriter>::setDefaultWriter(&w);
	
// 	SysLogWriter s("scara");
// 	s.show();
// 	Logger<LogWriter>::setDefaultWriter(&s);
	
	Logger<LogWriter> log('M');
	log.trace() << "SCARA Robot Control Application started";
	
	// Get HAL instance & initialize HW
	HAL& hal = HAL::instance();
	initHardware();

	// Get Safety System instance
	ScaraSafetyProperties properties;
	SafetySystem safetySystem(properties, dt);
	
	// Get Control System instance
	ScaraControlSystem controlSystem;
	controlSystem.start();
	
	// Get Sequencer
	Sequencer sequencer;
	ScaraSequenceMain mainSequence(&sequencer, &controlSystem, &safetySystem);
	sequencer.start(&mainSequence);
	
	while(running && sequencer.getState() != state::terminated) {
		usleep(1000000);
	}
	
	controlSystem.stop();
	safetySystem.shutdown();

	log.trace() << "SCARA Robot Control Application finished";
	
	return 0;
}
