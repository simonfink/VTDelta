#include <eeros/hal/HAL.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp> 
#include <eeros/core/Executor.hpp>
#include <eeros/task/Periodic.hpp>
#include <eeros/task/Lambda.hpp>
#include <eeros/control/PeripheralOutput.hpp>
#include <eeros/control/PeripheralInput.hpp>
#include <eeros/control/TimeDomain.hpp> 
#include <eeros/control/Gain.hpp>
#include <eeros/control/DeMux.hpp>
#include <eeros/control/Mux.hpp>
#include <eeros/control/Sum.hpp>
#include <eeros/control/Constant.hpp>

#include "../../control/hallsensors/HallSensorsDataAcquisition.hpp"
#include "../../control/kinematic/DirectKinematic.hpp"
#include "../../control/SetWatchdog.hpp"
#include "../../constants.hpp"
#include "../../types.hpp"
#include <unistd.h>
#include <signal.h>

using namespace parallelscara;
using namespace eeros;
using namespace eeros::hal;
using namespace eeros::control;
using namespace eeros::task;
using namespace eeros::logger;

class TestControlSystem {
public:
	TestControlSystem() :
		hall0("hall0"), hall1("hall1"), hall2("hall2"), hall3("hall3"),
		hallXout("hallXout"), hallYout("hallYout"),
		q0EncPos("enc0"), q1EncPos("enc1"),
		q0EncOffset(0.0), q1EncOffset(0.0),
		q0InvGearRatio(1/i), q1InvGearRatio(1/i),
		directKinematic(l1, l2),
		setWatchdog("Wdt"),
		timedomain("Main time domain", dt, true)
	{
		hallMux.getIn(0).connect(hall0.getOut());
		hallMux.getIn(1).connect(hall1.getOut());
		hallMux.getIn(2).connect(hall2.getOut());
		hallMux.getIn(3).connect(hall3.getOut());
		hallDataRead.getIn().connect(hallMux.getOut());	
		hallDataRead.getOut().getSignal().setName("hall signals");
		hallXout.getIn().connect(hallDataRead.getOutX());
		hallYout.getIn().connect(hallDataRead.getOutY());

		q0EncPos.getOut().getSignal().setName("q0 enc position raw");
		q1EncPos.getOut().getSignal().setName("q1 enc position raw");
		q0EncSumOffset.getIn(0).connect(q0EncPos.getOut());
		q0EncSumOffset.getIn(1).connect(q0EncOffset.getOut());
		q1EncSumOffset.getIn(0).connect(q1EncPos.getOut());
		q1EncSumOffset.getIn(1).connect(q1EncOffset.getOut());
		q0InvGearRatio.getIn().connect(q0EncSumOffset.getOut());
		q1InvGearRatio.getIn().connect(q1EncSumOffset.getOut());
		encPosMux.getOut().getSignal().setName("enc pos mux");
		encPosMux.getIn(0).connect(q0InvGearRatio.getOut());
		encPosMux.getIn(1).connect(q1InvGearRatio.getOut());
	
		directKinematic.getOut().getSignal().setName("xy pos");
		directKinematic.getIn().connect(encPosMux.getOut());  

		timedomain.addBlock(hall0);
		timedomain.addBlock(hall1);
		timedomain.addBlock(hall2);
		timedomain.addBlock(hall3);
		timedomain.addBlock(hallMux);
		timedomain.addBlock(hallDataRead);
		timedomain.addBlock(hallXout);
		timedomain.addBlock(hallYout);

		timedomain.addBlock(q0EncPos);
		timedomain.addBlock(q1EncPos);
		timedomain.addBlock(q0EncOffset);
		timedomain.addBlock(q1EncOffset);
		timedomain.addBlock(q0EncSumOffset);
		timedomain.addBlock(q1EncSumOffset);
		timedomain.addBlock(q0InvGearRatio);
		timedomain.addBlock(q1InvGearRatio);
		timedomain.addBlock(encPosMux);
		
		timedomain.addBlock(directKinematic);
		timedomain.addBlock(setWatchdog);
	};

	PeripheralInput<double> hall0, hall1, hall2, hall3;
	Mux<4,double> hallMux;
	HallSensorsDataAcquisition hallDataRead; 
	PeripheralOutput<double> hallXout, hallYout;
	
	PeripheralInput<double> q0EncPos, q1EncPos;
	Constant<double> q0EncOffset, q1EncOffset;
	Sum<2,double> q0EncSumOffset, q1EncSumOffset;
	Gain<double,double> q0InvGearRatio, q1InvGearRatio;
	Mux<2,double> encPosMux;

	DirectKinematic directKinematic;
	SetWatchdog setWatchdog;
	TimeDomain timedomain;
};

void signalHandler(int signum) {
	eeros::Executor::stop();
}

int main(int argc, char *argv[]) {
	signal(SIGINT, signalHandler);
	
	StreamLogWriter w(std::cout);
	w.show(LogLevel::TRACE);
	Logger::setDefaultWriter(&w);
	Logger log('M');
	log.info() << "Hardware test program";
	
	HAL& hal = HAL::instance();
	hal.readConfigFromFile(&argc, argv);
	
	// disable motors
	eeros::hal::Output<bool>* enable = hal.getLogicOutput("enable");
	enable->set(false);	 
	eeros::hal::Output<bool>* highSpeedLed = hal.getLogicOutput("greenLed_highSpeedButton");
	highSpeedLed->set(false);	 
	eeros::hal::Output<bool>* stopLed = hal.getLogicOutput("redLed_stopButton");
	stopLed->set(false);	 
	eeros::hal::Output<bool>* balancingLed = hal.getLogicOutput("blueLed_balancingButton");
	balancingLed->set(false);	 
	eeros::hal::Output<bool>* approvalLed = hal.getLogicOutput("redLed_approvalButton");
	approvalLed->set(false);	 
	eeros::hal::Output<bool>* testToggle = hal.getLogicOutput("testToggle");
	testToggle->set(false);	 
	eeros::hal::Input<bool>* highSpeedButton = hal.getLogicInput("highSpeedButton");
	eeros::hal::Input<bool>* stopButton = hal.getLogicInput("stopButton");
	eeros::hal::Input<bool>* balancingButton = hal.getLogicInput("balancingButton");
	eeros::hal::Input<bool>* approvalButton = hal.getLogicInput("approvalButton");
	eeros::hal::Input<bool>* emergencyButton = hal.getLogicInput("emergencyButton");

// 	uncomment the following lines to check if a positive voltage for the motors lead to a positive encoder reading	
// 	eeros::hal::ScalableOutput<double>* driveM1 = hal.getScalableOutput("dacOut0");
// 	driveM1->set(0.5);
// 	eeros::hal::ScalableOutput<double>* driveM2 = hal.getScalableOutput("dacOut1");
// 	driveM2->set(0.5);
//   	enable->set(true);	 // enable motors
	
	TestControlSystem cs;
	Periodic per1("per1", 0.05, cs.timedomain);
	
	auto &executor = eeros::Executor::instance();
	executor.setMainTask(per1);

	// create periodic function for logging
	Lambda l1 ([&] () { });
	Periodic per2("per2", 1.0, l1);
	per2.monitors.push_back([&](PeriodicCounter &pc, Logger &log){
// 		log.info() << cs.q0EncPos.getOut().getSignal();	// log encoder 0 raw signal
// 		log.info() << cs.q1EncPos.getOut().getSignal();	// log encoder 1 raw signal
		log.info() << cs.encPosMux.getOut().getSignal();	// log angular position
// 		log.info() << cs.hallDataRead.getOut().getSignal();	// log hall signals
// 		log.info() << cs.hall0.getOut().getSignal();	// log hall signals
// 		log.info() << cs.encPosMux.getOut().getSignal();	// log position of arm in xy
	//	log.info() << cs.directKinematic.getOut().getSignal();	// log position of arm in xy
		// blinks all leds
		highSpeedLed->set(!highSpeedLed->get());
		stopLed->set(!stopLed->get());
		balancingLed->set(!balancingLed->get());
		approvalLed->set(!approvalLed->get());
		// toggles testToggle pin
		testToggle->set(!testToggle->get());
		// log all input buttons
		log.warn() << "high speed button = " << highSpeedButton->get();
		log.warn() << "stop button = " << stopButton->get();
		log.warn() << "balancing button = " << balancingButton->get();
		log.warn() << "reset button = " << approvalButton->get();
		log.warn() << "emergency button = " << emergencyButton->get();
	});

	executor.add(per2);
	executor.run();

	log.trace() << "program end";
	return 0;
}
