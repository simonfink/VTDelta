#include "ParallelScaraSafetyProperties.hpp"
#include "../control/ParallelScaraControlSystem.hpp"
#include <eeros/hal/HAL.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/sequencer/Sequencer.hpp>

#include <unistd.h>
#include <iostream>
#include <vector>
#include <initializer_list>
#include "../constants.hpp"
#include <cmath>

using namespace parallelscara;
using namespace eeros;
using namespace eeros::hal;
using namespace eeros::safety;
using namespace eeros::sequencer;

ParallelScaraSafetyProperties::ParallelScaraSafetyProperties() : 
	log('H'),
	// levels
	off("System off"),
	shuttingDown("System shutting down"),
	swInitializing("SW initializing"),
	emergency("EMERGENCY"),
	resetEmergency("Reset EMERGENCY"),          
	systemOn("System on (motors disabled)"),
	poweringDown("Powering down"),
	waitForMotionStop("Wait for motion stop"),	
	powerOn("Power on"),
	homing("Homing"),
	homed("Homed"),
	readying("Readying"),
	systemReady("System ready"),
	balancing("Balancing mode"),
	highSpeed("High speed mode"),
	stopMoving("Stop moving"),
	
	// events
	switchOff("switch off"),
	shutDown("shut down"),
	initSw("init software"),
	initSwDone("init software done"),
	doEmergency("emergency button pressed or position out of limit or velocity too big"),
	doEmergencyReset("reset emergency"),
	emergencyResetDone("emergency reset done"),
	doPowerUp("start powering up"),
	startPoweringDown("start powering down"),
	startHoming("start homing"),
	homingDone("homing done"),
	doReady("do ready"),
	readyDone("ready done"), 
	startBalancing("start balancing"),
	startHighSpeed("start high speed"),
	stop("stop moving"),
	abort("abort")
{
	HAL& hal = HAL::instance();

	// ############ Define critical outputs ############
	enable = hal.getLogicOutput("enable");
	testToggle = hal.getLogicOutput("testToggle");
	criticalOutputs = { enable, testToggle };
	
	// get reference to other outputs which are controlled by the safety system
	blueLedBalancingButton  = hal.getLogicOutput("blueLed_balancingButton");
	redLedStopButton        = hal.getLogicOutput("redLed_stopButton");
	greenLedHighSpeedButton = hal.getLogicOutput("greenLed_highSpeedButton");
	redLedResetButton    = hal.getLogicOutput("redLed_approvalButton");
	
	// ############ Define critical inputs ############
	emergencyButton = hal.getLogicInput("emergencyButton");	// positive active
	criticalInputs = { emergencyButton };
	
	// get reference to other inputs which are read by the safety system
	highSpeedButton = hal.getLogicInput("highSpeedButton");	// negativ active
	balancingButton = hal.getLogicInput("balancingButton");	// negativ active
	stopButton      = hal.getLogicInput("stopButton");	// negativ active
	resetButton  = hal.getLogicInput("approvalButton");
	
	// ############ Add levels ############
	addLevel(off);
	addLevel(shuttingDown);
	addLevel(swInitializing);
	addLevel(emergency);
	addLevel(resetEmergency);
	addLevel(systemOn);
	addLevel(poweringDown);
	addLevel(waitForMotionStop);
	addLevel(powerOn);
	addLevel(homing);
	addLevel(homed);
	addLevel(readying);
	addLevel(systemReady);
	addLevel(balancing);
	addLevel(highSpeed);
	addLevel(stopMoving);
	
	// ############ Add events to the levels ############
	off.addEvent				(initSw,				swInitializing,		kPublicEvent);
	shuttingDown.addEvent		(switchOff,      		off,				kPrivateEvent);
	swInitializing.addEvent		(initSwDone,			systemOn,			kPrivateEvent);
	emergency.addEvent			(doEmergencyReset,		resetEmergency,		kPublicEvent);
	resetEmergency.addEvent		(emergencyResetDone,	systemOn,			kPrivateEvent); 
	systemOn.addEvent			(doPowerUp,				powerOn,			kPublicEvent);
	systemOn.addEvent			(abort,					poweringDown,		kPublicEvent);
	poweringDown.addEvent		(shutDown,    			shuttingDown,		kPrivateEvent);
	waitForMotionStop.addEvent	(startPoweringDown,		poweringDown,		kPrivateEvent); 
	powerOn.addEvent			(startPoweringDown,		poweringDown,		kPublicEvent);
	powerOn.addEvent			(startHoming,			homing,				kPublicEvent);
	powerOn.addEvent			(doReady,				readying,			kPublicEvent);
	homing.addEvent				(homingDone,			homed,				kPublicEvent);
	homed.addEvent				(doReady,				readying,			kPublicEvent);
	readying.addEvent			(readyDone,				systemReady,		kPublicEvent);
	systemReady.addEvent		(startBalancing,		balancing,			kPrivateEvent);
	systemReady.addEvent		(startHighSpeed, 		highSpeed,			kPrivateEvent);
	balancing.addEvent			(stop,					stopMoving,			kPrivateEvent); 
	highSpeed.addEvent			(stop,					stopMoving,			kPrivateEvent);
	stopMoving.addEvent			(doReady,				readying,			kPrivateEvent); 
	
	// Add events to multiple levels
	addEventToLevelAndAbove(systemOn, doEmergency, emergency, kPublicEvent);
	addEventToLevelAndAbove(powerOn, abort, waitForMotionStop, kPrivateEvent);
	addEventToAllLevelsBetween(emergency, resetEmergency, abort, shuttingDown, kPrivateEvent);
	
	// ############ Define input states and events for all levels ############
	off.setInputActions					({ ignore(emergencyButton) });
	shuttingDown.setInputActions		({ ignore(emergencyButton) });
	swInitializing.setInputActions		({ ignore(emergencyButton) });
	emergency.setInputActions			({ ignore(emergencyButton) });
	resetEmergency.setInputActions		({ ignore(emergencyButton) });
	systemOn.setInputActions			({ check (emergencyButton, false, doEmergency) });
	poweringDown.setInputActions		({ check (emergencyButton, false, doEmergency) });
	waitForMotionStop.setInputActions	({ check (emergencyButton, false, doEmergency) });
	powerOn.setInputActions				({ check (emergencyButton, false, doEmergency) });
	homing.setInputActions				({ check (emergencyButton, false, doEmergency) });
	homed.setInputActions				({ check (emergencyButton, false, doEmergency) });
	readying.setInputActions			({ check (emergencyButton, false, doEmergency) });
	systemReady.setInputActions			({ check (emergencyButton, false, doEmergency) });
	balancing.setInputActions			({ check (emergencyButton, false, doEmergency) });
	highSpeed.setInputActions			({ check (emergencyButton, false, doEmergency) });
	stopMoving.setInputActions			({ check (emergencyButton, false, doEmergency) });
		
	// Define output states and events for all levels 
	off.setOutputActions				({ set(enable, false), toggle(testToggle) });  
	shuttingDown.setOutputActions		({ set(enable, false), toggle(testToggle) });   
	swInitializing.setOutputActions		({ set(enable, false), toggle(testToggle) });  
	emergency.setOutputActions			({ set(enable, false), toggle(testToggle) });   
	resetEmergency.setOutputActions		({ set(enable, false), toggle(testToggle) });   
	systemOn.setOutputActions			({ set(enable, false), toggle(testToggle) });   
	poweringDown.setOutputActions		({ set(enable, false), toggle(testToggle) });   
	waitForMotionStop.setOutputActions	({ set(enable, true ), toggle(testToggle) });   
	powerOn.setOutputActions			({ set(enable, true ), toggle(testToggle) });  
	homing.setOutputActions				({ set(enable, true ), toggle(testToggle) });  
	homed.setOutputActions				({ set(enable, false), toggle(testToggle) });   
	readying.setOutputActions			({ set(enable, false ), toggle(testToggle) });   
	systemReady.setOutputActions		({ set(enable, false ), toggle(testToggle) });  
	balancing.setOutputActions			({ set(enable, false ), toggle(testToggle) });  
	highSpeed.setOutputActions			({ set(enable, false ), toggle(testToggle) });  
	stopMoving.setOutputActions			({ set(enable, false), toggle(testToggle) });  
	
	robotHomed = false;

	// *** Define and add level functions *** //
	off.setLevelAction([&](SafetyContext* privateContext) {
		Executor::stop();
	});	
	
	shuttingDown.setLevelAction([&](SafetyContext* privateContext) {
		privateContext->triggerEvent(switchOff);
	});	
	
	swInitializing.setLevelAction([this](SafetyContext* privateContext) {
		// Switch off all LEDs
		blueLedBalancingButton->set(false);
		greenLedHighSpeedButton->set(false);
		redLedStopButton->set(false);
		redLedResetButton->set(false);
		controlSys->stop();
		privateContext->triggerEvent(initSwDone); 
	});
	
	emergency.setLevelAction([this](SafetyContext* privateContext) { 
		blueLedBalancingButton->set(false);
		redLedStopButton->set(false);
		greenLedHighSpeedButton->set(false);
		
		if ((emergency.getNofActivations() % 500) == 0) redLedResetButton->set(true); 
		else redLedResetButton->set(false);
			
		if (!resetButton->get() && !emergencyButton->get()) {
			redLedResetButton->set(false);
			privateContext->triggerEvent(doEmergencyReset);
		}
	});
	
	resetEmergency.setLevelAction([this](SafetyContext* privateContext) { 
		if (!robotHomed) {
			controlSys->robotController.setInitializationSpeed({0.0, 0.0});
			controlSys->robotController.setSpeedSwitch(0); // choose speed control
		} else { 
			controlSys->robotController.setSpeedSwitch(1);                  // choose position control
			controlSys->pathPlanner.setInitPos(controlSys->directKinematic.getOut().getSignal().getValue()[0]);	// init path planner with actual position
			controlSys->xyRefPosSwitch.switchToInput(0);                                    // select path planner input for 'xyRefPosSwitch'
			controlSys->pendulumController.I_accToVel.disable();            // disable integrator
			controlSys->pendulumController.I_velToPos.disable();            // disable integrator
			controlSys->inverseKinematic.resetOutOfRange();			// reset out of range parameter before enabling inv kinematic
			controlSys->inverseKinematic.enable();				// enable inverse kinematic
		}
		
		AxisVector pp  = controlSys->robotController.getInEncRefPos().getSignal().getValue(); // change to output of previos block
		AxisVector enc = controlSys->robotController.getInEncPosAct().getSignal().getValue();
		AxisVector speed = controlSys->robotController.getActSpeed();
		
		double dac0 = controlSys->dac0.getIn().getSignal().getValue();
		double dac1 = controlSys->dac1.getIn().getSignal().getValue();
		
		if (fabs(pp(0)-enc(0)) > err || fabs(pp(1)-enc(1)) > err || fabs(speed(0)) > err || fabs(speed(1)) > err || fabs(dac0) > 0.009 || fabs(dac1) > 0.009){
			// Set path planner init pos
			controlSys->pathPlanner.setInitPos(controlSys->directKinematic.getOut().getSignal().getValue()[0]);
		} else {
			auto init_pp = controlSys->pathPlanner.getPosOut().getSignal().getValue();
			auto init_ik = controlSys->inverseKinematic.getOut().getSignal().getValue();
			auto sw_in = controlSys->xyRefPosSwitch.getIn(0).getSignal().getValue();
			auto sw_out = controlSys->xyRefPosSwitch.getCurrentInput();
// 			std::cout << "reset emergency -> init: " << init_pp(0) << ", " << init_pp(1) << " -> " << init_ik(0) << ", " << init_ik(1) << std::endl;
// 			std::cout << "reset emergency -> ref: "  << pp(0) << ", " << pp(1) << "; enc: " << enc(0) << "; " << enc(1) << std::endl;
			privateContext->triggerEvent(emergencyResetDone); 
		}
	});
	
	systemOn.setLevelAction([this](SafetyContext* privateContext) {
		if (systemOn.getNofActivations() == 1) {
			controlSys->robotController.setSpeedSwitch(0);	// switch to speed control
			controlSys->robotController.setInitializationSpeed({0, 0});
			controlSys->start();
		}
		if(fabs(controlSys->dac0.getIn().getSignal().getValue()) < 0.01 && fabs(controlSys->dac1.getIn().getSignal().getValue()) < 0.01 ) 
			privateContext->triggerEvent(doPowerUp);
	});

	poweringDown.setLevelAction([this](SafetyContext* privateContext) {
		blueLedBalancingButton->set(false);
		greenLedHighSpeedButton->set(false);
		redLedStopButton->set(false);
		redLedResetButton->set(false);
		controlSys->stop();
		privateContext->triggerEvent(shutDown);  
	});
	
	waitForMotionStop.setLevelAction([this](SafetyContext* privateContext) { 
		AxisVector speed = controlSys->robotController.getActSpeed();
		if (waitForMotionStop.getNofActivations() >= 500 && fabs(speed(0)) < err && fabs(speed(1)) < err)
			privateContext->triggerEvent(startPoweringDown); 
	});
	
	powerOn.setLevelAction([this](SafetyContext* privateContext) {
		if (!robotHomed) privateContext->triggerEvent(startHoming); 
		else privateContext->triggerEvent(doReady);
	});
	
	homing.setLevelAction([this](SafetyContext* privateContext) {
		if (homing.getNofActivations() == 1)
				Sequencer::instance().getSequenceByName("Homing Sequence")->start();
		controlSys->trace1.enable();
		controlSys->trace2.enable();
		controlSys->trace3.enable();
		controlSys->trace4.enable();
	});
	
	homed.setLevelAction([this](SafetyContext* privateContext) {
		if (homed.getNofActivations() == 1) {
			robotHomed = true;
			// Set encoder offset
			double actual0 = controlSys->q0EncPos.getOut().getSignal().getValue();
			double actual1 = controlSys->q1EncPos.getOut().getSignal().getValue();
			controlSys->q0EncOffset.setValue(-actual0 + controlSys->robotController.initPos(0) * i);
			controlSys->q1EncOffset.setValue(-actual1 + controlSys->robotController.initPos(1) * i);
			controlSys->timedomain.removeBlock(controlSys->checkVel);	// stop velocity check, as encoder position will jump due to offset setting
		}
		
		if (homed.getNofActivations() == 3) controlSys->timedomain.addBlock(controlSys->checkVel);	// restart velocity checking
						 
		controlSys->xyRefPosSwitch.switchToInput(0);	// path planner input
		controlSys->inverseKinematic.enable();			// enable inverse kinematic
		controlSys->robotController.setSpeedSwitch(1);	// position control
		controlSys->robotController.speedSaturation.enable();
		controlSys->pathPlanner.setInitPos(controlSys->directKinematic.getOut().getSignal().getValue()); 
			
		if (homed.getNofActivations() == 4) {
			privateContext->triggerEvent(doReady);
		}
	});
	
	readying.setLevelAction([this](SafetyContext* privateContext) {
		if (readying.getNofActivations() == 1) Sequencer::instance().getSequenceByName("Readying Sequence")->start();
		controlSys->trace1.disable();
		controlSys->trace2.disable();
		controlSys->trace3.disable();
		controlSys->trace4.disable();
	});

	systemReady.setLevelAction([this](SafetyContext* privateContext) {
		blueLedBalancingButton->set(true);
		greenLedHighSpeedButton->set(true);
		// check buttons (negative active)
		if (!balancingButton->get()) {
			blueLedBalancingButton->set(false);
			greenLedHighSpeedButton->set(false);
			redLedStopButton->set(true);
			privateContext->triggerEvent(startBalancing);
		}
		if (!highSpeedButton->get()) {
			blueLedBalancingButton->set(false);
			greenLedHighSpeedButton->set(false);
			redLedStopButton->set(true);
			privateContext->triggerEvent(startHighSpeed);
		}
	});
	
	balancing.setLevelAction([this](SafetyContext* privateContext) { 
		if (!stopButton->get()) privateContext->triggerEvent(stop); 

// 			AxisVector pp  = controlSys->robotController.getInEncRefPos().getSignal().getValue(); // change to output of previos block
// 			AxisVector enc = controlSys->robotController.getInEncPosAct().getSignal().getValue();
// 			controlSys->pathPlanner.setInitPos(controlSys->directKinematic.getOut().getSignal().getValue()[0]);
// 			
// 			if(fabs(pp(0)-enc(0)) > err || fabs(pp(1)-enc(1)) > err){
// 				controlSys->pathPlanner.setInitPos(controlSys->directKinematic.getOut().getSignal().getValue()[0]); 
// 			}
// 			else{
// 				controlSys->xyRefPosSwitch.switchToInput(1);                              // tip control input
// 				controlSys->robotController.setSpeedSwitch(1);            // position control
// 				controlSys->pendulumController.I_accToVel.setInitCondition(0.0);
// 				controlSys->pendulumController.I_velToPos.setInitCondition(controlSys->directKinematic.getOut().getSignal().getValue());
// 				
// 				controlSys->pendulumController.I_accToVel.enable();       // enable integrator
// 				controlSys->pendulumController.I_velToPos.enable();       // enable integrator
// 				privateContext->triggerEvent(startBalancing);
// 			}

// 		// Check if bar is on
// 		if(!controlSys->hallDataRead.isBarOn()){
// 			std::cout  << "Bar was removed" << std::endl;
// 			privateContext->triggerEvent(stopMoving); 
// 		}
// 		
// 		AxisVector ref = controlSys->robotController.getInEncRefPos().getSignal().getValue(); // change to output of previos block
// 		AxisVector enc = controlSys->robotController.getInEncPosAct().getSignal().getValue();
// 
// 		// Check inverse Kinematic and brake
// 		if(controlSys->inverseKinematic.isOutOfRange()) {
// 			controlSys->robotController.setInitializationSpeed(0.0);  // set speed = 0
// 			controlSys->robotController.setSpeedSwitch(0);            // speed control
// 			std::cout  << "  is Out of Range: "<< controlSys->inverseKinematic.isOutOfRange()<< "  ref: " << ref << "; enc: " << enc <<std::endl;
// 			privateContext->triggerEvent(doEmergency);
// 		}
	});
	
	
	highSpeed.setLevelAction([this](SafetyContext* privateContext) { 
		if (!stopButton->get()) privateContext->triggerEvent(stop); 

		if (highSpeed.getNofActivations() == 1) {
			controlSys->xyRefPosSwitch.switchToInput(0);	// path planner input
			controlSys->robotController.setSpeedSwitch(1);	// position control
		}
	});
	
	stopMoving.setLevelAction([this](SafetyContext* privateContext) { 
		redLedStopButton->set(false);

// 		if(stopMoveCounter++ > 0) {
// 			controlSys->pathPlanner.setInitPos(controlSys->directKinematic.getOut().getSignal().getValue()[0]);
// 			controlSys->xyRefPosSwitch.switchToInput(0);                              // path planner input (for comparison with balancing mode)
// 			controlSys->robotController.speedSwitch.switchToInput(0); // set pos control
// 			AxisVector pp  = controlSys->robotController.getInEncRefPos().getSignal().getValue(); // change to output of previos block
// 			AxisVector enc = controlSys->robotController.getInEncPosAct().getSignal().getValue();
// 			
// 			if(fabs(pp(0)-enc(0)) > err || fabs(pp(1)-enc(1)) > err){
// 				// Set path planner init pos
// 				controlSys->pathPlanner.setInitPos(controlSys->directKinematic.getOut().getSignal().getValue()[0]); 
// 			}
// 			else {
// 				controlSys->xyRefPosSwitch.switchToInput(0);                          // path planner input
// 				controlSys->robotController.setSpeedSwitch(1);        // position control
// 				controlSys->pendulumController.I_accToVel.disable();  // disable integrator
// 				controlSys->pendulumController.I_velToPos.disable();  // disable integrator
// 				privateContext->triggerEvent(doReady);
// 			}
// 		}
	});
	
	// Define entry level
	setEntryLevel(off);
	
	// Define action when exiting application with Ctrl-C 
	exitFunction = [&](SafetyContext* privateContext) {
//		if(privateContext->
		privateContext->triggerEvent(abort);
		
	};

}

ParallelScaraSafetyProperties::~ParallelScaraSafetyProperties() {
	// nothing to do
}