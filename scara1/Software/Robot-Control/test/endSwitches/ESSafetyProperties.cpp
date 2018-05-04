#include "ESSafetyProperties.hpp"
#include "ESControlSystem.hpp"
#include <eeros/hal/HAL.hpp>
#include <eeros/safety/InputAction.hpp>
#include <eeros/safety/OutputAction.hpp>

#include <unistd.h>
#include <iostream>
#include <vector>
#include <initializer_list>

using namespace scara;
using namespace eeros;
using namespace eeros::hal;
using namespace eeros::safety;

ESSafetyProperties::ESSafetyProperties(ESControlSystem* cs) : 
off("SW off"), 
waitForApproval("Wait for approval button"), 
emergency("Robot in emergency state"),
approvalOn("Approval button on "),
on("SW on "),
startSW("start switch"),
doEmergency("emergency button pressed"),
doApproval("green button pressed"),
doOn("control system started"),
doOff("control system stopped"),

controlSys(cs) {

	HAL& hal = HAL::instance();

	// ############ Define critical outputs ############
	watchdog = hal.getLogicPeripheralOutput("watchdog");
	enable0 = hal.getLogicPeripheralOutput("enable0");
	enable1 = hal.getLogicPeripheralOutput("enable1");
	enable2 = hal.getLogicPeripheralOutput("enable2");
	enable3 = hal.getLogicPeripheralOutput("enable3");
	brake0 = hal.getLogicPeripheralOutput("brake0");
	brake1 = hal.getLogicPeripheralOutput("brake1");
	brake2 = hal.getLogicPeripheralOutput("brake2");
	brake3 = hal.getLogicPeripheralOutput("brake3");
	
	criticalOutputs = { watchdog, enable0, enable1, enable2, enable3, brake0, brake1, brake2, brake3 };
	
	// ############ Define critical inputs ############
	approval = hal.getLogicPeripheralInput("approval");
	limitSwitchQ0p = hal.getLogicPeripheralInput("limitSwitchQ0p");
	limitSwitchQ0n = hal.getLogicPeripheralInput("limitSwitchQ0n");
	limitSwitchQ1p = hal.getLogicPeripheralInput("limitSwitchQ1p");
	limitSwitchQ1n = hal.getLogicPeripheralInput("limitSwitchQ1n");
	limitSwitchQ2p = hal.getLogicPeripheralInput("limitSwitchQ2p");
	limitSwitchQ2n = hal.getLogicPeripheralInput("limitSwitchQ2n");
	limitSwitchQ3p = hal.getLogicPeripheralInput("limitSwitchQ3p");
	limitSwitchQ3n = hal.getLogicPeripheralInput("limitSwitchQ3n");
	
	criticalInputs = { approval, 
					   limitSwitchQ0p, limitSwitchQ0n, limitSwitchQ1p, limitSwitchQ1n, 
					   limitSwitchQ2p, limitSwitchQ2n, limitSwitchQ3p, limitSwitchQ3n  };
	
	// ############ Other inputs/outputs ############
	
	
	// ############ Define Levels ############
	addLevel(off);
	addLevel(waitForApproval);
	addLevel(emergency);
	addLevel(approvalOn);
	addLevel(on);
		
	// ############ Add events to the levels ############
	off             .addEvent(startSW,    waitForApproval,  kPublicEvent  ); 
	waitForApproval .addEvent(doApproval, approvalOn,       kPublicEvent  ); 
	emergency       .addEvent(doOff,      off,              kPublicEvent  ); 
	approvalOn      .addEvent(doOn,       on,               kPublicEvent  ); 
	on              .addEvent(doOff,      off,              kPublicEvent  ); 
	
	// ############ Define input states and events for all levels ############
	off             .setInputActions({ignore(approval),                    ignore(limitSwitchQ0p), ignore(limitSwitchQ0n), ignore(limitSwitchQ1p), ignore(limitSwitchQ1n), ignore(limitSwitchQ2p), ignore(limitSwitchQ2n), ignore(limitSwitchQ3p), ignore(limitSwitchQ3n)});
	waitForApproval .setInputActions({check(approval, true , doApproval),  ignore(limitSwitchQ0p), ignore(limitSwitchQ0n), ignore(limitSwitchQ1p), ignore(limitSwitchQ1n), ignore(limitSwitchQ2p), ignore(limitSwitchQ2n), ignore(limitSwitchQ3p), ignore(limitSwitchQ3n)});
	emergency       .setInputActions({ignore(approval),                    ignore(limitSwitchQ0p), ignore(limitSwitchQ0n), ignore(limitSwitchQ1p), ignore(limitSwitchQ1n), ignore(limitSwitchQ2p), ignore(limitSwitchQ2n), ignore(limitSwitchQ3p), ignore(limitSwitchQ3n)});
	approvalOn      .setInputActions({check(approval, false, doEmergency), ignore(limitSwitchQ0p), ignore(limitSwitchQ0n), ignore(limitSwitchQ1p), ignore(limitSwitchQ1n), ignore(limitSwitchQ2p), ignore(limitSwitchQ2n), ignore(limitSwitchQ3p), ignore(limitSwitchQ3n)});
	on              .setInputActions({check(approval, false, doEmergency), ignore(limitSwitchQ0p), ignore(limitSwitchQ0n), ignore(limitSwitchQ1p), ignore(limitSwitchQ1n), ignore(limitSwitchQ2p), ignore(limitSwitchQ2n), ignore(limitSwitchQ3p), ignore(limitSwitchQ3n)});

	// Define output states and events for all levels 
	off             .setOutputActions({set(watchdog, false), set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false), set(brake0, true ), set(brake1, true ), set(brake2, true ), set(brake3, true ) });
	waitForApproval .setOutputActions({toggle(watchdog    ), set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false), set(brake0, true ), set(brake1, true ), set(brake2, true ), set(brake3, true ) });
	emergency       .setOutputActions({set(watchdog, false), set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false), set(brake0, true ), set(brake1, true ), set(brake2, true ), set(brake3, true ) });
	approvalOn      .setOutputActions({toggle(watchdog    ), set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false), set(brake0, true ), set(brake1, true ), set(brake2, true ), set(brake3, true ) });
	on              .setOutputActions({toggle(watchdog    ), set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false), set(brake0, false), set(brake1, false), set(brake2, false), set(brake3, false) });

	// *** Define and add level functions *** //
	
	off.setLevelAction([this](SafetyContext* privateContext) {
		static bool first = true; 
		if(first == true) {
			privateContext->triggerEvent(startSW);
			first = false;
		}
	});
	
	approvalOn.setLevelAction([this](SafetyContext* privateContext) {
		static bool first = true; 
		if(first == true) {
			privateContext->triggerEvent(doOn);
			first = false;
		}
	});
	
	on.setLevelAction([this](SafetyContext* privateContext) {
		static int count = 0;
		if(count > 100){
			std::cout << "axis0: "   << limitSwitchQ0p->get() << "; " << limitSwitchQ0n->get()
			          << ", axis1: " << limitSwitchQ1p->get() << "; " << limitSwitchQ1n->get() 
			          << ", axis2: " << limitSwitchQ2p->get() << "; " << limitSwitchQ2n->get() 
			          << ", axis3: " << limitSwitchQ3p->get() << "; " << limitSwitchQ3n->get() << std::endl;
			count = 0;
		}
		else
			count++;
	});

	// Define entry level
	setEntryLevel(off);
}

ESSafetyProperties::~ESSafetyProperties() {
	// nothing to do
}