#include "PPSafetyProperties.hpp"
#include "PPControlSystem.hpp"
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

PPSafetyProperties::PPSafetyProperties(PPControlSystem* cs) : 
off("SW off"), 
on("SW on "),
doOn("control system started"),
doOff("control system stopped"),

controlSys(cs) {

	HAL& hal = HAL::instance();

	// ############ Define critical outputs ############
	
	// ############ Define critical inputs ############
	
	// ############ Other inputs/outputs ############
	
	
	// ############ Define Levels ############
	addLevel(off);
	addLevel(on);
		
	// ############ Add events to the levels ############
	off.addEvent(doOn,  on,  kPublicEvent  ); 
	on .addEvent(doOff, off, kPublicEvent  ); 
	
	// ############ Define input states and events for all levels ############
	off .setInputActions({  });
	on  .setInputActions({  });
	
	// Define output states and events for all levels 
	off .setOutputActions({ });
	on  .setOutputActions({ });
	
	// *** Define and add level functions *** //
	
	// Boot
	off.setLevelAction([this](SafetyContext* privateContext) {
		static bool first = true; 
		if(first == true) {
			privateContext->triggerEvent(doOn);
			std::vector<AxisVector> pos = {{1.0, 2.0, 3.0, 4.0},{2.0, 4.0, 6.0, 8.0},{4.0, 8.0, 12.0, 16.0},{1.0, 2.0, 3.0, 4.0}};
			controlSys->pathPlannerJS.move(pos);
			first = false;
		}
	});
	
	on.setLevelAction([this](SafetyContext* privateContext) {
		static bool first = true;
		if(first){
			controlSys->speedInit.setValue({0.1,0.1, 2.0, 3.0});
			first = false;
		}
		
		
		static int count = 0;
		if(count > 100){
// 			auto pos = controlSys->pathPlannerJS.getPosOut().getSignal().getValue();
// 			std::cout << controlSys->pathPlannerJS.posReached() << "; " <<  pos(0) << "; " << pos(1) << "; " << pos(2) << "; " << pos(3) << std::endl;
			
			auto speed = controlSys->speedInit.getOut().getSignal().getValue();
			auto sw = controlSys->speedSwitch.getOut().getSignal().getValue();
			std::cout 	<< speed(0) << "; " << speed(1) << "; " << speed(2) << "; " << speed(3)
						<< sw(0) << "; " << sw(1) << "; " << sw(2) << "; " << sw(3)
						<< std::endl;
			
			count = 0;
		}
		else
			count++;
	});

	// Define entry level
	setEntryLevel(off);
}

PPSafetyProperties::~PPSafetyProperties() {
	// nothing to do
}