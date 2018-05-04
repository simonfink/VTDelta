#include <eeros/math/Matrix.hpp>
#include <iostream>
#include <unistd.h>
#include <eeros/hal/HAL.hpp>
#include <eeros/core/System.hpp>
#include <iostream>
#include "../joystick/XBoxController.hpp"
#include "SetPlasmaPen.hpp"

using namespace scara;
using namespace eeros;
using namespace eeros::hal;

SetPlasmaPen::SetPlasmaPen() {}

SetPlasmaPen::~SetPlasmaPen() { 
	// nothing to do...
}

void SetPlasmaPen::run() {
		
	HAL& hal = HAL::instance();
	
	bool red_button = in.getSignal().getValue()(1);
	bool plasmaIsOn = hal.getLogicPeripheralOutput("plasmaON")->get();
	
	if(enabled) {
		if(red_button && !plasmaIsOn){
			hal.getLogicPeripheralOutput("plasmaON")->set(true); 
		}
		else if(!red_button && plasmaIsOn) {
			hal.getLogicPeripheralOutput("plasmaON")->set(false); 
		}
	}
}

void SetPlasmaPen::enable() {
	enabled = true;
}

void SetPlasmaPen::disable() {
	enabled = false;
}
