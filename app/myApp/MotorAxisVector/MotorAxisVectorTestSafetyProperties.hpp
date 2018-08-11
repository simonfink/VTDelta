#ifndef MOUSESAFETYPROPERTIES_HPP_
#define MOUSESAFETYPROPERTIES_HPP_

#include <eeros/safety/SafetyProperties.hpp>
#include <eeros/hal/HAL.hpp>
#include "MotorAxisVectorTestControlSystem.hpp"

#include <eeros/safety/InputAction.hpp>
#include <eeros/safety/OutputAction.hpp>
#include <eeros/core/Executor.hpp>

#include <unistd.h>
#include <iostream>
#include <vector>
#include <initializer_list>

using namespace eeros;
using namespace eeros::hal;
using namespace eeros::safety;


class MotorAxisVectorTestSafetyProperties : public eeros::safety::SafetyProperties {
	
public:
	public:
	  MotorAxisVectorTestSafetyProperties(MotorAxisVectorTestControlSystem& controlSys, double ts):
	  slRun("Safety Level RUN"),
	  slStop("Safety Level STOP"),
	  seRun("Goto SL RUN"),
	  seStop("Goto SL STOP"),
	  controlSys(controlSys)
	  {
	    HAL& hal = HAL::instance();

	    runLed = hal.getLogicOutput("ledBlue");
	    stopLed = hal.getLogicOutput("ledRed");
	    
	    criticalOutputs={runLed, stopLed};
	    
	    runButton = hal.getLogicInput("buttonBlue");
	    stopButton= hal.getLogicInput("buttonRed");
	    
	    addLevel(slRun);
	    addLevel(slStop);

	    slRun.addEvent(seStop, slStop);
	    slStop.addEvent(seRun, slRun);
	    
	    slRun.setInputActions({ check(stopButton, true, seStop), ignore(runButton) });
	    slStop.setInputActions({ check(runButton, true, seRun), ignore(stopButton) });
	    
	    slRun.setOutputActions({set(runLed,true), set(stopLed,false)});
	    slStop.setOutputActions({set(runLed,false), set(stopLed, true)});
	    
	    

	    setEntryLevel(slRun);
	  }
	  eeros::safety::SafetyLevel slRun, slStop;
	  eeros::safety::SafetyEvent seRun, seStop;
	  
	  eeros::hal::Output<bool>* runLed;
	  eeros::hal::Output<bool>* stopLed;
	  eeros::hal::Input<bool>* runButton;
	  eeros::hal::Input<bool>* stopButton;
	  
	  MotorAxisVectorTestControlSystem& controlSys;
};


#endif // MOUSESAFETYPROPERTIES_HPP_