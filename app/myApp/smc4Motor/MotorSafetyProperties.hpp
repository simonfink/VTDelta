#ifndef MOTORSAFETYPROPERTIES_HPP_
#define MOTORSAFETYPROPERTIES_HPP_

#include <eeros/safety/SafetyProperties.hpp>
#include <eeros/hal/HAL.hpp>
#include "MotorControlSystem.hpp"

class MotorSafetyProperties : public eeros::safety::SafetyProperties {
	
public:
	MotorSafetyProperties(MotorControlSystem& controlSys, double ts);
	virtual ~MotorSafetyProperties();
	
	// Define all possible events
	eeros::safety::SafetyEvent doSystemOn;
	eeros::safety::SafetyEvent doSystemOff;
	eeros::safety::SafetyEvent startControl;
	eeros::safety::SafetyEvent stopControl;
	eeros::safety::SafetyEvent startControlDone;
	eeros::safety::SafetyEvent stopControlDone;
	eeros::safety::SafetyEvent startMoving;
	eeros::safety::SafetyEvent stopMoving;
	eeros::safety::SafetyEvent doEmergency;
	eeros::safety::SafetyEvent resetEmergency;	
	eeros::safety::SafetyEvent abort;
	
	// Name all levels
	eeros::safety::SafetyLevel slOff;
	eeros::safety::SafetyLevel slEmergency;
	eeros::safety::SafetyLevel slSystemOn;
	eeros::safety::SafetyLevel slStartingControl;
	eeros::safety::SafetyLevel slStoppingControl;
	eeros::safety::SafetyLevel slPowerOn;
	eeros::safety::SafetyLevel slMoving;
	
protected:
	// critical outputs
	eeros::hal::Output<bool>* enable;
	eeros::hal::Output<bool>* isEmergency;
	
	// critical inputs
	eeros::hal::Input<bool>* emergency;
	eeros::hal::Input<bool>* ready;
		
	MotorControlSystem& controlSys;
};

#endif // MOTORSAFETYPROPERTIES_HPP_