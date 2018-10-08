#pragma once

#include <eeros/sequencer/Sequence.hpp>
#include <eeros/safety/SafetySystem.hpp>

#include "../control/DeltaControlSystem.hpp"
#include "../safety/DeltaSafetyProperties.hpp"
#include "MouseSequence.hpp"
#include "../Calibration.hpp"

#include <vector>

namespace eeduro {
	namespace delta {
		class MouseExceptionSequence : public eeros::sequencer::Sequence{
		public:
			MouseExceptionSequence(std::string name, eeros::sequencer::Sequencer& sequencer,eeros::sequencer::BaseSequence* caller, eeros::safety::SafetySystem& safetySys, DeltaSafetyProperties& properties, DeltaControlSystem& controlSys):
			Sequence(name, sequencer, caller, true),
			controlSys(controlSys),
			safetySys(safetySys),
			properties(properties),
			sequencer(sequencer)
			{
			  			  
			}
			
			int action(){
			    sequencer.abort();
			    controlSys.start();
			    controlSys.pathPlanner.gotoPoint({0.0,0.0,-0.015,0.0});
			    while (!controlSys.pathPlanner.posReached()) {
				usleep(100000);
				std::this_thread::yield();
			    } 
			    controlSys.inputSwitch.switchToInput(1);
			    controlSys.voltageSwitch.switchToInput(0);
			    safetySys.triggerEvent(properties.doMouseTeaching);			    
			}

			
		private:

			eeros::safety::SafetySystem& safetySys;
			DeltaSafetyProperties& properties;
			DeltaControlSystem& controlSys;
			eeros::sequencer::Sequencer& sequencer;


		};
		
		class MouseTimeOutExceptionSequence : public eeros::sequencer::Sequence{
		public:
			MouseTimeOutExceptionSequence(std::string name, sequencer::Sequencer& sequencer, eeros::sequencer::BaseSequence* caller,DeltaControlSystem& controlSys, eeros::safety::SafetySystem& safetySys, DeltaSafetyProperties& properties):
			Sequence(name, sequencer, caller, true),
			safetySys(safetySys),
			properties(properties),
			controlSys(controlSys),
			sequencer(sequencer)
			{
			  
			}
			
			int action(){
 			    sequencer.abort();
			    controlSys.start();
			    controlSys.pathPlanner.gotoPoint({0.0,0.0,-0.015,0.0});
			    controlSys.inputSwitch.switchToInput(0);
			    controlSys.voltageSwitch.switchToInput(0);
			    while (!controlSys.pathPlanner.posReached()) {
				usleep(100000);
				std::this_thread::yield();
			    } 
			    safetySys.triggerEvent(properties.doAutoMoving);
			}
			
		private:
			eeros::safety::SafetySystem& safetySys;
			DeltaSafetyProperties& properties;
			DeltaControlSystem& controlSys;
			eeros::sequencer::Sequencer& sequencer;
		};
	}
}
