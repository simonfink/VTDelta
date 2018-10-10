#include "MouseSequence.hpp"
#include "../safety/DeltaSafetyProperties.hpp"
#include "ExceptionSequence.hpp"
#include <unistd.h>

using namespace eeduro::delta;
using namespace eeros::sequencer;
using namespace eeros::safety;
MouseSequence::MouseSequence(std::string name, eeros::sequencer::Sequencer& sequencer, eeros::sequencer::BaseSequence* caller, DeltaControlSystem& controlSys, eeros::safety::SafetySystem& safetySys, Calibration& calibration, DeltaSafetyProperties& properties) :
	Sequence(name, sequencer),
	controlSys(controlSys),
	safetySys(safetySys),
	emag("Set Elektromagnet", sequencer, this, controlSys),
	timeoutSeq("Mouse TimeOut Exception Sequence", sequencer, this, controlSys, safetySys, properties){
	  setTimeoutTime(2.0);
	  setTimeoutExceptionSequence(timeoutSeq);				
	  setTimeoutBehavior(eeros::sequencer::SequenceProp::abort);
	  controlSys.mouse.setInitPos(controlSys.pathPlanner.getLastPoint());
	  mouseNew = controlSys.mouse.getOut().getSignal().getValue();
	  mouseOld = mouseNew;
	  count = 0;
	}
	
bool MouseSequence::checkPreCondition()
{
  controlSys.inputSwitch.switchToInput(1);
  controlSys.voltageSwitch.switchToInput(0);
  return true;
}



int MouseSequence::action() {
	mouseNew = controlSys.mouse.getOut().getSignal().getValue();
	
	if(controlSys.mouse.getButtonOut().getSignal().getValue()[0]){
	    buttonPressed = true;
	    emag(true);
	}
	else{
	    buttonPressed = false;
	    emag(false);
	}
	
	while(!buttonPressed && mouseNew == mouseOld && count < (2000)){
	 mouseNew = controlSys.mouse.getOut().getSignal().getValue();
	 emag(false);
	 count++;
	}
	
	count = 0;
	
	mouseOld = mouseNew;
      
}
