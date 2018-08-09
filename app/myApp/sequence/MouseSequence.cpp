#include "MouseSequence.hpp"
#include "../safety/DeltaSafetyProperties.hpp"
#include <unistd.h>

using namespace eeduro::delta;
using namespace eeros::sequencer;
using namespace eeros::safety;

MouseSequence::MouseSequence(std::string name, eeros::sequencer::Sequencer& sequencer, DeltaControlSystem& controlSys, eeros::safety::SafetySystem& safetySys, Calibration& calibration) :
	Sequence(name, sequencer),
	controlSys(controlSys),
	safetySys(safetySys){
	  
	}
	



int MouseSequence::action() {
	controlSys.inputSwitch.switchToInput(1);
  
	while(Sequencer::running){
// 	  log.info() << "mouse running";
//     std::cout << "mouse sequencer running" << std::endl;
//       mouseNew = controlSys.mouse.getOut().getSignal().getValue();
  //     std::cout << "mouse action" << std::endl;
//       if(mouseNew == mouseOld) count++;
//       else count = 0;
      
      if(controlSys.mouse.getButtonOut().getSignal().getValue()[0]){
	  count = 0;
  // 	std::cout << "button pressed" << std::endl;
	  controlSys.emagVal.setValue(true);
      }
      else{
	  controlSys.emagVal.setValue(false);
      }
	}
      
//       mouseOld = mouseNew;
}
