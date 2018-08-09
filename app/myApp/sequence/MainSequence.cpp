#include "MainSequence.hpp"
#include "../safety/DeltaSafetyProperties.hpp"
#include <eeros/sequencer/Sequencer.hpp>
#include "../sequence/CalibrateSequence.hpp"
#include <unistd.h>

using namespace eeduro::delta;
using namespace eeros::sequencer;
using namespace eeros::safety;

MainSequence::MainSequence(std::string name, eeros::sequencer::Sequencer& sequencer, DeltaControlSystem& controlSys, eeros::safety::SafetySystem& safetySys, DeltaSafetyProperties properties, Calibration& calibration):
	Sequence(name, sequencer),
	controlSys(controlSys),
	properties(properties),
	calibration(calibration),
	safetySys(safetySys),
	sequencer(sequencer),
	sortSeq("Sort Sequence", sequencer, this, controlSys, safetySys, calibration),
	shuffSeq("Shuffle Sequence", sequencer, this, controlSys, safetySys, calibration),
	calibSeq("Calibration Sequence", sequencer, this, controlSys, safetySys, calibration),
	
	mmc(controlSys),
	mouseMove("MouseMoveMonitor", this, mmc, eeros::sequencer::SequenceProp::abort){
	  sequencer.addSequence(sortSeq);
	  sequencer.addSequence(shuffSeq);
	  sequencer.addSequence(calibSeq);
	  
	  addMonitor(&mouseMove);
	  }
	


int MainSequence::action() {
 
 
 while(safetySys.getCurrentLevel() < properties.slSystemReady);
 
//   auto& sequencer = sequencer::Sequencer::instance();
//   const AxisVector start_position{ 0, 0, -0.015, 0 };
	
//   controlSys.pathPlanner.setInitPos(start_position);  
//   controlSys.inputSwitch.switchToInput(0);		// set input to pathplanner

  
//   CalibrateSequence* calSeq = static_cast<CalibrateSequence>(sequencer.getSequenceByName("Calibration Sequence"));
  
  while(Sequencer::running){
   log.info() << safetySys.getCurrentLevel().getDescription();

   
   if(safetySys.getCurrentLevel() == properties.slAutoMoving){
    sortSeq.start();
    
    sortSeq.wait();
   
    shuffSeq.start();
    
    shuffSeq.wait();
   }
   
   if(safetySys.getCurrentLevel() == properties.slEmergency){
   }
   
   if(safetySys.getCurrentLevel() == properties.slCalibrating){
       calibSeq.start();
       calibSeq.waitAndTerminate();
   }
   
   if(safetySys.getCurrentLevel() == properties.slMouseTeaching){
//       sequencer.getSequenceByName("Mouse Sequence")->start();
   }
   
//    sequencer.getSequenceByName("Calibration Sequence")->start();
//    sequencer.getSequenceByName("Mouse Sequence")->wait();
 
  }
}


