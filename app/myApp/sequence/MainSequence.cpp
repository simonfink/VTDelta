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
	mmc(controlSys),
	mexSeq("Mouse Exception Sequence", sequencer, this,  safetySys, properties, controlSys),
// 	amexSeq("Mouse Exception Sequence", sequencer, this, controlSys,  safetySys, properties),
	mouseMove("MouseMoveMonitor", this, mmc, eeros::sequencer::SequenceProp::restart, &mexSeq),
	sortSeq("Sort Sequence", sequencer, this, controlSys, safetySys, calibration, mouseMove),
	shuffSeq("Shuffle Sequence", sequencer, this, controlSys, safetySys, calibration, mouseMove),
	mouseSeq("Mouse Sequence", sequencer, this, controlSys, safetySys, calibration, properties),
 	calibSeq("Calibration Sequence", sequencer, this, controlSys, safetySys, calibration)
	
	  {
	    sequencer.addSequence(sortSeq);
	    sequencer.addSequence(shuffSeq);
	    sequencer.addSequence(calibSeq);
	  }
	


int MainSequence::action() {
 
 
 while(safetySys.getCurrentLevel() < properties.slSystemReady);
  log.warn() << "starting mainsequence";
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
   
   else if(safetySys.getCurrentLevel() == properties.slCalibrating){
        calibSeq.start();
        calibSeq.waitAndTerminate();
   }
   
   else if(safetySys.getCurrentLevel() == properties.slMouseTeaching){
// 	 controlSys.emagVal.setValue(controlSys.mouse.getButtonOut().getSignal().getValue()[0]);
 	 mouseSeq.start();
 	 mouseSeq.wait();
//       sequencer.getSequenceByName("Mouse Sequence")->start();
   }
    
  }
}


