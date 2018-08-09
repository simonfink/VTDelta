#pragma once
#include <eeros/sequencer/Step.hpp>
#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/BaseSequence.hpp>
#include "../control/DeltaControlSystem.hpp"
#include "../Calibration.hpp"
#include <unistd.h>

namespace eeduro{
  namespace delta{
    class Grab : public eeros::sequencer::Step {
    public:
      Grab(std::string name,eeros::sequencer::Sequencer & seq, BaseSequence* caller, DeltaControlSystem& controlSys, Calibration& calibration) : Step(name, seq, caller), controlSys(controlSys), calibration(calibration){
      }
      int operator() () {return Step::start();}
      int action(){
// 	log.trace() << "grab block";
	controlSys.emagVal.setValue(true);
      };
      
      DeltaControlSystem & controlSys;
      Calibration calibration;
      
    };

  }
}