#pragma once
#include <eeros/sequencer/Step.hpp>
#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/BaseSequence.hpp>
#include "../control/DeltaControlSystem.hpp"
#include "../Calibration.hpp"
#include <unistd.h>

namespace eeduro{
  namespace delta{
    class Release : public eeros::sequencer::Step {
    public:
      Release(std::string name,eeros::sequencer::Sequencer & seq, BaseSequence* caller, DeltaControlSystem& controlSys, Calibration& calibration) : Step(name, seq, caller), controlSys(controlSys), calibration(calibration){
      }
      int operator() () {return Step::start();}
      int action(){
// 	log.trace() << "release block";
	controlSys.emagVal.setValue(false);
      };
      
      DeltaControlSystem & controlSys;
      Calibration calibration;
      
    };

  }
}