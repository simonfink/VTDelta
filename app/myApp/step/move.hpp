#pragma once
#include <eeros/sequencer/Step.hpp>
#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/BaseSequence.hpp>
#include "../control/DeltaControlSystem.hpp"
#include "../Calibration.hpp"
#include <unistd.h>

namespace eeduro{
  namespace delta{
    class Move : public eeros::sequencer::Step {
    public:
      Move(std::string name, eeros::sequencer::Sequencer & seq, BaseSequence* caller, DeltaControlSystem& controlSys, Calibration& calibration) : Step(name, seq, caller), controlSys(controlSys), calibration(calibration){
	this->position = 0;
      }
      int operator() (int pos) {this->position = pos; return Step::start();}
      int action(){
	auto p = controlSys.pathPlanner.getLastPoint();
	p[0] = calibration.position[position].x;
	p[1] = calibration.position[position].y;
	
// 	if(position == 0) log.error() << "phi0: " << calibration.position[position].r;
	
	
	if (p[3] > 1) {
		p[3] = calibration.position[position].r;
	}
	else {
// 		log.error() << "rotating from: " << p[3]  << " to: " << calibration.position[position].r+pi/8.0;
		p[3] = calibration.position[position].r + pi / 2.0;
		
	}
	controlSys.pathPlanner.gotoPoint(p);
	waitUntilPointReached();
      };
    private:
      void waitUntilPointReached(){
// 	std::cout << "move to position: " << controlSys.pathPlanner.getPosOut().getSignal().getValue() << std::endl;
	while (!controlSys.pathPlanner.posReached()) {
// 		std::cout << "move to: " << controlSys.pathPlanner.getPosOut().getSignal().getValue() << std::endl;
// 		std::cout << "moving: " << controlSys.directKin.getOut().getSignal().getValue() << std::endl;
		usleep(100000);
		std::this_thread::yield();
	} 
// 	std::cout << "moving to final: " << controlSys.pathPlanner.getPosOut().getSignal().getValue() << std::endl;
// 	std::cout << "moving final: " << controlSys.directKin.getOut().getSignal().getValue() << std::endl;
      };
      
      DeltaControlSystem & controlSys;
      Calibration calibration;
      int position;
      
    };

  }
}