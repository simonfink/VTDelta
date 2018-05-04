#ifndef CH_NTB_PARALLELSCARA_HOMING_HPP_
#define CH_NTB_PARALLELSCARA_HOMING_HPP_

#include "../control/ParallelScaraControlSystem.hpp"
#include "../safety/ParallelScaraSafetyProperties.hpp"
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/sequencer/Step.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include "../types.hpp"

using namespace eeros::sequencer;
using namespace eeros::safety;

namespace parallelscara {

	class MoveRight : public Step {
	public:
		MoveRight(std::string name, Sequencer& seq, BaseSequence* caller, ParallelScaraControlSystem& cs) : Step(name, seq, caller), cs(cs) { }
		int action() {
			cs.robotController.setInitializationSpeed({0.7, 0.7});
		}
		bool checkExitCondition() {return cs.robotController.reachedMechanicalLimit(maxTorque_mechLimit);}
	private:
		ParallelScaraControlSystem& cs;
	};
	
	class MoveLeft : public Step {
	public:
		MoveLeft(std::string name, Sequencer& seq, BaseSequence* caller, ParallelScaraControlSystem& cs) : Step(name, seq, caller), cs(cs) { }
		int action() {
			cs.robotController.setInitializationSpeed({-0.7, -0.7});
		}
		bool checkExitCondition() {return cs.robotController.reachedMechanicalLimit(maxTorque_mechLimit);}
	private:
		ParallelScaraControlSystem& cs;
	};
	
	class Wait : public Step {
	public:
		Wait(std::string name, Sequencer& seq, BaseSequence* caller) : Step(name, seq, caller) { }
		int operator() (double time) {this->waitingTime = time; return Step::start();}
		int action() {time = std::chrono::steady_clock::now();}
		bool checkExitCondition() {return ((std::chrono::duration<double>)(std::chrono::steady_clock::now() - time)).count() > waitingTime;}
	private:
		std::chrono::time_point<std::chrono::steady_clock> time;
		double waitingTime;
	};

	class Homing : public Sequence {
	public:
		Homing(std::string name, Sequencer& seq, ParallelScaraControlSystem& cs, SafetySystem& ss, ParallelScaraSafetyProperties& sp) :
			cs(&cs), ss(&ss), sp(&sp), Sequence(name, seq), moveRight("move right", seq, this, cs), moveLeft("move left", seq, this, cs), wait("wait", seq, this) { 
			setNonBlocking();
		}

		int action() {
			moveRight();
			cs->robotController.setInitializationSpeed({0, 0});
			wait(0.5);
			AxisVector initAnglesRight = cs->encPosMux.getOut().getSignal().getValue();
			moveLeft();
			cs->robotController.setInitializationSpeed({0, 0});
			wait(0.5);
			AxisVector initAnglesLeft = cs->encPosMux.getOut().getSignal().getValue();
			cs->robotController.setInitializationSpeed({0.57, 1.06});	// move out of mechanical limit
			wait(3.0);
			cs->robotController.setInitializationSpeed({0, 0});
			wait(1.0);
			AxisVector initAnglesDelta = cs->encPosMux.getOut().getSignal().getValue() - initAnglesLeft;
			cs->robotController.initPos = angles_mechLimit + initAnglesDelta;
			ss->triggerEvent(sp->homingDone);
		}
		
		bool checkExitCondition() {return true;}
		
	private:
		ParallelScaraControlSystem* cs;
		SafetySystem* ss;
		ParallelScaraSafetyProperties* sp;
		MoveRight moveRight;
		MoveLeft moveLeft;
		Wait wait;
	};
};

#endif // CH_NTB_PARALLELSCARA_HOMING_HPP_ 