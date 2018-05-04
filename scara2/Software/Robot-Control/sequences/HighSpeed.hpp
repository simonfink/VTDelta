#ifndef CH_NTB_PARALLELSCARA_HIGHSPEED_HPP_
#define CH_NTB_PARALLELSCARA_HIGHSPEED_HPP_

#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/sequencer/Step.hpp>
#include "../control/ParallelScaraControlSystem.hpp"
#include "../safety/ParallelScaraSafetyProperties.hpp"
#include "../types.hpp"
#include "../constants.hpp"

using namespace eeros::sequencer;
using namespace eeros::safety;

namespace parallelscara{
	class Move : public Step {
	public:
		Move(std::string name, Sequencer& seq, BaseSequence* caller, ParallelScaraControlSystem& cs) : Step(name, seq, caller), cs(cs) { }
		int operator() (AxisVector dest) {this->dest = dest; return Step::start();}
		int action() {cs.pathPlanner.move(dest);}
		bool checkExitCondition() {return cs.pathPlanner.endReached();}
	private:
		ParallelScaraControlSystem& cs;
		AxisVector dest;
	};

	class HighSpeed : public Sequence {
	public:
		HighSpeed(std::string name, Sequencer& seq, ParallelScaraControlSystem& cs, SafetySystem& ss, ParallelScaraSafetyProperties& sp) : 
			cs(&cs), ss(&ss), sp(&sp), Sequence(name, seq), move("move", seq, this, cs) { 
			setNonBlocking();
		}

		int action() { 
			cs->pathPlanner.setMaxSpeed(0.2);
			cs->pathPlanner.setMaxAcc(0.1);
			cs->pathPlanner.setMaxDec(0.1);
			while (Sequencer::running) {
				AxisVector pos = {0.20, 0.25};
				move(pos);
				pos = {0.20, 0.35};
				move(pos);
				pos = {-0.20, 0.35};
				move(pos);
				pos = {-0.20, 0.25};
				move(pos);
			}
		}
		
		bool checkExitCondition() {return true;}
		
	private:
		ParallelScaraControlSystem* cs;
		SafetySystem* ss;
		ParallelScaraSafetyProperties* sp;	
		Move move;
	};
};

#endif // CH_NTB_PARALLELSCARA_HIGHSPEED_HPP_ 