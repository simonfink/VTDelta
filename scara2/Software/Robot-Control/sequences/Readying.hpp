#ifndef CH_NTB_PARALLELSCARA_READYING_HPP_
#define CH_NTB_PARALLELSCARA_READYING_HPP_

#include <eeros/sequencer/Sequence.hpp>
#include <eeros/sequencer/Step.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include "../control/ParallelScaraControlSystem.hpp"
#include "../safety/ParallelScaraSafetyProperties.hpp"
#include "../types.hpp"

using namespace eeros::sequencer;
using namespace eeros::safety;

namespace parallelscara {

	class Readying : public Sequence {
	public:
		Readying(std::string name, Sequencer& seq, ParallelScaraControlSystem& cs, SafetySystem& ss, ParallelScaraSafetyProperties& sp) : cs(&cs), ss(&ss), sp(&sp), Sequence(name, seq) { 
			setNonBlocking();
		}

		int action() {
			cs->pathPlanner.setMaxSpeed(1.0);
			cs->pathPlanner.setMaxAcc(0.5);
			cs->pathPlanner.setMaxDec(0.5);
			cs->pathPlanner.move(ready_pos);
		}
		
		bool checkExitCondition() {
			bool end = cs->pathPlanner.endReached();
			if (end) ss->triggerEvent(sp->readyDone);
			return end;
		}
		
	private:
		parallelscara::ParallelScaraControlSystem* cs;
		eeros::safety::SafetySystem* ss;
		ParallelScaraSafetyProperties* sp;
	};
};

#endif // CH_NTB_PARALLELSCARA_READYING_HPP_ 