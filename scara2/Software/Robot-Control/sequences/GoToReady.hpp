#ifndef CH_NTB_PARALLELSCARA_GOTOREADY_HPP_
#define CH_NTB_PARALLELSCARA_GOTOREADY_HPP_

#include "../control/ParallelScaraControlSystem.hpp"
#include "../safety/ParallelScaraSafetyProperties.hpp"
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include "../types.hpp"

namespace parallelscara{
	class GoToReady : public eeros::sequencer::Sequence {

	public:
		GoToReady(eeros::sequencer::Sequencer* sequencer, ParallelScaraControlSystem* controlSys, eeros::safety::SafetySystem* safetySys, ParallelScaraSafetyProperties* safetyProp);

		virtual void init();
		virtual bool checkPreCondition();
		virtual void run();
		virtual bool checkPostCondition();
		virtual void exit();
		
	private:
		bool isTerminating();
		bool isStopping();
		
		ParallelScaraControlSystem* controlSys;
		eeros::safety::SafetySystem* safetySys;
		ParallelScaraSafetyProperties* safetyProp;
	};
};

#endif // CH_NTB_PARALLELSCARA_GOTOREADY_HPP_ 