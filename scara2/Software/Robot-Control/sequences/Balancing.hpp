#ifndef CH_NTB_PARALLELSCARA_BALANCING_HPP_
#define CH_NTB_PARALLELSCARA_BALANCING_HPP_

#include <eeros/sequencer/Sequence.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include "../control/ParallelScaraControlSystem.hpp"
#include "../safety/ParallelScaraSafetyProperties.hpp"
#include "../types.hpp"

namespace parallelscara{
	class Balancing : public eeros::sequencer::Sequence {

	public:
		Balancing(eeros::sequencer::Sequencer* sequencer, ParallelScaraControlSystem* controlSys, eeros::safety::SafetySystem* safetySys, ParallelScaraSafetyProperties* safetyProp);

		virtual void init();
		virtual bool checkPreCondition();
		virtual void run();
		virtual bool checkPostCondition();
		virtual void exit();
		
	private:
		bool isTerminating();
		bool isStopping();
		
		double t, delta_t;
		
		ParallelScaraControlSystem* controlSys;
		eeros::safety::SafetySystem* safetySys;
		ParallelScaraSafetyProperties* safetyProp;
	};
};

#endif // CH_NTB_PARALLELSCARA_BALANCING_HPP_ 