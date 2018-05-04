#ifndef CH_NTB_PARALLELSCARA_MAINSEQUENCE_HPP_
#define CH_NTB_PARALLELSCARA_MAINSEQUENCE_HPP_

#include <eeros/sequencer/Sequence.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include "../control/ParallelScaraControlSystem.hpp"

#include "Homing.hpp"
#include "GoToReady.hpp"
#include "HighSpeed.hpp"
#include "Balancing.hpp"

using namespace eeros::sequencer;

namespace parallelscara {
	
	class MainSequence : public Sequence {
	public:
		MainSequence(Sequencer& seq, ParallelScaraControlSystem* controlSys, SafetySystem* safetySys, ParallelScaraSafetyProperties* safetyProp);
		
//		virtual bool checkPreCondition();
		int action();
// 		virtual void exit();
		
	private:
// 		Homing 		homing_s;
// 		GoToReady 	goToReady_s;
// 		HighSpeed 	highSpeed_s;
// 		Balancing 	balancing_s;
		
// 		bool isTerminating();
		
		ParallelScaraControlSystem* controlSys;
		SafetySystem* safetySys;
		ParallelScaraSafetyProperties* safetyProp;
	};
};

#endif // CH_NTB_PARALLELSCARA_MAINSEQUENCE_HPP_ 