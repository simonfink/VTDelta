#ifndef CH_NTB_SCARA_SCARASEQUENCEMANUALPARK_HPP_
#define CH_NTB_SCARA_SCARASEQUENCEMANUALPARK_HPP_

#include "../../ScaraControlSystem.hpp"
#include "../../ScaraSafetyProperties.hpp"
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/sequencer/Sequence.hpp>

namespace scara{
	class ScaraSequenceManualPark : public eeros::sequencer::Sequence<> {

	public:
		ScaraSequenceManualPark(eeros::sequencer::Sequencer* sequencer, scara::ScaraControlSystem* controlSys, eeros::safety::SafetySystem* safetySys);
		
		virtual void init();
		virtual bool checkPreCondition();
		virtual void run();
		virtual bool checkPostCondition();
		virtual void exit();
	
	private:
		scara::ScaraControlSystem* controlSys;
		eeros::safety::SafetySystem* safetySys;
	};
};

#endif // CH_NTB_SCARA_SCARASEQUENCEMANUALPARK_HPP_ 