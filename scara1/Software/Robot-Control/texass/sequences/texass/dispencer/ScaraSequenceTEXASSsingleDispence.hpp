#ifndef CH_NTB_SCARA_SCARASEQUENCETEXASSSINGLEDISPENCE_HPP_
#define CH_NTB_SCARA_SCARASEQUENCETEXASSSINGLEDISPENCE_HPP_

#include "../../../ScaraControlSystem.hpp"
#include "../../../ScaraSafetyProperties.hpp"
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/safety/SafetySystem.hpp>

namespace scara{
	
	class ScaraSequenceTEXASSsingleDispence : public eeros::sequencer::Sequence<void, AxisVector, AxisVector> {

	public:
		ScaraSequenceTEXASSsingleDispence(eeros::sequencer::Sequencer* sequencer, scara::ScaraControlSystem* controlSys, eeros::safety::SafetySystem* safetySys);
		
		virtual void init();
		virtual bool checkPreCondition();
		virtual void run(AxisVector p_in, AxisVector vel);
		virtual bool checkPostCondition();
		virtual void exit();
	
	private:
		scara::ScaraControlSystem* controlSys;
		eeros::safety::SafetySystem* safetySys;
		
		AxisVector p;
	};
};

#endif // CH_NTB_SCARA_SCARASEQUENCETEXASSSINGLEDISPENCE_HPP_ 