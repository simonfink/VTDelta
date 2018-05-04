#ifndef CH_NTB_SCARA_SCARASEQUENCELOADREFSYSTEM_HPP_
#define CH_NTB_SCARA_SCARASEQUENCELOADREFSYSTEM_HPP_

#include "../../ScaraControlSystem.hpp"
#include "../../ScaraSafetyProperties.hpp"
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/safety/SafetySystem.hpp>

namespace scara{
	class ScaraSequenceLoadRefSystem : public eeros::sequencer::Sequence<> {

	public:
		ScaraSequenceLoadRefSystem(eeros::sequencer::Sequencer* sequencer, scara::ScaraControlSystem* controlSys, eeros::safety::SafetySystem* safetySys);
		
		virtual void init();
		virtual void run();
		virtual void exit();
	
	private:
		eeros::math::Frame* frame;
		scara::ScaraControlSystem* controlSys;
		eeros::safety::SafetySystem* safetySys;
	};
};

#endif // CH_NTB_SCARA_SCARASEQUENCELOADREFSYSTEM_HPP_
