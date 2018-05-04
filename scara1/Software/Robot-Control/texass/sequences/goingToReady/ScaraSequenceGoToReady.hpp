#ifndef CH_NTB_SCARA_SCARASEQUENCEGOTOREADY_HPP_
#define CH_NTB_SCARA_SCARASEQUENCEGOTOREADY_HPP_

#include "../../ScaraControlSystem.hpp"
#include "../../ScaraSafetyProperties.hpp"
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include "../../types.hpp"

namespace scara{
	class ScaraSequenceGoToReady : public eeros::sequencer::Sequence<> {

	public:
		ScaraSequenceGoToReady(eeros::sequencer::Sequencer* sequencer, scara::ScaraControlSystem* controlSys, eeros::safety::SafetySystem* safetySys);

		virtual void init();
		virtual bool checkPreCondition();
		virtual void run();
		virtual bool checkPostCondition();
		virtual void exit();
		
	private:
		AxisVector posUp, posReady; 
		scara::ScaraControlSystem* controlSys;
		eeros::safety::SafetySystem* safetySys;
	};
};

#endif // CH_NTB_SCARA_SCARASEQUENCEGOTOREADY_HPP_ 