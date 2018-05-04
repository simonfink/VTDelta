#ifndef CH_NTB_SCARA_SCARASEQUENCEHOMETOREADY_HPP_
#define CH_NTB_SCARA_SCARASEQUENCEHOMETOREADY_HPP_

#include "../../ScaraControlSystem.hpp"
#include "../../ScaraSafetyProperties.hpp"
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include "../../types.hpp"

namespace scara{
	class ScaraSequenceHomeToReady : public eeros::sequencer::Sequence<> {

	public:
		ScaraSequenceHomeToReady(eeros::sequencer::Sequencer* sequencer, scara::ScaraControlSystem* controlSys, eeros::safety::SafetySystem* safetySys);

		virtual void init();
		virtual bool checkPreCondition();
		virtual void run();
		virtual bool checkPostCondition();
		virtual void exit();
		
	private:
		AxisVector posReady1, posReady2; 
		scara::ScaraControlSystem* controlSys;
		eeros::safety::SafetySystem* safetySys;
	};
}; 

#endif // CH_NTB_SCARA_SCARASEQUENCEHOMETOREADY_HPP_ 
