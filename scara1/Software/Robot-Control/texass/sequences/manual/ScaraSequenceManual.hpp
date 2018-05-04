#ifndef CH_NTB_SCARA_SCARASEQUENCEMANUAL_HPP_
#define CH_NTB_SCARA_SCARASEQUENCEMANUAL_HPP_

#include <eeros/safety/SafetySystem.hpp>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include "../../ScaraControlSystem.hpp"
#include "../../ScaraSafetyProperties.hpp"
#include <eeros/sequencer/Sequence.hpp>

#include "../goingToReady/ScaraSequenceGoToReady.hpp"

namespace scara{
	class ScaraSequenceManual : public eeros::sequencer::Sequence<> {

	public:
		ScaraSequenceManual(eeros::sequencer::Sequencer* sequencer, scara::ScaraControlSystem* controlSys, eeros::safety::SafetySystem* safetySys);
		
		virtual void init();
		virtual bool checkPreCondition();
		virtual void run();
		virtual bool checkPostCondition();
		virtual void exit();
	
	private: 
		ScaraSequenceGoToReady 		goToReady;
		
		scara::ScaraControlSystem* controlSys;
		eeros::safety::SafetySystem* safetySys;
	};
}; 

#endif // CH_NTB_SCARA_SCARASEQUENCEMANUAL_HPP_ 
