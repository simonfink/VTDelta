#ifndef CH_NTB_SCARA_SCARASEQUENCESAVEREFSYSTEMS_HPP_
#define CH_NTB_SCARA_SCARASEQUENCESAVEREFSYSTEMS_HPP_

#include <eeros/safety/SafetySystem.hpp>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include "../../ScaraControlSystem.hpp"
#include "../../ScaraSafetyProperties.hpp"
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/math/CoordinateSystem.hpp>

#include "ScaraSequenceCalculateRefSystem.hpp"
#include "../tools/ScaraSequenceCheckTool.hpp"
#include "../tools/ScaraSequenceGoToToolChange.hpp"
#include "../goingToReady/ScaraSequenceGoToReady.hpp"

namespace scara{
	class ScaraSequenceSaveRefSystems : public eeros::sequencer::Sequence<> {

	public:
		ScaraSequenceSaveRefSystems(eeros::sequencer::Sequencer* sequencer, scara::ScaraControlSystem* controlSys, eeros::safety::SafetySystem* safetySys);
		
		virtual void init();
		virtual bool checkPreCondition();
		virtual void run();
		virtual bool checkPostCondition();
		virtual void exit();
	
	private: 
		ScaraSequenceCalculateRefSystem calcRefSystem;
		ScaraSequenceCheckTool 			checkTool;
		ScaraSequenceGoToToolChange		goToTool;
		ScaraSequenceGoToReady			goToReady;
		
		
		scara::ScaraControlSystem* controlSys;
		eeros::safety::SafetySystem* safetySys;
	};
}; 

#endif // CH_NTB_SCARA_SCARASEQUENCESAVEREFSYSTEMS_HPP_ 
