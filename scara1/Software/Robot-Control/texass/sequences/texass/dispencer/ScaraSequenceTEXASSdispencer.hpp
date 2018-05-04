#ifndef CH_NTB_SCARA_SCARASEQUENCETEXASSDISPENCER_HPP_
#define CH_NTB_SCARA_SCARASEQUENCETEXASSDISPENCER_HPP_

#include "../../../ScaraControlSystem.hpp"
#include "../../../ScaraSafetyProperties.hpp"
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/safety/SafetySystem.hpp>

#include "ScaraSequenceTEXASSdispenceProcess.hpp"
#include "../../tools/ScaraSequenceGoToToolChange.hpp"
#include "../../tools/ScaraSequenceCheckTool.hpp"
#include "../../goingToReady/ScaraSequenceGoToReady.hpp"

namespace scara{
	
	class ScaraSequenceTEXASSdispencer : public eeros::sequencer::Sequence<> {

	public:
		ScaraSequenceTEXASSdispencer(eeros::sequencer::Sequencer* sequencer, scara::ScaraControlSystem* controlSys, eeros::safety::SafetySystem* safetySys);
		
		virtual void init();
		virtual bool checkPreCondition();
		virtual void run();
		virtual bool checkPostCondition();
		virtual void exit();
	
	private:
		scara::ScaraControlSystem* 			controlSys;
		eeros::safety::SafetySystem* 		safetySys;
		
		ScaraSequenceGoToToolChange 		goToTool;
		ScaraSequenceGoToReady 				goToReady;
		ScaraSequenceCheckTool				checkTool;
		ScaraSequenceTEXASSdispenceProcess	dispenceProcess;
	};
};

#endif // CH_NTB_SCARA_SCARASEQUENCETEXASSDISPENCER_HPP_ 