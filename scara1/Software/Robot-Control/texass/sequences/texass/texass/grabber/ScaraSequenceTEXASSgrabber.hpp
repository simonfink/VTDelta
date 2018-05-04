#ifndef CH_NTB_SCARA_SCARASEQUENCETEXASSGRABBER_HPP_
#define CH_NTB_SCARA_SCARASEQUENCETEXASSGRABBER_HPP_

#include "../../../ScaraControlSystem.hpp"
#include "../../../ScaraSafetyProperties.hpp"
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/safety/SafetySystem.hpp>

#include "ScaraSequenceTEXASSsingleGrab.hpp"
#include "ScaraSequenceTEXASSgrabProcess.hpp"
#include "../../tools/ScaraSequenceGoToToolChange.hpp"
#include "../../tools/ScaraSequenceCheckTool.hpp"
#include "../../goingToReady/ScaraSequenceGoToReady.hpp"

namespace scara{
	
	class ScaraSequenceTEXASSgrabber : public eeros::sequencer::Sequence<> {

	public:
		ScaraSequenceTEXASSgrabber(eeros::sequencer::Sequencer* sequencer, scara::ScaraControlSystem* controlSys, eeros::safety::SafetySystem* safetySys);
		
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
		ScaraSequenceTEXASSgrabProcess		grabProcess;
	};
};

#endif // CH_NTB_SCARA_SCARASEQUENCETEXASSGRABBER_HPP_ 
