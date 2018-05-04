#ifndef CH_NTB_SCARA_SCARASEQUENCETEXASS_HPP_
#define CH_NTB_SCARA_SCARASEQUENCETEXASS_HPP_

#include "../../ScaraControlSystem.hpp"
#include "../../ScaraSafetyProperties.hpp"
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/safety/SafetySystem.hpp>

#include "../tools/ScaraSequenceGoToToolChange.hpp"
#include "../goingToReady/ScaraSequenceGoToReady.hpp"
#include "../tools/ScaraSequenceCheckTool.hpp"
#include "plasmapen/ScaraSequenceTEXASSplasmapen.hpp"
#include "grabber/ScaraSequenceTEXASSgrabber.hpp"
#include "dispencer/ScaraSequenceTEXASSdispencer.hpp"

namespace scara{
	
	class ScaraSequenceTEXASS : public eeros::sequencer::Sequence<> {

	public:
		ScaraSequenceTEXASS(eeros::sequencer::Sequencer* sequencer, scara::ScaraControlSystem* controlSys, eeros::safety::SafetySystem* safetySys);
		
		virtual void init();
		virtual bool checkPreCondition();
		virtual void run();
		virtual bool checkPostCondition();
		virtual void exit();
		
	private:
		scara::ScaraControlSystem* 			controlSys;
		eeros::safety::SafetySystem* 		safetySys;
		
		ScaraSequenceGoToToolChange 		goToTool;
		ScaraSequenceGoToReady				goToReady;
		ScaraSequenceCheckTool				checkTool;
		ScaraSequenceTEXASSplasmapen  		plasmapen;
		ScaraSequenceTEXASSdispencer 		dispencer;
		ScaraSequenceTEXASSgrabber			grabber;
	};
};

#endif // CH_NTB_SCARA_SCARASEQUENCETEXASS_HPP_ 
