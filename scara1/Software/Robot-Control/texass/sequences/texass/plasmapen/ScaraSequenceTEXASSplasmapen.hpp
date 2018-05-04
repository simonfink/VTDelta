#ifndef CH_NTB_SCARA_SCARASEQUENCETEXASSPLASMAPEN_HPP_
#define CH_NTB_SCARA_SCARASEQUENCETEXASSPLASMAPEN_HPP_

#include "../../../ScaraControlSystem.hpp"
#include "../../../ScaraSafetyProperties.hpp"
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/safety/SafetySystem.hpp>

#include "ScaraSequenceTEXASSsinglePlasmaLine.hpp"
#include "ScaraSequenceTEXASSsinglePlasmaPoint.hpp"

#include "../../tools/ScaraSequenceGoToToolChange.hpp"
#include "../../tools/ScaraSequenceCheckTool.hpp"
#include "../../goingToReady/ScaraSequenceGoToReady.hpp"
#include "ScaraSequenceTEXASSplasmaProcess.hpp"

namespace scara{
	
	class ScaraSequenceTEXASSplasmapen : public eeros::sequencer::Sequence<void> {

	public:
		ScaraSequenceTEXASSplasmapen(eeros::sequencer::Sequencer* sequencer, scara::ScaraControlSystem* controlSys, eeros::safety::SafetySystem* safetySys);
		
		virtual void init();
		virtual bool checkPreCondition();
		
		virtual void run();
		
		virtual bool checkPostCondition();
		virtual void exit();
		
	private:
		scara::ScaraControlSystem* 			controlSys;
		eeros::safety::SafetySystem* 		safetySys;
		
		ScaraSequenceGoToToolChange 			goToTool;
		ScaraSequenceGoToReady 					goToReady;
		ScaraSequenceCheckTool					checkTool;
		ScaraSequenceTEXASSplasmaProcess		plasmaProcess;
	};
};

#endif // CH_NTB_SCARA_SCARASEQUENCETEXASSPLASMAPEN_HPP_ 
