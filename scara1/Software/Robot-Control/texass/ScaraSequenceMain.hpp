#ifndef CH_NTB_SCARA_SCARASEQUENCEMAIN_HPP_
#define CH_NTB_SCARA_SCARASEQUENCEMAIN_HPP_

#include <eeros/sequencer/Sequence.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include "ScaraControlSystem.hpp"

#include "sequences/parking/ScaraSequenceManualPark.hpp"
#include "sequences/homing/ScaraSequenceHoming.hpp"
#include "sequences/goingToReady/ScaraSequenceHomeToReady.hpp"
#include "sequences/referenceSystems/ScaraSequenceLoadRefSystem.hpp"
#include "sequences/workspace/ScaraSequenceLoadWorkspaceLimitation.hpp"
#include "sequences/tools/ScaraSequenceGoToToolChange.hpp"
#include "sequences/goingToReady/ScaraSequenceGoToReady.hpp"
#include "sequences/referenceSystems/ScaraSequenceSaveRefSystems.hpp"
#include "sequences/manual/ScaraSequenceManual.hpp"
#include "sequences/calibration/ScaraSequenceSaveCalibratedPartsStorage.hpp"
#include "sequences/calibration/ScaraSequenceSaveCalibratedMesh.hpp"
#include "sequences/tools/ScaraSequenceCheckTool.hpp"

#include "sequences/texass/dispencer/ScaraSequenceTEXASSdispencer.hpp"
#include "sequences/texass/grabber/ScaraSequenceTEXASSgrabber.hpp"
#include "sequences/texass/plasmapen/ScaraSequenceTEXASSplasmapen.hpp"
#include "sequences/texass/ScaraSequenceTEXASS.hpp"

#include "sequences/ScaraSequenceMove.hpp"
#include "sequences/parking/ScaraSequenceAutoPark.hpp"

namespace scara{
	class ScaraSequenceMain : public eeros::sequencer::Sequence<void> {

	public:
		ScaraSequenceMain(eeros::sequencer::Sequencer* sequencer, scara::ScaraControlSystem* controlSys, eeros::safety::SafetySystem* safetySys);
		
		virtual bool checkPreCondition();
		virtual void run();
		virtual void exit();
		
	private:
		ScaraSequenceManualPark 					manualPark;
		ScaraSequenceHoming 						homing;
		ScaraSequenceHomeToReady 					homeToReady;
		ScaraSequenceLoadRefSystem 					loadRefSystems;
		ScaraSequenceLoadWorkspaceLimitation		loadWorkspaceLimitation;
		ScaraSequenceGoToToolChange					goToTool;
		ScaraSequenceGoToReady 						goToReady;
		ScaraSequenceSaveRefSystems					saveRefSystem;
		ScaraSequenceManual 						joystickMove;
		ScaraSequenceSaveCalibratedPartsStorage		calibratePartsStorage;
		ScaraSequenceSaveCalibratedMesh 			calibrateMesh;
		ScaraSequenceCheckTool						checkTool;
		ScaraSequenceTEXASSdispencer    			dispencer;
		ScaraSequenceTEXASSgrabber    				grabber;
		ScaraSequenceTEXASSplasmapen   				plasmapen;
		ScaraSequenceTEXASS 						texass;
		
		ScaraSequenceMove 							move;
		ScaraSequenceAutoPark 						autopark;
		
		scara::ScaraControlSystem* controlSys;
		eeros::safety::SafetySystem* safetySys;
	};
};

#endif // CH_NTB_SCARA_SCARASEQUENCEMAIN_HPP_ 