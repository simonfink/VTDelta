#ifndef CH_NTB_SCARA_SCARASEQUENCEMAIN_HPP_
#define CH_NTB_SCARA_SCARASEQUENCEMAIN_HPP_

#include <eeros/sequencer/Sequence.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include "ScaraSafetyProperties.hpp"
#include "ScaraControlSystem.hpp"

#include "sequences/goingToReady/HomeToReady.hpp"
#include "sequences/goingToReady/GoToReady.hpp"
#include "sequences/moving/Move.hpp"
#include "sequences/moving/Move_Joystick.hpp"

namespace scara{
	class ScaraSequenceMain : public eeros::sequencer::Sequence<void> {

	public:
		ScaraSequenceMain(eeros::sequencer::Sequencer* sequencer, scara::ScaraControlSystem* controlSys, eeros::safety::SafetySystem* safetySys, scara::ScaraSafetyProperties* safetyProp);
		
		virtual bool checkPreCondition();
		virtual void run();
		virtual void exit();
		
	private:
		bool isTerminating();
		bool isEmergency();
		
		HomeToReady   homeToReady;
		GoToReady     goToReady;
		Move          moveSequence;  
		Move_Joystick moveSequenceJoystick;  
		
		scara::ScaraControlSystem* controlSys;
		eeros::safety::SafetySystem* safetySys;
		scara::ScaraSafetyProperties* safetyProp;
	};
};

#endif // CH_NTB_SCARA_SCARASEQUENCEMAIN_HPP_ 