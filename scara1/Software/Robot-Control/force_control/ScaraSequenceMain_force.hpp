#ifndef CH_NTB_SCARA_SCARASEQUENCEMAIN_FORCE_HPP_
#define CH_NTB_SCARA_SCARASEQUENCEMAIN_FORCE_HPP_

#include <eeros/sequencer/Sequence.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include "ScaraSafetyProperties_force.hpp"
#include "ScaraControlSystem_force.hpp"

#include "sequences/goingToReady/HomeToReady.hpp"
#include "sequences/goingToReady/GoToReady.hpp"
#include "sequences/moving/Move.hpp"
#include "sequences/moving/Move_Joystick.hpp"
#include "sequences/forceControl/ForceControl.hpp"


namespace scara{
	class ScaraSequenceMain_force : public eeros::sequencer::Sequence<void> {

	public:
		ScaraSequenceMain_force(eeros::sequencer::Sequencer* sequencer, scara::ScaraControlSystem_force* controlSys, eeros::safety::SafetySystem* safetySys, scara::ScaraSafetyProperties_force* safetyProp);
		
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
		ForceControl  forceControlSequence;
		
		scara::ScaraControlSystem_force* controlSys;
		eeros::safety::SafetySystem* safetySys;
		scara::ScaraSafetyProperties_force* safetyProp;
	};
};

#endif // CH_NTB_SCARA_SCARASEQUENCEMAIN_FORCE_HPP_ 