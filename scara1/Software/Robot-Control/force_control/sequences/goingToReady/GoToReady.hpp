#ifndef CH_NTB_SCARA_GOTOREADY_FORCE_HPP_
#define CH_NTB_SCARA_GOTOREADY_FORCE_HPP_

#include "../../ScaraControlSystem_force.hpp"
#include "../../ScaraSafetyProperties_force.hpp"
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include "../../../types.hpp"

namespace scara{
	class GoToReady : public eeros::sequencer::Sequence<> {

	public:
		GoToReady(eeros::sequencer::Sequencer* sequencer, scara::ScaraControlSystem_force* controlSys, eeros::safety::SafetySystem* safetySys, scara::ScaraSafetyProperties_force* safetyProp);

		virtual void init();
		virtual bool checkPreCondition();
		virtual void run();
		virtual bool checkPostCondition();
		virtual void exit();
		
	private:
		bool isEmergency();
		bool isTerminating();
		AxisVector posUp, posReady; 
		
		scara::ScaraControlSystem_force* controlSys;
		scara::ScaraSafetyProperties_force* safetyProp;
		eeros::safety::SafetySystem* safetySys;
	};
};

#endif // CH_NTB_SCARA_GOTOREADY_FORCE_HPP_ 