
#ifndef CH_NTB_SCARA_SCARASEQUENCETEXASSDISPENCEPROCESS_HPP_
#define CH_NTB_SCARA_SCARASEQUENCETEXASSDISPENCEPROCESS_HPP_

#include "../../../ScaraControlSystem.hpp"
#include "../../../ScaraSafetyProperties.hpp"
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/safety/SafetySystem.hpp>

#include "ScaraSequenceTEXASSsingleDispence.hpp"

namespace scara{
	
	class ScaraSequenceTEXASSdispenceProcess : public eeros::sequencer::Sequence<void, eeros::math::Vector2> {

	public:
		ScaraSequenceTEXASSdispenceProcess(eeros::sequencer::Sequencer* sequencer, scara::ScaraControlSystem* controlSys, eeros::safety::SafetySystem* safetySys);
		
		virtual void init();
		virtual bool checkPreCondition();
		virtual void run(eeros::math::Vector2 shift_in);
		virtual bool checkPostCondition();
		virtual void exit();
	
	private:
		scara::ScaraControlSystem* 			controlSys;
		eeros::safety::SafetySystem* 		safetySys;
		
		ScaraSequenceTEXASSsingleDispence	singleDispence;
		
		double z_high = 0.055;
		double z_low = 0.002;
		AxisVector vel = 1.0;
		AxisVector p;
		AxisVector shift;
	};
};

#endif // CH_NTB_SCARA_SCARASEQUENCETEXASSDISPENCEPROCESS_HPP_  
