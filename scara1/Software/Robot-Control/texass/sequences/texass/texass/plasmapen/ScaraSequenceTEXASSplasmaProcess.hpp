#ifndef CH_NTB_SCARA_SCARASEQUENCETEXASSPLASMAPROCESS_HPP_
#define CH_NTB_SCARA_SCARASEQUENCETEXASSPLASMAPROCESS_HPP_

#include "../../../ScaraControlSystem.hpp"
#include "../../../ScaraSafetyProperties.hpp"
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/safety/SafetySystem.hpp>

#include "ScaraSequenceTEXASSsinglePlasmaLine.hpp"
#include "ScaraSequenceTEXASSsinglePlasmaPoint.hpp"

#include "../../tools/ScaraSequenceGoToToolChange.hpp"
#include "../../tools/ScaraSequenceCheckTool.hpp"
#include "../../goingToReady/ScaraSequenceGoToReady.hpp"

namespace scara{
	
	class ScaraSequenceTEXASSplasmaProcess : public eeros::sequencer::Sequence<void, eeros::math::Vector2> {

	public:
		ScaraSequenceTEXASSplasmaProcess(eeros::sequencer::Sequencer* sequencer, scara::ScaraControlSystem* controlSys, eeros::safety::SafetySystem* safetySys);
		
		virtual void init();
		virtual bool checkPreCondition();
		
		virtual void run(eeros::math::Vector2 shift_in);
		
		virtual bool checkPostCondition();
		virtual void exit();
		
	private:
		scara::ScaraControlSystem* 			controlSys;
		eeros::safety::SafetySystem* 		safetySys;
		
		ScaraSequenceTEXASSsinglePlasmaLine	 	singlePlasmaLine;
		ScaraSequenceTEXASSsinglePlasmaPoint	singlePlasmaPoint;
		
		double z_high = 0.055;
		double z_low = 0.0003;
		AxisVector vel = 0.5;
		AxisVector p, p1, p2;
		
		AxisVector shift;
	};
};

#endif // CH_NTB_SCARA_SCARASEQUENCETEXASSPLASMAPROCESS_HPP_ 
