#ifndef CH_NTB_SCARA_SCARASEQUENCETEXASSGRABPROCESS_HPP_
#define CH_NTB_SCARA_SCARASEQUENCETEXASSGRABPROCESS_HPP_

#include "../../../ScaraControlSystem.hpp"
#include "../../../ScaraSafetyProperties.hpp"
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/safety/SafetySystem.hpp>

#include "ScaraSequenceTEXASSsingleGrab.hpp"
#include "../../tools/ScaraSequenceGoToToolChange.hpp"
#include "../../tools/ScaraSequenceCheckTool.hpp"
#include "../../goingToReady/ScaraSequenceGoToReady.hpp"

namespace scara{
	
	class ScaraSequenceTEXASSgrabProcess : public eeros::sequencer::Sequence<void, eeros::math::Vector2, eeros::math::Vector2> {

	public:
		ScaraSequenceTEXASSgrabProcess(eeros::sequencer::Sequencer* sequencer, scara::ScaraControlSystem* controlSys, eeros::safety::SafetySystem* safetySys);
		
		virtual void init();
		virtual bool checkPreCondition();
		virtual void run(eeros::math::Vector2 shift_parts_in, eeros::math::Vector2 shift_mesh_in);
		virtual bool checkPostCondition();
		virtual void exit();
	
	private:
		scara::ScaraControlSystem* 			controlSys;
		eeros::safety::SafetySystem* 		safetySys;
		
		ScaraSequenceTEXASSsingleGrab   	singleGrab;
		
		AxisVector vel = 1.0;
		double z_high = 0.055;
		double z_low_mesh = 0.003;
		double z_low_parts = 0.000;
		eeros::math::Vector2 xy_parts, xy_mesh;
		eeros::math::Vector2 shift_parts, shift_mesh;
	};
};

#endif // CH_NTB_SCARA_SCARASEQUENCETEXASSGRABPROCESS_HPP_ 
