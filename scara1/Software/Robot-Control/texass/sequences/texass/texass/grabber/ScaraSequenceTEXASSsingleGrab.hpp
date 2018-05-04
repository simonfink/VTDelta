#ifndef CH_NTB_SCARA_SCARASEQUENCETEXASSSINGLEGRAB_HPP_
#define CH_NTB_SCARA_SCARASEQUENCETEXASSSINGLEGRAB_HPP_

#include "../../../ScaraControlSystem.hpp"
#include "../../../ScaraSafetyProperties.hpp"
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/safety/SafetySystem.hpp>

namespace scara{
	
	class ScaraSequenceTEXASSsingleGrab : public eeros::sequencer::Sequence<void, eeros::math::Vector2, eeros::math::Vector2, double, double, double> {

	public:
		ScaraSequenceTEXASSsingleGrab(eeros::sequencer::Sequencer* sequencer, scara::ScaraControlSystem* controlSys, eeros::safety::SafetySystem* safetySys);
		
		virtual void init();
		virtual bool checkPreCondition();
		virtual void run(eeros::math::Vector2 xy_parts, eeros::math::Vector2 xy_mesh, double z_high, double z_low_parts, double z_low_mesh);
		virtual bool checkPostCondition();
		virtual void exit();
	
	private:
		scara::ScaraControlSystem* controlSys;
		eeros::safety::SafetySystem* safetySys;
		
		AxisVector p;
		AxisVector vel_low = 0.5;
		AxisVector vel_high = 1.0;
	};
};

#endif // CH_NTB_SCARA_SCARASEQUENCETEXASSSINGLEGRAB_HPP_ 