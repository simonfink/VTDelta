#ifndef CH_NTB_SCARA_SCARASEQUENCETEXASSSINGLEPLASMAPOINT_HPP_
#define CH_NTB_SCARA_SCARASEQUENCETEXASSSINGLEPLASMAPOINT_HPP_

#include "../../../ScaraControlSystem.hpp"
#include "../../../ScaraSafetyProperties.hpp"
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/safety/SafetySystem.hpp>

namespace scara{
	
	class ScaraSequenceTEXASSsinglePlasmaPoint : public eeros::sequencer::Sequence<void, AxisVector> { //, int> {

	public:
		ScaraSequenceTEXASSsinglePlasmaPoint(eeros::sequencer::Sequencer* sequencer, scara::ScaraControlSystem* controlSys, eeros::safety::SafetySystem* safetySys);
		
		virtual void init();
		virtual bool checkPreCondition();
		virtual void run(AxisVector p_in); //, int mode);
		virtual bool checkPostCondition();
		virtual void exit();
	
	private:
		scara::ScaraControlSystem* controlSys;
		eeros::safety::SafetySystem* safetySys;
		
		AxisVector p;
		AxisVector vel_low = 0.0003;
		AxisVector vel_high = 1.0;
		
		int mode;
	};
};

#endif // CH_NTB_SCARA_SCARASEQUENCETEXASSSINGLEPLASMAPOINT_HPP_ 