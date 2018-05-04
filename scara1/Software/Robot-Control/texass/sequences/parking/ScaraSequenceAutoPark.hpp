#ifndef CH_NTB_SCARA_SCARASEQUENCEAUTOPARK_HPP_
#define CH_NTB_SCARA_SCARASEQUENCEAUTOPARK_HPP_

#include "../../ScaraControlSystem.hpp"
#include "../../ScaraSafetyProperties.hpp"
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/safety/SafetySystem.hpp>

namespace scara{
	class ScaraSequenceAutoPark : public eeros::sequencer::Sequence<> {

	public:
		ScaraSequenceAutoPark(eeros::sequencer::Sequencer* sequencer, scara::ScaraControlSystem* controlSys, eeros::safety::SafetySystem* safetySys);
		
		virtual void init();
		virtual bool checkPreCondition();
		virtual void run();
		virtual bool checkPostCondition();
		virtual void exit();
		
	private:
		AxisVector qPark;
		scara::ScaraControlSystem* controlSys;
		eeros::safety::SafetySystem* safetySys;
	};
};

#endif // CH_NTB_SCARA_SCARASEQUENCEAUTOPARK_HPP_ 