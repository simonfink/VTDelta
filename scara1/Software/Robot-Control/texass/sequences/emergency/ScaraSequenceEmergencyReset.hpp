#ifndef CH_NTB_SCARA_SCARASEQUENCEEMERGENCYRESET_HPP_
#define CH_NTB_SCARA_SCARASEQUENCEEMERGENCYRESET_HPP_

#include "../../ScaraControlSystem.hpp"
#include "../../ScaraSafetyProperties.hpp"
#include "../../ScaraSequence.hpp"
#include <eeros/safety/SafetySystem.hpp>

namespace scara{
	class ScaraSequenceEmergencyReset : public ScaraSequence {

	public:
		ScaraSequenceEmergencyReset(std::string name, SafetySystem& safetySys, ScaraControlSystem& controlSys);
		
		virtual bool checkPreCondition();
		virtual bool checkPostCondition();
		
		virtual void init();
		virtual void exit();
	};
};

#endif // CH_NTB_SCARA_SCARASEQUENCEEMERGENCYRESET_HPP_ 
