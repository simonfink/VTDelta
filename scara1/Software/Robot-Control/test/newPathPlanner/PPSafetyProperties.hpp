#ifndef CH_NTB_SCARA_PP_SAFETYPROPERTIES_HPP_
#define CH_NTB_SCARA_PP_SAFETYPROPERTIES_HPP_

#include <eeros/safety/SafetyProperties.hpp>
#include <eeros/hal/HAL.hpp>
#include <eeros/hal/ScalablePeripheralInput.hpp>

namespace scara{
			
		class PPControlSystem;
		
	class PPSafetyProperties : public eeros::safety::SafetyProperties {

		public:
		
			PPSafetyProperties(PPControlSystem* cs);
			virtual ~PPSafetyProperties();
		
			// critical outputs
			// critical inputs
			
		private: 
			eeros::safety::SafetyLevel off;
			eeros::safety::SafetyLevel on;
			eeros::safety::SafetyEvent doOn;
			eeros::safety::SafetyEvent doOff;
			
			PPControlSystem* controlSys;
			
			double q3_init; 
			double q2r_init;
			double q1_init; 
			double q0_init;
	};
};
#endif // CH_NTB_SCARA_PP_SAFETYPROPERTIES_HPP_