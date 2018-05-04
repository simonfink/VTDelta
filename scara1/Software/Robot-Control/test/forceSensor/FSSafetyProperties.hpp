#ifndef CH_NTB_SCARA_FS_SAFETYPROPERTIES_HPP_
#define CH_NTB_SCARA_FS_SAFETYPROPERTIES_HPP_

#include <eeros/safety/SafetyProperties.hpp>
#include <eeros/safety/SafetyLevel.hpp>
#include <eeros/hal/HAL.hpp>
#include <eeros/hal/ScalablePeripheralInput.hpp>

namespace scara{
			
	class FSControlSystem;
		
		
	class FSSafetyProperties : public eeros::safety::SafetyProperties {

		public:
		
			FSSafetyProperties(FSControlSystem* cs);
			virtual ~FSSafetyProperties();
		
			// critical outputs			
			eeros::hal::PeripheralOutput<bool>* watchdog;
			eeros::hal::PeripheralOutput<bool>* enable0;
			eeros::hal::PeripheralOutput<bool>* enable1;
			eeros::hal::PeripheralOutput<bool>* enable2;
			eeros::hal::PeripheralOutput<bool>* enable3;
			eeros::hal::PeripheralOutput<bool>* brake0;
			eeros::hal::PeripheralOutput<bool>* brake1;
			eeros::hal::PeripheralOutput<bool>* brake2;
			eeros::hal::PeripheralOutput<bool>* brake3;
			
			// critical inputs
			eeros::hal::PeripheralInput<bool>* approval;
			eeros::hal::ScalablePeripheralInput<double>* q0;
			eeros::hal::ScalablePeripheralInput<double>* q1;
			eeros::hal::ScalablePeripheralInput<double>* q2r;
			eeros::hal::ScalablePeripheralInput<double>* q3;
			eeros::hal::PeripheralInput<bool>* limitSwitchQ0p;
			eeros::hal::PeripheralInput<bool>* limitSwitchQ0n;
			eeros::hal::PeripheralInput<bool>* limitSwitchQ1p;
			eeros::hal::PeripheralInput<bool>* limitSwitchQ1n;
			eeros::hal::PeripheralInput<bool>* limitSwitchQ2p;
			eeros::hal::PeripheralInput<bool>* limitSwitchQ2n;
			eeros::hal::PeripheralInput<bool>* limitSwitchQ3p;
			eeros::hal::PeripheralInput<bool>* limitSwitchQ3n;
			
		private: 
			eeros::safety::SafetyLevel off;
			eeros::safety::SafetyLevel waitForApproval;
			eeros::safety::SafetyLevel emergency;
			eeros::safety::SafetyLevel approvalOn;
			eeros::safety::SafetyLevel on;
			
			eeros::safety::SafetyEvent startSW;
			eeros::safety::SafetyEvent doEmergency;
			eeros::safety::SafetyEvent doApproval;
			eeros::safety::SafetyEvent doOn;
			eeros::safety::SafetyEvent doOff;

			FSControlSystem* controlSys;
			
			double q3_init; 
			double q2r_init;
			double q1_init; 
			double q0_init;
	};
};
#endif // CH_NTB_SCARA_FS_SAFETYPROPERTIES_HPP_