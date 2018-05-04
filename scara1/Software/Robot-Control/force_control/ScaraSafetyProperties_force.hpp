#ifndef CH_NTB_SCARA_SCARASAFETYPROPERTIES_FORCE_HPP_
#define CH_NTB_SCARA_SCARASAFETYPROPERTIES_FORCE_HPP_

#include <eeros/safety/SafetyProperties.hpp>
#include <eeros/safety/SafetyLevel.hpp>
#include <eeros/hal/HAL.hpp>
#include <eeros/hal/ScalableInput.hpp>

namespace scara{
			
	class ScaraControlSystem_force;

	class ScaraSafetyProperties_force : public eeros::safety::SafetyProperties {

		public:
		
			ScaraSafetyProperties_force();
			virtual ~ScaraSafetyProperties_force();
		
			// critical outputs
			eeros::hal::Output<bool>* watchdog;
			eeros::hal::Output<bool>* enable0;
			eeros::hal::Output<bool>* enable1;
			eeros::hal::Output<bool>* enable2;
			eeros::hal::Output<bool>* enable3;
			eeros::hal::Output<bool>* brake0;
			eeros::hal::Output<bool>* brake1;
			eeros::hal::Output<bool>* brake2;
			eeros::hal::Output<bool>* brake3;
			
			eeros::hal::Output<bool>* safeUserLight;
			
			// critical inputs
			eeros::hal::Input<bool>* approval;
			eeros::hal::Input<bool>* limitSwitchQ0p;
			eeros::hal::Input<bool>* limitSwitchQ0n;
			eeros::hal::Input<bool>* limitSwitchQ1p;
			eeros::hal::Input<bool>* limitSwitchQ1n;
			eeros::hal::Input<bool>* limitSwitchQ2p;
			eeros::hal::Input<bool>* limitSwitchQ2n;
			eeros::hal::Input<bool>* limitSwitchQ3p;
			eeros::hal::Input<bool>* limitSwitchQ3n;
			
			eeros::hal::Input<bool>* safeUserButton;
			
			// safety levels
			eeros::safety::SafetyLevel off;
			eeros::safety::SafetyLevel notinit_emergency;
			eeros::safety::SafetyLevel notinit_waitingForApproval;
			eeros::safety::SafetyLevel notinit_systemOn;

			eeros::safety::SafetyLevel manualParking3;
			eeros::safety::SafetyLevel manualParking2;
			eeros::safety::SafetyLevel manualParking1;
			eeros::safety::SafetyLevel manualParking0;
			eeros::safety::SafetyLevel robotParked;

			eeros::safety::SafetyLevel homing3;
			eeros::safety::SafetyLevel homing2;
			eeros::safety::SafetyLevel homing1;
			eeros::safety::SafetyLevel homing0;
			eeros::safety::SafetyLevel robotHomed;

			eeros::safety::SafetyLevel emergency; 
			eeros::safety::SafetyLevel resetEmergency;
			eeros::safety::SafetyLevel waitingForApproval;
			
			eeros::safety::SafetyLevel set_autoParking;
			eeros::safety::SafetyLevel autoParking_shutdown3;
			eeros::safety::SafetyLevel autoParking_shutdown2;
			eeros::safety::SafetyLevel autoParking_shutdown1;
			eeros::safety::SafetyLevel autoParking_shutdown0;
			
			eeros::safety::SafetyLevel systemOn; 
			eeros::safety::SafetyLevel powerOn;
			eeros::safety::SafetyLevel goingToReady;
			eeros::safety::SafetyLevel ready;
			eeros::safety::SafetyLevel set_moving;
			eeros::safety::SafetyLevel moving;
			eeros::safety::SafetyLevel set_moving_joystick;
			eeros::safety::SafetyLevel moving_joystick;
			eeros::safety::SafetyLevel set_force_control;
			eeros::safety::SafetyLevel force_control;
			
			// safety events
			eeros::safety::SafetyEvent notinit_goToWaitingForApproval;
			eeros::safety::SafetyEvent doOff;
			eeros::safety::SafetyEvent approvalIsOn;

			eeros::safety::SafetyEvent doManualParking;
			eeros::safety::SafetyEvent manualParkingDone3;
			eeros::safety::SafetyEvent manualParkingDone2;
			eeros::safety::SafetyEvent manualParkingDone1;
			eeros::safety::SafetyEvent manualParkingDone0;
			
			eeros::safety::SafetyEvent doHoming;
			eeros::safety::SafetyEvent homingDone3;  
			eeros::safety::SafetyEvent homingDone2;      
			eeros::safety::SafetyEvent homingDone1;   
			eeros::safety::SafetyEvent homingDone0;      

			eeros::safety::SafetyEvent doSystemOn;
			eeros::safety::SafetyEvent doPowerUp;
			eeros::safety::SafetyEvent doPowerDown;
			eeros::safety::SafetyEvent goToReady;
			eeros::safety::SafetyEvent isReady;
			
			eeros::safety::SafetyEvent doEmergency;
			eeros::safety::SafetyEvent doResetEmergency;
			eeros::safety::SafetyEvent resetEmergencyDone;
			eeros::safety::SafetyEvent doEmergency_posOutRange;
			eeros::safety::SafetyEvent doEmergency_velOutRange;
			
			eeros::safety::SafetyEvent doSetMoving;
			eeros::safety::SafetyEvent doStartMoving;
			eeros::safety::SafetyEvent doStopMoving;
			
			eeros::safety::SafetyEvent doSetMoving_joystick;
			eeros::safety::SafetyEvent doStartMoving_joystick;
			eeros::safety::SafetyEvent doStopMoving_joystick;
			
			eeros::safety::SafetyEvent doSetForce_control;
			eeros::safety::SafetyEvent doStartForce_control;
			eeros::safety::SafetyEvent doStopForce_control;

			eeros::safety::SafetyEvent doAutoParking_shutdown;
			eeros::safety::SafetyEvent doStartAutoParking;
			eeros::safety::SafetyEvent autoParking_shutdownDone3;
			eeros::safety::SafetyEvent autoParking_shutdownDone2;
			eeros::safety::SafetyEvent autoParking_shutdownDone1;
			eeros::safety::SafetyEvent autoParking_shutdownDone0;

			ScaraControlSystem_force* controlSys;
		
		private: 
// 			void reset_all_checkers();
			
			double q3_init; 
			double q2r_init;
			double q1_init; 
			double q0_init;
	};
};
#endif // CH_NTB_SCARA_SCARASAFETYPROPERTIES_FORCE_HPP_