#ifndef CH_NTB_SCARA_SCARASAFETYPROPERTIES_HPP_
#define CH_NTB_SCARA_SCARASAFETYPROPERTIES_HPP_

#include <eeros/safety/SafetyProperties.hpp>
#include <eeros/hal/HAL.hpp>
#include <eeros/hal/ScalablePeripheralInput.hpp>

namespace scara{
			
		// Define all possible events
		enum {
			doSwInit = 10,
			swInitDone = 11,
			swShutDownDone = 12,
			doOff = 13,
			controlStoppingDone = 14,
			doBaseResetEmergency = 15,
			baseResetEmergencyDone = 16,
			goToBaseWaitingForApproval = 17,
			approvalIsOn = 18,
			doStopControl = 19,
			basePoweringDownDone = 20,
			doPoweringDown = 21,
			
			doManualParking = 30,
			manualParkingDone3 = 31, 
			manualParkingDone2 = 32, 
			manualParkingDone1 = 33,
			manualParkingDone0 = 34, 
			doHoming = 35,
			homingDone3 = 36,  
			homingDone2 = 37,      
			homingDone1 = 38,   
			homingDone0 = 39,      
			doRobotHomed = 40, 
			
			doControlStart = 50,
			controlStartingDone = 51,
			doResetEmergency = 52,
			resetEmergencyDone = 53,
			doSystemOn = 54,
			poweringDownDone = 55,
			doPoweringUp = 56,
			poweringUpDone = 57,
			goToReady = 58,
			isReady = 59,
			doTeaching = 60, 
			doEndTeaching = 61,
			doStartingMotion = 62,
			motionStartingDone = 63,
			doMotionStopping = 64,
			motionStoppingDone = 65,
			
			doAutoParkingBeforeShutdown = 70,
			autoParkingBeforeShutdownDone3 = 71,
			autoParkingBeforeShutdownDone2 = 72,
			autoParkingBeforeShutdownDone1 = 73,
			autoParkingBeforeShutdownDone0 = 74,

			doEmergency = 80, 
		};
		
		// Name all levels
		enum {
			off = 10,
			swShutingDown = 11,
			swInitializing = 12,
			swInitialized = 13,
			baseEmergency = 14,
			baseResetEmergency = 15,
			baseWaitingForApproval = 16,
			baseSystemOn = 17,
			basePoweringDown = 18,
			
			manualParking3 = 20,
			manualParking2 = 21,
			manualParking1 = 22,
			manualParking0 = 23,
			robotParked = 24,
			
			homing3 = 25,
			homing2 = 26,
			homing1 = 27,
			homing0 = 28,
			robotHomed = 29,
			
			controlStopping = 40,
			controlStarting = 41,	
			emergency = 42, 
			resetEmergency = 43,
			waitingForApproval = 44,
			systemOn = 45, 
			poweringDown = 46,	
			poweringUp = 47, 
			powerOn = 48,
			goingToReady = 49,
			ready = 50, 
			autoParkingBeforeShutdown3 = 51,
			autoParkingBeforeShutdown2 = 52,
			autoParkingBeforeShutdown1 = 53,
			autoParkingBeforeShutdown0 = 54,
			motionStopping = 60,
			motionStarting = 61,
			teaching = 62,
			moving = 63
		};
		
	class ScaraSafetyProperties : public eeros::safety::SafetyProperties {

		public:
			ScaraSafetyProperties();
			virtual ~ScaraSafetyProperties();
		
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
			
			double q3_init; 
			double q2r_init;
			double q1_init; 
			double q0_init;
	};
};
#endif // CH_NTB_SCARA_SCARASAFETYPROPERTIES_HPP_