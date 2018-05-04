#ifndef CH_NTB_PARALLELSCARA_SAFETYPROPERTIES_HPP_
#define CH_NTB_PARALLELSCARA_SAFETYPROPERTIES_HPP_

#include <eeros/safety/SafetyProperties.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/hal/HAL.hpp>

namespace parallelscara{
	
	class ParallelScaraControlSystem;
	
	class ParallelScaraSafetyProperties : public eeros::safety::SafetyProperties {
	public:
		ParallelScaraSafetyProperties();
		virtual ~ParallelScaraSafetyProperties();
		
		//  outputs
		eeros::hal::Output<bool>* enable;	// motor enable, wdt must be set as well to be active
		eeros::hal::Output<bool>* testToggle;	// toggle spare pin to check for running safety system
		eeros::hal::Output<bool>* blueLedBalancingButton; 
		eeros::hal::Output<bool>* redLedStopButton;       
		eeros::hal::Output<bool>* greenLedHighSpeedButton;
		eeros::hal::Output<bool>* redLedResetButton;   
		
		//  inputs
		eeros::hal::Input<bool>* balancingButton;
		eeros::hal::Input<bool>* highSpeedButton;
		eeros::hal::Input<bool>* stopButton;     
		eeros::hal::Input<bool>* resetButton; 
		eeros::hal::Input<bool>* emergencyButton;
			
		eeros::safety::SafetyLevel off;
		eeros::safety::SafetyLevel shuttingDown;
		eeros::safety::SafetyLevel swInitializing;
		eeros::safety::SafetyLevel emergency ;
		eeros::safety::SafetyLevel resetEmergency;
		eeros::safety::SafetyLevel systemOn;				// control systems runs, motor disable
		eeros::safety::SafetyLevel poweringDown;
		eeros::safety::SafetyLevel waitForMotionStop;		// wait for motion to stop
		eeros::safety::SafetyLevel powerOn;					// power is on
		eeros::safety::SafetyLevel homing;
		eeros::safety::SafetyLevel homed;
		eeros::safety::SafetyLevel readying;		// system moves to middle position
		eeros::safety::SafetyLevel systemReady;				// system is homed and ready to run
		eeros::safety::SafetyLevel balancing;
		eeros::safety::SafetyLevel highSpeed;
		eeros::safety::SafetyLevel stopMoving;
		
		eeros::safety::SafetyEvent switchOff;			
		eeros::safety::SafetyEvent shutDown;
		eeros::safety::SafetyEvent initSw;
		eeros::safety::SafetyEvent initSwDone;
		eeros::safety::SafetyEvent doEmergency;
		eeros::safety::SafetyEvent doEmergencyReset;
		eeros::safety::SafetyEvent emergencyResetDone;
		eeros::safety::SafetyEvent doPowerUp;
		eeros::safety::SafetyEvent startPoweringDown;
		eeros::safety::SafetyEvent startHoming;
		eeros::safety::SafetyEvent homingDone;
		eeros::safety::SafetyEvent doReady;
		eeros::safety::SafetyEvent readyDone;
		eeros::safety::SafetyEvent startBalancing;
		eeros::safety::SafetyEvent startHighSpeed;
		eeros::safety::SafetyEvent stop;
		eeros::safety::SafetyEvent abort;
		
		ParallelScaraControlSystem* controlSys;
	private:
		double err = 0.00001;
		bool robotHomed; 
		eeros::logger::Logger log;
	};
};
#endif // CH_NTB_PARALLELSCARA_SAFETYPROPERTIES_HPP_
