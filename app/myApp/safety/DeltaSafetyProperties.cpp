#include "DeltaSafetyProperties.hpp"

#include <eeros/hal/HAL.hpp>
#include <eeros/safety/InputAction.hpp>
#include <eeros/safety/OutputAction.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/Sequence.hpp>

#include <unistd.h>
#include <iostream>
#include <vector>
#include <initializer_list>

using namespace eeros;
using namespace eeros::hal;
using namespace eeros::safety;
using namespace eeduro::delta;

DeltaSafetyProperties::DeltaSafetyProperties(DeltaControlSystem& controlSys, double td) : 
	controlSys(controlSys),
	
	// ### Events ###
	doOff("Do off"),
	doSwInit("Do software init"),
	swInitDone("Software initialized"),
	doEmergency("Do emergency"),
	doEmergencyReset("Do emergency reset"),
	emergencyResetDone("Emergency reset done"),
	doControlStart("Do control start"),
	doControlStop("Do control stop"),
	controlStoppingDone("Control stop done"),
	controlStartingDone("Control start done"),
	doPoweringUp("Do power up"),
	poweringUpDone("Power up done"),
	doPoweringDown("Do power down"),
	poweringDownDone("Power down done"),
	doHoming("Do homing"),
	doParking("Do parking"),
	parkingDone("Parking done"),
	homeingDone("Homing done"),
	doSystemReady("Do system ready"),
	doAutoMoving("Do auto moving"),
	doMouseTeaching("Do mouse teaching"),
	stopMoving("Stop moving"),
	doCalibrating("Do calibrating"),
	
	// ### Levels ###
	slOff("Off"),
	slSwInitializing("Software initializing"),
	slSwInitialized("Software initialized"),
	slEmergency("Emergency"),
	slResetingEmergency("Reseting emergency"),
      //waitingForApproval = 10,
	slControlStopping("Stop control"),
	slControlStarting("Start control"),
	slSystemOn("System on"),
	slPoweringDown("Powering down"),
	slPoweringUp("Powering up"),
	slPowerOn("Power on"),
	slHomeing("Homing"),
	slAxesHomed("Axes homed"),
	slParking("Parking"),
	slParked("Parked"),
	slSystemReady("System ready"),
	slAutoMoving("Auto moving"),
	slMouseTeaching("Mouse teaching"),
	slCalibrating("Calibrating"),
	tdn(td)
	{
	  
	HAL& hal = HAL::instance();

	// ############ Define critical outputs ############
	led = hal.getLogicOutput("onBoardRed");		 //equals ledGreen of extern Led (GPIO)
	//power = hal.getLogicOutput("ledGreen");
	errorLed = hal.getLogicOutput("ledRed");
	
	criticalOutputs = { led , errorLed};
	
	// ############ Define critical inputs ############
	emergencyStop = hal.getLogicInput("buttonRed");
	approval = hal.getLogicInput("buttonGreen");
	
	//startAgain = hal.getLogicInput("buttonGreen");
	
	criticalInputs = { emergencyStop, approval};//, startAgain};
	
	addLevel(slOff);
	addLevel(slEmergency);
	addLevel(slSwInitializing);
	addLevel(slSwInitialized);
	addLevel(slResetingEmergency);
	addLevel(slControlStopping);
	addLevel(slControlStarting);
	addLevel(slSystemOn);
	addLevel(slPoweringDown);
	addLevel(slPoweringUp);
	addLevel(slPowerOn);
	addLevel(slHomeing);
	addLevel(slAxesHomed);
	addLevel(slParked);
	addLevel(slParking);
	addLevel(slSystemReady);
	addLevel(slAutoMoving);
	addLevel(slMouseTeaching);
	addLevel(slCalibrating);
	
	//############Add events to the levels############
    slOff.addEvent(doSwInit, slSwInitializing, safety::kPublicEvent);
    slSwInitializing.addEvent(swInitDone, slSwInitialized, safety::kPrivateEvent);
    slSwInitialized.addEvent(doControlStart, slControlStarting, safety::kPublicEvent);
    slEmergency.addEvent(doEmergencyReset, slResetingEmergency, safety::kPublicEvent);
    slEmergency.addEvent(doSwInit, slSwInitializing, safety::kPublicEvent);
    slEmergency.addEvent(doCalibrating, slCalibrating, eeros::safety::kPublicEvent);
    
    slResetingEmergency.addEvent(emergencyResetDone, slSystemOn, safety::kPrivateEvent);
    slResetingEmergency.addEvent(doEmergency, slEmergency, safety::kPrivateEvent);
    slControlStopping.addEvent(controlStoppingDone, slOff, safety::kPublicEvent);
    slControlStarting.addEvent(controlStartingDone, slSystemOn, safety::kPrivateEvent);
    slSystemOn.addEvent(doControlStop, slControlStopping, safety::kPublicEvent);
    slSystemOn.addEvent(doPoweringUp,slPoweringUp,safety::kPublicEvent);

    slPoweringDown.addEvent(poweringDownDone,slSystemOn,safety::kPrivateEvent);
    slPoweringUp.addEvent(poweringUpDone,slPowerOn,safety::kPrivateEvent);
    slPowerOn.addEvent(doPoweringDown,slPoweringDown,safety::kPublicEvent);
    slPowerOn.addEvent(doHoming,slHomeing,safety::kPublicEvent);

    slHomeing.addEvent(homeingDone,slAxesHomed,safety::kPrivateEvent);
    slAxesHomed.addEvent(doSystemReady,slSystemReady,safety::kPrivateEvent);
    slParking.addEvent(parkingDone,slParked,safety::kPrivateEvent);
    slParked.addEvent(doSystemReady,slSystemReady,safety::kPublicEvent);

    slSystemReady.addEvent(doAutoMoving,slAutoMoving,safety::kPublicEvent);
    slSystemReady.addEvent(doMouseTeaching,slMouseTeaching,safety::kPublicEvent);
    slAutoMoving.addEvent(doMouseTeaching, slMouseTeaching, eeros::safety::kPublicEvent);
    slMouseTeaching.addEvent(doAutoMoving, slAutoMoving, safety::kPublicEvent);
    //slSystemReady.addEvent(doJoystickTeaching,slAutoMoving,safety::kPublicEvent);
    slSystemReady.addEvent(doParking,slParking,kPublicEvent);

    slAutoMoving.addEvent(stopMoving,slSystemReady,safety::kPublicEvent);
    slMouseTeaching.addEvent(stopMoving,slSystemReady,safety::kPublicEvent);

    
    slCalibrating.addEvent(doSystemReady, slSystemReady, eeros::safety::kPublicEvent);

    addEventToLevelAndAbove(slSystemOn,doEmergency,slEmergency,safety::kPublicEvent);

    //############Define input states and events for all levels############
    slOff.setInputActions({ignore(emergencyStop),ignore(approval)});//, ignore(startAgain)});
    slSwInitializing.setInputActions({ignore(emergencyStop),ignore(approval)});//, ignore(startAgain)});
    slSwInitialized.setInputActions({ignore(emergencyStop),ignore(approval)});//, ignore(startAgain)});
    //slEmergency.setInputActions({ignore(emergencyStop),check(approval,true,doEmergencyReset)});
    slEmergency.setInputActions({ignore(emergencyStop),check(approval, false, doSwInit)});//, check(startAgain,false,doControlStart)});
    slResetingEmergency.setInputActions({check(emergencyStop,true,doEmergency),ignore(approval)});//, ignore(startAgain)});
    //level(waitingForApproval).setInputActions({ignore(emergencyStop),check(approval,false,doControlStart),ignore(q0),ignore(q1),ignore(q2),ignore(q3)});
    slControlStopping.setInputActions({ignore(emergencyStop),ignore(approval)});//, ignore(startAgain)});
    slControlStarting.setInputActions({ignore(emergencyStop),ignore(approval)});//, ignore(startAgain)});
    slSystemOn.setInputActions({ignore(emergencyStop),check(approval,false,doPoweringUp)});//, ignore(startAgain)});
    slPoweringDown.setInputActions({check(emergencyStop,false,doEmergency),ignore(approval)});//, ignore(startAgain)});
    slPoweringUp.setInputActions({check(emergencyStop,false,doEmergency),ignore(approval)});//, ignore(startAgain)});
    slPowerOn.setInputActions({check(emergencyStop,false,doEmergency),ignore(approval)});//, ignore(startAgain)});
    slHomeing.setInputActions({check(emergencyStop,false,doEmergency),ignore(approval)});//, ignore(startAgain)});
    slAxesHomed.setInputActions({check(emergencyStop,false,doEmergency),ignore(approval)});//, ignore(startAgain)});
    slParking.setInputActions({check(emergencyStop,false,doEmergency),ignore(approval)});//, ignore(startAgain)});
    slParked.setInputActions({check(emergencyStop,false,doEmergency),ignore(approval)});//, ignore(startAgain)});
    slSystemReady.setInputActions({check(emergencyStop,false,doEmergency),ignore(approval)});//, ignore(startAgain)});
    slAutoMoving.setInputActions({check(emergencyStop,false,doEmergency),check(approval, false, doMouseTeaching)});//, ignore(startAgain)});
    slMouseTeaching.setInputActions({check(emergencyStop,false,doEmergency),ignore(approval)});//, ignore(startAgain)});
    slCalibrating.setInputActions({ignore(emergencyStop), check(approval, false, doSystemReady)});

    //############Define output states and events for all levels############
    slOff.setOutputActions({set(led,false), set(errorLed, false)});//, set(power,false)});
    slSwInitializing.setOutputActions({set(led,false), set(errorLed, false)});//, set(power,false)});
    slSwInitialized.setOutputActions({set(led,true), set(errorLed, false)});//, set(power,false)});
    slEmergency.setOutputActions({set(led,false), set(errorLed, true)});//, set(power,false)});
    slResetingEmergency.setOutputActions({set(led,true), set(errorLed, true)});//, set(power,false)});
    //level(waitingForApproval).setOutputActions({set(enable0,false),set(enable1,false),set(enable2,false),set(enable3,false),set(led,false)});
    slControlStopping.setOutputActions({set(led,false), set(errorLed, false)});//, set(power,false)});
    slControlStarting.setOutputActions({set(led,true), set(errorLed, false)});//, set(power,false)});
    slSystemOn.setOutputActions({set(led,false), set(errorLed, false)});//, set(power,false)});
    slPoweringDown.setOutputActions({set(led,false), set(errorLed, false)});//, set(power,false)});
    slPoweringUp.setOutputActions({set(led,true), set(errorLed, false)});//, set(power,false)});
    slPowerOn.setOutputActions({set(led,true), set(errorLed, false)});//, set(power,false)});
    slHomeing.setOutputActions({set(led,true), set(errorLed, false)});//, set(power,false)});
    slAxesHomed.setOutputActions({set(led,true), set(errorLed, false)});//, set(power,false)});
    slParking.setOutputActions({set(led,false), set(errorLed, false)});//, set(power,false)});
    slParked.setOutputActions({set(led,false), set(errorLed, false)});//, set(power,false)});
    slSystemReady.setOutputActions({set(led,true), set(errorLed, false)});//, set(power,false)});
    slAutoMoving.setOutputActions({set(led,true), set(errorLed, false)});//, set(power,false)});
    slMouseTeaching.setOutputActions({set(led,true), set(errorLed, false)});//, set(power,false)});
    slCalibrating.setOutputActions({set(led,true), set(errorLed,true)});

    //Defineandaddlevelfunctions
    slOff.setLevelAction([&](SafetyContext*privateContext){
      privateContext->triggerEvent(doSwInit);
    });

    slSwInitializing.setLevelAction([&](SafetyContext*privateContext){
      privateContext->triggerEvent(swInitDone);
    });

    slSwInitialized.setLevelAction([&](SafetyContext*privateContext){
      privateContext->triggerEvent(doControlStart);
    });

    slEmergency.setLevelAction([&](SafetyContext*privateContext){
      static int count=0;
      
       controlSys.voltageSetPoint.setValue({0.0,0.0,0.0,0.0});
       controlSys.voltageSwitch.switchToInput(1);
       controlSys.inputSwitch.switchToInput(1);
       controlSys.stop();
       
       eeros::sequencer::Sequencer::instance().abort();


      if(emergencyStop->get())
	count++;
      else
	count=0;

      if(count>static_cast<unsigned int>(2.0/tdn)){
	count = 0;
 	auto& sequencer = sequencer::Sequencer::instance();
  	sequencer.getSequenceByName("Main Sequence")->start();
 	controlSys.start();
	controlSys.inputSwitch.switchToInput(1);
	controlSys.voltageSwitch.switchToInput(0);
	privateContext->triggerEvent(doCalibrating);
      }

    });

    slResetingEmergency.setLevelAction([&](SafetyContext*privateContext){
	exit(2);
    });
    
    slCalibrating.setLevelAction([&](SafetyContext*privateContext){
//        std::cout << "mouse: " << controlSys.mouse.getOut().getSignal().getValue() << std::endl;
//        std::cout << "inputSwitch: " << controlSys.inputSwitch.getOut().getSignal().getValue() << std::endl;
//       std::cout << "enc4: " << controlSys.enc4.getOut().getSignal().getValue() << std::endl;
//        std::cout << "mot: " << controlSys.voltageSwitch.getOut().getSignal().getValue() << std::endl;
      controlSys.emagVal.setValue(controlSys.mouse.getButtonOut().getSignal().getValue()[0]);
//       std::cout << "mouse: " << controlSys.mouse.getOut().getSignal().getValue() << std::endl;
      
    });

    slControlStarting.setLevelAction([&](SafetyContext*privateContext){
       controlSys.start();
       controlSys.torqueLimitation.setLimit({-q012gearTorqueLimit,-q012gearTorqueLimit,-q012gearTorqueLimit,-q3gearTorqueLimit},{q012gearTorqueLimit,q012gearTorqueLimit,q012gearTorqueLimit,q3gearTorqueLimit});
      privateContext->triggerEvent(controlStartingDone);
    });

    slPoweringDown.setLevelAction([&](SafetyContext*privateContext){
     controlSys.disableAxis();
    privateContext->triggerEvent(poweringDownDone);
    });

    slSystemOn.setLevelAction([&](SafetyContext*privateContext){
//       controlSys.enableAxis();
      privateContext->triggerEvent(doPoweringUp);
    });
    
    slPoweringUp.setLevelAction([&](SafetyContext*privateContext){
     controlSys.enableAxis();
    privateContext->triggerEvent(poweringUpDone);
    });

    slPowerOn.setLevelAction([&](SafetyContext*privateContext){
       
       static unsigned int count=0;

      
      if(count==1){
	controlSys.homed=false;
	controlSys.voltageSwitch.switchToInput(1);
 	controlSys.setVoltageForInitializing({q012InitVoltage,q012InitVoltage,q012InitVoltage,q3InitVoltage});
      }      
      else if(count>static_cast<unsigned int>(1.8/tdn)){ // 2.0/td
 	if(controlSys.switchToPosControl()){
	  controlSys.enc1.callInputFeature<>("resetFqd");
	  controlSys.enc2.callInputFeature<>("resetFqd");
	  controlSys.enc3.callInputFeature<>("resetFqd");
	  controlSys.enc4.callInputFeature<>("resetFqd");
	  count = 0;
 	  privateContext->triggerEvent(doHoming);
 	}
	count=0;
      }
      count++;;
     
    });
    
    slHomeing.setLevelAction([&](SafetyContext*privateContext){
      static unsigned int count=0;
      static double mot1V = 0.0;
      static double mot2V = 0.0;
      static double mot3V = 0.0;
      static bool mot1Homed = false;
      static bool mot2Homed = false;
      static bool mot3Homed = false;

      
      double mot1 = controlSys.enc1.getOut().getSignal().getValue();
      double mot2 = controlSys.enc2.getOut().getSignal().getValue();
      double mot3 = controlSys.enc3.getOut().getSignal().getValue();

      if(count < (0.2/tdn)){
	controlSys.setVoltageForInitializing({-2,-2,-2,-7});
      }
      else{
	
	if(mot1 < -0.453785606 && mot1 > -0.47){
	  mot1V = 0;
	  mot1Homed = true;
	}else{
	  if(mot1 >= -0.453785606) mot1V = -2;
	  else mot1V = 2;
	  mot1Homed = false;
	}
	
	if(mot2 < -0.453785606 && mot2 > -0.47){
	  mot2V = 0;
	  mot2Homed = true;
	}else{
	  if(mot2 >= -0.453785606) mot2V = -2;
	  else mot2V = 2;
	  mot2Homed = false;
	}
	
	if(mot3 < -0.453785606 && mot3 > -0.47){
	  mot3V = 0;
	  mot3Homed = true;
	}else{
	  if(mot3 >= -0.453785606) mot3V = -2;
	  else mot3V = 2;
	  mot3Homed = false;
	}
	
	
	if(mot1Homed && mot2Homed && mot3Homed  && count > (2.0/tdn)){
	  controlSys.setVoltageForInitializing({0,0,0,0});
	  controlSys.enc1.callInputFeature<>("resetFqd");
	  controlSys.enc2.callInputFeature<>("resetFqd");
	  controlSys.enc3.callInputFeature<>("resetFqd");
// 	  controlSys.enc4.callInputFeature<>("resetFqd");
 	  controlSys.mouse.reset(0,0,0,0);
  	  controlSys.voltageSwitch.switchToInput(0);
	  controlSys.homed = true;
	  count=0;
	  privateContext->triggerEvent(homeingDone);
	  mot1Homed = false;
	  mot2Homed = false;
	  mot3Homed = false;
	}else{
	   controlSys.voltageSetPoint.setValue({mot1V, mot2V, mot3V, -5.0});
	}

      }
      count++;	
    });

    slAxesHomed.setLevelAction([&](SafetyContext*privateContext){
	  static int count = 0;
	  static bool mot4Homed = false;
	  double motV = 0.0;
	  double motEnc = controlSys.enc4.getOut().getSignal().getValue();
	  
	  
	  if(motEnc <= -2.8 && motEnc > -2.82){
	      mot4Homed = true;
	      motV = 0;
	  }else {
	      if(motEnc < -2.8) motV = 4.0;
	      else motV = -4.0;
	  }
	  
	  controlSys.voltageSetPoint.setValue({0.5,0.5,0.5,motV});
	  
	  if(mot4Homed){
	    mot4Homed = false;
	    controlSys.enc4.callInputFeature<>("resetFqd");
	    controlSys.setVoltageForInitializing({0.0,0.0,0.0,0.0});
 	    controlSys.start();
	    count = 0;
	    privateContext->triggerEvent(doSystemReady);
	  }
	  count++;

    });
    
    slSystemReady.setLevelAction([&](SafetyContext*privateContext){ 
// 	  eeros::sequencer::Sequencer::instance().getSequenceByName("Mouse Sequence")->start();
//    	  controlSys.inputSwitch.switchToInput(1);
//  	  controlSys.voltageSwitch.switchToInput(0);
       auto& sequencer = eeros::sequencer::Sequencer::instance();
       sequencer.abort();
       controlSys.inputSwitch.switchToInput(0);
       controlSys.voltageSwitch.switchToInput(0);
       
	
//  	if(sequencer.getSequenceByName("Main Sequence")->getRunningState() != eeros::sequencer::SequenceState::running){
//  	  std::cout << "slAutoMoving starting Main Sequence" << std::endl;
// 	sequencer.getSequenceByName("Main Sequence")->start();
	  
	    
//  	}
//  	  controlSys.emagVal.setValue(controlSys.mouse.getButtonOut().getSignal().getValue()[0]);
// 	  std::cout << "move: " << controlSys.inputSwitch.getOut().getSignal().getValue() << std::endl;
// 	  std::cout << "mouse: " << controlSys.mouse.getOut().getSignal().getValue() << std::endl;
// 	  controlSys.emagVal.setValue(false);
	privateContext->triggerEvent(doAutoMoving);

	
	

    });
    
    slAutoMoving.setLevelAction([&](SafetyContext*privateContext){
 	static auto& sequencer = eeros::sequencer::Sequencer::instance();
// 	
 	if(sequencer.getSequenceByName("Main Sequence")->getRunningState() != eeros::sequencer::SequenceState::running){
 	  sequencer.getSequenceByName("Main Sequence")->start();
	  
	}
	 
//       static std::string runningSeq = "Sort Sequence";
//       if(sequencer::Sequencer::instance().getSequenceByName("Sort Sequence")->getRunningState() == eeros::sequencer::SequenceState::running){
// 	runningSeq = "Sort Sequence";
//       }
//       if(sequencer::Sequencer::instance().getSequenceByName("Shuffle Sequence")->getRunningState() == eeros::sequencer::SequenceState::running){
// 	runningSeq = "Shuffle Sequence";
//       }
//       
//       if(sequencer::Sequencer::instance().getSequenceByName(runningSeq)->getRunningState() != eeros::sequencer::SequenceState::running){
// 	privateContext->triggerEvent(stopMoving);
//       }
      
      
//       auto& sequencer = sequencer::Sequencer::instance();
      
      //std::cout<< sequencer.getSequenceByName("Mouse Sequence")->getRunningState() << std::endl;
      
//       if(sequencer.getSequenceByName("Sort Sequence")->getRunningState() != eeros::sequencer::SequenceState::running){
// 	privateContext->triggerEvent(stopMoving);
//       }
    });
    
    slMouseTeaching.setLevelAction([&](SafetyContext*privateContext){
      
    });
    
    

    //Defineentrylevel
    setEntryLevel(slOff);
	
	exitFunction = ([&](SafetyContext* privateContext){
 		controlSys.voltageSetPoint.setValue({0.0,0.0,0.0,0.0});
 		controlSys.voltageSwitch.switchToInput(1);
		privateContext->triggerEvent(doOff);
	});
	
}

DeltaSafetyProperties::~DeltaSafetyProperties() { }