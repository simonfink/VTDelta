#include "ScaraSafetyProperties.hpp"
#include "ScaraControlSystem.hpp"
#include <eeros/hal/HAL.hpp>
#include <eeros/safety/InputAction.hpp>
#include <eeros/safety/inputActions.hpp>
#include <eeros/safety/OutputAction.hpp>

#include <unistd.h>
#include <iostream>
#include <vector>
#include <initializer_list>

using namespace scara;
using namespace eeros;
using namespace eeros::hal;
using namespace eeros::safety;

ScaraSafetyProperties::ScaraSafetyProperties() {

	HAL& hal = HAL::instance();

	// ############ Define critical outputs ############
	watchdog = hal.getLogicPeripheralOutput("watchdog");
	enable0 = hal.getLogicPeripheralOutput("enable0");
	enable1 = hal.getLogicPeripheralOutput("enable1");
	enable2 = hal.getLogicPeripheralOutput("enable2");
	enable3 = hal.getLogicPeripheralOutput("enable3");
	brake0 = hal.getLogicPeripheralOutput("brake0");
	brake1 = hal.getLogicPeripheralOutput("brake1");
	brake2 = hal.getLogicPeripheralOutput("brake2");
	brake3 = hal.getLogicPeripheralOutput("brake3");
	
	criticalOutputs = { watchdog, enable0, enable1, enable2, enable3, brake0, brake1, brake2, brake3 };
	
	// ############ Define critical inputs ############
	approval = hal.getLogicPeripheralInput("approval");
	q0  = dynamic_cast<ScalablePeripheralInput<double>*>(hal.getRealPeripheralInput("q0"));
	q1  = dynamic_cast<ScalablePeripheralInput<double>*>(hal.getRealPeripheralInput("q1"));
	q2r = dynamic_cast<ScalablePeripheralInput<double>*>(hal.getRealPeripheralInput("q2r"));
	q3  = dynamic_cast<ScalablePeripheralInput<double>*>(hal.getRealPeripheralInput("q3"));
	limitSwitchQ0p = hal.getLogicPeripheralInput("limitSwitchQ0p");
	limitSwitchQ0n = hal.getLogicPeripheralInput("limitSwitchQ0n");
	limitSwitchQ1p = hal.getLogicPeripheralInput("limitSwitchQ1p");
	limitSwitchQ1n = hal.getLogicPeripheralInput("limitSwitchQ1n");
	limitSwitchQ2p = hal.getLogicPeripheralInput("limitSwitchQ2p");
	limitSwitchQ2n = hal.getLogicPeripheralInput("limitSwitchQ2n");
	limitSwitchQ3p = hal.getLogicPeripheralInput("limitSwitchQ3p");
	limitSwitchQ3n = hal.getLogicPeripheralInput("limitSwitchQ3n");
		
	criticalInputs = { approval, q0, q1, q2r, q3, limitSwitchQ0p, limitSwitchQ0n, limitSwitchQ1p, limitSwitchQ1n, limitSwitchQ2p, limitSwitchQ2n, limitSwitchQ3p, limitSwitchQ3n };
	
	// ############ Define Levels ############
	levels = {
			{ off,                        "SW off",                                       },
			{ swShutingDown,              "SW shuting down",                              },
			{ swInitializing,             "SW starting",                                  },
			{ swInitialized,              "SW initialized",                               },
			{ baseEmergency,              "Emergency NOT INIT",                           },
			{ baseResetEmergency,         "Emergency reset NOT INIT",                     },
			{ baseWaitingForApproval,     "Waiting for approval, press WHITE button",     },
			{ baseSystemOn,               "System on NOT INIT",                           },
			{ basePoweringDown,           "Power switch down NOT INIT",                   },
			{ manualParking3,             "Manual parking q3",                            },
			{ manualParking2,             "Manual parking q2",                            },
			{ manualParking1,             "Manual parking q1",                            },
			{ manualParking0,             "Manual parking q0",                            },
			{ robotParked,                "Robot is in park position, press GREEN button" },
			{ homing3,                    "Homing q3",                                    },
			{ homing2,                    "Homing q2",                                    },
			{ homing1,                    "Homing q1",                                    },
			{ homing0,                    "Homing q0",                                    },
			{ robotHomed,                 "Robot homed",                                  },
			{ controlStopping,            "Control stop",                                 },
			{ controlStarting,            "Control start",                                },
			{ emergency,                  "Emergency",                                    },
			{ resetEmergency,             "Emergency reset",                              },
			{ waitingForApproval,         "Waiting for approval",                         },
			{ systemOn,                   "System on",                                    },
			{ poweringDown,               "Power switch down",                            },
			{ poweringUp,                 "Power switch on",                              },
			{ powerOn,                    "Power on",                                     },
			{ goingToReady,               "System going to ready",                        },
			{ ready,                      "System ready",                                 },
			{ motionStopping,             "Motion stopping",                              },
			{ teaching,                   "Teaching mode"                                 },
			{ motionStarting,             "Motion starting",                              },
			{ moving,                     "Moving",                                       },
			{ autoParkingBeforeShutdown3, "Autoparking q3",                               },
			{ autoParkingBeforeShutdown2, "Autoparking q2",                               },
			{ autoParkingBeforeShutdown1, "Autoparking q1",                               },
			{ autoParkingBeforeShutdown0, "Autoparking q0",                               }
		};
		
		// ############ Add events to the levels ############
		level(off                        ).addEvent(doSwInit,                       swInitializing,             kPublicEvent  ); // Alternative: safetySys.level(off).addEvent(doSwInit, swInitializing, kPublicEvent);
		level(swShutingDown              ).addEvent(swShutDownDone,                 off,                        kPrivateEvent );
		level(swInitializing             ).addEvent(swInitDone,                     swInitialized,              kPrivateEvent );
		level(swInitialized              ).addEvent(doOff,                          swShutingDown,              kPublicEvent  );
		level(swInitialized              ).addEvent(goToBaseWaitingForApproval,     baseWaitingForApproval,     kPublicEvent  );
		level(baseEmergency              ).addEvent(doBaseResetEmergency,           baseResetEmergency,         kPublicEvent  );
		level(baseResetEmergency         ).addEvent(baseResetEmergencyDone,         baseWaitingForApproval,     kPrivateEvent );
		level(baseWaitingForApproval     ).addEvent(approvalIsOn,                   baseSystemOn,               kPublicEvent  );
		level(baseSystemOn               ).addEvent(doStopControl,                  controlStopping,            kPublicEvent  );
		level(baseSystemOn               ).addEvent(doManualParking,                manualParking3,             kPublicEvent  );  
		level(basePoweringDown           ).addEvent(basePoweringDownDone,           baseSystemOn,               kPrivateEvent );
		level(manualParking3             ).addEvent(manualParkingDone3,             manualParking2,             kPrivateEvent );
		level(manualParking2             ).addEvent(manualParkingDone2,             manualParking1,             kPrivateEvent );
		level(manualParking1             ).addEvent(manualParkingDone1,             manualParking0,             kPrivateEvent );
		level(manualParking0             ).addEvent(manualParkingDone0,             robotParked,                kPrivateEvent );
		level(robotParked                ).addEvent(doHoming,                       homing3,                    kPublicEvent  );

		level(homing3                    ).addEvent(homingDone3,                    homing2,                    kPrivateEvent );
		level(homing2                    ).addEvent(homingDone2,                    homing1,                    kPrivateEvent );
		level(homing1                    ).addEvent(homingDone1,                    homing0,                    kPrivateEvent );
		level(homing0                    ).addEvent(homingDone0,                    robotHomed,                 kPrivateEvent );
		level(robotHomed                 ).addEvent(doControlStart,                 controlStarting,            kPublicEvent  );
		
		level(controlStopping            ).addEvent(controlStoppingDone,            swShutingDown,              kPrivateEvent );
		level(controlStarting            ).addEvent(controlStartingDone,            systemOn,                   kPrivateEvent );
		level(emergency                  ).addEvent(doResetEmergency,               resetEmergency,             kPublicEvent  );
		level(resetEmergency             ).addEvent(resetEmergencyDone,             waitingForApproval,         kPrivateEvent );
		level(waitingForApproval         ).addEvent(approvalIsOn,                   systemOn,                   kPublicEvent  );
		level(systemOn                   ).addEvent(doStopControl,                  controlStopping,            kPublicEvent  );
		level(systemOn                   ).addEvent(doPoweringUp,                   poweringUp,                 kPublicEvent  );
		level(poweringDown               ).addEvent(poweringDownDone,               systemOn,                   kPrivateEvent );
		level(poweringUp                 ).addEvent(poweringUpDone,                 powerOn,                    kPrivateEvent );
		level(powerOn                    ).addEvent(doPoweringDown,                 poweringDown,               kPublicEvent  );
		level(powerOn                    ).addEvent(goToReady,                      goingToReady,               kPublicEvent  );
		level(goingToReady               ).addEvent(isReady,                        ready,                      kPublicEvent  );
		level(ready                      ).addEvent(doStartingMotion,               motionStarting,             kPublicEvent  );
		level(ready                      ).addEvent(doTeaching,                     teaching,                   kPublicEvent  );
		level(ready                      ).addEvent(doAutoParkingBeforeShutdown,    autoParkingBeforeShutdown3, kPublicEvent  );
		level(motionStopping             ).addEvent(motionStoppingDone,             ready,                      kPrivateEvent );
		level(motionStarting             ).addEvent(motionStartingDone,             moving,                     kPrivateEvent );
		level(teaching                   ).addEvent(doMotionStopping,               motionStopping,             kPublicEvent  );
		level(moving                     ).addEvent(doMotionStopping,               motionStopping,             kPublicEvent  );
		level(autoParkingBeforeShutdown3 ).addEvent(autoParkingBeforeShutdownDone3, autoParkingBeforeShutdown2, kPrivateEvent ); 
		level(autoParkingBeforeShutdown2 ).addEvent(autoParkingBeforeShutdownDone2, autoParkingBeforeShutdown1, kPrivateEvent ); 
		level(autoParkingBeforeShutdown1 ).addEvent(autoParkingBeforeShutdownDone1, autoParkingBeforeShutdown0, kPrivateEvent ); 
		level(autoParkingBeforeShutdown0 ).addEvent(autoParkingBeforeShutdownDone0, systemOn,                   kPrivateEvent );
		
		// Add events to multiple levels
		addEventToAllLevelsBetween(off, homing0, doEmergency, baseEmergency, kPublicEvent);
		addEventToLevelAndAbove(robotHomed, doEmergency, emergency, kPublicEvent);
		
		// ############ Define input states and events for all levels ############
		level(off                        ).setInputActions({ ignore(approval),                     ignore(limitSwitchQ0p), ignore(limitSwitchQ0n), ignore(limitSwitchQ1p), ignore(limitSwitchQ1n), ignore(limitSwitchQ2p), ignore(limitSwitchQ2n), ignore(limitSwitchQ3p), ignore(limitSwitchQ3n), ignore(q0), ignore(q1), ignore(q2r), ignore(q3) });
		level(swShutingDown              ).setInputActions({ ignore(approval),                     ignore(limitSwitchQ0p), ignore(limitSwitchQ0n), ignore(limitSwitchQ1p), ignore(limitSwitchQ1n), ignore(limitSwitchQ2p), ignore(limitSwitchQ2n), ignore(limitSwitchQ3p), ignore(limitSwitchQ3n), ignore(q0), ignore(q1), ignore(q2r), ignore(q3) });
		level(swInitializing             ).setInputActions({ ignore(approval),                     ignore(limitSwitchQ0p), ignore(limitSwitchQ0n), ignore(limitSwitchQ1p), ignore(limitSwitchQ1n), ignore(limitSwitchQ2p), ignore(limitSwitchQ2n), ignore(limitSwitchQ3p), ignore(limitSwitchQ3n), ignore(q0), ignore(q1), ignore(q2r), ignore(q3) });
		level(swInitialized              ).setInputActions({ ignore(approval),                     ignore(limitSwitchQ0p), ignore(limitSwitchQ0n), ignore(limitSwitchQ1p), ignore(limitSwitchQ1n), ignore(limitSwitchQ2p), ignore(limitSwitchQ2n), ignore(limitSwitchQ3p), ignore(limitSwitchQ3n), ignore(q0), ignore(q1), ignore(q2r), ignore(q3) });
		level(baseEmergency              ).setInputActions({ ignore(approval),                     ignore(limitSwitchQ0p), ignore(limitSwitchQ0n), ignore(limitSwitchQ1p), ignore(limitSwitchQ1n), ignore(limitSwitchQ2p), ignore(limitSwitchQ2n), ignore(limitSwitchQ3p), ignore(limitSwitchQ3n), ignore(q0), ignore(q1), ignore(q2r), ignore(q3) });
		level(baseResetEmergency         ).setInputActions({ ignore(approval),                     ignore(limitSwitchQ0p), ignore(limitSwitchQ0n), ignore(limitSwitchQ1p), ignore(limitSwitchQ1n), ignore(limitSwitchQ2p), ignore(limitSwitchQ2n), ignore(limitSwitchQ3p), ignore(limitSwitchQ3n), ignore(q0), ignore(q1), ignore(q2r), ignore(q3) });
		level(baseWaitingForApproval     ).setInputActions({ check(approval, true , approvalIsOn), ignore(limitSwitchQ0p), ignore(limitSwitchQ0n), ignore(limitSwitchQ1p), ignore(limitSwitchQ1n), ignore(limitSwitchQ2p), ignore(limitSwitchQ2n), ignore(limitSwitchQ3p), ignore(limitSwitchQ3n), ignore(q0), ignore(q1), ignore(q2r), ignore(q3) });
		level(baseSystemOn               ).setInputActions({ check(approval, false, doEmergency),  ignore(limitSwitchQ0p), ignore(limitSwitchQ0n), ignore(limitSwitchQ1p), ignore(limitSwitchQ1n), ignore(limitSwitchQ2p), ignore(limitSwitchQ2n), ignore(limitSwitchQ3p), ignore(limitSwitchQ3n), ignore(q0), ignore(q1), ignore(q2r), ignore(q3) });
		level(basePoweringDown           ).setInputActions({ check(approval, false, doEmergency),  ignore(limitSwitchQ0p), ignore(limitSwitchQ0n), ignore(limitSwitchQ1p), ignore(limitSwitchQ1n), ignore(limitSwitchQ2p), ignore(limitSwitchQ2n), ignore(limitSwitchQ3p), ignore(limitSwitchQ3n), ignore(q0), ignore(q1), ignore(q2r), ignore(q3) });
		
		level(manualParking3             ).setInputActions({ check(approval, false, doEmergency),  ignore(limitSwitchQ0p), ignore(limitSwitchQ0n),                           ignore(limitSwitchQ1p), ignore(limitSwitchQ1n),                           ignore(limitSwitchQ2p),                           ignore(limitSwitchQ2n), ignore(limitSwitchQ3p), check(limitSwitchQ3n, true , manualParkingDone3), ignore(q0), ignore(q1), ignore(q2r), ignore(q3) });
		level(manualParking2             ).setInputActions({ check(approval, false, doEmergency),  ignore(limitSwitchQ0p), ignore(limitSwitchQ0n),                           ignore(limitSwitchQ1p), ignore(limitSwitchQ1n),                           check(limitSwitchQ2p, true , manualParkingDone2), ignore(limitSwitchQ2n), ignore(limitSwitchQ3p), ignore(limitSwitchQ3n),                           ignore(q0), ignore(q1), ignore(q2r), ignore(q3) });
		level(manualParking1             ).setInputActions({ check(approval, false, doEmergency),  ignore(limitSwitchQ0p), ignore(limitSwitchQ0n),                           ignore(limitSwitchQ1p), check(limitSwitchQ1n, true , manualParkingDone1), ignore(limitSwitchQ2p),                           ignore(limitSwitchQ2n), ignore(limitSwitchQ3p), ignore(limitSwitchQ3n),                           ignore(q0), ignore(q1), ignore(q2r), ignore(q3) });
		level(manualParking0             ).setInputActions({ check(approval, false, doEmergency),  ignore(limitSwitchQ0p), check(limitSwitchQ0n, true , manualParkingDone0), ignore(limitSwitchQ1p), ignore(limitSwitchQ1n),                           ignore(limitSwitchQ2p),                           ignore(limitSwitchQ2n), ignore(limitSwitchQ3p), ignore(limitSwitchQ3n),                           ignore(q0), ignore(q1), ignore(q2r), ignore(q3) });
		level(robotParked                ).setInputActions({ check(approval, false, doEmergency),  ignore(limitSwitchQ0p), ignore(limitSwitchQ0n),                           ignore(limitSwitchQ1p), ignore(limitSwitchQ1n),                           ignore(limitSwitchQ2p),                           ignore(limitSwitchQ2n), ignore(limitSwitchQ3p), ignore(limitSwitchQ3n),                           ignore(q0), ignore(q1), ignore(q2r), ignore(q3) });
		
		level(homing3                    ).setInputActions({ check(approval, false, doEmergency),  ignore(limitSwitchQ0p), ignore(limitSwitchQ0n),                           ignore(limitSwitchQ1p), ignore(limitSwitchQ1n),                           ignore(limitSwitchQ2p),                           ignore(limitSwitchQ2n), ignore(limitSwitchQ3p), check(limitSwitchQ3n, false, homingDone3),        ignore(q0), ignore(q1), ignore(q2r), ignore(q3) });
		level(homing2                    ).setInputActions({ check(approval, false, doEmergency),  ignore(limitSwitchQ0p), ignore(limitSwitchQ0n),                           ignore(limitSwitchQ1p), ignore(limitSwitchQ1n),                           check(limitSwitchQ2p, false, homingDone2),        ignore(limitSwitchQ2n), ignore(limitSwitchQ3p), ignore(limitSwitchQ3n),                           ignore(q0), ignore(q1), ignore(q2r), ignore(q3) });
		level(homing1                    ).setInputActions({ check(approval, false, doEmergency),  ignore(limitSwitchQ0p), ignore(limitSwitchQ0n),                           ignore(limitSwitchQ1p), check(limitSwitchQ1n, false, homingDone1),        ignore(limitSwitchQ2p),                           ignore(limitSwitchQ2n), ignore(limitSwitchQ3p), ignore(limitSwitchQ3n),                           ignore(q0), ignore(q1), ignore(q2r), ignore(q3) });
		level(homing0                    ).setInputActions({ check(approval, false, doEmergency),  ignore(limitSwitchQ0p), check(limitSwitchQ0n, false, homingDone0),        ignore(limitSwitchQ1p), ignore(limitSwitchQ1n),                           ignore(limitSwitchQ2p),                           ignore(limitSwitchQ2n), ignore(limitSwitchQ3p), ignore(limitSwitchQ3n),                           ignore(q0), ignore(q1), ignore(q2r), ignore(q3) });
		level(robotHomed                 ).setInputActions({ check(approval, false, doEmergency),  ignore(limitSwitchQ0p), ignore(limitSwitchQ0n),                           ignore(limitSwitchQ1p), ignore(limitSwitchQ1n),                           ignore(limitSwitchQ2p),                           ignore(limitSwitchQ2n), ignore(limitSwitchQ3p), ignore(limitSwitchQ3n),                           ignore(q0), ignore(q1), ignore(q2r), ignore(q3) });
		
		level(controlStopping            ).setInputActions({ check(approval, false, doEmergency),  ignore(limitSwitchQ0p), ignore(limitSwitchQ0n), ignore(limitSwitchQ1p), ignore(limitSwitchQ1n), ignore(limitSwitchQ2p), ignore(limitSwitchQ2n), ignore(limitSwitchQ3p), ignore(limitSwitchQ3n), ignore(q0), ignore(q1), ignore(q2r), ignore(q3) });
		level(controlStarting            ).setInputActions({ check(approval, false, doEmergency),  ignore(limitSwitchQ0p), ignore(limitSwitchQ0n), ignore(limitSwitchQ1p), ignore(limitSwitchQ1n), ignore(limitSwitchQ2p), ignore(limitSwitchQ2n), ignore(limitSwitchQ3p), ignore(limitSwitchQ3n), ignore(q0), ignore(q1), ignore(q2r), ignore(q3) });
		level(emergency                  ).setInputActions({ ignore(approval),                      ignore(limitSwitchQ0p), ignore(limitSwitchQ0n), ignore(limitSwitchQ1p), ignore(limitSwitchQ1n), ignore(limitSwitchQ2p), ignore(limitSwitchQ2n), ignore(limitSwitchQ3p), ignore(limitSwitchQ3n), ignore(q0), ignore(q1), ignore(q2r), ignore(q3) });
		level(resetEmergency             ).setInputActions({ ignore(approval),                      ignore(limitSwitchQ0p), ignore(limitSwitchQ0n), ignore(limitSwitchQ1p), ignore(limitSwitchQ1n), ignore(limitSwitchQ2p), ignore(limitSwitchQ2n), ignore(limitSwitchQ3p), ignore(limitSwitchQ3n), ignore(q0), ignore(q1), ignore(q2r), ignore(q3) });
		level(waitingForApproval         ).setInputActions({ check(approval, true , approvalIsOn),  ignore(limitSwitchQ0p), ignore(limitSwitchQ0n), ignore(limitSwitchQ1p), ignore(limitSwitchQ1n), ignore(limitSwitchQ2p), ignore(limitSwitchQ2n), ignore(limitSwitchQ3p), ignore(limitSwitchQ3n), ignore(q0), ignore(q1), ignore(q2r), ignore(q3) });
		level(systemOn                   ).setInputActions({ check(approval, false, doEmergency),   ignore(limitSwitchQ0p), ignore(limitSwitchQ0n), ignore(limitSwitchQ1p), ignore(limitSwitchQ1n), ignore(limitSwitchQ2p), ignore(limitSwitchQ2n), ignore(limitSwitchQ3p), ignore(limitSwitchQ3n), ignore(q0), ignore(q1), ignore(q2r), ignore(q3) });
		level(poweringDown               ).setInputActions({ check(approval, false, doEmergency),   ignore(limitSwitchQ0p), ignore(limitSwitchQ0n), ignore(limitSwitchQ1p), ignore(limitSwitchQ1n), ignore(limitSwitchQ2p), ignore(limitSwitchQ2n), ignore(limitSwitchQ3p), ignore(limitSwitchQ3n), ignore(q0), ignore(q1), ignore(q2r), ignore(q3) });
		level(poweringUp                 ).setInputActions({ check(approval, false, doEmergency),   ignore(limitSwitchQ0p), ignore(limitSwitchQ0n), ignore(limitSwitchQ1p), ignore(limitSwitchQ1n), ignore(limitSwitchQ2p), ignore(limitSwitchQ2n), ignore(limitSwitchQ3p), ignore(limitSwitchQ3n), ignore(q0), ignore(q1), ignore(q2r), ignore(q3) });
		level(powerOn                    ).setInputActions({ check(approval, false, doEmergency),   ignore(limitSwitchQ0p), ignore(limitSwitchQ0n), ignore(limitSwitchQ1p), ignore(limitSwitchQ1n), ignore(limitSwitchQ2p), ignore(limitSwitchQ2n), ignore(limitSwitchQ3p), ignore(limitSwitchQ3n), ignore(q0), ignore(q1), ignore(q2r), ignore(q3) });
		level(goingToReady               ).setInputActions({ check(approval, false, doEmergency),   check(limitSwitchQ0p, true , doEmergency), check(limitSwitchQ0n, true , doEmergency), check(limitSwitchQ1p, true , doEmergency), check(limitSwitchQ1n, true, doEmergency), check(limitSwitchQ2p, true, doEmergency), check(limitSwitchQ2n, true , doEmergency), check(limitSwitchQ3p, true , doEmergency), check(limitSwitchQ3n, true , doEmergency), ignore(q0), ignore(q1), ignore(q2r), ignore(q3) });		
		level(ready                      ).setInputActions({ check(approval, false, doEmergency),   check(limitSwitchQ0p, true , doEmergency), check(limitSwitchQ0n, true , doEmergency), check(limitSwitchQ1p, true , doEmergency), check(limitSwitchQ1n, true, doEmergency), check(limitSwitchQ2p, true, doEmergency), check(limitSwitchQ2n, true , doEmergency), check(limitSwitchQ3p, true , doEmergency), check(limitSwitchQ3n, true , doEmergency), ignore(q0), ignore(q1), ignore(q2r), ignore(q3) });
		level(motionStopping             ).setInputActions({ check(approval, false, doEmergency),   check(limitSwitchQ0p, true , doEmergency), check(limitSwitchQ0n, true , doEmergency), check(limitSwitchQ1p, true , doEmergency), check(limitSwitchQ1n, true, doEmergency), check(limitSwitchQ2p, true, doEmergency), check(limitSwitchQ2n, true , doEmergency), check(limitSwitchQ3p, true , doEmergency), check(limitSwitchQ3n, true , doEmergency), ignore(q0), ignore(q1), ignore(q2r), ignore(q3) });
		level(motionStarting             ).setInputActions({ check(approval, false, doEmergency),   check(limitSwitchQ0p, true , doEmergency), check(limitSwitchQ0n, true , doEmergency), check(limitSwitchQ1p, true , doEmergency), check(limitSwitchQ1n, true, doEmergency), check(limitSwitchQ2p, true, doEmergency), check(limitSwitchQ2n, true , doEmergency), check(limitSwitchQ3p, true , doEmergency), check(limitSwitchQ3n, true , doEmergency), ignore(q0), ignore(q1), ignore(q2r), ignore(q3) });
		level(teaching                   ).setInputActions({ check(approval, false, doEmergency),   check(limitSwitchQ0p, true , doEmergency), check(limitSwitchQ0n, true , doEmergency), check(limitSwitchQ1p, true , doEmergency), check(limitSwitchQ1n, true, doEmergency), check(limitSwitchQ2p, true, doEmergency), check(limitSwitchQ2n, true , doEmergency), check(limitSwitchQ3p, true , doEmergency), check(limitSwitchQ3n, true , doEmergency), ignore(q0), ignore(q1), ignore(q2r), ignore(q3) });
		level(moving                     ).setInputActions({ check(approval, false, doEmergency),   check(limitSwitchQ0p, true , doEmergency), check(limitSwitchQ0n, true , doEmergency), check(limitSwitchQ1p, true , doEmergency), check(limitSwitchQ1n, true, doEmergency), check(limitSwitchQ2p, true, doEmergency), check(limitSwitchQ2n, true , doEmergency), check(limitSwitchQ3p, true , doEmergency), check(limitSwitchQ3n, true , doEmergency), ignore(q0), ignore(q1), ignore(q2r), ignore(q3) });
		level(autoParkingBeforeShutdown3 ).setInputActions({ check(approval, false, doEmergency),   ignore(limitSwitchQ0p), ignore(limitSwitchQ0n),                                      ignore(limitSwitchQ1p), ignore(limitSwitchQ1n),                                      ignore(limitSwitchQ2p),                                      ignore(limitSwitchQ2n), ignore(limitSwitchQ3p), check(limitSwitchQ3n, true, autoParkingBeforeShutdownDone3), ignore(q0), ignore(q1), ignore(q2r), ignore(q3) });
		level(autoParkingBeforeShutdown2 ).setInputActions({ check(approval, false, doEmergency),   ignore(limitSwitchQ0p), ignore(limitSwitchQ0n),                                      ignore(limitSwitchQ1p), ignore(limitSwitchQ1n),                                      check(limitSwitchQ2p, true, autoParkingBeforeShutdownDone2), ignore(limitSwitchQ2n), ignore(limitSwitchQ3p), ignore(limitSwitchQ3n),                                      ignore(q0), ignore(q1), ignore(q2r), ignore(q3) });
		level(autoParkingBeforeShutdown1 ).setInputActions({ check(approval, false, doEmergency),   ignore(limitSwitchQ0p), ignore(limitSwitchQ0n),                                      ignore(limitSwitchQ1p), check(limitSwitchQ1n, true, autoParkingBeforeShutdownDone1), ignore(limitSwitchQ2p),                                      ignore(limitSwitchQ2n), ignore(limitSwitchQ3p), ignore(limitSwitchQ3n),                                      ignore(q0), ignore(q1), ignore(q2r), ignore(q3) });
		level(autoParkingBeforeShutdown0 ).setInputActions({ check(approval, false, doEmergency),   ignore(limitSwitchQ0p), check(limitSwitchQ0n, true, autoParkingBeforeShutdownDone0), ignore(limitSwitchQ1p), ignore(limitSwitchQ1n),                                      ignore(limitSwitchQ2p),                                      ignore(limitSwitchQ2n), ignore(limitSwitchQ3p), ignore(limitSwitchQ3n),                                      ignore(q0), ignore(q1), ignore(q2r), ignore(q3) });
	
		// Define output states and events for all levels 
		level(off                        ).setOutputActions({ set(watchdog, false), set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false), set(brake0, true ), set(brake1, true ), set(brake2, true ), set(brake3, true ) });
		level(swShutingDown              ).setOutputActions({ set(watchdog, false), set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false), set(brake0, true ), set(brake1, true ), set(brake2, true ), set(brake3, true ) });
		level(swInitializing             ).setOutputActions({ set(watchdog, false), set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false), set(brake0, true ), set(brake1, true ), set(brake2, true ), set(brake3, true ) });
		level(swInitialized              ).setOutputActions({ set(watchdog, false), set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false), set(brake0, true ), set(brake1, true ), set(brake2, true ), set(brake3, true ) });
		level(baseEmergency              ).setOutputActions({ set(watchdog, false), set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false), set(brake0, true ), set(brake1, true ), set(brake2, true ), set(brake3, true ) });
		level(baseResetEmergency         ).setOutputActions({ set(watchdog, false), set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false), set(brake0, true ), set(brake1, true ), set(brake2, true ), set(brake3, true ) });
		level(baseWaitingForApproval     ).setOutputActions({ toggle(watchdog    ), set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false), set(brake0, true ), set(brake1, true ), set(brake2, true ), set(brake3, true ) });
		level(baseSystemOn               ).setOutputActions({ toggle(watchdog    ), set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false), set(brake0, true ), set(brake1, true ), set(brake2, true ), set(brake3, true ) });
		level(basePoweringDown           ).setOutputActions({ toggle(watchdog    ), set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false), set(brake0, true ), set(brake1, true ), set(brake2, true ), set(brake3, true ) });
		
		level(manualParking3             ).setOutputActions({ toggle(watchdog    ), set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false), set(brake0, true ), set(brake1, true ), set(brake2, true ), set(brake3, false) });
		level(manualParking2             ).setOutputActions({ toggle(watchdog    ), set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false), set(brake0, true ), set(brake1, true ), set(brake2, false), set(brake3, true ) });
		level(manualParking1             ).setOutputActions({ toggle(watchdog    ), set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false), set(brake0, true ), set(brake1, false), set(brake2, true ), set(brake3, true ) });
		level(manualParking0             ).setOutputActions({ toggle(watchdog    ), set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false), set(brake0, false), set(brake1, true ), set(brake2, true ), set(brake3, true ) });
		level(robotParked                ).setOutputActions({ toggle(watchdog    ), set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false), set(brake0, true ), set(brake1, true ), set(brake2, true ), set(brake3, true ) });
		
		level(homing3                    ).setOutputActions({ toggle(watchdog    ), set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, true ), set(brake0, true ), set(brake1, true ), set(brake2, true ), set(brake3, false) });	
		level(homing2                    ).setOutputActions({ toggle(watchdog    ), set(enable0, false), set(enable1, false), set(enable2, true ), set(enable3, false), set(brake0, true ), set(brake1, true ), set(brake2, false), set(brake3, true ) });	
		level(homing1                    ).setOutputActions({ toggle(watchdog    ), set(enable0, false), set(enable1, true ), set(enable2, false), set(enable3, false), set(brake0, true ), set(brake1, false), set(brake2, true ), set(brake3, true) });
		level(homing0                    ).setOutputActions({ toggle(watchdog    ), set(enable0, true ), set(enable1, false), set(enable2, false), set(enable3, false), set(brake0, false), set(brake1, true ), set(brake2, true ), set(brake3, true ) });
		level(robotHomed                 ).setOutputActions({ toggle(watchdog    ), set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false), set(brake0, true ), set(brake1, true ), set(brake2, true ), set(brake3, true ) });
		
		level(controlStopping            ).setOutputActions({ set(watchdog, false), set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false), set(brake0, true ), set(brake1, true ), set(brake2, true ), set(brake3, true ) });
		level(controlStarting            ).setOutputActions({ toggle(watchdog    ), set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false), set(brake0, true ), set(brake1, true ), set(brake2, true ), set(brake3, true ) });
		level(emergency                  ).setOutputActions({ set(watchdog, false), set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false), set(brake0, true ), set(brake1, true ), set(brake2, true ), set(brake3, true ) });
		level(resetEmergency             ).setOutputActions({ set(watchdog, false), set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false), set(brake0, true ), set(brake1, true ), set(brake2, true ), set(brake3, true ) });
		level(waitingForApproval         ).setOutputActions({ toggle(watchdog    ), set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false), set(brake0, true ), set(brake1, true ), set(brake2, true ), set(brake3, true ) });
		level(systemOn                   ).setOutputActions({ toggle(watchdog    ), set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false), set(brake0, true ), set(brake1, true ), set(brake2, true ), set(brake3, true ) });
		level(poweringDown               ).setOutputActions({ toggle(watchdog    ), set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false), set(brake0, true ), set(brake1, true ), set(brake2, true ), set(brake3, true ) });
		level(poweringUp                 ).setOutputActions({ toggle(watchdog    ), set(enable0, true ), set(enable1, true ), set(enable2, true ), set(enable3, true ), set(brake0, false), set(brake1, false), set(brake2, false), set(brake3, false) });
		level(powerOn                    ).setOutputActions({ toggle(watchdog    ), set(enable0, true ), set(enable1, true ), set(enable2, true ), set(enable3, true ), set(brake0, false), set(brake1, false), set(brake2, false), set(brake3, false) });
		level(goingToReady               ).setOutputActions({ toggle(watchdog    ), set(enable0, true ), set(enable1, true ), set(enable2, true ), set(enable3, true ), set(brake0, false), set(brake1, false), set(brake2, false), set(brake3, false) });
		level(ready                      ).setOutputActions({ toggle(watchdog    ), set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false), set(brake0, true ), set(brake1, true ), set(brake2, true ), set(brake3, true ) });	
		level(motionStopping             ).setOutputActions({ toggle(watchdog    ), set(enable0, true ), set(enable1, true ), set(enable2, true ), set(enable3, true ), set(brake0, false), set(brake1, false), set(brake2, false), set(brake3, false) });
		level(motionStarting             ).setOutputActions({ toggle(watchdog    ), set(enable0, true ), set(enable1, true ), set(enable2, true ), set(enable3, true ), set(brake0, false), set(brake1, false), set(brake2, false), set(brake3, false) });
		level(teaching                   ).setOutputActions({ toggle(watchdog    ), set(enable0, true ), set(enable1, true ), set(enable2, true ), set(enable3, true ), set(brake0, false), set(brake1, false), set(brake2, false), set(brake3, false) });
		level(moving                     ).setOutputActions({ toggle(watchdog    ), set(enable0, true ), set(enable1, true ), set(enable2, true ), set(enable3, true ), set(brake0, false), set(brake1, false), set(brake2, false), set(brake3, false) });
		level(autoParkingBeforeShutdown3 ).setOutputActions({ toggle(watchdog    ), set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, true ), set(brake0, true ), set(brake1, true ), set(brake2, true ), set(brake3, false) });
		level(autoParkingBeforeShutdown2 ).setOutputActions({ toggle(watchdog    ), set(enable0, false), set(enable1, false), set(enable2, true ), set(enable3, false), set(brake0, true ), set(brake1, true ), set(brake2, false), set(brake3, true ) });
		level(autoParkingBeforeShutdown1 ).setOutputActions({ toggle(watchdog    ), set(enable0, false), set(enable1, true ), set(enable2, false), set(enable3, false), set(brake0, true ), set(brake1, false), set(brake2, true ), set(brake3, true ) });
		level(autoParkingBeforeShutdown0 ).setOutputActions({ toggle(watchdog    ), set(enable0, true ), set(enable1, false), set(enable2, false), set(enable3, false), set(brake0, false), set(brake1, true ), set(brake2, true ), set(brake3, true ) });
	
		// *** Define and add level functions *** //
		
		// Boot
		level(off).setLevelAction([&](SafetyContext* privateContext) {
			static bool first = true; 
			if(first == true) {
				privateContext->triggerEvent(doSwInit);
				first = false;
			}
		});
		level(swInitializing).setLevelAction([&](SafetyContext* privateContext) {
			privateContext->triggerEvent(swInitDone); 
		});
		level(swInitialized).setLevelAction([&](SafetyContext* privateContext) {
			privateContext->triggerEvent(goToBaseWaitingForApproval); 
		});
		
		// Shut Down
		level(swShutingDown).setLevelAction([&](SafetyContext* privateContext) {
			privateContext->triggerEvent(swShutDownDone); 
		});

		// Base Emergency
		level(baseEmergency).setLevelAction([&](SafetyContext* privateContext) {
			throw EEROSException("EMERGENCY! Start the program again");
// 			privateContext->triggerEvent(doBaseResetEmergency); 
		});
		level(baseResetEmergency).setLevelAction([&](SafetyContext* privateContext) {
			if(approval->get())
				privateContext->triggerEvent(baseResetEmergencyDone); 
		});
		level(basePoweringDown).setLevelAction([&](SafetyContext* privateContext) {
			privateContext->triggerEvent(basePoweringDownDone); 
		});	

		level(robotParked).setLevelAction([&](SafetyContext* privateContext) {
			hal.getLogicPeripheralOutput("safeUserLight")->set(true);
			if(hal.getLogicPeripheralInput("safeUserButton")->get())
				privateContext->triggerEvent(doHoming); 
		});
		level(homing3).setLevelAction([&](SafetyContext* privateContext) {
			if(limitSwitchQ3n->get() == true){
				q3_init = q3->get();
			}
		});
		level(homing2).setLevelAction([&](SafetyContext* privateContext) {
			if(limitSwitchQ2p->get() == true){
				q2r_init = q2r->get();
			}
		});
		level(homing1).setLevelAction([&](SafetyContext* privateContext) {
			if(limitSwitchQ1n->get() == true){
				q1_init = q1->get();
			}
		});		
		level(homing0).setLevelAction([&](SafetyContext* privateContext) {
			if(limitSwitchQ0n->get() == true){
				q0_init = q0->get();
			}
		});
		level(robotHomed).setLevelAction([&](SafetyContext* privateContext) { 
			q3->setOffset(-q3->get() + 0.0 + (q3->get() - q3_init));
			q2r->setOffset(-q2r->get() - (2*M_PI*0.099/0.025)*(-1.5) + (q2r->get() - q2r_init));
			q1->setOffset(-q1->get() + (2.45035171-0.0461+0.017075+0.0032)*(100) + (q1->get() - q1_init));
			q0->setOffset(-q0->get() + (2.60743134-0.01945+0.0015+0.003)*(100) + (q0->get() - q0_init));
			
			privateContext->triggerEvent(doControlStart);
		});

		level(controlStopping).setLevelAction([&](SafetyContext* privateContext) {
			privateContext->triggerEvent(controlStoppingDone);
		});
		level(controlStarting).setLevelAction([&](SafetyContext* privateContext) {
			privateContext->triggerEvent(controlStartingDone); 
		});
		
		level(emergency).setLevelAction([&](SafetyContext* privateContext) {
			throw EEROSException("EMERGENCY! Start the program again");
// 			privateContext->triggerEvent(doResetEmergency);  
		});
		level(resetEmergency).setLevelAction([&](SafetyContext* privateContext) {		
			if(approval->get())
				privateContext->triggerEvent(resetEmergencyDone);
		});
		level(waitingForApproval).setLevelAction([&](SafetyContext* privateContext) {
			static bool first = true;
			if(first) {
				std::cout << "press the white button" << std::endl;
				first = false;
			}
		});
		level(systemOn).setLevelAction([&](SafetyContext* privateContext) {
// 			hal.getLogicPeripheralOutput("safeUserLight")->set(true);
// 			if(hal.getLogicPeripheralInput("safeUserButton")->get())
// 				privateContext->triggerEvent(doPoweringUp); 
		});
		
		level(goingToReady).setLevelAction([&](SafetyContext* privateContext) {
		});
		
		level(poweringDown).setLevelAction([&](SafetyContext* privateContext) {
			privateContext->triggerEvent(poweringDownDone);
		});
		level(poweringUp).setLevelAction([&](SafetyContext* privateContext) {
			privateContext->triggerEvent(poweringUpDone); 
		});
		level(ready).setLevelAction([&](SafetyContext* privateContext) {
			// TODO leave only trigger event
			hal.getLogicPeripheralOutput("safeUserLight")->set(true);
			if(hal.getLogicPeripheralInput("safeUserButton")->get())
				privateContext->triggerEvent(doStartingMotion); // green button takes safety system into moving state
		});
		level(motionStarting).setLevelAction([&](SafetyContext* privateContext) {
			privateContext->triggerEvent(motionStartingDone);
		});
		level(motionStopping).setLevelAction([&](SafetyContext* privateContext) {
			privateContext->triggerEvent(motionStoppingDone); 
		});	

		// Define entry level
		entryLevel = off;
}

ScaraSafetyProperties::~ScaraSafetyProperties() {
	// nothing to do
}