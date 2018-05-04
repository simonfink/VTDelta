#include "ScaraSafetyProperties.hpp"
#include "ScaraControlSystem.hpp"
#include <eeros/hal/HAL.hpp>
#include <eeros/safety/InputAction.hpp>
#include <eeros/safety/OutputAction.hpp>
#include <eeros/safety/ControlInput.hpp>
#include <eeros/math/Matrix.hpp>

#include <unistd.h>
#include <iostream>
#include <vector>
#include <initializer_list>
#include <cmath>

#include "constants.hpp"

using namespace scara;
using namespace eeros;
using namespace eeros::hal;
using namespace eeros::safety;

ScaraSafetyProperties::ScaraSafetyProperties() :

off                        ("off"                                              ),
notinit_emergency          ("emergency, not init"                              ),
notinit_waitingForApproval ("waitingForApproval, not init, press WHITE button" ),
notinit_systemOn           ("systemOn, not init"                               ),

manualParking3             ("manualParking 3" ),             
manualParking2             ("manualParking 2" ),
manualParking1             ("manualParking 1" ),
manualParking0             ("manualParking 0" ),
robotParked                ("robotParked"     ),

homing3                    ("homing 3"                                 ),
homing2                    ("homing 2"                                 ),
homing1                    ("homing 1"                                 ),
homing0                    ("homing 0"                                 ),
robotHomed                 ("robot homed, press GREEN button to start" ),

emergency                  ("Emergency"            ),
resetEmergency             ("Emergency reset"      ),
waitingForApproval         ("Waiting for approval" ),

set_autoParking            ("Set Autoparking axes" ),
autoParking_shutdown3      ("Autoparking q3"       ),
autoParking_shutdown2      ("Autoparking q2"       ),
autoParking_shutdown1      ("Autoparking q1"       ),
autoParking_shutdown0      ("Autoparking q0"       ),

systemOn                   ("System on"            ),
powerOn                    ("Power on"             ),
goingToReady               ("System going to ready"),
ready                      ("System ready"         ),

set_moving                 ("Set Moving"           ),
moving                     ("Moving"               ),
set_moving_joystick        ("Set Moving joystick"  ),
moving_joystick            ("Moving joystick"      ),

notinit_goToWaitingForApproval ("notinit_goToWaitingForApproval"),
doOff                          ("doOff"),
approvalIsOn                   ("approvalIsOn"),

doManualParking                ("doManualParking          "),
manualParkingDone3             ("manualParkingDone3       "),
manualParkingDone2             ("manualParkingDone2       "),
manualParkingDone1             ("manualParkingDone1       "),
manualParkingDone0             ("manualParkingDone0       "),
                                                         
doHoming                       ("doHoming                 "),
homingDone3                    ("homingDone3              "), 
homingDone2                    ("homingDone2              "),     
homingDone1                    ("homingDone1              "),  
homingDone0                    ("homingDone0              "), 
                                                         
doEmergency                    ("doEmergency              "),
doResetEmergency               ("doResetEmergency         "),
resetEmergencyDone             ("resetEmergencyDone       "),
doEmergency_posOutRange        ("doEmergency_posOutRange  "),
doEmergency_velOutRange        ("doEmergency_velOutRange  "),
                                                         
doSystemOn                     ("doSystemOn               "),
doPowerUp                      ("doPowerUp                "),
doPowerDown                    ("doPowerDown              "),
goToReady                      ("goToReady                "),
isReady                        ("isReady                  "),
          
doSetMoving                    ("doSetMotion              "),
doStartMoving                  ("doStartMotion            "),
doStopMoving                   ("doStopMotion             "),
doSetMoving_joystick           ("doSetMoving_joystick     "),
doStartMoving_joystick         ("doStartMoving_joystick   "),
doStopMoving_joystick          ("doStopMoving_joystick    "),

doAutoParking_shutdown         ("doAutoParking_shutdown   "),
doStartAutoParking             ("doStartAutoParking       "),
autoParking_shutdownDone3      ("autoParking_shutdownDone3"),
autoParking_shutdownDone2      ("autoParking_shutdownDone2"),
autoParking_shutdownDone1      ("autoParking_shutdownDone1"),
autoParking_shutdownDone0      ("autoParking_shutdownDone0")

{
	HAL& hal = HAL::instance();

	// ############ Define critical outputs ############
	watchdog = hal.getLogicOutput("watchdog");
	enable0 =  hal.getLogicOutput("enable0");
	enable1 =  hal.getLogicOutput("enable1");
	enable2 =  hal.getLogicOutput("enable2");
	enable3 =  hal.getLogicOutput("enable3");
	brake0 =   hal.getLogicOutput("brake0");
	brake1 =   hal.getLogicOutput("brake1");
	brake2 =   hal.getLogicOutput("brake2");
	brake3 =   hal.getLogicOutput("brake3");
	
	criticalOutputs = { watchdog, enable0, enable1, enable2, enable3, brake0, brake1, brake2, brake3 };
	
	// ############ Define critical inputs ############
	approval = hal.getLogicInput("approval");
	limitSwitchQ0p = hal.getLogicInput("limitSwitchQ0p");
	limitSwitchQ0n = hal.getLogicInput("limitSwitchQ0n");
	limitSwitchQ1p = hal.getLogicInput("limitSwitchQ1p");
	limitSwitchQ1n = hal.getLogicInput("limitSwitchQ1n");
	limitSwitchQ2p = hal.getLogicInput("limitSwitchQ2p");
	limitSwitchQ2n = hal.getLogicInput("limitSwitchQ2n");
	limitSwitchQ3p = hal.getLogicInput("limitSwitchQ3p");
	limitSwitchQ3n = hal.getLogicInput("limitSwitchQ3n");
	
	criticalInputs = {  approval, 
						limitSwitchQ0p, limitSwitchQ0n, limitSwitchQ1p, limitSwitchQ1n, 
						limitSwitchQ2p, limitSwitchQ2n, limitSwitchQ3p, limitSwitchQ3n
	};
	
	// ############ Other inputs/outputs ############
	safeUserLight = hal.getLogicOutput("safeUserLight", false);
	safeUserButton = hal.getLogicInput("safeUserButton");
	
	// ############ Define Levels ############
	addLevel(off                        );
	addLevel(notinit_emergency          );
	addLevel(notinit_waitingForApproval );
	addLevel(notinit_systemOn           );
	addLevel(manualParking3             );            
	addLevel(manualParking2             );
	addLevel(manualParking1             );
	addLevel(manualParking0             );
	addLevel(robotParked                );
	addLevel(homing3                    );
	addLevel(homing2                    );
	addLevel(homing1                    );
	addLevel(homing0                    );
	addLevel(robotHomed                 );
	addLevel(emergency                  );
	addLevel(resetEmergency             );
	
	addLevel(waitingForApproval         );
	addLevel(systemOn                   );
	addLevel(powerOn                    );
	addLevel(goingToReady               );
	addLevel(ready                      );
	addLevel(set_moving                 );
	addLevel(moving                     );
	addLevel(set_moving_joystick        );
	addLevel(moving_joystick            );
	
	addLevel(set_autoParking            );
	addLevel(autoParking_shutdown3      );
	addLevel(autoParking_shutdown2      );
	addLevel(autoParking_shutdown1      );
	addLevel(autoParking_shutdown0      );
		
	// ############ Add events to the levels ############
	off                        .addEvent(notinit_goToWaitingForApproval, notinit_waitingForApproval, kPublicEvent  );
	notinit_waitingForApproval .addEvent(approvalIsOn,                   notinit_systemOn,           kPublicEvent  );
	notinit_systemOn           .addEvent(doManualParking,                manualParking3,             kPublicEvent  );  
	notinit_emergency          .addEvent(doOff,                          off,                        kPrivateEvent );
	manualParking3             .addEvent(manualParkingDone3,             manualParking2,             kPrivateEvent );
	manualParking2             .addEvent(manualParkingDone2,             manualParking1,             kPrivateEvent );
	manualParking1             .addEvent(manualParkingDone1,             manualParking0,             kPrivateEvent );
	manualParking0             .addEvent(manualParkingDone0,             robotParked,                kPrivateEvent );
	robotParked                .addEvent(doHoming,                       homing3,                    kPublicEvent  );
	homing3                    .addEvent(homingDone3,                    homing2,                    kPrivateEvent );
	homing2                    .addEvent(homingDone2,                    homing1,                    kPrivateEvent ); 
	homing1                    .addEvent(homingDone1,                    homing0,                    kPrivateEvent ); 
	homing0                    .addEvent(homingDone0,                    robotHomed,                 kPrivateEvent );
	robotHomed                 .addEvent(doSystemOn,                     systemOn,                   kPublicEvent  );
	
	emergency                  .addEvent(doResetEmergency,               resetEmergency,             kPublicEvent  );
	resetEmergency             .addEvent(resetEmergencyDone,             waitingForApproval,         kPrivateEvent );
	waitingForApproval         .addEvent(approvalIsOn,                   systemOn,                   kPublicEvent  );
	systemOn                   .addEvent(doOff,                          off,                        kPublicEvent  );
	systemOn                   .addEvent(doPowerUp,                      powerOn,                    kPublicEvent  );
	powerOn                    .addEvent(doPowerDown,                    systemOn,                   kPublicEvent  );
	powerOn                    .addEvent(goToReady,                      goingToReady,               kPublicEvent  );
	goingToReady               .addEvent(isReady,                        ready,                      kPublicEvent  );
	ready                      .addEvent(doSetMoving_joystick,           set_moving_joystick,        kPublicEvent  );
	ready                      .addEvent(doSetMoving,                    set_moving,                 kPublicEvent  );
	ready                      .addEvent(doAutoParking_shutdown,         set_autoParking,            kPublicEvent  );
	
	set_moving                 .addEvent(doStartMoving,                  moving,                     kPublicEvent  );
	moving                     .addEvent(doStopMoving,                   ready,                      kPublicEvent  );
	set_moving_joystick        .addEvent(doStartMoving_joystick,         moving_joystick,            kPublicEvent  );
	moving_joystick            .addEvent(doStopMoving_joystick,          ready,                      kPublicEvent  );
	
	set_autoParking            .addEvent(doStartAutoParking,             autoParking_shutdown3,      kPrivateEvent ); 
	autoParking_shutdown3      .addEvent(autoParking_shutdownDone3,      autoParking_shutdown2,      kPrivateEvent ); 
	autoParking_shutdown2      .addEvent(autoParking_shutdownDone2,      autoParking_shutdown1,      kPrivateEvent ); 
	autoParking_shutdown1      .addEvent(autoParking_shutdownDone1,      autoParking_shutdown0,      kPrivateEvent ); 
	autoParking_shutdown0      .addEvent(autoParking_shutdownDone0,      off,                        kPrivateEvent );
		
	// Add events to multiple levels
// 	addEventToAllLevelsBetween(off,            robotHomed,            doEmergency, notinit_emergency, kPublicEvent );
// 	addEventToAllLevelsBetween(resetEmergency, autoParking_shutdown0, doEmergency, emergency,         kPublicEvent );
	
	off                        .addEvent(doEmergency, notinit_emergency, kPublicEvent  );
	notinit_waitingForApproval .addEvent(doEmergency, notinit_emergency, kPublicEvent  );
	notinit_systemOn           .addEvent(doEmergency, notinit_emergency, kPublicEvent  );  
	notinit_emergency          .addEvent(doEmergency, notinit_emergency, kPrivateEvent );
	manualParking3             .addEvent(doEmergency, notinit_emergency, kPrivateEvent );
	manualParking2             .addEvent(doEmergency, notinit_emergency, kPrivateEvent );
	manualParking1             .addEvent(doEmergency, notinit_emergency, kPrivateEvent );
	manualParking0             .addEvent(doEmergency, notinit_emergency, kPrivateEvent );
	robotParked                .addEvent(doEmergency, notinit_emergency, kPublicEvent  );
	homing3                    .addEvent(doEmergency, notinit_emergency, kPrivateEvent );
	homing2                    .addEvent(doEmergency, notinit_emergency, kPrivateEvent ); 
	homing1                    .addEvent(doEmergency, notinit_emergency, kPrivateEvent ); 
	homing0                    .addEvent(doEmergency, notinit_emergency, kPrivateEvent );
	robotHomed                 .addEvent(doEmergency, notinit_emergency, kPublicEvent  );
	resetEmergency             .addEvent(doEmergency, emergency,         kPrivateEvent );
	waitingForApproval         .addEvent(doEmergency, emergency,         kPublicEvent  );
	systemOn                   .addEvent(doEmergency, emergency,         kPublicEvent  );
	powerOn                    .addEvent(doEmergency, emergency,         kPublicEvent  );
	goingToReady               .addEvent(doEmergency, emergency,         kPublicEvent  );
	ready                      .addEvent(doEmergency, emergency,         kPublicEvent  );
	set_moving                 .addEvent(doEmergency, emergency,         kPublicEvent  );
	moving                     .addEvent(doEmergency, emergency,         kPublicEvent  );
	set_moving_joystick        .addEvent(doEmergency, emergency,         kPublicEvent  );
	moving_joystick            .addEvent(doEmergency, emergency,         kPublicEvent  );
	set_autoParking            .addEvent(doEmergency, emergency,         kPrivateEvent ); 
	autoParking_shutdown3      .addEvent(doEmergency, emergency,         kPrivateEvent ); 
	autoParking_shutdown2      .addEvent(doEmergency, emergency,         kPrivateEvent ); 
	autoParking_shutdown1      .addEvent(doEmergency, emergency,         kPrivateEvent ); 
	autoParking_shutdown0      .addEvent(doEmergency, emergency,         kPrivateEvent );
	
	// Emergency checker levels
	systemOn                   .addEvent(doEmergency_posOutRange,        emergency,                  kPublicEvent  );
	systemOn                   .addEvent(doEmergency_velOutRange,        emergency,                  kPublicEvent  );
	powerOn                    .addEvent(doEmergency_posOutRange,        emergency,                  kPublicEvent  );
	powerOn                    .addEvent(doEmergency_velOutRange,        emergency,                  kPublicEvent  );
	goingToReady               .addEvent(doEmergency_posOutRange,        emergency,                  kPublicEvent  );
	goingToReady               .addEvent(doEmergency_velOutRange,        emergency,                  kPublicEvent  );
	ready                      .addEvent(doEmergency_posOutRange,        emergency,                  kPublicEvent  );
	ready                      .addEvent(doEmergency_velOutRange,        emergency,                  kPublicEvent  );
	set_moving                 .addEvent(doEmergency_posOutRange,        emergency,                  kPublicEvent  );
	set_moving                 .addEvent(doEmergency_velOutRange,        emergency,                  kPublicEvent  );
	moving                     .addEvent(doEmergency_posOutRange,        emergency,                  kPublicEvent  );
	moving                     .addEvent(doEmergency_velOutRange,        emergency,                  kPublicEvent  );
	moving_joystick            .addEvent(doEmergency_posOutRange,        emergency,                  kPublicEvent  );
	moving_joystick            .addEvent(doEmergency_velOutRange,        emergency,                  kPublicEvent  );
	
	// ############ Define input states and events for all levels ############
	off                        .setInputActions({ ignore(approval),                     ignore(limitSwitchQ0p), ignore(limitSwitchQ0n), ignore(limitSwitchQ1p),  ignore(limitSwitchQ1n), ignore(limitSwitchQ2p), ignore(limitSwitchQ2n), ignore(limitSwitchQ3p), ignore(limitSwitchQ3n),  });
	notinit_emergency          .setInputActions({ ignore(approval),                     ignore(limitSwitchQ0p), ignore(limitSwitchQ0n), ignore(limitSwitchQ1p),  ignore(limitSwitchQ1n), ignore(limitSwitchQ2p), ignore(limitSwitchQ2n), ignore(limitSwitchQ3p), ignore(limitSwitchQ3n),  });
	notinit_waitingForApproval .setInputActions({ check(approval, true , approvalIsOn), ignore(limitSwitchQ0p), ignore(limitSwitchQ0n), ignore(limitSwitchQ1p),  ignore(limitSwitchQ1n), ignore(limitSwitchQ2p), ignore(limitSwitchQ2n), ignore(limitSwitchQ3p), ignore(limitSwitchQ3n),  });
	notinit_systemOn           .setInputActions({ check(approval, false, doEmergency),  ignore(limitSwitchQ0p), ignore(limitSwitchQ0n), ignore(limitSwitchQ1p),  ignore(limitSwitchQ1n), ignore(limitSwitchQ2p), ignore(limitSwitchQ2n), ignore(limitSwitchQ3p), ignore(limitSwitchQ3n),  });
	
	manualParking3             .setInputActions({ check(approval, false, doEmergency),  ignore(limitSwitchQ0p), ignore(limitSwitchQ0n),                           ignore(limitSwitchQ1p), ignore(limitSwitchQ1n),                           ignore(limitSwitchQ2p),                           ignore(limitSwitchQ2n), ignore(limitSwitchQ3p), check(limitSwitchQ3n, true , manualParkingDone3),});
	manualParking2             .setInputActions({ check(approval, false, doEmergency),  ignore(limitSwitchQ0p), ignore(limitSwitchQ0n),                           ignore(limitSwitchQ1p), ignore(limitSwitchQ1n),                           check(limitSwitchQ2p, true , manualParkingDone2), ignore(limitSwitchQ2n), ignore(limitSwitchQ3p), ignore(limitSwitchQ3n),                          });
	manualParking1             .setInputActions({ check(approval, false, doEmergency),  ignore(limitSwitchQ0p), ignore(limitSwitchQ0n),                           ignore(limitSwitchQ1p), check(limitSwitchQ1n, true , manualParkingDone1), ignore(limitSwitchQ2p),                           ignore(limitSwitchQ2n), ignore(limitSwitchQ3p), ignore(limitSwitchQ3n),                          });
	manualParking0             .setInputActions({ check(approval, false, doEmergency),  ignore(limitSwitchQ0p), check(limitSwitchQ0n, true , manualParkingDone0), ignore(limitSwitchQ1p), ignore(limitSwitchQ1n),                           ignore(limitSwitchQ2p),                           ignore(limitSwitchQ2n), ignore(limitSwitchQ3p), ignore(limitSwitchQ3n),                          });
	robotParked                .setInputActions({ check(approval, false, doEmergency),  ignore(limitSwitchQ0p), ignore(limitSwitchQ0n),                           ignore(limitSwitchQ1p), ignore(limitSwitchQ1n),                           ignore(limitSwitchQ2p),                           ignore(limitSwitchQ2n), ignore(limitSwitchQ3p), ignore(limitSwitchQ3n),                          });
	
	homing3                    .setInputActions({ check(approval, false, doEmergency),  ignore(limitSwitchQ0p), ignore(limitSwitchQ0n), ignore(limitSwitchQ1p),   ignore(limitSwitchQ1n), ignore(limitSwitchQ2p), ignore(limitSwitchQ2n), ignore(limitSwitchQ3p), ignore(limitSwitchQ3n), });
	homing2                    .setInputActions({ check(approval, false, doEmergency),  ignore(limitSwitchQ0p), ignore(limitSwitchQ0n), ignore(limitSwitchQ1p),   ignore(limitSwitchQ1n), ignore(limitSwitchQ2p), ignore(limitSwitchQ2n), ignore(limitSwitchQ3p), ignore(limitSwitchQ3n), });
	homing1                    .setInputActions({ check(approval, false, doEmergency),  ignore(limitSwitchQ0p), ignore(limitSwitchQ0n), ignore(limitSwitchQ1p),   ignore(limitSwitchQ1n), ignore(limitSwitchQ2p), ignore(limitSwitchQ2n), ignore(limitSwitchQ3p), ignore(limitSwitchQ3n), });
	homing0                    .setInputActions({ check(approval, false, doEmergency),  ignore(limitSwitchQ0p), ignore(limitSwitchQ0n), ignore(limitSwitchQ1p),   ignore(limitSwitchQ1n), ignore(limitSwitchQ2p), ignore(limitSwitchQ2n), ignore(limitSwitchQ3p), ignore(limitSwitchQ3n), });
	robotHomed                 .setInputActions({ check(approval, false, doEmergency),  ignore(limitSwitchQ0p), ignore(limitSwitchQ0n), ignore(limitSwitchQ1p),   ignore(limitSwitchQ1n), ignore(limitSwitchQ2p), ignore(limitSwitchQ2n), ignore(limitSwitchQ3p), ignore(limitSwitchQ3n), });

	emergency                  .setInputActions({ ignore(approval),                     ignore(limitSwitchQ0p), ignore(limitSwitchQ0n), ignore(limitSwitchQ1p),   ignore(limitSwitchQ1n), ignore(limitSwitchQ2p), ignore(limitSwitchQ2n), ignore(limitSwitchQ3p), ignore(limitSwitchQ3n), });
	resetEmergency             .setInputActions({ ignore(approval),                     ignore(limitSwitchQ0p), ignore(limitSwitchQ0n), ignore(limitSwitchQ1p),   ignore(limitSwitchQ1n), ignore(limitSwitchQ2p), ignore(limitSwitchQ2n), ignore(limitSwitchQ3p), ignore(limitSwitchQ3n), });

	waitingForApproval         .setInputActions({ check(approval, true , approvalIsOn), ignore(limitSwitchQ0p), ignore(limitSwitchQ0n), ignore(limitSwitchQ1p),   ignore(limitSwitchQ1n), ignore(limitSwitchQ2p), ignore(limitSwitchQ2n), ignore(limitSwitchQ3p), ignore(limitSwitchQ3n), });
	systemOn                   .setInputActions({ check(approval, false, doEmergency),  ignore(limitSwitchQ0p), ignore(limitSwitchQ0n), ignore(limitSwitchQ1p),   ignore(limitSwitchQ1n), ignore(limitSwitchQ2p), ignore(limitSwitchQ2n), ignore(limitSwitchQ3p), ignore(limitSwitchQ3n), });
	
	powerOn                    .setInputActions({ check(approval, false, doEmergency),  check(limitSwitchQ0p, true , doEmergency), check(limitSwitchQ0n, true , doEmergency),        check(limitSwitchQ1p, true , doEmergency), check(limitSwitchQ1n, true, doEmergency),         check(limitSwitchQ2p, true, doEmergency),         check(limitSwitchQ2n, true , doEmergency), check(limitSwitchQ3p, true , doEmergency), check(limitSwitchQ3n, true , doEmergency),       });                                                                                                                                                                                                                                                                                                                                                                                                                                                                
	goingToReady               .setInputActions({ check(approval, false, doEmergency),  check(limitSwitchQ0p, true , doEmergency), check(limitSwitchQ0n, true , doEmergency),        check(limitSwitchQ1p, true , doEmergency), check(limitSwitchQ1n, true, doEmergency),         check(limitSwitchQ2p, true, doEmergency),         check(limitSwitchQ2n, true , doEmergency), check(limitSwitchQ3p, true , doEmergency), check(limitSwitchQ3n, true , doEmergency),       });
	ready                      .setInputActions({ check(approval, false, doEmergency),  check(limitSwitchQ0p, true , doEmergency), check(limitSwitchQ0n, true , doEmergency),        check(limitSwitchQ1p, true , doEmergency), check(limitSwitchQ1n, true, doEmergency),         check(limitSwitchQ2p, true, doEmergency),         check(limitSwitchQ2n, true , doEmergency), check(limitSwitchQ3p, true , doEmergency), check(limitSwitchQ3n, true , doEmergency),       });
	set_moving                 .setInputActions({ check(approval, false, doEmergency),  check(limitSwitchQ0p, true , doEmergency), check(limitSwitchQ0n, true , doEmergency),        check(limitSwitchQ1p, true , doEmergency), check(limitSwitchQ1n, true, doEmergency),         check(limitSwitchQ2p, true, doEmergency),         check(limitSwitchQ2n, true , doEmergency), check(limitSwitchQ3p, true , doEmergency), check(limitSwitchQ3n, true , doEmergency),       });
	moving                     .setInputActions({ check(approval, false, doEmergency),  check(limitSwitchQ0p, true , doEmergency), check(limitSwitchQ0n, true , doEmergency),        check(limitSwitchQ1p, true , doEmergency), check(limitSwitchQ1n, true, doEmergency),         check(limitSwitchQ2p, true, doEmergency),         check(limitSwitchQ2n, true , doEmergency), check(limitSwitchQ3p, true , doEmergency), check(limitSwitchQ3n, true , doEmergency),       });
	set_moving_joystick        .setInputActions({ check(approval, false, doEmergency),  check(limitSwitchQ0p, true , doEmergency), check(limitSwitchQ0n, true , doEmergency),        check(limitSwitchQ1p, true , doEmergency), check(limitSwitchQ1n, true, doEmergency),         check(limitSwitchQ2p, true, doEmergency),         check(limitSwitchQ2n, true , doEmergency), check(limitSwitchQ3p, true , doEmergency), check(limitSwitchQ3n, true , doEmergency),       });
	moving_joystick            .setInputActions({ check(approval, false, doEmergency),  check(limitSwitchQ0p, true , doEmergency), check(limitSwitchQ0n, true , doEmergency),        check(limitSwitchQ1p, true , doEmergency), check(limitSwitchQ1n, true, doEmergency),         check(limitSwitchQ2p, true, doEmergency),         check(limitSwitchQ2n, true , doEmergency), check(limitSwitchQ3p, true , doEmergency), check(limitSwitchQ3n, true , doEmergency),       });
	
	set_autoParking            .setInputActions({ check(approval, false, doEmergency),  ignore(limitSwitchQ0p), ignore(limitSwitchQ0n), ignore(limitSwitchQ1p), ignore(limitSwitchQ1n), ignore(limitSwitchQ2p), ignore(limitSwitchQ2n), ignore(limitSwitchQ3p), ignore(limitSwitchQ3n), });
	autoParking_shutdown3      .setInputActions({ check(approval, false, doEmergency),  ignore(limitSwitchQ0p), ignore(limitSwitchQ0n), ignore(limitSwitchQ1p), ignore(limitSwitchQ1n), ignore(limitSwitchQ2p), ignore(limitSwitchQ2n), ignore(limitSwitchQ3p), ignore(limitSwitchQ3n), });
	autoParking_shutdown2      .setInputActions({ check(approval, false, doEmergency),  ignore(limitSwitchQ0p), ignore(limitSwitchQ0n), ignore(limitSwitchQ1p), ignore(limitSwitchQ1n), ignore(limitSwitchQ2p), ignore(limitSwitchQ2n), ignore(limitSwitchQ3p), ignore(limitSwitchQ3n), });
	autoParking_shutdown1      .setInputActions({ check(approval, false, doEmergency),  ignore(limitSwitchQ0p), ignore(limitSwitchQ0n), ignore(limitSwitchQ1p), ignore(limitSwitchQ1n), ignore(limitSwitchQ2p), ignore(limitSwitchQ2n), ignore(limitSwitchQ3p), ignore(limitSwitchQ3n), });
	autoParking_shutdown0      .setInputActions({ check(approval, false, doEmergency),  ignore(limitSwitchQ0p), ignore(limitSwitchQ0n), ignore(limitSwitchQ1p), ignore(limitSwitchQ1n), ignore(limitSwitchQ2p), ignore(limitSwitchQ2n), ignore(limitSwitchQ3p), ignore(limitSwitchQ3n), });
                                                                                                                                                                                                                                                                                                                                                                                                                                                                       
	// Define output states and event for all levels 
	off                        .setOutputActions({ set(watchdog, false), set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false), set(brake0, true ), set(brake1, true ), set(brake2, true ), set(brake3, true ) });
	notinit_emergency          .setOutputActions({ set(watchdog, false), set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false), set(brake0, true ), set(brake1, true ), set(brake2, true ), set(brake3, true ) });
	notinit_waitingForApproval .setOutputActions({ toggle(watchdog    ), set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false), set(brake0, true ), set(brake1, true ), set(brake2, true ), set(brake3, true ) });
	notinit_systemOn           .setOutputActions({ toggle(watchdog    ), set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false), set(brake0, true ), set(brake1, true ), set(brake2, true ), set(brake3, true ) });
	manualParking3             .setOutputActions({ toggle(watchdog    ), set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false), set(brake0, true ), set(brake1, true ), set(brake2, true ), set(brake3, false) });
	manualParking2             .setOutputActions({ toggle(watchdog    ), set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false), set(brake0, true ), set(brake1, true ), set(brake2, false), set(brake3, true ) });
	manualParking1             .setOutputActions({ toggle(watchdog    ), set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false), set(brake0, true ), set(brake1, false), set(brake2, true ), set(brake3, true ) });
	manualParking0             .setOutputActions({ toggle(watchdog    ), set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false), set(brake0, false), set(brake1, true ), set(brake2, true ), set(brake3, true ) });
	robotParked                .setOutputActions({ toggle(watchdog    ), set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false), set(brake0, true ), set(brake1, true ), set(brake2, true ), set(brake3, true ) });
	homing3                    .setOutputActions({ toggle(watchdog    ), set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, true ), set(brake0, true ), set(brake1, true ), set(brake2, true ), set(brake3, false) });
	homing2                    .setOutputActions({ toggle(watchdog    ), set(enable0, false), set(enable1, false), set(enable2, true ), set(enable3, false), set(brake0, true ), set(brake1, true ), set(brake2, false), set(brake3, true ) });
	homing1                    .setOutputActions({ toggle(watchdog    ), set(enable0, false), set(enable1, true ), set(enable2, false), set(enable3, false), set(brake0, true ), set(brake1, false), set(brake2, true ), set(brake3, true)  });
	homing0                    .setOutputActions({ toggle(watchdog    ), set(enable0, true ), set(enable1, false), set(enable2, false), set(enable3, false), set(brake0, false), set(brake1, true ), set(brake2, true ), set(brake3, true ) });
	robotHomed                 .setOutputActions({ toggle(watchdog    ), set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false), set(brake0, true ), set(brake1, true ), set(brake2, true ), set(brake3, true ) });
	
	emergency                  .setOutputActions({ set(watchdog, false), set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false), set(brake0, true ), set(brake1, true ), set(brake2, true ), set(brake3, true ) });
	resetEmergency             .setOutputActions({ set(watchdog, false), set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false), set(brake0, true ), set(brake1, true ), set(brake2, true ), set(brake3, true ) });
	waitingForApproval         .setOutputActions({ toggle(watchdog    ), set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false), set(brake0, true ), set(brake1, true ), set(brake2, true ), set(brake3, true ) });
	systemOn                   .setOutputActions({ toggle(watchdog    ), set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false), set(brake0, true ), set(brake1, true ), set(brake2, true ), set(brake3, true ) });
	powerOn                    .setOutputActions({ toggle(watchdog    ), set(enable0, true ), set(enable1, true ), set(enable2, true ), set(enable3, true ), set(brake0, false), set(brake1, false), set(brake2, false), set(brake3, false) });
	goingToReady               .setOutputActions({ toggle(watchdog    ), set(enable0, true ), set(enable1, true ), set(enable2, true ), set(enable3, true ), set(brake0, false), set(brake1, false), set(brake2, false), set(brake3, false) });
	ready                      .setOutputActions({ toggle(watchdog    ), set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false), set(brake0, true ), set(brake1, true ), set(brake2, true ), set(brake3, true ) });	
	set_moving                 .setOutputActions({ toggle(watchdog    ), set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false), set(brake0, true ), set(brake1, true ), set(brake2, true ), set(brake3, true ) });
	moving                     .setOutputActions({ toggle(watchdog    ), set(enable0, true ), set(enable1, true ), set(enable2, true ), set(enable3, true ), set(brake0, false), set(brake1, false), set(brake2, false), set(brake3, false) });
	set_moving_joystick        .setOutputActions({ toggle(watchdog    ), set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false), set(brake0, true ), set(brake1, true ), set(brake2, true ), set(brake3, true ) });
	moving_joystick            .setOutputActions({ toggle(watchdog    ), set(enable0, true ), set(enable1, true ), set(enable2, true ), set(enable3, true ), set(brake0, false), set(brake1, false), set(brake2, false), set(brake3, false) });
	
	set_autoParking            .setOutputActions({ toggle(watchdog    ), set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false), set(brake0, true ), set(brake1, true ), set(brake2, true ), set(brake3, true ) });
	autoParking_shutdown3      .setOutputActions({ toggle(watchdog    ), set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, true ), set(brake0, true ), set(brake1, true ), set(brake2, true ), set(brake3, false) });
	autoParking_shutdown2      .setOutputActions({ toggle(watchdog    ), set(enable0, false), set(enable1, false), set(enable2, true ), set(enable3, false), set(brake0, true ), set(brake1, true ), set(brake2, false), set(brake3, true ) });
	autoParking_shutdown1      .setOutputActions({ toggle(watchdog    ), set(enable0, false), set(enable1, true ), set(enable2, false), set(enable3, false), set(brake0, true ), set(brake1, false), set(brake2, true ), set(brake3, true ) });
	autoParking_shutdown0      .setOutputActions({ toggle(watchdog    ), set(enable0, true ), set(enable1, false), set(enable2, false), set(enable3, false), set(brake0, false), set(brake1, true ), set(brake2, true ), set(brake3, true ) });
		
	// *** Define and add level functions *** //
	
	// Boot
	off.setLevelAction([this](SafetyContext* privateContext) 
	{
		static bool first = true; 
		if(first == true && off.getNofActivations()>5) {
			privateContext->triggerEvent(notinit_goToWaitingForApproval);
			first = false;
		}
	});

	// Emergency, not initialized
	notinit_emergency.setLevelAction([](SafetyContext* privateContext) 
	{
		throw Fault("EMERGENCY! Start the program again");
	});

	notinit_systemOn.setLevelAction([this](SafetyContext* privateContext) 
	{
		privateContext->triggerEvent(doManualParking); 
	});	
	
	robotParked.setLevelAction([this](SafetyContext* privateContext) 
	{
		safeUserLight->set(true);
		
		if(safeUserButton->get())
		{
			privateContext->triggerEvent(doHoming); 
			controlSys->initSwitch.switchToInput(0); // velocity input ref
		}
	});
	
	homing3.setLevelAction([this](SafetyContext* privateContext) 
	{
		static bool first = true;
		if(first == true)
		{
			AxisVector speed_init3 = {0.0, 0.0, 0.0, home_speed3};
			controlSys->initSpeed.setValue(speed_init3);
			first = false;
		}
		if(limitSwitchQ3n->get() == true)
		{
			q3_init = controlSys->enc3.getOut().getSignal().getValue();
			privateContext->triggerEvent(homingDone3);
		}
	});
	
	homing2.setLevelAction([this](SafetyContext* privateContext) 
	{
		static bool first = true;
		if(first == true)
		{
			AxisVector speed_init2 = {0.0, 0.0, home_speed2, 0.0};
			controlSys->initSpeed.setValue(speed_init2);
			first = false;
		}
		if(limitSwitchQ2p->get() == true)
		{
			q2r_init = controlSys->enc2.getOut().getSignal().getValue();
			privateContext->triggerEvent(homingDone2);
		}
		
	});
	
	homing1.setLevelAction([this](SafetyContext* privateContext) 
	{
		static bool first = true;
		if(first == true)
		{
			AxisVector speed_init1 = {0.0, home_speed1, 0.0, 0.0};
			controlSys->initSpeed.setValue(speed_init1);
			first = false;
		}
		if(limitSwitchQ1n->get() == true)
		{
			q1_init = controlSys->enc1.getOut().getSignal().getValue();
			privateContext->triggerEvent(homingDone1);
		}
	});
	
	homing0.setLevelAction([this](SafetyContext* privateContext) 
	{
		static bool first = true;
		if(first == true)
		{
			AxisVector speed_init0 = {home_speed0, 0.0, 0.0, 0.0};
			controlSys->initSpeed.setValue(speed_init0);
			first = false;
		}
		if(limitSwitchQ0n->get() == true)
		{
			q0_init = controlSys->enc0.getOut().getSignal().getValue();
			privateContext->triggerEvent(homingDone0);
		}
	});
	
	robotHomed.setLevelAction([this](SafetyContext* privateContext) 
	{ 
		double q3_motor  = controlSys->enc3.getOut().getSignal().getValue();
		double q2r_motor = controlSys->enc2.getOut().getSignal().getValue();
		double q1_motor  = controlSys->enc1.getOut().getSignal().getValue();
		double q0_motor  = controlSys->enc0.getOut().getSignal().getValue();
		
		static bool first = true;                                                                                  
		if(first)
		{   
			// Set offset 
			controlSys->enc3_offset.setValue( -q3_motor  +  0.0                        + (q3_motor  - q3_init ) );
			controlSys->enc2_offset.setValue( -q2r_motor - (2*M_PI*0.099/0.025)*(-1.5) + (q2r_motor - q2r_init) );
			controlSys->enc1_offset.setValue( -q1_motor  + (2.4245671 )*(100)          + (q1_motor  - q1_init ) );
			controlSys->enc0_offset.setValue( -q0_motor  + (2.59248134)*(100)          + (q0_motor  - q0_init ) );
			first = false;        
		}   
		
		// Check encoder after setting offset
		double q0 = controlSys->muxEncPos.getIn(0).getSignal().getValue();
		double q1 = controlSys->muxEncPos.getIn(1).getSignal().getValue();
		double q2 = controlSys->muxEncPos.getIn(2).getSignal().getValue();
		double q3 = controlSys->muxEncPos.getIn(3).getSignal().getValue();
		
		if(fabs(q0-home_check_angle0) < home_check_error && fabs(q1-home_check_angle1) < home_check_error && fabs(q2-home_check_angle2) < home_check_error && fabs(q3-home_check_angle3) < home_check_error)
		{
			// Set path planner and check output values
			controlSys->pathPlannerJS.setInitPos(controlSys->muxEncPos.getOut().getSignal().getValue());
			
			// Set also other path planners for kinematic (safety, not needed)
			controlSys->pathPlannerCS.setInitPos(controlSys->dirKin.getOut().getSignal().getValue());
			controlSys->xbox.setInitPos(controlSys->dirKin.getOut().getSignal().getValue());
			
			// Check Path Planner / Encoder
			AxisVector err = controlSys->pathPlannerJS.getPosOut().getSignal().getValue() - controlSys->muxEncPos.getOut().getSignal().getValue();
			
			if(fabs(err(0))>0.0005 || fabs(err(1))>0.0005 || fabs(err(2))>0.01 || fabs(err(3))>0.01)
			{
				controlSys->pathPlannerJS.setInitPos(controlSys->muxEncPos.getOut().getSignal().getValue());
			}
			else
			{
				controlSys->jointsRefSwitch.switchToInput(0);  // joint path planner active
				controlSys->initSwitch.switchToInput(1);       // position control
				
				// Check Pos Controller Sum = 0
				AxisVector err = controlSys->posSum.getIn(0).getSignal().getValue() - controlSys->posSum.getIn(1).getSignal().getValue();
				if(fabs(err(0))>0.0005 || fabs(err(1))>0.0005 || fabs(err(2))>0.01 || fabs(err(3))>0.01){
					controlSys->initSwitch.switchToInput(1);
				}
				else
				{
					std::cout << "Init angles " << controlSys->muxEncPos.getOut().getSignal().getValue() << std::endl;
					privateContext->triggerEvent(doSystemOn);
				}
			}
		}
	});

	resetEmergency.setLevelAction([this](SafetyContext* privateContext) 
	{
		AxisVector actualPos_phi = controlSys->muxEncPos.getOut().getSignal().getValue();
		AxisVector actualPos_xyz = controlSys->dirKin.getOut().getSignal().getValue();
		
		controlSys->pathPlannerJS.setInitPos(actualPos_phi);
		controlSys->pathPlannerCS.setInitPos(actualPos_xyz);
		controlSys->xbox.setInitPos(actualPos_xyz);
		
		AxisVector pathJS = controlSys->pathPlannerJS.getPosOut().getSignal().getValue();
		AxisVector pathCS = controlSys->pathPlannerCS.getPosOut().getSignal().getValue();
		AxisVector xbox   = controlSys->xbox.getOut().getSignal().getValue();
		
		// Check settings before going to moving
		AxisVector err_JS   = pathJS - actualPos_phi;
		AxisVector err_CS   = pathCS - actualPos_xyz;
		AxisVector err_xbox = xbox   - actualPos_xyz;
		
		if( fabs(err_JS  (0))>0.0005 || fabs(err_JS  (1))>0.0005 || fabs(err_JS  (2))>0.01 || fabs(err_JS  (3))>0.01 ||
			fabs(err_CS  (0))>0.0005 || fabs(err_CS  (1))>0.0005 || fabs(err_CS  (2))>0.01 || fabs(err_CS  (3))>0.01 ||
			fabs(err_xbox(0))>0.0005 || fabs(err_xbox(1))>0.0005 || fabs(err_xbox(2))>0.01 || fabs(err_xbox(3))>0.01    )
		{
			controlSys->pathPlannerCS.setInitPos(actualPos_xyz);
			controlSys->xbox.setInitPos(actualPos_xyz);
			controlSys->pathPlannerJS.setInitPos(actualPos_phi);
		}
		else
		{
			// Check Pos Controller Sum = 0
			AxisVector err = controlSys->posSum.getIn(0).getSignal().getValue() - controlSys->posSum.getIn(1).getSignal().getValue();
			if(fabs(err(0))>0.0005 || fabs(err(1))>0.0005 || fabs(err(2))>0.01 || fabs(err(3))>0.01)
			{
				controlSys->initSwitch.switchToInput(1);      // position control
				controlSys->cartesRefSwitch.switchToInput(0); // path planner cartesian
				controlSys->jointsRefSwitch.switchToInput(1); // inv kinematic
			}
			else
			{
				privateContext->triggerEvent(resetEmergencyDone);
			}
		}
	});
	
	waitingForApproval.setLevelAction([](SafetyContext* privateContext) 
	{
		static bool first = true;
		if(first) {
			std::cout << "Press the white button" << std::endl;
			first = false;
		}
	});
	
	systemOn.setLevelAction([this](SafetyContext* privateContext) 
	{
		safeUserLight->set(true);
		if(safeUserButton->get()) 
		{
			privateContext->triggerEvent(doPowerUp); 
		}
	});
	
	powerOn.setLevelAction([this](SafetyContext* privateContext)
	{
		if(powerOn.getNofActivations() > 5)
			privateContext->triggerEvent(goToReady);
	});
	
	set_moving.setLevelAction([this](SafetyContext* privateContext) 
	{
		AxisVector x_actual = controlSys->dirKin.getOut().getSignal().getValue();
		controlSys->pathPlannerCS.setInitPos(x_actual);
		
		// Check settings before going to moving
		AxisVector err = controlSys->pathPlannerCS.getPosOut().getSignal().getValue() - controlSys->dirKin.getOut().getSignal().getValue();
		
		if(fabs(err(0))>0.0005 || fabs(err(1))>0.0005 || fabs(err(2))>0.01 || fabs(err(3))>0.01)
		{
			controlSys->pathPlannerCS.setInitPos(controlSys->dirKin.getOut().getSignal().getValue());
		}
		else
		{
			controlSys->jointsRefSwitch.switchToInput(1);  // cartesian path planner active
			controlSys->initSwitch.switchToInput(1);       // position control
			
			// Check Pos Controller Sum = 0
			AxisVector err = controlSys->posSum.getIn(0).getSignal().getValue() - controlSys->posSum.getIn(1).getSignal().getValue();
			if(fabs(err(0))>0.0005 || fabs(err(1))>0.0005 || fabs(err(2))>0.01 || fabs(err(3))>0.01){
				controlSys->initSwitch.switchToInput(1);
			}
			else
			{
				privateContext->triggerEvent(doStartMoving);
			}
		}
	});
	
	set_moving_joystick.setLevelAction([this](SafetyContext* privateContext) 
	{
		AxisVector x_actual = controlSys->dirKin.getOut().getSignal().getValue();
		controlSys->xbox.setInitPos(x_actual);
		
		// Check settings before going to moving
		AxisVector err = controlSys->xbox.getOut().getSignal().getValue() - controlSys->dirKin.getOut().getSignal().getValue();
		
		if(fabs(err(0))>0.0005 || fabs(err(1))>0.0005 || fabs(err(2))>0.01 || fabs(err(3))>0.01)
		{
			controlSys->xbox.setInitPos(controlSys->dirKin.getOut().getSignal().getValue());
		}
		else
		{
			controlSys->jointsRefSwitch.switchToInput(0);  // xbox (cartesian) active
			controlSys->initSwitch.switchToInput(1);       // position control
			
			// Check Pos Controller Sum = 0
			AxisVector err = controlSys->posSum.getIn(0).getSignal().getValue() - controlSys->posSum.getIn(1).getSignal().getValue();
			if(fabs(err(0))>0.0005 || fabs(err(1))>0.0005 || fabs(err(2))>0.01 || fabs(err(3))>0.01){
				controlSys->initSwitch.switchToInput(1);
			}
			else
			{
				privateContext->triggerEvent(doStartMoving_joystick);
			}
		}
	});
	
	set_autoParking.setLevelAction([this](SafetyContext* privateContext) 
	{
		AxisVector actualPos = controlSys->muxEncPos.getOut().getSignal().getValue();
		controlSys->pathPlannerJS.setInitPos(actualPos);
		
		// Check path planner setting
		AxisVector err = controlSys->pathPlannerJS.getPosOut().getSignal().getValue() - controlSys->muxEncPos.getOut().getSignal().getValue();
		
		if(fabs(err(0))>0.0005 || fabs(err(1))>0.0005 || fabs(err(2))>0.01 || fabs(err(3))>0.01)
		{
			controlSys->pathPlannerJS.setInitPos(controlSys->muxEncPos.getOut().getSignal().getValue());
		}
		else
		{
			controlSys->jointsRefSwitch.switchToInput(0);  // joint path planner active
			controlSys->initSwitch.switchToInput(1);       // position control
			
			// Check Pos Controller Sum = 0
			AxisVector err = controlSys->posSum.getIn(0).getSignal().getValue() - controlSys->posSum.getIn(1).getSignal().getValue();
			if(fabs(err(0))>0.0005 || fabs(err(1))>0.0005 || fabs(err(2))>0.01 || fabs(err(3))>0.01){
				controlSys->initSwitch.switchToInput(1);
			}
			else
			{
				privateContext->triggerEvent(doStartAutoParking);
			}
		}
	});
	
	autoParking_shutdown3.setLevelAction([this](SafetyContext* privateContext) 
	{
		static bool first = true;
		if(first == true)
		{
			AxisVector actualPos = controlSys->muxEncPos.getOut().getSignal().getValue();
			AxisVector pos; pos << actualPos(0), actualPos(1), 0.105, 0.78; 
			controlSys->pathPlannerJS.move(pos);
			first = false;
		}
		
		if(controlSys->pathPlannerJS.posReached())
			privateContext->triggerEvent(autoParking_shutdownDone3);
	});
	
	autoParking_shutdown2.setLevelAction([this](SafetyContext* privateContext) 
	{
		static bool first = true;
		if(first == true)
		{
			AxisVector actualPos = controlSys->muxEncPos.getOut().getSignal().getValue();
			AxisVector pos; pos << actualPos(0), actualPos(1), 0.105, 0.78;  
			controlSys->pathPlannerJS.move(pos);
			first = false;
		}
		
		if(controlSys->pathPlannerJS.posReached())
			privateContext->triggerEvent(autoParking_shutdownDone2);
	});
	
	autoParking_shutdown1.setLevelAction([this](SafetyContext* privateContext) 
	{
		static bool first = true;
		if(first == true)
		{
			AxisVector actualPos = controlSys->muxEncPos.getOut().getSignal().getValue();
			AxisVector pos; pos << actualPos(0), 2.4, actualPos(2), actualPos(3); 
			controlSys->pathPlannerJS.move(pos);
			first = false;
		}
		
		if(controlSys->pathPlannerJS.posReached())
			privateContext->triggerEvent(autoParking_shutdownDone1);
	});
	
	autoParking_shutdown0.setLevelAction([this](SafetyContext* privateContext) 
	{
		static bool first = true;
		if(first == true)
		{
			AxisVector actualPos = controlSys->muxEncPos.getOut().getSignal().getValue();
			AxisVector pos; pos << 2.4, actualPos(1), actualPos(2), actualPos(3); 
			controlSys->pathPlannerJS.move(pos);
			first = false;
		}
		
		if(controlSys->pathPlannerJS.posReached())
			privateContext->triggerEvent(autoParking_shutdownDone0);
	});
	
	// Define entry level
	setEntryLevel(off);
}

ScaraSafetyProperties::~ScaraSafetyProperties() {
	// nothing to do
}

// TEST GREEN BUTTON
// 			safeUserLight->set(true);
// 			if(safeUserButton->get()){
// 				controlSys->switchToInitSpeed();               // TODO 
// 				privateContext->triggerEvent(doControlStart);  // TODO
// 			}
