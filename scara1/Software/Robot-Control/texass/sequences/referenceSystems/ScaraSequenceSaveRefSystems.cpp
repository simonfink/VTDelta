#include "ScaraSequenceSaveRefSystems.hpp"
#include "ScaraSequenceCalculateRefSystem.hpp"
#include "../tools/ScaraSequenceCheckTool.hpp"
#include "../../ScaraSafetyProperties.hpp"
#include "../../ScaraControlSystem.hpp"
#include <eeros/safety/SafetySystem.hpp>
#include <iostream>
#include <string.h>
#include <unistd.h>
#include "../../constants.hpp"

using namespace scara;
using namespace eeros::control;
using namespace eeros::sequencer;
using namespace eeros::safety;
using namespace eeros::math;

ScaraSequenceSaveRefSystems::ScaraSequenceSaveRefSystems(Sequencer* sequencer, ScaraControlSystem* controlSys, SafetySystem* safetySys) : 
											Sequence<>("main", sequencer), controlSys(controlSys), safetySys(safetySys), 
											calcRefSystem(sequencer, controlSys, safetySys), checkTool(sequencer, controlSys, safetySys), 
											goToTool(sequencer, controlSys, safetySys), goToReady(sequencer, controlSys, safetySys) {
	// nothing to do
}

void ScaraSequenceSaveRefSystems::init() {
	std::bind(&ScaraSequenceSaveRefSystems::init, *this);
}

bool ScaraSequenceSaveRefSystems::checkPreCondition() {
	return safetySys->getCurrentLevel().getId() == ready;
}

void ScaraSequenceSaveRefSystems::run() {
	log.info() << "[ Save Ref System ] started";
	
	goToTool();
	goToReady();
	safetySys->triggerEvent(doTeaching);
	usleep(100000);
	
	AxisVector x_actual = controlSys->dirKin.getOut().getSignal().getValue();
	controlSys->pathPlannerCS.setInitPos(x_actual);
	controlSys->posIntegral.setInitCondition(x_actual);
	controlSys->xbox.setInitPos(x_actual);
	controlSys->pathPlannerPosSwitch.switchToInput(1);	// cartesian path planner
	controlSys->autoToManualSwitch.switchToInput(1);	// manual mode
	usleep(100000); 
	
	controlSys->posIntegral.enable(); 
	
	char a = 0; double inputScale;
	AxisVector actPos, bauPos;
	while(a != 'b') {
		log.info() << "'1' = change frame 'Gestell' (KG)";
		log.info() << "'2' = change frame 'Spannrahmen' (KS)";
		log.info() << "'3' = change frame 'Bauteile' (KB)"; 
		log.info() << "'4' = change frame 'Mesh' (KM)"; 
		log.info() << "'v' = change joystick speed (scaling factor, default = 1.0)";
		log.info() << "'b' = go back to the main menu";
	
		std::cin >> a;
		switch(a) {
			case '1':
				calcRefSystem(&controlSys->KG); 
				break;
			case '2':
				calcRefSystem(&controlSys->KS);
				break;
			case '3':
				calcRefSystem(&controlSys->KB); 
				break;
			case '4':
				checkTool('c', controlSys->toBasisCoordinate(controlSys->crossRefPoint, 'n', controlSys->bauteile));
				safetySys->triggerEvent(doTeaching);
				calcRefSystem(&controlSys->KM);
				break;
			case 'v':
				log.warn() << "write a positive scaling factor for the speed (default = 1.0) and then press ENTER";
				std::cin >> inputScale; 
				controlSys->xbox.setSpeedScaleFactor(inputScale);
				log.info() << "Velocity scale set to: " << inputScale;
				break;
			default:
				// nothing to do
				break;
		}
	}
	
	controlSys->posIntegral.disable(); 
	safetySys->triggerEvent(doMotionStopping);
	while(!(safetySys->getCurrentLevel().getId() == ready));
	
	// Go to reeady
	goToReady();
}

bool ScaraSequenceSaveRefSystems::checkPostCondition() {
	return safetySys->getCurrentLevel().getId() == ready;
	return true;
}

void ScaraSequenceSaveRefSystems::exit() {
	AxisVector x_actual = controlSys->dirKin.getOut().getSignal().getValue();
	controlSys->pathPlannerCS.setInitPos(x_actual);
	controlSys->autoToManualSwitch.switchToInput(0);
	controlSys->xbox.setSpeedScaleFactor(1.0);
	log.info() << "[ Save Ref System ] exit done";
}
