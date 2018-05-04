#include <eeros/safety/SafetySystem.hpp>
#include "ScaraSequenceMain.hpp"
#include "ScaraSafetyProperties.hpp"
#include "ScaraControlSystem.hpp"
#include <unistd.h>
#include <queue>

using namespace scara;
using namespace eeros::sequencer;
using namespace eeros::safety;
using namespace eeros::math;

ScaraSequenceMain::ScaraSequenceMain(Sequencer* sequencer, ScaraControlSystem* controlSys, SafetySystem* safetySys) : 
									Sequence<void>("main", sequencer), controlSys(controlSys), safetySys(safetySys), 
									move(sequencer, controlSys, safetySys), manualPark(sequencer, controlSys, safetySys), 
									homing(sequencer, controlSys, safetySys), homeToReady(sequencer, controlSys, safetySys),
									loadRefSystems(sequencer, controlSys, safetySys), autopark(sequencer, controlSys, safetySys), 
									goToTool(sequencer, controlSys, safetySys), goToReady(sequencer, controlSys, safetySys),
									saveRefSystem(sequencer, controlSys, safetySys), joystickMove(sequencer, controlSys, safetySys),
									calibratePartsStorage(sequencer, controlSys, safetySys), calibrateMesh(sequencer, controlSys, safetySys), 
									checkTool(sequencer, controlSys, safetySys), dispencer(sequencer, controlSys, safetySys), 
									grabber(sequencer, controlSys, safetySys), plasmapen(sequencer, controlSys, safetySys),
									texass(sequencer, controlSys, safetySys), loadWorkspaceLimitation(sequencer, controlSys, safetySys) {
	// nothing to do
}

bool ScaraSequenceMain::checkPreCondition() {
	return safetySys->getCurrentLevel().getId() >= off;
}

void ScaraSequenceMain::run() {
	log.trace() << "Sequencer '" << name << "': started.";
	
	while(safetySys->getCurrentLevel().getId() != baseSystemOn);	// 1. Boot
	manualPark(); 													// 2. Manual Parking
	homing();														// 3. Automatic Homing
	homeToReady();													// 4. Going to Ready Position
	
	// 5. Set all path planners and controllers to READY position
	AxisVector q_actual = controlSys->muxEncPos.getOut().getSignal().getValue();
	AxisVector x_actual = controlSys->dirKin.getOut().getSignal().getValue();
	controlSys->pathPlannerJS.setInitPos(q_actual);
	controlSys->pathPlannerCS.setInitPos(x_actual);
	controlSys->xbox.setInitPos(x_actual);
	usleep(100000);
	
	// 6. Load default reference systems, workspace limitations & calibration tables
	loadRefSystems();
	loadWorkspaceLimitation();
	controlSys->meshCalibrationLut.load("mesh_calibrationTable.txt");
	controlSys->partsCalibrationLut.load("parts_calibrationTable.txt");
	
	// 7. Run sequences (texass, plasmapen, dispencer, etc.)
	char m = 0;
	while(m != 'k') {
		log.info() << "1 = Change reference systems";
		log.info() << "2 = Calibrate parts storage field";
		log.info() << "3 = Calibrate mesh field";
		log.info() << "4 = Load again calibration tables (to do after modifying calibration tables on txt files)";
// 		log.info() << "t = run TEXASS (only plasmapen) sequence";
		log.info() << "e = run 'plasmapen' sequence";
		log.info() << "d = run 'dispencer' sequence";
		log.info() << "g = run 'greifer' sequence";
		log.info() << "m = manual mode";
		log.info() << "p = park the robot and shut down";
		
		std::cin >> m;
		switch(m) {
			case '1':
				saveRefSystem();
				break;
			case '2':
				calibratePartsStorage();
				break;
			case '3':
				calibrateMesh();
				break;
			case '4':		// Load again calibration tables
				controlSys->meshCalibrationLut.load("mesh_calibrationTable.txt");
				controlSys->partsCalibrationLut.load("parts_calibrationTable.txt");
				break;
// 			case 't':
// 				texass();
// 				break;
			case 'e':
				plasmapen();
				break;
			case 'd':
				dispencer();
				break;
			case 'g':
				grabber(); 
				break;
			case 'm':	// Manual Mode
				safetySys->triggerEvent(doTeaching); 
				joystickMove();
				break;
			case 'p':	// Exit from loop
				m = 'k';
			default:
				// nothing to do
				break;
		}
	}

	// 12. Autopark
	autopark();
	// 13. Shut down
	safetySys->triggerEvent(doStopControl);
	while(!(safetySys->getCurrentLevel().getId() == off));

	log.trace() << "Sequencer '" << name << "': finished";
}

void ScaraSequenceMain::exit() {
	log.info() << "[ Exit Main Sequence ]";
}