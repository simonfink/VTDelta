#include "ScaraSequenceSaveCalibratedPartsStorage.hpp"
#include "../../ScaraSafetyProperties.hpp"
#include "../../ScaraControlSystem.hpp"
#include <eeros/safety/SafetySystem.hpp>
#include <iostream>
#include <string.h>
#include <unistd.h>
#include <fstream>

#include "../tools/ScaraSequenceGoToToolChange.hpp"
#include "../goingToReady/ScaraSequenceGoToReady.hpp"

using namespace scara;
using namespace eeros::control;
using namespace eeros::sequencer;
using namespace eeros::safety;
using namespace eeros::math;

ScaraSequenceSaveCalibratedPartsStorage::ScaraSequenceSaveCalibratedPartsStorage(Sequencer* sequencer, ScaraControlSystem* controlSys, SafetySystem* safetySys) : 
										Sequence<>("main", sequencer), controlSys(controlSys), safetySys(safetySys),
										checkTool(sequencer, controlSys, safetySys), goToReady(sequencer, controlSys, safetySys),
										goToTool(sequencer, controlSys, safetySys) {
	// nothing to do
}

void ScaraSequenceSaveCalibratedPartsStorage::init() {
	std::bind(&ScaraSequenceSaveCalibratedPartsStorage::init, *this);
}

bool ScaraSequenceSaveCalibratedPartsStorage::checkPreCondition() {
	return safetySys->getCurrentLevel().getId() == ready;
}

void ScaraSequenceSaveCalibratedPartsStorage::run() {
	log.info() << "[ Calibration Parts-Storage ] started";
	
	goToTool();
	goToReady();
	usleep(100000); 
	
	AxisVector x_actual = controlSys->dirKin.getOut().getSignal().getValue();
	controlSys->pathPlannerCS.setInitPos(x_actual);
	controlSys->xbox.setInitPos(x_actual);
	controlSys->pathPlannerPosSwitch.switchToInput(1);	// cartesian path planner
	controlSys->autoToManualSwitch.switchToInput(1);	// manual mode
	
	static int count = 0;
	
	// load parts reference points in a matrix
	int line = 0;
	std::fstream file;
	file.open("parts_calibrationReferencePoints.txt", std::fstream::in);
	if(!file.is_open()) throw EEROSException("File for loading calibration ref. points is not open!");
	
	int j = 0; 
	while (!file.eof() && j < 24) {
		for(int i = 0; i<4; i++)
				file >> parts_referencePoints(j,i);
		j++;
	}
	file.close();
	
	// check camera offset and set it - check tool
	safetySys->triggerEvent(doTeaching);
	checkTool('c', controlSys->toBasisCoordinate(controlSys->crossRefPoint, 'n', controlSys->bauteile));
	goToReady();

	// run calibration for every point
	char m = 0;
	while (count < 24) {
		// *** automatic movement *** //
		// get position 
		AxisVector pos_parts; pos_parts << parts_referencePoints(count, 0), parts_referencePoints(count, 1), parts_referencePoints(count, 2), parts_referencePoints(count, 3);
		log.info() << "Parts Storage  -> Count: " << count << " , Pos: " << pos_parts(0) << "\t " << pos_parts(1) << "\t " << pos_parts(2) << "\t " << pos_parts(3);
		AxisVector pos_robot; pos_robot = controlSys->toBasisCoordinate(pos_parts, 'c', controlSys->bauteile); 
		
		// init path planner and integral
		x_actual = controlSys->dirKin.getOut().getSignal().getValue();
		controlSys->pathPlannerCS.setInitPos(x_actual);
		controlSys->posIntegral.setInitCondition(x_actual);
		controlSys->posIntegral.enable(); 
		controlSys->autoToManualSwitch.switchToInput(0);	// automatic mode
		// go to position
		safetySys->triggerEvent(doStartingMotion); 
		while(safetySys->getCurrentLevel().getId() < moving); 

		controlSys->pathPlannerCS.gotoPoint(pos_robot);
		while (!controlSys->pathPlannerCS.posReached()) {
			usleep(100000);
		}
		controlSys->posIntegral.disable(); 
		safetySys->triggerEvent(doMotionStopping);
		while(!(safetySys->getCurrentLevel().getId() == ready));
		
		// *** manual calibration *** //
		x_actual = controlSys->dirKin.getOut().getSignal().getValue();
		controlSys->pathPlannerCS.setInitPos(x_actual);
		controlSys->pathPlannerPosSwitch.switchToInput(1);	// cartesian path planner
		controlSys->xbox.setInitPos(x_actual);
		controlSys->autoToManualSwitch.switchToInput(1);	// manual mode
		safetySys->triggerEvent(doTeaching);
		controlSys->posIntegral.setInitCondition(x_actual);
		controlSys->posIntegral.enable();
		controlSys->xbox.setSpeedScaleFactor(0.01);

		log.info() << "Press 's' + ENTER to save a point";

		AxisVector pos_robot_out; AxisVector pos_parts_out; std::fstream file;
		std::cin >> m;

		switch(m) {
			case 's':
				pos_robot_out = controlSys->dirKin.getOut().getSignal().getValue();
				pos_parts_out = controlSys->toUserCoordinate(pos_robot_out, 'c', controlSys->bauteile); 
				
	 			// Save calibrated position to file
				file.open("parts_calibrationList.txt", std::fstream::out | std::fstream::app);
				if(!file.is_open()) throw EEROSException("File for saving calibration list is not open!");
		
				file << count; file << "\t";
				file << " ( " << pos_parts(0) << " , " << pos_parts(1) << " ) => ( " << pos_parts_out(0) << " , " << pos_parts_out(1) << " ) ;";
				file << "\n"; file.close();
				log.info() << "Reference point and calibrated point saved into 'parts_calibrationList.txt' ";
				break;
			default:
				// nothing to do
				break;
		}
		// do motion doMotionStopping
		controlSys->posIntegral.disable(); 
		safetySys->triggerEvent(doMotionStopping);
		while(!(safetySys->getCurrentLevel().getId() == ready));

		// counter
		count++;
	}
	
	goToReady();
}

bool ScaraSequenceSaveCalibratedPartsStorage::checkPostCondition() {
	return safetySys->getCurrentLevel().getId() == ready;
}

void ScaraSequenceSaveCalibratedPartsStorage::exit() {
	AxisVector x_actual = controlSys->dirKin.getOut().getSignal().getValue();
	controlSys->pathPlannerCS.setInitPos(x_actual);
	controlSys->autoToManualSwitch.switchToInput(0);
	log.info() << "[ Calibration Parts-Storage ] exit done";
}
