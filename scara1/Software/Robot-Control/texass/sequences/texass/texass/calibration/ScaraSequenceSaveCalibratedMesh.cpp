#include "ScaraSequenceSaveCalibratedMesh.hpp"
#include "../../ScaraSafetyProperties.hpp"
#include "../../ScaraControlSystem.hpp"
#include <eeros/safety/SafetySystem.hpp>
#include <iostream>
#include <string.h>
#include <unistd.h>
#include <fstream>

using namespace scara;
using namespace eeros::control;
using namespace eeros::sequencer;
using namespace eeros::safety;
using namespace eeros::math;

ScaraSequenceSaveCalibratedMesh::ScaraSequenceSaveCalibratedMesh(Sequencer* sequencer, ScaraControlSystem* controlSys, SafetySystem* safetySys) : 
										Sequence<>("main", sequencer), controlSys(controlSys), safetySys(safetySys),
										checkTool(sequencer, controlSys, safetySys), goToReady(sequencer, controlSys, safetySys),
										goToTool(sequencer, controlSys, safetySys) {
	// nothing to do
}

void ScaraSequenceSaveCalibratedMesh::init() {
	std::bind(&ScaraSequenceSaveCalibratedMesh::init, *this);
}

bool ScaraSequenceSaveCalibratedMesh::checkPreCondition() {
	return safetySys->getCurrentLevel().getId() == ready;
}

void ScaraSequenceSaveCalibratedMesh::run() {
	log.info() << "[ Calibration Mesh ] started";
	
	goToTool();
	goToReady();
	usleep(100000); 
	
	AxisVector x_actual = controlSys->dirKin.getOut().getSignal().getValue();
	controlSys->pathPlannerCS.setInitPos(x_actual);
	controlSys->xbox.setInitPos(x_actual);
	controlSys->pathPlannerPosSwitch.switchToInput(1);	// cartesian path planner
	controlSys->autoToManualSwitch.switchToInput(1);	// manual mode
	
	static int count = 0;
	
	// load mesh reference points in a matrix
	int line = 0;
	std::fstream file;
	file.open("mesh_calibrationReferencePoints.txt", std::fstream::in);
	if(!file.is_open()) throw EEROSException("File for loading calibration ref. points is not open!");
	
	int j = 0; 
	while (!file.eof() && j < 9) {
		for(int i = 0; i<4; i++)
				file >> mesh_referencePoints(j,i);
		j++;
	}
	file.close();
	
	// check camera offset and set it - check tool
	safetySys->triggerEvent(doTeaching);
	checkTool('c', controlSys->toBasisCoordinate(controlSys->crossRefPoint, 'n', controlSys->bauteile));
	goToReady();

	// run calibration for every point
	char m = 0;
	while (count < 9) {
		// *** automatic movement *** //
		// get position 
		AxisVector pos_mesh; pos_mesh << mesh_referencePoints(count, 0), mesh_referencePoints(count, 1), mesh_referencePoints(count, 2), mesh_referencePoints(count, 3);
		log.info() << "Mesh  -> Count: " << count << " , Pos: " << pos_mesh(0) << "\t " << pos_mesh(1) << "\t " << pos_mesh(2) << "\t " << pos_mesh(3);
		AxisVector pos_robot; pos_robot = controlSys->toBasisCoordinate(pos_mesh, 'c', controlSys->mesh); 
		
		// init path planner and integral
		x_actual = controlSys->dirKin.getOut().getSignal().getValue();
		controlSys->pathPlannerCS.setInitPos(x_actual);
		controlSys->posIntegral.setInitCondition(x_actual);
		controlSys->posIntegral.enable(); 
		controlSys->autoToManualSwitch.switchToInput(0);		// automatic mode
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
		AxisVector pos_robot_out; AxisVector pos_mesh_out; std::fstream file;
		std::cin >> m;
		switch(m) {
			case 's':
				pos_robot_out = controlSys->dirKin.getOut().getSignal().getValue();
				pos_mesh_out = controlSys->toUserCoordinate(pos_robot_out, 'c', controlSys->mesh); 
				
	 			// Save calibrated position to file
				file.open("mesh_calibrationList.txt", std::fstream::out | std::fstream::app);
				if(!file.is_open()) throw EEROSException("File for saving calibration list is not open!");
		
				file << count; file << "\t";
				file << " ( " << pos_mesh(0) << " , " << pos_mesh(1) << " ) => ( " << pos_mesh_out(0) << " , " << pos_mesh_out(1) << " ) ;";
				file << "\n"; file.close();
				log.info() << "Reference point and calibrated point saved into 'mesh_calibrationList.txt' ";
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

bool ScaraSequenceSaveCalibratedMesh::checkPostCondition() {
	return safetySys->getCurrentLevel().getId() == ready;
}

void ScaraSequenceSaveCalibratedMesh::exit() {
	AxisVector x_actual = controlSys->dirKin.getOut().getSignal().getValue();
	controlSys->pathPlannerCS.setInitPos(x_actual);
	controlSys->autoToManualSwitch.switchToInput(0);
	log.info() << "[ Calibration Mesh ] exit done";
}
