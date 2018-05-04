#include "ScaraSequenceTEXASSsingleGrab.hpp"
#include "../../../ScaraControlSystem.hpp"
#include "../../../ScaraSafetyProperties.hpp"
#include <eeros/safety/SafetySystem.hpp>
#include <unistd.h>
#include <iostream>
#include "../../../types.hpp"
#include <eeros/hal/HAL.hpp>

using namespace scara;
using namespace eeros::control;
using namespace eeros::sequencer;
using namespace eeros::safety;
using namespace eeros::hal;
using namespace eeros::math;

ScaraSequenceTEXASSsingleGrab::ScaraSequenceTEXASSsingleGrab(Sequencer* sequencer, ScaraControlSystem* controlSys, SafetySystem* safetySys) : 
													Sequence<void, Vector2, Vector2, double, double, double>
													("main", sequencer), controlSys(controlSys), safetySys(safetySys) {
	// nothing to do
}

void ScaraSequenceTEXASSsingleGrab::init() {
	std::bind(&ScaraSequenceTEXASSsingleGrab::init, *this);
}

bool ScaraSequenceTEXASSsingleGrab::checkPreCondition() {
	return safetySys->getCurrentLevel().getId() == moving;
}

void ScaraSequenceTEXASSsingleGrab::run(Vector2 xy_parts, Vector2 xy_mesh, double z_high, double z_low_parts, double z_low_mesh) {
	HAL& hal = HAL::instance();
	
	// bauteile - high
	p <<  xy_parts(0), xy_parts(1), z_high, 1;
	p = controlSys->toCalibratedValue(p, controlSys->partsCalibrationLut);
	p = controlSys->toBasisCoordinate(p, 'g', controlSys->bauteile);
	controlSys->pathPlannerCS.gotoPoint(p);
	while (!controlSys->pathPlannerCS.posReached()) {usleep(100000);}
	
	// Set workspace limitation parameters / enable workspace limitation
	AxisVector offset; offset << controlSys->greiferX, controlSys->greiferY, controlSys->greiferZ, controlSys->greiferAlpha;
	controlSys->cartesianWorkspaceLimit.setParameters(controlSys->parts_cartesianLimitation_parameters, offset);
	controlSys->cartesianWorkspaceLimit.enable();
	
	// bauteile - low
	p <<  xy_parts(0), xy_parts(1), z_low_parts, 1;
	p = controlSys->toCalibratedValue(p, controlSys->partsCalibrationLut);
	p = controlSys->toBasisCoordinate(p, 'g', controlSys->bauteile);
	controlSys->pathPlannerCS.gotoPoint(p);
	while (!controlSys->pathPlannerCS.posReached()) {usleep(100000);}
	
	// vacuum on
	hal.getLogicPeripheralOutput("vakuum")->set(true);
	sleep(1);
	
	// bauteile - high
	p <<  xy_parts(0), xy_parts(1), z_high, 1;
	p = controlSys->toCalibratedValue(p, controlSys->partsCalibrationLut);
	p = controlSys->toBasisCoordinate(p, 'g', controlSys->bauteile);
	controlSys->pathPlannerCS.gotoPoint(p);
	while (!controlSys->pathPlannerCS.posReached()) {usleep(100000);}
	
	// Set workspace limitation parameters / enable workspace limitation	
	controlSys->cartesianWorkspaceLimit.disable();
	
	// mesh - high
	p << xy_mesh(0), xy_mesh(1), z_high, 1;
	p = controlSys->toCalibratedValue(p, controlSys->meshCalibrationLut);
	p = controlSys->toBasisCoordinate(p, 'g', controlSys->mesh);
	controlSys->pathPlannerCS.gotoPoint(p);
	while (!controlSys->pathPlannerCS.posReached()) {usleep(100000);}
	
	// Set workspace limitation parameters / enable workspace limitation
	controlSys->cartesianWorkspaceLimit.setParameters(controlSys->mesh_cartesianLimitation_parameters, offset);
	controlSys->cartesianWorkspaceLimit.enable();
			
	// mesh - low
	p << xy_mesh(0), xy_mesh(1), z_low_mesh, 1;
	p = controlSys->toCalibratedValue(p, controlSys->meshCalibrationLut);
	p = controlSys->toBasisCoordinate(p, 'g', controlSys->mesh);
	controlSys->pathPlannerCS.gotoPoint(p);
	while (!controlSys->pathPlannerCS.posReached()) {usleep(100000);}
		
	// vacuum off
	hal.getLogicPeripheralOutput("vakuum")->set(false);
	sleep(1);
	hal.getLogicPeripheralOutput("vakuumAusblasen")->set(true);
	sleep(1);
	hal.getLogicPeripheralOutput("vakuumAusblasen")->set(false);
	sleep(1);
	
	// mesh - high
	p << xy_mesh(0), xy_mesh(1), z_high, 1;
	p = controlSys->toCalibratedValue(p, controlSys->meshCalibrationLut);
	p = controlSys->toBasisCoordinate(p, 'g', controlSys->mesh);
	// set speed slow
	controlSys->pathPlannerCS.setVelMax(vel_low);
	// go to point
	controlSys->pathPlannerCS.gotoPoint(p);
	while (!controlSys->pathPlannerCS.posReached()) {usleep(100000);}
	// set speed high
	controlSys->pathPlannerCS.setVelMax(vel_high);
	
	// Set workspace limitation parameters / enable workspace limitation	
	controlSys->cartesianWorkspaceLimit.disable();
}

bool ScaraSequenceTEXASSsingleGrab::checkPostCondition() {
	return true;
}

void ScaraSequenceTEXASSsingleGrab::exit() {
// 	log.info() << "[ Single Grab ] exit done";
}