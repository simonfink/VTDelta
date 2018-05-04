#include "ScaraSequenceTEXASSgrabProcess.hpp"
#include "../../../ScaraControlSystem.hpp"
#include "../../../ScaraSafetyProperties.hpp"
#include <eeros/safety/SafetySystem.hpp>
#include <unistd.h>
#include <iostream>

#include <eeros/hal/HAL.hpp>

using namespace scara;
using namespace eeros::control;
using namespace eeros::sequencer;
using namespace eeros::safety;
using namespace eeros::hal;
using namespace eeros::math;

ScaraSequenceTEXASSgrabProcess::ScaraSequenceTEXASSgrabProcess(Sequencer* sequencer, ScaraControlSystem* controlSys, SafetySystem* safetySys) : 
													Sequence<void, Vector2, Vector2>("main", sequencer), controlSys(controlSys), safetySys(safetySys), 
													singleGrab(sequencer, controlSys, safetySys) {
	// nothing to do
}

void ScaraSequenceTEXASSgrabProcess::init() {
	std::bind(&ScaraSequenceTEXASSgrabProcess::init, *this);
}

bool ScaraSequenceTEXASSgrabProcess::checkPreCondition() {
	return safetySys->getCurrentLevel().getId() >= ready;
}

void ScaraSequenceTEXASSgrabProcess::run(Vector2 shift_parts_in, Vector2 shift_mesh_in) {
	HAL& hal = HAL::instance();
	
	shift_parts = shift_parts_in;
	shift_mesh = shift_mesh_in;
	
	// 3. Init path planner parameters & start motion
	AxisVector x_actual = controlSys->dirKin.getOut().getSignal().getValue();
	controlSys->pathPlannerCS.setInitPos(x_actual);
	controlSys->posIntegral.setInitCondition(x_actual);
	controlSys->pathPlannerPosSwitch.switchToInput(1);	// cartesian path planner
	controlSys->autoToManualSwitch.switchToInput(0);	// automatic mode
	usleep(100000); 
	
	controlSys->posIntegral.enable();
	safetySys->triggerEvent(doStartingMotion); 
	while(safetySys->getCurrentLevel().getId() < moving); 

	// 4. Run process

	// First led
	xy_parts << 0.01,     0.020  ; xy_parts = xy_parts + shift_parts;
	xy_mesh  << 0.003175, 0.00254; xy_mesh  = xy_mesh  + shift_mesh;
	singleGrab(xy_parts, xy_mesh, z_high, z_low_parts, z_low_mesh); 

	// Second led
	xy_parts << 0.01,     0.025  ; xy_parts = xy_parts + shift_parts;
	xy_mesh  << 0.003175, 0.02286; xy_mesh  = xy_mesh  + shift_mesh;
	singleGrab(xy_parts, xy_mesh, z_high, z_low_parts, z_low_mesh); 
	
	// Third led
	xy_parts << 0.01,     0.03   ; xy_parts = xy_parts + shift_parts;
	xy_mesh  << 0.022225, 0.02286; xy_mesh  = xy_mesh  + shift_mesh;
	singleGrab(xy_parts, xy_mesh, z_high, z_low_parts, z_low_mesh); 
	
	// Fourth led
	xy_parts << 0.0097,   0.035  ; xy_parts = xy_parts + shift_parts;
	xy_mesh  << 0.022225, 0.00254; xy_mesh  = xy_mesh  + shift_mesh;
	singleGrab(xy_parts, xy_mesh, z_high, z_low_parts, z_low_mesh); 
	
	// R75 - 1
	xy_parts << 0.01,    0.07    ; xy_parts = xy_parts + shift_parts;
	xy_mesh  << 0.00762, 0.001905; xy_mesh  = xy_mesh  + shift_mesh;
	singleGrab(xy_parts, xy_mesh, z_high, z_low_parts, z_low_mesh); 
	
	// R75 - 2
	xy_parts << 0.01,    0.075   ; xy_parts = xy_parts + shift_parts;
	xy_mesh  << 0.01778, 0.001905; xy_mesh  = xy_mesh  + shift_mesh;
	singleGrab(xy_parts, xy_mesh, z_high, z_low_parts, z_low_mesh); 
	
	// R100 
	xy_parts << 0.01,     0.120  ; xy_parts = xy_parts + shift_parts; 
	xy_mesh  << 0.012065, 0.02159; xy_mesh  = xy_mesh  + shift_mesh;
	singleGrab(xy_parts, xy_mesh, z_high, z_low_parts, z_low_mesh); 
	
	// Fototransistor
	xy_parts << 0.060,  0.02  ; xy_parts = xy_parts + shift_parts;
	xy_mesh  << 0.0127, 0.0127; xy_mesh  = xy_mesh  + shift_mesh;
	singleGrab(xy_parts, xy_mesh, z_high, z_low_parts, z_low_mesh); 
	
	// Transistor
	xy_parts << 0.060,   0.075  ; xy_parts = xy_parts + shift_parts;
	xy_mesh  << 0.01793, 0.01651; xy_mesh  = xy_mesh  + shift_mesh;
	singleGrab(xy_parts, xy_mesh, z_high, z_low_parts, z_low_mesh); 

	// Last point HIGH
	AxisVector p; p <<  xy_mesh(0), xy_mesh(1), z_high, 1;
	p = controlSys->toCalibratedValue(p, controlSys->meshCalibrationLut);
	p = controlSys->toBasisCoordinate(p, 'g', controlSys->mesh);
	controlSys->pathPlannerCS.setVelMax(0.5);
	controlSys->pathPlannerCS.gotoPoint(p);
	while (!controlSys->pathPlannerCS.posReached()) {usleep(100000);}
	controlSys->pathPlannerCS.setVelMax(vel);

	// 5. Stop motion
	controlSys->posIntegral.disable();
	safetySys->triggerEvent(doMotionStopping);
	while(!(safetySys->getCurrentLevel().getId() == ready));
}

bool ScaraSequenceTEXASSgrabProcess::checkPostCondition() {
	return safetySys->getCurrentLevel().getId() == ready;
}

void ScaraSequenceTEXASSgrabProcess::exit() {
// 	log.info() << "[ Grabber ] exit done";
}
