#include "ScaraSequenceManualPark.hpp"
#include "../../ScaraControlSystem.hpp"
#include "../../ScaraSafetyProperties.hpp"

using namespace scara;
using namespace eeros::control;
using namespace eeros::sequencer;
using namespace eeros::safety;

ScaraSequenceManualPark::ScaraSequenceManualPark(Sequencer* sequencer, ScaraControlSystem* controlSys, SafetySystem* safetySys) : Sequence<>("main", sequencer), controlSys(controlSys), safetySys(safetySys) {
	// nothing to do
}

void ScaraSequenceManualPark::init() {
	std::bind(&ScaraSequenceManualPark::init, *this);
}

bool ScaraSequenceManualPark::checkPreCondition() {
	return safetySys->getCurrentLevel().getId() == baseSystemOn;
}

void ScaraSequenceManualPark::run() {
	log.info() << "[ Manual Parking Sequence Started ]";
	safetySys->triggerEvent(doManualParking);
	while(safetySys->getCurrentLevel().getId() < robotParked);
}

bool ScaraSequenceManualPark::checkPostCondition() {
	return safetySys->getCurrentLevel().getId() == robotParked;
// 	return true;
}

void ScaraSequenceManualPark::exit() {
	AxisVector phi_actual = controlSys->muxEncPos.getOut().getSignal().getValue();
	AxisVector x_actual = controlSys->dirKin.getOut().getSignal().getValue();
	controlSys->pathPlannerJS.setInitPos(phi_actual);
	controlSys->pathPlannerCS.setInitPos(x_actual);
	controlSys->pathPlannerPosSwitch.switchToInput(0);			// joint path planner
	controlSys->autoToManualSwitch.switchToInput(0);			// automatic mode
	log.info() << "[ Manual Parking Sequence Exit Done ]";
}