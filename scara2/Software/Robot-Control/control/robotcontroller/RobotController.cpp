#include "RobotController.hpp"
#include "../../constants.hpp"

RobotController::RobotController(SafetySystem& safetySys, ParallelScaraSafetyProperties& ssProperties) :
	bufEncPosAct(1.0), 
	posController(kp_robCtrl),
	speedSwitch(0),
	velController(kv_robCtrl),
	massMatrix(J),
	invKm(1/km),
	invI(1/i)
{
	posSum.negateInput(1);
	posSum.getIn(1).connect(bufEncPosAct.getOut());
	encPosDiff.getIn().connect(bufEncPosAct.getOut());
	posController.getIn().connect(posSum.getOut());
	velFfwSum.getIn(0).connect(posController.getOut());
	speedInitSetPoint.getOut().getSignal().setName("speed init set point");
	speedSwitch.getIn(0).connect(speedInitSetPoint.getOut());
	speedSwitch.getIn(1).connect(velFfwSum.getOut());
	speedSwitch.getOut().getSignal().setName("speed switch");
	speedSaturation.getIn_veloDes().connect(speedSwitch.getOut());
	speedSaturation.getIn_posActual().connect(bufEncPosAct.getOut());
	velSum.negateInput(1);
	velSum.getIn(0).connect(speedSaturation.getOut_veloDesSaturation());
	velSum.getIn(1).connect(encPosDiff.getOut());
	velController.getIn().connect(velSum.getOut());
	massMatrix.getIn().connect(velController.getOut());
	invKm.getIn().connect(massMatrix.getOut());
	invI.getIn().connect(invKm.getOut());
}

RobotController::~RobotController() {
  	// nothing to do
}

void RobotController::run() {
	bufEncPosAct.run();
	posSum.run();
	encPosDiff.run();
	posController.run(); 
	velFfwSum.run();
	speedInitSetPoint.run();
	speedSwitch.run();
	speedSaturation.run();
	velSum.run();
	velController.run(); 
	massMatrix.run();
	invKm.run();
	invI.run();
}

bool RobotController::reachedMechanicalLimit(double maxTorque) {
	for(int i = 0; i < nofAxis; i++) {
		if(fabs(massMatrix.getOut().getSignal().getValue()[i]) < maxTorque) return false;
	}
	return true;
}

void RobotController::setInitializationSpeed(AxisVector u) {
	speedInitSetPoint.setValue(u);
}

void RobotController::setSpeedSwitch(bool s) {
	speedSwitch.switchToInput(s);
}

Output<AxisVector>& RobotController::getOutSetpointMotors() {return invI.getOut();}

Input<AxisVector>& RobotController::getInEncPosAct() {
	return bufEncPosAct.getIn();
}

Input<AxisVector>& RobotController::getInEncRefPos() {
	return posSum.getIn(0);
}

Input<AxisVector>& RobotController::getInEncRefVel() {
	return velFfwSum.getIn(1);
}

AxisVector RobotController::getActSpeed() {
	return encPosDiff.getOut().getSignal().getValue();
}


