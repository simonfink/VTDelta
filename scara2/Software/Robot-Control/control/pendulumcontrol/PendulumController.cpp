#include "PendulumController.hpp"
#include "../../constants.hpp"

PendulumController::PendulumController() :
	xy_refPos_in(1.0),
	xy_actPos_in(1.0),
	phi_hallSensor_in(1.0),
	angleToTipPos(l_pendulum),
	phiFilter(0.4),
	Pgain_tip(kp_tipCtrl),
	Dgain_tip(kv_tipCtrl),
	Pgain_angle(kp_phiCtrl),
	Dgain_angle(kv_phiCtrl),
	accFilter(0.2),
	angleAccToTipAcc(l_pendulum, cog_pendulum, J_pendulum, m_pendulum)
// 	enableIntegral(1.0)
{
	phiFilter.getIn().connect(phi_hallSensor_in.getOut());
	angleToTipPos.getIn_phi().connect(phiFilter.getOut());       
	angleToTipPos.getIn_xyRobot().connect(xy_actPos_in.getOut());
	tipSum.negateInput(1);
	tipSum.getIn(0).connect(xy_refPos_in.getOut());                  
	tipSum.getIn(1).connect(angleToTipPos.getOut_xyPendulum());
	
	// the following blocks build a PD (position)
	Pgain_tip.getIn().connect(tipSum.getOut());
	diff_tip.getIn().connect(tipSum.getOut());
	Dgain_tip.getIn().connect(diff_tip.getOut());
	sumPD_tip.getIn(0).connect(Pgain_tip.getOut());
	sumPD_tip.getIn(1).connect(Dgain_tip.getOut());
	
	tipAccToAngle.getIn().connect(sumPD_tip.getOut());
	
	phiSum.negateInput(1);
	phiSum.getIn(0).connect(tipAccToAngle.getOut());
	phiSum.getIn(1).connect(phi_hallSensor_in.getOut());          
	
	// the following blocks build a PD (phi)
	Pgain_angle.getIn().connect(phiSum.getOut());
	diff_angle.getIn().connect(phiSum.getOut());
	Dgain_angle.getIn().connect(diff_angle.getOut());
	sumPD_angle.getIn(0).connect(Pgain_angle.getOut());
	sumPD_angle.getIn(1).connect(Dgain_angle.getOut());
	
	accFilter.getIn().connect(sumPD_angle.getOut());                 
	angleAccToTipAcc.getIn_ddphi().connect(accFilter.getOut());
	angleAccToTipAcc.getIn_phi().connect(phi_hallSensor_in.getOut());  
	
	I_accToVel.getIn().connect(angleAccToTipAcc.getOut());
// 	I_accToVel.getEnable().connect(enableIntegral.getOut());
	I_velToPos.getIn().connect(I_accToVel.getOut());
// 	I_velToPos.getEnable().connect(enableIntegral.getOut());
}

void PendulumController::run() {
	phi_hallSensor_in.run();
	xy_actPos_in.run();
	xy_refPos_in.run();
	phiFilter.run();
	angleToTipPos.run();
	tipSum.run();
	diff_tip.run();
	Pgain_tip.run(); 
	Dgain_tip.run(); 
	sumPD_tip.run();
	tipAccToAngle.run();
	phiSum.run();
	diff_angle.run();
	Pgain_angle.run(); 
	Dgain_angle.run(); 
	sumPD_angle.run();
	accFilter.run();              
	angleAccToTipAcc.run();
// 	enableIntegral.run();
	I_accToVel.run();
	I_velToPos.run();
}

Input<AxisVector>& PendulumController::getIn_refPos_xy() {
	return xy_refPos_in.getIn();
}
Input<AxisVector>& PendulumController::getIn_actPos_xy() {
	return xy_actPos_in.getIn();
}
Input<AxisVector>& PendulumController::getIn_phiHall() {
	return phi_hallSensor_in.getIn();
}

// Input<bool>& PendulumController::getEnableIntegral() {
// 	return enableIntegral.getIn();
// }

Output<AxisVector>& PendulumController::getOutRefPos() {
	return I_velToPos.getOut();
}
Output<AxisVector>& PendulumController::getOut_refAcc() {
	return angleAccToTipAcc.getOut();
}
Output<AxisVector>& PendulumController::getOut_refVel() {
	return I_accToVel.getOut();
}

PendulumController::~PendulumController() {
  	// nothing to do
}
