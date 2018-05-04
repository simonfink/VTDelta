#include "ScaraControlSystem_force.hpp"
#include "ScaraSafetyProperties_force.hpp"
#include <eeros/math/Matrix.hpp>
#include <iostream>
#include <unistd.h>
#include <eeros/core/Fault.hpp>
#include <eeros/hal/HAL.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/task/Periodic.hpp>
#include <eeros/core/PeriodicCounter.hpp>
#include "../joystick/XBoxController.hpp"
#include "../constants.hpp"

using namespace eeros::control;
using namespace eeros::math;
using namespace eeros::hal;
using namespace scara;

ScaraControlSystem_force::ScaraControlSystem_force(SafetySystem& safetySys, ScaraSafetyProperties_force& ssProperties) :
	fx("force_fx"),
	fy("force_fy"),
	fz("force_fz"),
	mx("force_mx"),
	my("force_my"),
	mz("force_mz"),
	
	initSpeedConst(0.0, 0.0, 0.0, 0.0),
	initForceConst(0, 0.0, 0.0, 0.0),
	initForcePosConst(0.78, -1.57, 0.105, 1.57),
	posIntGain(5.0, 5.0, 5.0, 5.0),
	velCtrlGain(220.0, 220.0, 220.0, 350.0),
	//posCtrlGain(75.0,  75.0, 75.0, 150.0),
	posCtrlGain(75.0,  75.0,0, 150.0),
	forceCrtlGain(10,  10, 0, 0),
		
	forceCrtlGain_f(10,  10,  0, 0),
	forceCrtlGain_dx(100, 100,  0, 0),
		
	forceCrtlGain_f1(0.01,  0.01,  0, 0),
	forceCrtlGain_x(300,  300,  0, 0),
	forceCrtlGain_x_dx(200,  200,  0, 0),
		
	Mmatrix(J0*i0*i0, J1*i1*i1, J2*i2*i2, J3*i3*i3),
	Imatrix(1/i0, 1/i1, 1/(-1*i2), 1/i3),
	Kmatrix(1/km0, 1/km1, 1/km2, 1/km3),
	
	xbox("/dev/input/js0"),
	xbox_buttons("/dev/input/js0"),
	enc0("q0"),
	enc1("q1"),
	enc2("q2r"),
	enc3("q3"),
	dac0("setCurrent0"),
	dac1("setCurrent1"),
	dac2("setCurrent2"),
	dac3("setCurrent3"),
	
	enc0_offset(0.0),
	enc1_offset(0.0),
	enc2_offset(0.0),
	enc3_offset(0.0),
	
	pathPlannerJS(velMaxPpJS, accMaxPpJS, -accMaxPpJS, dt),
	pathPlannerCS(velMaxPpCS, accMaxPpCS, -accMaxPpCS, dt),
	dirKin(l1, l2),
	invKin(l1, l2),
	invKinForce_dx(l1, l2),
	invKinForce_x(l1, l2),
	transJ(l1,l2),
	transJ1(l1,l2),
	transJ2(l1,l2),
	initSwitch(0),
	jointsRefSwitch(0),
	cartesRefSwitch(0),
	positionOrForceControlSwitch(0),
	invIencPos0(1/i0),
	invIencPos1(1/i1),
	invIencPos2(1/i2),
	invIencPos3(1/i3),
	invSenForce0(ifx),
	invSenForce1(ify),
	invSenForce2(ifz),
	invSenMomentz(imz),
	q2Gain(radToM),
	initSpeed(0.0),
	initForce(0.0),
	
	//PT-1
	kp_pt1(1),
	T_pt1(25),
	
	// DynModell
	DynModell(5,5),
	SatPos({0.1, -0.3, -0.050, -1},{0.45, 0.3, -0.250, 0}),
	
	checker_q0(-2.7, 2.7, safetySys, ssProperties.doEmergency_posOutRange),   
	checker_q1(-2.6, 2.6, safetySys, ssProperties.doEmergency_posOutRange),   
	checker_q2(-0.2, 0.3, safetySys, ssProperties.doEmergency_posOutRange),   
	checker_q3(-0.2, 6.3, safetySys, ssProperties.doEmergency_posOutRange),   
	checker_q0_speed(-1.05, 1.05, safetySys, ssProperties.doEmergency_velOutRange),
	checker_q1_speed(-1.05, 1.05, safetySys, ssProperties.doEmergency_velOutRange),
	checker_q2_speed(-1.05, 1.05, safetySys, ssProperties.doEmergency_velOutRange),
	checker_q3_speed(-1.05, 1.05, safetySys, ssProperties.doEmergency_velOutRange),
	
	// Trace
	trace0(5000), 
	trace1(5000),
	
	timedomain("Main time domain", dt, true)

	{				
		// Gains
		velController.setGain(velCtrlGain);
		posController.setGain(posCtrlGain);
		forceController.setGain(forceCrtlGain);
		forceController_f.setGain(forceCrtlGain_f);
		forceController_dx.setGain(forceCrtlGain_dx);
		forceController_x.setGain(forceCrtlGain_x);
		forceController_x_dx.setGain(forceCrtlGain_x_dx);
		forceController_f1.setGain(forceCrtlGain_f1);
		massMatrix.setGain(Mmatrix);
		invI.setGain(Imatrix);
		invKm.setGain(Kmatrix);
		initSpeed.setValue(initSpeedConst);
		initForce.setValue(initForceConst);
		initForcePos.setValue(initForcePosConst);
		// Sums
		posSum.negateInput(1);
		velSum.negateInput(1);
		forceSum.negateInput(1);
		q2Sum.negateInput(0);
		pt1Sum.negateInput(1);
		
		forceSum_dx.negateInput(1);
		forceSum_x.negateInput(1);
		forceSum_x_dx.negateInput(1);
		
		// Connect blocks
		enc0_offset_sum.getIn(0).connect(enc0.getOut());
		enc0_offset_sum.getIn(1).connect(enc0_offset.getOut());
		enc1_offset_sum.getIn(0).connect(enc1.getOut());
		enc1_offset_sum.getIn(1).connect(enc1_offset.getOut());
		enc2_offset_sum.getIn(0).connect(enc2.getOut());
		enc2_offset_sum.getIn(1).connect(enc2_offset.getOut());
		enc3_offset_sum.getIn(0).connect(enc3.getOut());
		enc3_offset_sum.getIn(1).connect(enc3_offset.getOut());
		
		invIencPos0.getIn().connect(enc0_offset_sum.getOut());
		invIencPos1.getIn().connect(enc1_offset_sum.getOut());
		invIencPos2.getIn().connect(enc2_offset_sum.getOut());
		invIencPos3.getIn().connect(enc3_offset_sum.getOut());
		
		q2Sum.getIn(0).connect(invIencPos2.getOut());
		q2Sum.getIn(1).connect(invIencPos3.getOut());
		q2Gain.getIn().connect(q2Sum.getOut());
		muxEncPos.getIn(0).connect(invIencPos0.getOut());
		muxEncPos.getIn(1).connect(invIencPos1.getOut());
		muxEncPos.getIn(2).connect(q2Gain.getOut());
		muxEncPos.getIn(3).connect(invIencPos3.getOut());
		
		///////////////////////////////////////
		// Force Control direkt
		invSenForce0.getIn().connect(fx.getOut());
		invSenForce1.getIn().connect(fy.getOut());
		invSenForce2.getIn().connect(fz.getOut());
		invSenMomentz.getIn().connect(mz.getOut());
		muxSenForce.getIn(0).connect(invSenForce0.getOut());
		muxSenForce.getIn(1).connect(invSenForce1.getOut());
		muxSenForce.getIn(2).connect(invSenForce2.getOut());
		muxSenForce.getIn(3).connect(invSenMomentz.getOut());
//		filter.getIn().connect(muxSenForce.getOut());
		
		forceSum.getIn(0).connect(initForce.getOut());
//		forceSum.getIn(1).connect(filter.getOut());
		// PT1_Glied
		pt1Sum.getIn(0).connect(muxSenForce.getOut());
		pt1Sum.getIn(1).connect(I_pt1.getOut());
		T_pt1.getIn().connect(pt1Sum.getOut());
		I_pt1.getIn().connect(T_pt1.getOut());
		forceSum.getIn(1).connect(I_pt1.getOut());
		
		transJ.getInForce().connect(forceSum.getOut());
		transJ.getInJointPos().connect(muxEncPos.getOut());
		forceController.getIn().connect(transJ.getOutJacobi());
		positionOrForceControlSwitch.getIn(1).connect(forceController.getOut());
		
		//Force Control velocity loop
		forceController_f.getIn().connect(forceSum.getOut());
		transJ1.getInForce().connect(forceController_f.getOut());
		transJ1.getInJointPos().connect(muxEncPos.getOut());
		//refPosDiffForce_dx.getIn().connect(invKinForce_dx.getOut());
		forceSum_dx.getIn(0).connect(transJ1.getOutJacobi());
		forceSum_dx.getIn(1).connect(muxEncPos.getOut());
		forceController_dx.getIn().connect(forceSum_dx.getOut());
		positionOrForceControlSwitch.getIn(2).connect(forceSum_dx.getOut());
		
		
		// Force Control position loop
		forceController_f1.getIn().connect(forceSum.getOut());
		transJ2.getInJointPos().connect(muxEncPos.getOut());
		transJ2.getInForce().connect(forceController_f1.getOut());
		forceSum_x.getIn(0).connect(transJ2.getOutJacobi());
		forceSum_x.getIn(1).connect(muxEncPos.getOut());
		forceSum_x.getIn(2).connect(initForcePos.getOut());
		forceController_x.getIn().connect(forceSum_x.getOut());
		//forceSum_x_dx.getIn(0).connect(forceController_x.getOut());
		//forceSum_x_dx.getIn(1).connect(encPosDiff.getOut());
		//forceController_x_dx.getIn().connect(forceSum_x_dx.getOut());
		positionOrForceControlSwitch.getIn(3).connect(forceController_x.getOut());
		
		// Forc Control DynModell
		DynModell.getInForce().connect(forceSum.getOut());
		DynModell.getInCartVel().connect(I_acc_to_vel.getOut());
		DynModell.getInCartPos().connect(I_vel_to_pos.getOut());
		I_acc_to_vel.getIn().connect(DynModell.getOut());
		I_vel_to_pos.getIn().connect(I_acc_to_vel.getOut());
		SatPos.getIn().connect(I_vel_to_pos.getOut());
		
		//////////////////////////////////////////
		dirKin.getIn().connect(muxEncPos.getOut());
		trace1.getIn().connect(dirKin.getOut());
			
		encPosDiff.getIn().connect(muxEncPos.getOut());
		
		demux_encPosDiff.getIn().connect(encPosDiff.getOut());
		
		cartesRefSwitch.getIn(0).connect(pathPlannerCS.getPosOut());
		trace0.getIn().connect(pathPlannerCS.getPosOut());
		cartesRefSwitch.getIn(1).connect(xbox.getOut());
		cartesRefSwitch.getIn(2).connect(SatPos.getOut());
		invKin.getIn().connect(cartesRefSwitch.getOut());
		
		jointsRefSwitch.getIn(0).connect(pathPlannerJS.getPosOut());
		jointsRefSwitch.getIn(1).connect(invKin.getOut());

		refPosDiff.getIn().connect(jointsRefSwitch.getOut());
		posSum.getIn(0).connect(jointsRefSwitch.getOut());
		posSum.getIn(1).connect(muxEncPos.getOut());
		
// 		posIntegral.getIn().connect(posSum.getOut());
// 		posIntegralGain.getIn().connect(posIntegral.getOut());
// 		posIntegralSum.getIn(0).connect(posIntegralGain.getOut());
// 		posIntegralSum.getIn(1).connect(posSum.getOut());
// 		posController.getIn().connect(posIntegralSum.getOut());
		
		posController.getIn().connect(posSum.getOut());
		velFfwSum.getIn(0).connect(posController.getOut());
		velFfwSum.getIn(1).connect(refPosDiff.getOut());
		
		initSwitch.getIn(0).connect(initSpeed.getOut());
		initSwitch.getIn(1).connect(velFfwSum.getOut());
		
		velSum.getIn(0).connect(initSwitch.getOut());
		velSum.getIn(1).connect(encPosDiff.getOut());
		velController.getIn().connect(velSum.getOut());
		
		massMatrix.getIn().connect(velController.getOut());
		positionOrForceControlSwitch.getIn(0).connect(massMatrix.getOut());
		
		invI.getIn().connect(positionOrForceControlSwitch.getOut());
		invKm.getIn().connect(invI.getOut());
		
		demuxDac.getIn().connect(invKm.getOut());
		dac0.getIn().connect(demuxDac.getOut(0));
		dac1.getIn().connect(demuxDac.getOut(1));
		dac2.getIn().connect(demuxDac.getOut(2));
		dac3.getIn().connect(demuxDac.getOut(3));
		
		checker_q0.getIn().connect(invIencPos0.getOut());
		checker_q1.getIn().connect(invIencPos1.getOut());
		checker_q2.getIn().connect(q2Gain.getOut());
		checker_q3.getIn().connect(invIencPos3.getOut());
		
		checker_q0_speed.getIn().connect(demux_encPosDiff.getOut(0));
		checker_q1_speed.getIn().connect(demux_encPosDiff.getOut(1));
		checker_q2_speed.getIn().connect(demux_encPosDiff.getOut(2));
		checker_q3_speed.getIn().connect(demux_encPosDiff.getOut(3));
		
		// Run blocks		
		timedomain.addBlock(&fx);
		timedomain.addBlock(&fy);
		timedomain.addBlock(&fz);
		timedomain.addBlock(&mx);
		timedomain.addBlock(&my);
		timedomain.addBlock(&mz);
		
		timedomain.addBlock(&invSenForce0);
		timedomain.addBlock(&invSenForce1);
		timedomain.addBlock(&invSenForce2);
		timedomain.addBlock(&invSenMomentz);
		timedomain.addBlock(&muxSenForce);
		timedomain.addBlock(&initForce);
		timedomain.addBlock(&initForcePos);
		timedomain.addBlock(&pt1Sum);
		timedomain.addBlock(&T_pt1);
		timedomain.addBlock(&I_pt1);
		timedomain.addBlock(&forceSum);
		
		timedomain.addBlock(&xbox);
		timedomain.addBlock(&xbox_buttons);
		
		timedomain.addBlock(&enc0);
		timedomain.addBlock(&enc1);
		timedomain.addBlock(&enc2);
		timedomain.addBlock(&enc3);
		
		timedomain.addBlock(&enc0_offset);
		timedomain.addBlock(&enc1_offset);
		timedomain.addBlock(&enc2_offset);
		timedomain.addBlock(&enc3_offset);
		
		timedomain.addBlock(&enc0_offset_sum);
		timedomain.addBlock(&enc1_offset_sum);
		timedomain.addBlock(&enc2_offset_sum);
		timedomain.addBlock(&enc3_offset_sum);
		
		timedomain.addBlock(&invIencPos0);
		timedomain.addBlock(&invIencPos1);
		timedomain.addBlock(&invIencPos2);
		timedomain.addBlock(&invIencPos3);
		
		timedomain.addBlock(&q2Sum);
		timedomain.addBlock(&q2Gain);
		timedomain.addBlock(&muxEncPos);
		
		
		timedomain.addBlock(&encPosDiff);
		timedomain.addBlock(&demux_encPosDiff);
		
		timedomain.addBlock(&dirKin);
		timedomain.addBlock(&trace1);
		
		timedomain.addBlock(&pathPlannerCS);
		timedomain.addBlock(&trace0);
		
		timedomain.addBlock(&DynModell);
		timedomain.addBlock(&I_acc_to_vel);
		timedomain.addBlock(&I_vel_to_pos);
		timedomain.addBlock(&SatPos);
		timedomain.addBlock(&cartesRefSwitch);
		timedomain.addBlock(&invKin);
		
		timedomain.addBlock(&pathPlannerJS);
		timedomain.addBlock(&jointsRefSwitch);
		
		timedomain.addBlock(&refPosDiff);
		timedomain.addBlock(&posSum);
//		timedomain.addBlock(&posIntegral);
// 		timedomain.addBlock(&posIntegralGain);
// 		timedomain.addBlock(&posIntegralSum);
		timedomain.addBlock(&posController);
		
		timedomain.addBlock(&velFfwSum);
		
		timedomain.addBlock(&initSpeed);
		timedomain.addBlock(&initSwitch);
		
		timedomain.addBlock(&velSum);
		timedomain.addBlock(&velController);
		/////////////////////////////////
		timedomain.addBlock(&transJ);
		timedomain.addBlock(&forceController);
		
		timedomain.addBlock(&forceController_f);
		timedomain.addBlock(&transJ1);
		//timedomain.addBlock(&refPosDiffForce_dx);
		timedomain.addBlock(&forceSum_dx);
		timedomain.addBlock(&forceController_dx);
		
		timedomain.addBlock(&forceController_f1);
		//timedomain.addBlock(&invKinForce_x);
		timedomain.addBlock(&transJ2);
		
		timedomain.addBlock(&forceSum_x);
		timedomain.addBlock(&forceController_x);
		//timedomain.addBlock(&forceSum_x_dx);
		//timedomain.addBlock(&forceController_x_dx);
		/////////////////////////////////
		
		timedomain.addBlock(&massMatrix);
		timedomain.addBlock(&positionOrForceControlSwitch);
		timedomain.addBlock(&invI);
		timedomain.addBlock(&invKm);
		
		timedomain.addBlock(&demuxDac);
		timedomain.addBlock(&dac0);
		timedomain.addBlock(&dac1);
		timedomain.addBlock(&dac2);
		timedomain.addBlock(&dac3);
		
		timedomain.addBlock(&checker_q0);
		timedomain.addBlock(&checker_q1);
		timedomain.addBlock(&checker_q2);
		timedomain.addBlock(&checker_q3);
		
		timedomain.addBlock(&checker_q0_speed);
		timedomain.addBlock(&checker_q1_speed);
		timedomain.addBlock(&checker_q2_speed);
		timedomain.addBlock(&checker_q3_speed);
		
// 		eeros::task::Periodic per(timedomain.getName().c_str(), timedomain.getPeriod(), &timedomain, timedomain.getRealtime());
// 		
// 		per.monitors.push_back([](eeros::PeriodicCounter &c, eeros::logger::Logger<eeros::logger::LogWriter> &log){
// 			static int count = 0;
// 			if (++count < 10000) return;
// 			count = 0;
// 			log.info() << "cs: max. period = " << c.period.max << " min. period = " << c.period.min << " mean period = " << c.period.mean;
// 			c.reset();
// 		});

		eeros::Executor::instance().add(timedomain); // td
	}
	
void ScaraControlSystem_force::setInitSpeed(scara::AxisVector v) {
	initSpeed.setValue(v);
}

void ScaraControlSystem_force::switchToInitSpeed() {
	initSwitch.switchToInput(0);
}

void ScaraControlSystem_force::switchToPositionControl() {
	initSwitch.switchToInput(1);
}