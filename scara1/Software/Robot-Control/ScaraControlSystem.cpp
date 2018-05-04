#include "ScaraControlSystem.hpp"
#include "ScaraSafetyProperties.hpp"
#include <eeros/math/Matrix.hpp>
#include <iostream>
#include <unistd.h>
#include <eeros/core/Fault.hpp>
#include <eeros/hal/HAL.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/task/Periodic.hpp>
#include <eeros/core/PeriodicCounter.hpp>
#include "joystick/XBoxController.hpp"
#include "constants.hpp"

using namespace eeros::control;
using namespace eeros::math;
using namespace eeros::hal;
using namespace scara;

ScaraControlSystem::ScaraControlSystem(SafetySystem& safetySys, ScaraSafetyProperties& ssProperties) :
	
	initSpeedConst(0.0, 0.0, 0.0, 0.0),
	posIntGain(5.0, 5.0, 5.0, 5.0),
	velCtrlGain(220.0, 220.0, 220.0, 350.0),
	posCtrlGain(75.0,  75.0, 75.0, 150.0),
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
	initSwitch(0),
	jointsRefSwitch(0),
	cartesRefSwitch(0),
	invIencPos0(1/i0),
	invIencPos1(1/i1),
	invIencPos2(1/i2),
	invIencPos3(1/i3),
	q2Gain(radToM),
	initSpeed(0.0),
	
	checker_q0(-2.7, 2.7, safetySys, ssProperties.doEmergency_posOutRange),   
	checker_q1(-2.6, 2.6, safetySys, ssProperties.doEmergency_posOutRange),   
	checker_q2(-0.2, 0.3, safetySys, ssProperties.doEmergency_posOutRange),   
	checker_q3(-0.2, 6.3, safetySys, ssProperties.doEmergency_posOutRange),   
	checker_q0_speed(-1.05, 1.05, safetySys, ssProperties.doEmergency_velOutRange),
	checker_q1_speed(-1.05, 1.05, safetySys, ssProperties.doEmergency_velOutRange),
	checker_q2_speed(-1.05, 1.05, safetySys, ssProperties.doEmergency_velOutRange),
	checker_q3_speed(-1.05, 1.05, safetySys, ssProperties.doEmergency_velOutRange),
	
	timedomain("Main time domain", dt, true)

	{				
		// Gains
		velController.setGain(velCtrlGain);
		posController.setGain(posCtrlGain);
		massMatrix.setGain(Mmatrix);
		invI.setGain(Imatrix);
		invKm.setGain(Kmatrix);
		initSpeed.setValue(initSpeedConst);
		// Sums
		posSum.negateInput(1);
		velSum.negateInput(1);
		q2Sum.negateInput(0);
		
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
		dirKin.getIn().connect(muxEncPos.getOut());
		
		encPosDiff.getIn().connect(muxEncPos.getOut());
		demux_encPosDiff.getIn().connect(encPosDiff.getOut());
		
		cartesRefSwitch.getIn(0).connect(pathPlannerCS.getPosOut());
		cartesRefSwitch.getIn(1).connect(xbox.getOut());
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
		invI.getIn().connect(massMatrix.getOut());
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
		
		timedomain.addBlock(&pathPlannerCS);
		timedomain.addBlock(&cartesRefSwitch);
		timedomain.addBlock(&invKin);
		
		timedomain.addBlock(&pathPlannerJS);
		timedomain.addBlock(&jointsRefSwitch);
		
		timedomain.addBlock(&refPosDiff);
		timedomain.addBlock(&posSum);
// 		timedomain.addBlock(&posIntegral);
// 		timedomain.addBlock(&posIntegralGain);
// 		timedomain.addBlock(&posIntegralSum);
		timedomain.addBlock(&posController);
		
		timedomain.addBlock(&velFfwSum);
		
		timedomain.addBlock(&initSpeed);
		timedomain.addBlock(&initSwitch);
		
		timedomain.addBlock(&velSum);
		timedomain.addBlock(&velController);
		timedomain.addBlock(&massMatrix);
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
	
void ScaraControlSystem::setInitSpeed(scara::AxisVector v) {
	initSpeed.setValue(v);
}

void ScaraControlSystem::switchToInitSpeed() {
	initSwitch.switchToInput(0);
}

void ScaraControlSystem::switchToPositionControl() {
	initSwitch.switchToInput(1);
}