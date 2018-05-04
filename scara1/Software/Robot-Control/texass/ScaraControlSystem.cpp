#include "ScaraControlSystem.hpp"
#include "ScaraSafetyProperties.hpp"
#include <eeros/math/Matrix.hpp>
#include <iostream>
#include <unistd.h>
#include <eeros/core/EEROSException.hpp>
#include <eeros/hal/HAL.hpp>

#include "joystick/XBoxController.hpp"

using namespace scara;
using namespace eeros::control;
using namespace eeros::math;
using namespace eeros::hal;
using namespace scara;

	ScaraControlSystem::ScaraControlSystem() :
	
	readyPositionCartesian(0.35, -0.05,  -0.105, -0.78),
	tipRefPoint(0.095, 0.005, 0.00296, -1.57), // in bauteile coordinate system
	crossRefPoint(0.05, 0.005, -0.005, -1.57), // in bauteile coordinate system (-0.0005)
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
	invIencPos0(1/i0),
	invIencPos1(1/i1),
	invIencPos2(1/i2),
	invIencPos3(1/i3),
	q2Gain(radToM),
	pathPlannerJS(velMaxPpJS, accMaxPpJS, dt),
	pathPlannerCS(velMaxPpCS, accMaxPpCS, dt),
	pathPlannerPosSwitch(0),
	autoToManualSwitch(0),
	dirKin(l1, l2),
	invKin(l1, l2),
	dac0("setCurrent0"),
	dac1("setCurrent1"),
	dac2("setCurrent2"),
	dac3("setCurrent3"),
	roboter("roboter"),
	gestell("gestell"),
	spannrahmen("spannrahmen"),
	bauteile("bauteile"),
	mesh("mesh"),
	KG(roboter, gestell),
	KS(roboter, spannrahmen),
	KB(roboter, bauteile),
	KM(roboter, mesh),
	
	timedomain("Main time domain", dt, true)

	{		
		// Configure blocks
		setPlasmaPen.getIn().connect(xbox_buttons.getOut());
		
		posIntegralGain.setGain(posIntGain);
		velController.setGain(velCtrlGain);
		posController.setGain(posCtrlGain);
		massMatrix.setGain(Mmatrix);
		invI.setGain(Imatrix);
		invKm.setGain(Kmatrix);
		posSum.negateInput(1);
		velSum.negateInput(1);
		q2Sum.negateInput(0);
		autoToManualSwitch.switchToInput(0);
		pathPlannerPosSwitch.switchToInput(0);
		
		// Connect blocks
		invIencPos0.getIn().connect(enc0.getOut());
		invIencPos1.getIn().connect(enc1.getOut());
		invIencPos2.getIn().connect(enc2.getOut());
		invIencPos3.getIn().connect(enc3.getOut());
		q2Sum.getIn(0).connect(invIencPos2.getOut());
		q2Sum.getIn(1).connect(invIencPos3.getOut());
		q2Gain.getIn().connect(q2Sum.getOut());
		muxEncPos.getIn(0).connect(invIencPos0.getOut());
		muxEncPos.getIn(1).connect(invIencPos1.getOut());
		muxEncPos.getIn(2).connect(q2Gain.getOut());
		muxEncPos.getIn(3).connect(invIencPos3.getOut());
		encPosDiff.getIn().connect(muxEncPos.getOut());
		autoToManualSwitch.getIn(0).connect(pathPlannerCS.getPosOut());
		autoToManualSwitch.getIn(1).connect(xbox.getOut());
		dirKin.getIn().connect(muxEncPos.getOut());
		
		cartesianWorkspaceLimit.getIn().connect(autoToManualSwitch.getOut()); // check if in the right place

		invKin.getIn().connect(autoToManualSwitch.getOut());
		
		pathPlannerPosSwitch.getIn(0).connect(pathPlannerJS.getPosOut());
		pathPlannerPosSwitch.getIn(1).connect(invKin.getOut());
		refPosDiff.getIn().connect(pathPlannerPosSwitch.getOut());
		posSum.getIn(0).connect(pathPlannerPosSwitch.getOut());
		posSum.getIn(1).connect(muxEncPos.getOut());
		
		posIntegral.getIn().connect(posSum.getOut());
		posIntegralGain.getIn().connect(posIntegral.getOut());
		posIntegralSum.getIn(0).connect(posIntegralGain.getOut());
		posIntegralSum.getIn(1).connect(posSum.getOut());
		
		posController.getIn().connect(posIntegralSum.getOut());
// 		posController.getIn().connect(posSum.getOut());
		velFfwSum.getIn(0).connect(posController.getOut());
		velFfwSum.getIn(1).connect(refPosDiff.getOut());
		velSum.getIn(0).connect(velFfwSum.getOut());
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
		
		// Run blocks
		timedomain.addBlock(&enc0);
		timedomain.addBlock(&enc1);
		timedomain.addBlock(&enc2);
		timedomain.addBlock(&enc3);
		timedomain.addBlock(&invIencPos0);
		timedomain.addBlock(&invIencPos1);
		timedomain.addBlock(&invIencPos2);
		timedomain.addBlock(&invIencPos3);
		timedomain.addBlock(&q2Sum);
		timedomain.addBlock(&q2Gain);
		timedomain.addBlock(&muxEncPos);
		timedomain.addBlock(&encPosDiff);
		timedomain.addBlock(&dirKin);
		timedomain.addBlock(&pathPlannerJS);
		timedomain.addBlock(&pathPlannerCS);
		
		timedomain.addBlock(&xbox);
		timedomain.addBlock(&xbox_buttons);
		timedomain.addBlock(&setPlasmaPen);
		
		timedomain.addBlock(&autoToManualSwitch);
		timedomain.addBlock(&cartesianWorkspaceLimit); // check if in the right place
		timedomain.addBlock(&invKin);
		timedomain.addBlock(&pathPlannerPosSwitch);
		timedomain.addBlock(&refPosDiff);
		timedomain.addBlock(&posSum);
		timedomain.addBlock(&posIntegral);
		timedomain.addBlock(&posIntegralGain);
		timedomain.addBlock(&posIntegralSum);
		timedomain.addBlock(&posController);
		timedomain.addBlock(&velFfwSum);
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
	}
	
	void ScaraControlSystem::start() {
		timedomain.start();
	}

	void ScaraControlSystem::stop() {
		timedomain.stop();
		timedomain.join();
	}
	
	AxisVector ScaraControlSystem::toBasisCoordinate(AxisVector pos, char tool, CoordinateSystem& c) {
		AxisVector out, posTr; Frame* frame;

		// TOOL: p = plasmapen; g = greifer; d = dispencer; n = no tool
		if (tool == 'p'){
			toolOffset << plasmaX, plasmaY, plasmaZ, 0; 
			angleOffset = plasmaAlpha; 
		}
		else if (tool == 'g'){
			toolOffset << greiferX, greiferY, greiferZ, 0; 
			angleOffset = greiferAlpha;
		}
		else if (tool == 'd'){
			toolOffset << dispencerX, dispencerY, dispencerZ, 0; 
			angleOffset = dispencerAlpha;
		}
		else if (tool == 'c'){
			toolOffset << -cameraX, -cameraY, -cameraZ, 0;
			angleOffset = cameraAlpha; 
		}
// 		else if (tool == 'k'){
// 			toolOffset << kameraX, kameraY, kameraZ, 0; 
// 			angleOffset = cameraAlpha; 
// 		}
		else if (tool == 'n'){
			toolOffset << 0, 0, 0, 0;
			angleOffset = 0;
		}
		else if (tool == 't'){
			toolOffset << tasterX, tasterY, tasterZ, 0;
			angleOffset = 0;
		}
		else
			throw EEROSException("Wrong tool id selected");

		// COORD. SYSTEM
		if(c == roboter) {
			out = pos + toolOffset; 
		}
		else{
			frame = Frame::getFrame(roboter, c);
			if(frame == nullptr) throw EEROSException("Frame not found!");
			AxisSquareMatrix T = frame->get();
			
			posTr << pos(0), pos(1), pos(2), 1;
			out = T * posTr;
			out = out + toolOffset;
			out(3) = angleOffset;
			
			if(tool == 'n')
				out(3) = pos(3);
		}
		return out;
	}
	
	AxisVector ScaraControlSystem::toUserCoordinate(AxisVector pos, char tool, CoordinateSystem& c) {
		AxisVector out, posTr; Frame* frame;

		// TOOL: p = plasmapen; g = greifer; d = dispencer; n = no tool
		if (tool == 'p'){
			toolOffset << plasmaX, plasmaY, plasmaZ, 0; 
			angleOffset = plasmaAlpha; 
		}
		else if (tool == 'g'){
			toolOffset << greiferX, greiferY, greiferZ, 0; 
			angleOffset = greiferAlpha;
		}
		else if (tool == 'd'){
			toolOffset << dispencerX, dispencerY, dispencerZ, 0; 
			angleOffset = dispencerAlpha;
		}
		else if (tool == 'c'){
			toolOffset << -cameraX, -cameraY, -cameraZ, 0; 
			angleOffset = cameraAlpha; 
		}
// 		else if (tool == 'k'){
// 			toolOffset << kameraX, kameraY, kameraZ, 0; 
// 			angleOffset = cameraAlpha; 
// 		}
		else if (tool == 'n'){
			toolOffset << 0, 0, 0, 0;
			angleOffset = 0;
		}
		else if (tool == 't'){
			toolOffset << tasterX, tasterY, tasterZ, 0;
			angleOffset = 0;
		}
		else
			throw EEROSException("Wrong tool id selected");
		
		// COORD. SYSTEM
		if(c == roboter) {
			out = pos; 
		}
		else{
			frame = Frame::getFrame(roboter, c);
			if(frame == nullptr) throw EEROSException("Frame not found!");
			AxisSquareMatrix T = frame->get();
			
			posTr << pos(0), pos(1), pos(2), 1;
			out = !T * (posTr - toolOffset);
// 			out = out - toolOffset;
			out(3) = angleOffset;
			
			if(tool == 'n')
				out(3) = pos(3);
		}
		return out;
	}
	
	AxisVector ScaraControlSystem::toCalibratedValue(AxisVector pos, Vector2Corrector& calibrationLut) {
		Vector2 xy; Vector2 xy_out2; Vector4 xy_out;
		
		xy << pos(0), pos(1);
		xy_out2 = calibrationLut.get(xy);
		
		xy_out << xy_out2(0), xy_out2(1), pos(2), pos(3);
		return xy_out;
	}

	