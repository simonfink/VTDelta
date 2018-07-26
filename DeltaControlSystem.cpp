#include "DeltaControlSystem.hpp"
#include <eeros/core/Executor.hpp>
#include <eeros/logger/Logger.hpp>

using namespace eeros::control;

DeltaControlSystem::DeltaControlSystem(double td) : 
	mouse("/dev/input/event1"),
	i(i1524, i1524, i1524, i0816),
	kM(kM1524, kM1524, kM1524, kM0816),
	RA(RA1524, RA1524, RA1524, RA0816),
	
	jacobian(kinematic.get_offset()),
	
	gainMot4(1.8),
	
	posController(kp),
	speedController(kd),

	inertia(jacobian),
	jacobi(jacobian),
	motorModel(kM, RA),
	voltageSwitch(1),
	directKin(kinematic),
	
	voltageSetPoint({0.0,0.0,0.0,0.0}),

	enc1("enc1"),
	enc2("enc2"),
	enc3("enc3"),
	enc4("enc4"),


	mot1("motor1"),
	mot2("motor2"),
	mot3("motor3"),
	mot4("motor4"),
	timedomain("Main time domain", dt, true) {
	
	enc1.setName("encoder input 1");
	enc1.getOut().getSignal().setName("phi1_actual");
	enc2.setName("encoder input 2");
	enc2.getOut().getSignal().setName("phi2_actual");
	enc3.setName("encoder input 3");
	enc3.getOut().getSignal().setName("phi3_actual");
	enc4.setName("encoder input 4");
	enc4.getOut().getSignal().setName("phi4_actual");
	
	mouse.setName("mouse");
	mouse.getOut().getSignal().setName("mouse axis output");
	mouse.getButtonOut().getSignal().setName("mouse button output");
	
	muxEnc.setName("Mux 4 encoder");
	muxEnc.getOut().getSignal().setName("AxisVector Encodervalues");
	
	posSum.setName("posSum");
	posController.setName("posController");
	posDiff.setName("posDiff");
	speedSum.setName("speedSum");
	speedLimitation.setName("speedLimitation");
	speedController.setName("speedController");
	inertia.setName("inertia");
	inertia.getOut().getSignal().setName("inertia signal");
	forceLimitation.setName("forceLimitation");
	jacobi.setName("jacobi");
	torqueLimitation.setName("torqueLimitation");
	angleGear.setName("angleGear");
	motorModel.setName("motorModel");
	voltageSwitch.setName("voltage switch");
	directKin.setName("directKin");
	demuxMot.setName("DeMux 4 Motor");
	mot1.setName("motor 1");
	mot2.setName("motor 2");
	mot3.setName("motor 3");
	mot4.setName("motor 4");
	voltageSetPoint.setName("voltage set point");
	torqueGear.setName("torqueGear");
	  
	torqueLimitation.enable();
	torqueGear.setGain(1.0/i);
	angleGear.setGain(1.0/i);
	
	posSum.negateInput(1);
	speedSum.negateInput(1);
	
	muxEnc.getIn(0).connect(enc1.getOut());
	muxEnc.getIn(1).connect(enc2.getOut());
	muxEnc.getIn(2).connect(enc3.getOut());
	muxEnc.getIn(3).connect(enc4.getOut());

	posSum.getIn(0).connect(mouse.getOut());
	posSum.getIn(1).connect(directKin.getOut());
	posController.getIn().connect(posSum.getOut());
	posDiff.getIn().connect(directKin.getOut());
	
	speedSum.getIn(0).connect(posController.getOut());
	speedSum.getIn(1).connect(posDiff.getOut());
	speedLimitation.getIn().connect(speedSum.getOut());
	speedController.getIn().connect(speedLimitation.getOut());
	
	inertia.getAccelerationInput().connect(speedController.getOut());
	inertia.getJointPosInput().connect(angleGear.getOut());
	inertia.getTcpPosInput().connect(directKin.getOut());
	
	forceLimitation.getIn().connect(inertia.getOut());
	
	jacobi.getForceInput().connect(forceLimitation.getOut());
	jacobi.getJointPosInput().connect(angleGear.getOut());
	jacobi.getTcpPosInput().connect(directKin.getOut());
	
	torqueLimitation.getIn().connect(jacobi.getOut());
	
	torqueGear.getIn().connect(torqueLimitation.getOut());
	
	angleGear.getIn().connect(muxEnc.getOut());
	
	motorModel.getTorqueIn().connect(torqueGear.getOut());
	motorModel.getSpeedIn().connect(voltageSwitch.getOut());
	
	voltageSwitch.getIn(0).connect(motorModel.getOut());
	voltageSwitch.getIn(1).connect(voltageSetPoint.getOut());
	
	directKin.getIn().connect(angleGear.getOut());

	demuxMot.getIn().connect(voltageSwitch.getOut());
	
	mot1.getIn().connect(demuxMot.getOut(0));
	mot2.getIn().connect(demuxMot.getOut(1));
	mot3.getIn().connect(demuxMot.getOut(2));
	gainMot4.getIn().connect(demuxMot.getOut(3));
// 	mot4.getIn().connect(demuxMot.getOut(3));
	mot4.getIn().connect(gainMot4.getOut());
	
	
	timedomain.addBlock(mouse);
	
	timedomain.addBlock(muxEnc);
	timedomain.addBlock(enc1);
	timedomain.addBlock(enc2);
	timedomain.addBlock(enc3);
	timedomain.addBlock(enc4);
	
	timedomain.addBlock(angleGear);
	timedomain.addBlock(directKin);
	
	timedomain.addBlock(posSum);
	timedomain.addBlock(posDiff);
	timedomain.addBlock(posController);
	
	timedomain.addBlock(speedSum);
	timedomain.addBlock(speedLimitation);
	timedomain.addBlock(speedController);
	
	timedomain.addBlock(inertia);
	timedomain.addBlock(forceLimitation);
	timedomain.addBlock(jacobi);
	timedomain.addBlock(torqueLimitation);
	timedomain.addBlock(torqueGear);
	
	timedomain.addBlock(motorModel);
	timedomain.addBlock(voltageSetPoint);
	timedomain.addBlock(voltageSwitch);

	
	timedomain.addBlock(demuxMot);
	timedomain.addBlock(mot1);
	timedomain.addBlock(mot2);
	timedomain.addBlock(mot3);
	timedomain.addBlock(gainMot4);
	timedomain.addBlock(mot4);
	

	eeros::task::Periodic tdPer("tdPer",(dt*100), timedomain);
	tdPer.monitors.push_back([&](eeros::PeriodicCounter &c, eeros::logger::Logger &log) {
 	  //log.info() <<  mouse.getButtonOut().getSignal();
	  //log.info() <<  mouse.getOut().getSignal();
	  //log.info() << "motorModel: " << motorModel.getOut().getSignal().getValue();
	  //log.info() << "voltage set point: " << voltageSetPoint.getOut().getSignal().getValue();
	});
	eeros::Executor::instance().add(timedomain);
	//eeros::Executor::instance().add(tdPer);
	
}

void DeltaControlSystem::start() {
	timedomain.start();
}

void DeltaControlSystem::stop() {
	timedomain.stop();
	//timedomain.join();
}

void DeltaControlSystem::enableAxis() {
	//board.setEnable(true);
	//board.setReset(false);
}

void DeltaControlSystem::disableAxis() {
	//board.setEnable(false);
	//board.setReset(true);
}

void DeltaControlSystem::setVoltageForInitializing(AxisVector u) {
	voltageSetPoint.setValue(u);
}

bool DeltaControlSystem::switchToPosControl() {
	//if(homed || !allAxisStopped()) return false;
	//board.resetPositions(q012homingOffset, q012homingOffset, q012homingOffset, q3homingOffset);	
	setVoltageForInitializing({0, 0, 0, 0});
	homed = true;
	return true;
}


void DeltaControlSystem::goToPos(double x, double y, double z, double phi) {
	AxisVector p;
	p << x, y, z, phi;
	//pathPlanner.gotoPoint(p);
}

void DeltaControlSystem::initBoard() {
	/*if(!board.open("/dev/spidev1.0"))
		throw EEROSException("failed to open SPI device");*/
}

AxisVector DeltaControlSystem::getTcpPos() {
	return directKin.getOut().getSignal().getValue();
}

AxisVector DeltaControlSystem::getAxisPos() {
	return angleGear.getOut().getSignal().getValue();
}

bool DeltaControlSystem::allAxisStopped(double maxSpeed) {
	for(int i = 0; i < nofAxis; i++) {
		if(demuxMot.getOut(i).getSignal().getValue() > maxSpeed) return false;
	}
	return true;
}

bool DeltaControlSystem::axisHomed() {
	return homed;
}

DeltaControlSystem::~DeltaControlSystem() { }