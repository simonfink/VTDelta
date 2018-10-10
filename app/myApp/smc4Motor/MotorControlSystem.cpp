#include "MotorControlSystem.hpp"
#include <eeros/core/Executor.hpp>

using namespace eeros::control;

MotorControlSystem::MotorControlSystem(double ts) : 
	setpoint({0.0,0.0,0.0,0.0}),
	enc1("enc1"),
	enc2("enc2"),
	enc3("enc3"),
	enc4("enc4"),
	//posController(174.5),
	//speedController(565.48),
	//inertia(9.49e-7),
	//invMotConst(1/16.3e-3),
	mot1("motor1"),
	mot2("motor2"),
	mot3("motor3"),
	mot4("motor4"),
	timedomain("Main time domain", ts, true) {
	
	setpoint.getOut().getSignal().setName("phi_desired");

	enc1.getOut().getSignal().setName("phi1_actual");
	enc2.getOut().getSignal().setName("phi2_actual");
	enc3.getOut().getSignal().setName("phi3_actual");
	enc4.getOut().getSignal().setName("phi4_actual");

	//diff1.getOut().getSignal().setName("phi_d_actual");
	//sum1.negateInput(1);
	//sum1.getOut().getSignal().setName("phi_e");
	
	//posController.getOut().getSignal().setName("phi_d_set");
	
	//diff2.getOut().getSignal().setName("phi_d_set_ff");
	
	//sum2.negateInput(1);
	//sum2.getOut().getSignal().setName("phi_d_e");
	
	//speedController.getOut().getSignal().setName("phi_dd_set");

	//inertia.getOut().getSignal().setName("M");

	//invMotConst.getOut().getSignal().setName("i");	
	
	muxEnc.getIn(0).connect(enc1.getOut());
	muxEnc.getIn(1).connect(enc2.getOut());
	muxEnc.getIn(2).connect(enc3.getOut());
	muxEnc.getIn(3).connect(enc4.getOut());
	
	/*diff1.getIn().connect(muxEnc.getOut());
	
	
	sum1.getIn(0).connect(setpoint.getOut());
	
	sum1.getIn(1).connect(muxEnc.getOut());
	
	posController.getIn().connect(sum1.getOut());
	diff2.getIn().connect(setpoint.getOut());
	sum2.getIn(0).connect(posController.getOut());
	sum2.getIn(1).connect(diff1.getOut());
	sum2.getIn(2).connect(diff2.getOut());
	speedController.getIn().connect(sum2.getOut());
	inertia.getIn().connect(speedController.getOut());
	invMotConst.getIn().connect(inertia.getOut());
	
	demuxMot.getIn().connect(invMotConst.getOut());*/
	
	demuxMot.getIn().connect(setpoint.getOut());
	
	mot1.getIn().connect(demuxMot.getOut(0));
	mot2.getIn().connect(demuxMot.getOut(1));
	mot3.getIn().connect(demuxMot.getOut(2));
	mot4.getIn().connect(demuxMot.getOut(3));

	
	timedomain.addBlock(setpoint);
	timedomain.addBlock(muxEnc);
	timedomain.addBlock(enc1);
	timedomain.addBlock(enc2);
	timedomain.addBlock(enc3);
	timedomain.addBlock(enc4);
	/*timedomain.addBlock(diff1);
	timedomain.addBlock(sum1);
	timedomain.addBlock(diff2);
	timedomain.addBlock(posController);
	timedomain.addBlock(sum2);
	timedomain.addBlock(speedController);
	timedomain.addBlock(inertia);
	timedomain.addBlock(invMotConst);*/
	timedomain.addBlock(demuxMot);
	timedomain.addBlock(mot1);
	timedomain.addBlock(mot2);
	timedomain.addBlock(mot3);
	timedomain.addBlock(mot4);

	eeros::Executor::instance().add(timedomain);
}

MotorControlSystem::~MotorControlSystem() { }