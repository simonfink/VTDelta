#include <eeros/control/TimeDomain.hpp> 
#include <eeros/control/Constant.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp> 
#include <eeros/logger/SysLogWriter.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/control/Mux.hpp>
#include <eeros/control/PathPlannerTrapezoid.hpp>
#include "../../control/kinematic/InverseKinematic.hpp"
#include "../../control/kinematic/DirectKinematic.hpp"
#include "../../constants.hpp"
#include "../../types.hpp"
#include <unistd.h>
#include <iostream>
#include <signal.h>
#include <fstream>

using namespace parallelscara;
using namespace eeros;
using namespace eeros::hal;
using namespace eeros::safety;
using namespace eeros::control;
using namespace eeros::logger;

class TestControlSystem {	
public:
	TestControlSystem() :
		c({0.0, 0.3}),
		pathPlanner(velMax, accMax, accMax, dt),
		dirKin(l1, l2),
		invKin(l1, l2, q0_limit, q1_limit, deltaQ_limMin, deltaQ_limMax),
		timedomain("Main time domain", dt, true)
	{
		
		invKin.getInActPosEnc().connect(c.getOut());
		invKin.getInActPosXY().connect(pathPlanner.getPosOut());
		dirKin.getIn().connect(invKin.getOut());
		
		timedomain.addBlock(c);
		timedomain.addBlock(pathPlanner);
		timedomain.addBlock(invKin);
		timedomain.addBlock(dirKin);
	}
		
	Constant<AxisVector> c;
	PathPlannerTrapezoid<AxisVector> pathPlanner; 
	InverseKinematic invKin; 
	DirectKinematic dirKin; 
	TimeDomain timedomain;	
};


int main() {
	StreamLogWriter w(std::cout);
	w.show();
	Logger::setDefaultWriter(&w);
	Logger log('M');
	log.info() << "Direct kinematic test started";
	TestControlSystem controlSystem;
	
	controlSystem.pathPlanner.setMaxSpeed(10.0);
	controlSystem.pathPlanner.setMaxAcc(10.0);
	controlSystem.pathPlanner.setInitPos(ready_pos);
	controlSystem.timedomain.run();
	sleep(1.0);
	
	controlSystem.invKin.enable();
	
	AxisVector pos = {ready_pos(0), ready_pos(1) + 0.1 };
	controlSystem.pathPlanner.move(pos);
	while(!controlSystem.pathPlanner.endReached()){
		controlSystem.timedomain.run();
		auto time = controlSystem.pathPlanner.getPosOut().getSignal().getTimestamp();
		auto path = controlSystem.pathPlanner.getPosOut().getSignal().getValue();
		auto dirKin = controlSystem.dirKin.getOut().getSignal().getValue();
		auto invKin = controlSystem.invKin.getOut().getSignal().getValue();
		log.info() << time << " panned pos:" << path << " dir kin:" << dirKin << " inv kin:" << invKin;
	}
	
	pos = {ready_pos(0) - 0.1, ready_pos(1)};
	controlSystem.pathPlanner.move(pos);
	while(!controlSystem.pathPlanner.endReached()){
		controlSystem.timedomain.run();
		auto time = controlSystem.pathPlanner.getPosOut().getSignal().getTimestamp();
		auto path = controlSystem.pathPlanner.getPosOut().getSignal().getValue();
		auto dirKin = controlSystem.dirKin.getOut().getSignal().getValue();
		auto invKin = controlSystem.invKin.getOut().getSignal().getValue();
		log.info() << time << " panned pos:" << path << " dir kin:" << dirKin << " inv kin:" << invKin;
	}

	log.info() << "test end";
	return 0;
}
