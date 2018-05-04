#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp> 
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/control/TimeDomain.hpp> 
#include <eeros/control/PathPlannerTrapezoid.hpp>
#include "../../constants.hpp"
#include "../../types.hpp"

using namespace parallelscara;
using namespace eeros;
using namespace eeros::safety;
using namespace eeros::control;
using namespace eeros::logger;

const double period = 1.0;

int main() {	
	StreamLogWriter w(std::cout);
	w.show();
	Logger::setDefaultWriter(&w);
	Logger log('M');
	log.info() << "Path planner test started";
	
	PathPlannerTrapezoid<AxisVector> p1(velMax, accMax, decMax, period);
	p1.setMaxSpeed(5.0);
	p1.setMaxAcc(1.0);
	p1.setMaxDec(1.0);
	
	p1.setInitPos({0,0});
	p1.move(50.0);
	while (!p1.endReached()) {
		p1.run();
		auto time = p1.getPosOut().getSignal().getTimestamp();
		AxisVector pos = p1.getPosOut().getSignal().getValue();
		AxisVector vel = p1.getVelOut().getSignal().getValue();
		AxisVector acc = p1.getAccOut().getSignal().getValue();
		log.info() << pos << "  " << vel << "  " << acc;
	}
	AxisVector dst = {40, 10};
	p1.move(dst);
	while (!p1.endReached()) {
		p1.run();
		auto time = p1.getPosOut().getSignal().getTimestamp();
		AxisVector pos = p1.getPosOut().getSignal().getValue();
		AxisVector vel = p1.getVelOut().getSignal().getValue();
		AxisVector acc = p1.getAccOut().getSignal().getValue();
		log.info() << pos << "  " << vel << "  " << acc;
	}

	log.info() << "Path planner test end";
	return 0;
}
