#include "PPControlSystem.hpp"
#include "PPSafetyProperties.hpp"
#include <eeros/math/Matrix.hpp>
#include <iostream>
#include <unistd.h>
#include <eeros/core/EEROSException.hpp>
#include <eeros/hal/HAL.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/task/Periodic.hpp>
#include <eeros/core/PeriodicCounter.hpp>
#include "../../constants.hpp"

using namespace scara;
using namespace eeros::control;
using namespace eeros::math;
using namespace eeros::hal;
using namespace scara;

	PPControlSystem::PPControlSystem() :
	
	speedSwitch(0),
	speedInit(1.0),
	
	pathPlannerJS(velMaxPpJS, accMaxPpJS, -accMaxPpJS, dt),
// 	pathPlannerCS(velMaxPpCS, accMaxPpCS, -accMaxPpJS, dt),
	timedomain("Main time domain", dt, true)

	{
		// Connect blocks
		speedSwitch.getIn(0).connect(speedInit.getOut());

		// Run blocks
		timedomain.addBlock(&speedInit);
		timedomain.addBlock(&speedSwitch);
		timedomain.addBlock(&pathPlannerJS);
// 		timedomain.addBlock(&pathPlannerCS);
		
// 		eeros::task::Periodic td("control system",dt, timedomain);
		eeros::Executor::instance().add(timedomain); // td
	}
	