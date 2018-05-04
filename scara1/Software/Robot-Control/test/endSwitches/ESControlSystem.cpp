#include "ESControlSystem.hpp"
#include "ESSafetyProperties.hpp"
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

ESControlSystem::ESControlSystem() :
	
	timedomain("Main time domain", dt, true)

	{
		// Connect blocks

		// Run blocks
		
// 		eeros::task::Periodic td("control system",dt, timedomain);
		eeros::Executor::instance().add(timedomain); // td
	}
	