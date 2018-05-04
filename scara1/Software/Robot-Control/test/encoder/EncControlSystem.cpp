#include "EncControlSystem.hpp"
#include "EncSafetyProperties.hpp"
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

EncControlSystem::EncControlSystem() :
	enc0("q0"),
	enc1("q1"),
	enc2("q2r"),
	enc3("q3"),
	invIencPos0(1/i0),
	invIencPos1(1/i1),
	invIencPos2(1/i2),
	invIencPos3(1/i3),
	timedomain("Main time domain", dt, true)

	{
		// Connect blocks
		invIencPos0.getIn().connect(enc0.getOut());
		invIencPos1.getIn().connect(enc1.getOut());
		invIencPos2.getIn().connect(enc2.getOut());
		invIencPos3.getIn().connect(enc3.getOut());

		// Run blocks
		timedomain.addBlock(&enc0);
		timedomain.addBlock(&enc1);
		timedomain.addBlock(&enc2);
		timedomain.addBlock(&enc3);
		timedomain.addBlock(&invIencPos0);
		timedomain.addBlock(&invIencPos1);
		timedomain.addBlock(&invIencPos2);
		timedomain.addBlock(&invIencPos3);
		
// 		eeros::task::Periodic td("control system",dt, timedomain);
		eeros::Executor::instance().add(timedomain); // td
	}
	