#ifndef CH_NTB_SCARA_ENC_CONTROLSYSTEM_HPP_
#define CH_NTB_SCARA_ENC_CONTROLSYSTEM_HPP_

#include <eeros/math/Matrix.hpp>
#include <eeros/control/TimeDomain.hpp> 
#include <eeros/control/PathPlanner.hpp>
#include <eeros/control/PeripheralInput.hpp>
#include <eeros/control/Gain.hpp>

#include "../../constants.hpp"
#include "../../types.hpp"

namespace scara {
	
	class EncControlSystem {
		
	protected:
		eeros::control::TimeDomain timedomain;
		
	public:
		EncControlSystem();
		
		eeros::control::PeripheralInput<double> enc0;
		eeros::control::PeripheralInput<double> enc1;
		eeros::control::PeripheralInput<double> enc2;
		eeros::control::PeripheralInput<double> enc3;
		eeros::control::Gain<double> invIencPos0;
		eeros::control::Gain<double> invIencPos1;
		eeros::control::Gain<double> invIencPos2;
		eeros::control::Gain<double> invIencPos3;
		
		
	}; // END class
}; // END namespace

#endif // CH_NTB_SCARA_ENC_CONTROLSYSTEM_HPP_