#ifndef CH_NTB_SCARA_FS_CONTROLSYSTEM_HPP_
#define CH_NTB_SCARA_FS_CONTROLSYSTEM_HPP_

#include <eeros/math/Matrix.hpp>
#include <eeros/control/TimeDomain.hpp> 
#include <eeros/control/PathPlanner.hpp>
#include <eeros/control/PeripheralInput.hpp>
#include <eeros/control/Gain.hpp>

#include "../../constants.hpp"
#include "../../types.hpp"

namespace scara {
	
	class FSControlSystem {
		
	protected:
		eeros::control::TimeDomain timedomain;
		
	public:
		FSControlSystem();
		
		eeros::control::PeripheralInput<double> fx;
		eeros::control::PeripheralInput<double> fy;
		eeros::control::PeripheralInput<double> fz;
		eeros::control::PeripheralInput<double> mx;
		eeros::control::PeripheralInput<double> my;
		eeros::control::PeripheralInput<double> mz;
		
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

#endif // CH_NTB_SCARA_FS_CONTROLSYSTEM_HPP_