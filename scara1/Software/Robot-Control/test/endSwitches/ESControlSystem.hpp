#ifndef CH_NTB_SCARA_ES_CONTROLSYSTEM_HPP_
#define CH_NTB_SCARA_ES_CONTROLSYSTEM_HPP_

#include <eeros/math/Matrix.hpp>
#include <eeros/control/TimeDomain.hpp> 
#include <eeros/control/PathPlanner.hpp>

#include "../../constants.hpp"
#include "../../types.hpp"

namespace scara {
	
	class ESControlSystem {
		
	protected:
		eeros::control::TimeDomain timedomain;
		
	public:
		ESControlSystem();
		
	}; // END class
}; // END namespace

#endif // CH_NTB_SCARA_ES_CONTROLSYSTEM_HPP_