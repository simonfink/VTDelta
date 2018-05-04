#ifndef CH_NTB_SCARA_PP_CONTROLSYSTEM_HPP_
#define CH_NTB_SCARA_PP_CONTROLSYSTEM_HPP_

#include <eeros/math/Matrix.hpp>
#include <eeros/control/TimeDomain.hpp> 
#include <eeros/control/PathPlanner.hpp>
#include <eeros/control/Switch.hpp>
#include <eeros/control/Constant.hpp>

#include "../../constants.hpp"
#include "../../types.hpp"

namespace scara {
	
	class PPControlSystem {
		
	protected:
		eeros::control::TimeDomain timedomain;		
		//path planner data
		AxisVector velMaxPpCS = 1.0;
		AxisVector accMaxPpCS = 1.0; 
		AxisVector velMaxPpJS = 0.5;  // set 0.2 speed for initialization!!
		AxisVector accMaxPpJS = 0.2;
		
	public:
		PPControlSystem();
		
		eeros::control::PathPlanner pathPlannerJS;
// 		eeros::control::PathPlanner pathPlannerCS;
		
		eeros::control::Switch<2,AxisVector> speedSwitch;
		eeros::control::Constant<AxisVector> speedInit;
		
	}; // END class
}; // END namespace

#endif // CH_NTB_SCARA_PP_CONTROLSYSTEM_HPP_