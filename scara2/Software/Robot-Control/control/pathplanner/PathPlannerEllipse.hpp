#ifndef CH_NTB_PARALLELSCARA_PATHPLANNERELLIPSE_HPP_
#define CH_NTB_PARALLELSCARA_PATHPLANNERELLIPSE_HPP_

#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/control/ConstantAccTrajectoryGenerator.hpp>
#include <mutex>

#include "../../types.hpp"

namespace parallelscara {
		class PathPlannerEllipse: public eeros::control::Block {
			
		public:
			PathPlannerEllipse(double a, double b, double dt, AxisVector centrePos);
			
			virtual eeros::control::Output<AxisVector>& getPosOut();
			virtual eeros::control::Input<AxisVector>& getIn_actPos_xy();
			
			virtual void run();
			virtual void enable();
			virtual void disable();
			virtual bool posReached();
		
// 		private: 
			bool enabled = 0;
			bool first = 1; 
			bool done = 0;
			
			AxisVector centrePos;
			AxisVector posPrev;
			double a0, b0, a, b, t, dt;
			
			double count = 1.0;
			double countLim1 = 8.0;
			double countLim2 = 8.0; 
			double t_incr = 0.001;
			
		protected:
			eeros::control::Output<AxisVector> posOut;
			eeros::control::Input<AxisVector> actPos_xy;
			std::mutex mtx;
			double tOld;
		};
	}
#endif /* CH_NTB_PARALLELSCARA_PATHPLANNERELLIPSE_HPP_ */
