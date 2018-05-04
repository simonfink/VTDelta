#ifndef CH_NTB_SCARA_PATHPLANNER_HPP_
#define CH_NTB_SCARA_PATHPLANNER_HPP_

#include <eeros/control/Block.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/control/ConstantAccTrajectoryGenerator.hpp>
#include <mutex>
#include "../../types.hpp"

using namespace eeros;
using namespace eeros::control;

namespace scara {
		class PathPlanner: public Block {
			
		public:
			PathPlanner(AxisVector velMax, AxisVector accMax, double dt);
			
			virtual Output<AxisVector>& getPosOut();
			virtual Output<AxisVector>& getVelOut();
			virtual Output<AxisVector>& getAccOut();
			
			virtual void gotoPoint(AxisVector p);
			virtual bool posReached();
			virtual void setInitPos(AxisVector initPos);
			
			virtual void setVelMax(AxisVector velMax); 
			virtual void setAccMax(AxisVector accMax); 
			
			virtual void run();
		
		protected:
			Output<AxisVector> posOut;
			Output<AxisVector> velOut;
			Output<AxisVector> accOut;
			std::mutex mtx;
			double tOld;
			
			ConstantAccTrajectoryGenerator<AxisVector> trajectoryGen;
		};
	}
#endif /* CH_NTB_SCARA_PATHPLANNER_HPP_ */