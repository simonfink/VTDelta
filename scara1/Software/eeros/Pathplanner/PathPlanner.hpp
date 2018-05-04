#ifndef CH_NTB_SCARA_PATHPLANNER_HPP_
#define CH_NTB_SCARA_PATHPLANNER_HPP_

#include <eeros/types.hpp>
#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>
#include <vector>
#include <eeros/math/Matrix.hpp>
#include <eeros/core/Mutex.hpp>

#include <eeros/safety/SafetySystem.hpp>

using namespace eeros;
using namespace eeros::control;
using namespace eeros::math;
using namespace eeros::safety;

namespace scara {
	
		enum TrajectoryType { linearVelocity = 0, limitedJerk = 1, limitedJerkSquare = 2, limitedSnap = 3, trigonometric = 4 };
		
		class PathPlanner: public Block {
  
		public:
			PathPlanner(Vector4 velMax, Vector4 accMax, Vector4 decMax, double dt, uint32_t minSafetyLevel, TrajectoryType trajType = linearVelocity);
			PathPlanner(Vector4 velMax, Vector4 accMax, double dt, uint32_t minSafetyLevel, TrajectoryType trajType = linearVelocity);
			virtual ~PathPlanner();
			
			virtual void run();
			virtual void reset();
			virtual void enable();
			virtual void disable();
			virtual void setVelMax(Vector4 velMax);
			virtual void setAccMax(Vector4 accMax);
			virtual void setDecMax(Vector4 decMax);
			virtual void setInitialPosition(Vector4 posInit);
			virtual bool goPoint(Vector4 posFinal);
			
			virtual Output<Vector4>& getOutPos();
			virtual Output<Vector4>& getOutVel();
			virtual Output<Vector4>& getOutAcc();
			
// 		protected:
			bool enabled = false;	
			bool first = true;
			bool trajParamSet = false;
			bool pathEnded = false;
			
			Output<Vector4> acc;
			Output<Vector4> vel;
			Output<Vector4> pos;
			
// 		private:
			double dt;
			Vector4 velMax, accMax, decMax; 
			TrajectoryType trajType;
			Mutex mutex;
			int indexAddPos, indexReadPos;
			Vector4 distance, posPrev, posFinalPrev;
			static constexpr int dimBuffer = 200; 
			Vector4 posFinalBuffer[dimBuffer];
			Matrix<dimBuffer, 1, bool> isNewValue;
			double trajCoeff = 1;
			double velNorm, accNorm, decNorm; 
			double dT1, dT2, dT3; 
			double tOffset, time, timePrev, timeScaled;
			uint32_t minSafetyLevel;
	
			virtual void addPosition(Vector4 posFinal);
			virtual void calculateTrajectoryParameters();
			virtual void readPosition();
			virtual void setTrajCoeff(TrajectoryType trajType);	
			virtual double setPosGain(double k, double dK);
			virtual double setVelGain(double k, double dK);
			virtual double setAccGain(double k, double dK);
			virtual void updateIndexReadPos();
			virtual void checkPath();	
			
			SafetySystem& safetySys = SafetySystem::instance();
		};
} // END namespace scara

#endif /* CH_NTB_SCARA_PATHPLANNER_HPP_ */