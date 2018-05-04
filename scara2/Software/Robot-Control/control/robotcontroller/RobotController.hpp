#ifndef CH_NTB_PARALLELSCARA_ROBOTCONTROLLER_HPP_
#define CH_NTB_PARALLELSCARA_ROBOTCONTROLLER_HPP_

#include <eeros/control/Block.hpp>
#include <eeros/control/Switch.hpp>
#include <eeros/control/D.hpp>
#include <eeros/control/Sum.hpp>
#include <eeros/control/Gain.hpp>
#include <eeros/control/DeMux.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/control/Switch.hpp>
#include <eeros/control/SignalChecker.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include "../../safety/ParallelScaraSafetyProperties.hpp"
#include "SpeedSaturation.hpp"

#include "../../types.hpp"

using namespace eeros;
using namespace eeros::control;
using namespace parallelscara;

namespace parallelscara {
	class RobotController : public Block {
	public: 
		RobotController(SafetySystem& safetySys, ParallelScaraSafetyProperties& ssProperties);
		virtual ~RobotController();
	
		virtual void run();
		virtual Input<AxisVector>& getInEncPosAct();
		virtual Input<AxisVector>& getInEncRefPos();
		virtual Input<AxisVector>& getInEncRefVel();
		virtual Output<AxisVector>& getOutSetpointMotors();
		void setInitializationSpeed(AxisVector u);
		void setSpeedSwitch(bool s);
		bool reachedMechanicalLimit(double maxTorque = 1.4);
		AxisVector getActSpeed();
		
		AxisVector initPos;
		
		Sum<2, AxisVector> posSum, velFfwSum, velSum;
		D<AxisVector> encPosDiff;
		Constant<AxisVector> speedInitSetPoint;
		Switch<2,AxisVector> speedSwitch;
		SpeedSaturation speedSaturation;
		Gain<AxisVector,AxisVector,true> bufEncPosAct, posController, velController, massMatrix, invKm, invI;
	};
};

#endif /* CH_NTB_PARALLELSCARA_ROBOTCONTROLLER_HPP_ */