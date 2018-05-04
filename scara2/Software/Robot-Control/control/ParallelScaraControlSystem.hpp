#ifndef CH_NTB_PARALLELSCARA_CONTROLSYSTEM_HPP_
#define CH_NTB_PARALLELSCARA_CONTROLSYSTEM_HPP_

#include <eeros/math/Matrix.hpp>
#include <eeros/control/TimeDomain.hpp> 
#include <eeros/control/Gain.hpp>
#include <eeros/control/PeripheralInput.hpp>
#include <eeros/control/PeripheralOutput.hpp>
#include <eeros/control/Mux.hpp>
#include <eeros/control/DeMux.hpp>
#include <eeros/control/Switch.hpp>
#include <eeros/control/PathPlannerTrapezoid.hpp>
#include <eeros/control/SignalChecker.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/control/Trace.hpp>

#include "../safety/ParallelScaraSafetyProperties.hpp"
#include "kinematic/InverseKinematic.hpp"
#include "kinematic/DirectKinematic.hpp"
#include "robotcontroller/RobotController.hpp"
#include "pendulumcontrol/PendulumController.hpp"
#include "hallsensors/HallSensorsDataAcquisition.hpp"
#include "hallsensors/HallPhiRotationMatrix.hpp"
#include "SetWatchdog.hpp"

#include "../constants.hpp"
#include "../types.hpp"

namespace parallelscara {	
	class ParallelScaraControlSystem {
	public:
		ParallelScaraControlSystem(SafetySystem& safetySys, ParallelScaraSafetyProperties& ssProperties);
	
		void start();
		void stop();
		
		PeripheralInput<double> hall0, hall1, hall2, hall3;
		Mux<4,double> hallMux;
		HallSensorsDataAcquisition hallDataRead; 
		PeripheralOutput<double> hallXout, hallYout;
		
		PeripheralInput<double> q0EncPos, q1EncPos;
		Constant<double> q0EncOffset, q1EncOffset;
		Sum<2,double> q0EncSumOffset, q1EncSumOffset;
		Gain<double,double> q0InvGearRatio, q1InvGearRatio;
		Mux<2,double> encPosMux;
		
		HallPhiRotationMatrix rotAngleTransformation; 
		DirectKinematic directKinematic;
		PathPlannerTrapezoid<AxisVector> pathPlanner;
// 		PathPlannerEllipse pathPlannerEllipse; 
// 		Switch<2,AxisVector> pathPlannerSwitch; 
		PendulumController pendulumController;
		Switch<2,AxisVector> xyRefPosSwitch;
		InverseKinematic inverseKinematic;
		D<AxisVector> velPathPl;
		
		RobotController robotController;
		DeMux<2,double> demuxDAC;
		PeripheralOutput<double> dac0, dac1;
		
		D<AxisVector> encPosDiff;
		SignalChecker<AxisVector> checkPos, checkVel;
		SetWatchdog setWatchdog;
		Trace<AxisVector> trace1, trace2, trace3, trace4;
		
		TimeDomain timedomain;
		
	}; // END class
}; // END namespace

#endif // CH_NTB_PARALLELSCARA_CONTROLSYSTEM_HPP_