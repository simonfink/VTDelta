#ifndef CH_NTB_SCARA_SCARACONTROLSYSTEM_FORCE_HPP_
#define CH_NTB_SCARA_SCARACONTROLSYSTEM_FORCE_HPP_

#include <eeros/math/Matrix.hpp>
#include <eeros/control/TimeDomain.hpp> 
#include <eeros/control/Sum.hpp>
#include <eeros/control/D.hpp>
#include <eeros/control/Gain.hpp>
#include <eeros/control/Switch.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/control/PeripheralInput.hpp>
#include <eeros/control/PeripheralOutput.hpp>
#include <eeros/control/Mux.hpp>
#include <eeros/control/DeMux.hpp>
#include <eeros/control/Vector2Corrector.hpp>
#include <eeros/math/Frame.hpp>
#include <eeros/control/PathPlanner.hpp>
#include <eeros/control/I.hpp>
#include <eeros/control/SignalChecker.hpp>
#include <eeros/control/Trace.hpp>
#include <eeros/control/Saturation.hpp>

#include <eeros/safety/SafetySystem.hpp>

#include "ScaraSafetyProperties_force.hpp"
#include "../control/workspaceLimitation/CartesianWorkspaceLimitation.hpp"
#include "../control/kinematicDynamic/ScaraInverseKinematic.hpp"
#include "../control/kinematicDynamic/ScaraDirectKinematic.hpp"
#include "../control/kinematicDynamic/ScaraTransposeJacobi.hpp"
#include "../control/kinematicDynamic/DynModell.hpp"
#include "../control/Filter.hpp"
#include "../joystick/XBoxInput.hpp"
#include "../joystick/XBoxButtons.hpp"
#include "../constants.hpp"
#include "../types.hpp"

using namespace eeros::safety;

namespace scara {
	
	class ScaraControlSystem_force {
		
		public:
			ScaraControlSystem_force(SafetySystem& safetySys, ScaraSafetyProperties_force& ssProperties);
			
			void setInitSpeed(AxisVector v);
			void switchToInitSpeed();
			void switchToPositionControl();
			
			eeros::control::Mux<4, double> muxEncPos;
			eeros::control::Mux<4, double> muxSenForce;
			eeros::control::DeMux<4, double> demux_encPosDiff; // needed for check in safety system (can access only doubles)
			
			eeros::control::Constant<AxisVector> initSpeed;   // TODO shift down
			eeros::control::Constant<AxisVector> initForce;   // TODO shift down
			eeros::control::Constant<AxisVector> initForcePos;   // TODO shift down
			eeros::control::Switch<2, AxisVector> initSwitch; // TODO shift down
			eeros::control::DeMux<4, double> demuxDac;        // TODO shift down
			
// 		private:
			AxisVector Mmatrix, Imatrix, Kmatrix;
			// controller data
			AxisVector initSpeedConst;
			AxisVector initForceConst;
			AxisVector initForcePosConst;
			AxisVector posCtrlGain; 
			AxisVector velCtrlGain;
			AxisVector forceCrtlGain;
			AxisVector forceCrtlGain_f;
			AxisVector forceCrtlGain_dx;
			AxisVector forceCrtlGain_f1;
			AxisVector forceCrtlGain_x;
			AxisVector forceCrtlGain_x_dx;
			AxisVector posIntGain; 
			//path planner data
			AxisVector velMaxPpCS = 1.0;
			AxisVector accMaxPpCS = 1.0; 
			AxisVector velMaxPpJS = 0.5;  // set 0.2 speed for initialization!!
			AxisVector accMaxPpJS = 0.2;
			
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
			
			eeros::control::Constant<double> enc0_offset;
			eeros::control::Constant<double> enc1_offset;
			eeros::control::Constant<double> enc2_offset;
			eeros::control::Constant<double> enc3_offset;
			eeros::control::Sum<2, double> enc0_offset_sum;
			eeros::control::Sum<2, double> enc1_offset_sum;
			eeros::control::Sum<2, double> enc2_offset_sum;
			eeros::control::Sum<2, double> enc3_offset_sum;
			
			eeros::control::Gain<double> invIencPos0;
			eeros::control::Gain<double> invIencPos1;
			eeros::control::Gain<double> invIencPos2;
			eeros::control::Gain<double> invIencPos3;
			eeros::control::Gain<double> invSenForce0;
			eeros::control::Gain<double> invSenForce1;
			eeros::control::Gain<double> invSenForce2;
			eeros::control::Gain<double> invSenMomentz;
			eeros::control::Sum<2, double> q2Sum;
			eeros::control::Gain<double> q2Gain;
			
			scara::XBoxInput   xbox;
			scara::XBoxButtons xbox_buttons;
			
			eeros::control::D<AxisVector> encPosDiff;
			
			eeros::control::PathPlanner pathPlannerJS;
			eeros::control::PathPlanner pathPlannerCS;
			scara::ScaraInverseKinematic invKin;
			scara::ScaraInverseKinematic invKinForce_dx;
			scara::ScaraInverseKinematic invKinForce_x;
			scara::ScaraDirectKinematic dirKin;
			scara::ScaraTransposeJacobi transJ;
			scara::ScaraTransposeJacobi transJ1;
			scara::ScaraTransposeJacobi transJ2;
			//scara::Filter filter;
			//PT1-Filter
			eeros::control::Gain<AxisVector> kp_pt1;
			eeros::control::Gain<AxisVector> T_pt1;
			eeros::control::I<AxisVector> I_pt1;
			eeros::control::Sum<2, AxisVector> pt1Sum;
			
			eeros::control::Switch<2, AxisVector> jointsRefSwitch;
			eeros::control::Switch<3, AxisVector> cartesRefSwitch;
			eeros::control::D<AxisVector> refPosDiff;
			eeros::control::Sum<2, AxisVector> posSum;
			
			eeros::control::Gain<AxisVector, AxisVector, true> posController; 
			eeros::control::Sum<2, AxisVector> velFfwSum;
			
			eeros::control::Sum<2, AxisVector> velSum;
			eeros::control::Gain<AxisVector, AxisVector, true> velController;
			eeros::control::Sum<2, AxisVector> forceSum;
			eeros::control::Gain<AxisVector, AxisVector, true> forceController; 
			eeros::control::Gain<AxisVector, AxisVector, true> forceController_f; 
			eeros::control::Gain<AxisVector, AxisVector, true> forceController_dx; 
			eeros::control::Sum<2, AxisVector> forceSum_dx;
			eeros::control::D<AxisVector> refPosDiffForce_dx;
			eeros::control::Gain<AxisVector, AxisVector, true> forceController_f1; 
			eeros::control::Gain<AxisVector, AxisVector, true> forceController_x;
			eeros::control::Gain<AxisVector, AxisVector, true> forceController_x_dx;
			eeros::control::Sum<3, AxisVector> forceSum_x;
			eeros::control::D<AxisVector> refPosDiffForce_x;
			eeros::control::Sum<2, AxisVector> forceSum_x_dx;
			
			eeros::control::Switch<4, AxisVector> positionOrForceControlSwitch;
			eeros::control::Gain<AxisVector, AxisVector, true> massMatrix;
			eeros::control::Gain<AxisVector, AxisVector, true> invKm;
			eeros::control::Gain<AxisVector, AxisVector, true> invI;
			eeros::control::PeripheralOutput<double> dac0;
			eeros::control::PeripheralOutput<double> dac1;
			eeros::control::PeripheralOutput<double> dac2;
			eeros::control::PeripheralOutput<double> dac3;
			
			eeros::control::SignalChecker<double> checker_q0;
			eeros::control::SignalChecker<double> checker_q1;
			eeros::control::SignalChecker<double> checker_q2;
			eeros::control::SignalChecker<double> checker_q3;
			
			eeros::control::SignalChecker<double> checker_q0_speed;
			eeros::control::SignalChecker<double> checker_q1_speed;
			eeros::control::SignalChecker<double> checker_q2_speed;
			eeros::control::SignalChecker<double> checker_q3_speed;
			using Vector4 = eeros::math::Matrix<4, 1>;
			eeros::control::Trace<Vector4> trace0, trace1;
			
			// DynModell
			
			scara::DynModell DynModell;
			eeros::control::I<AxisVector> I_acc_to_vel;
			eeros::control::I<AxisVector> I_vel_to_pos;
			eeros::control::Saturation<AxisVector> SatPos;
			
			
			
					
	// 		AxisVector readyPositionCartesian; // TODO needed?
			eeros::control::TimeDomain timedomain;
		
	}; // END class
}; // END namespace

#endif // CH_NTB_SCARA_SCARACONTROLSYSTEM_FORCE_HPP_