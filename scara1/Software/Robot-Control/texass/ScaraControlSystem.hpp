#ifndef CH_NTB_SCARA_SCARACONTROLSYSTEM_HPP_
#define CH_NTB_SCARA_SCARACONTROLSYSTEM_HPP_

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

#include "control/workspaceLimitation/CartesianWorkspaceLimitation.hpp"
#include "control/kinematicDynamic/ScaraInverseKinematic.hpp"
#include "control/kinematicDynamic/ScaraDirectKinematic.hpp"
#include "control/pathPlanner/PathPlanner.hpp"
#include "control/I.hpp"
#include "control/SetPlasmaPen.hpp"
#include "joystick/XBoxInput.hpp"
#include "joystick/XBoxButtons.hpp"
#include "constants.hpp"
#include "types.hpp"

namespace scara {
	
	class ScaraControlSystem {
		
	protected:
		AxisVector Mmatrix, Imatrix, Kmatrix;
		// controller data
		AxisVector posCtrlGain; 
		AxisVector velCtrlGain; 
		AxisVector posIntGain; 
		//path planner data
		AxisVector velMaxPpCS = 1.0;
		AxisVector accMaxPpCS = 1.0; 
		AxisVector velMaxPpJS = 0.5;  // set 0.2 speed for initialization!!
		AxisVector accMaxPpJS = 0.2;
		
		eeros::control::TimeDomain timedomain;
		
	public:
		ScaraControlSystem();
	
		void start();
		void stop();
		
		AxisVector toBasisCoordinate(AxisVector pos, char tool, eeros::math::CoordinateSystem& c);
		AxisVector toUserCoordinate(AxisVector pos, char tool, eeros::math::CoordinateSystem& c);
		AxisVector toCalibratedValue(AxisVector pos, eeros::control::Vector2Corrector& calibrationTable);
			
		eeros::control::Vector2Corrector meshCalibrationLut;
		eeros::control::Vector2Corrector partsCalibrationLut;
		
		scara::XBoxInput   xbox;
		scara::XBoxButtons xbox_buttons;
		scara::SetPlasmaPen setPlasmaPen;
		eeros::control::PeripheralInput<double> enc0;
		eeros::control::PeripheralInput<double> enc1;
		eeros::control::PeripheralInput<double> enc2;
		eeros::control::PeripheralInput<double> enc3;
		eeros::control::Gain<double> invIencPos0;
		eeros::control::Gain<double> invIencPos1;
		eeros::control::Gain<double> invIencPos2;
		eeros::control::Gain<double> invIencPos3;
		eeros::control::Sum<2, double> q2Sum;
		eeros::control::Gain<double> q2Gain;
		eeros::control::Mux<4, double> muxEncPos;
		eeros::control::D<AxisVector> encPosDiff;
		scara::PathPlanner pathPlannerJS;
		scara::PathPlanner pathPlannerCS;
		scara::ScaraDirectKinematic dirKin;
		scara::ScaraInverseKinematic invKin;
		eeros::control::Switch<2, AxisVector> autoToManualSwitch;
		eeros::control::Switch<2, AxisVector> pathPlannerPosSwitch;
		scara::CartesianWorkspaceLimitation cartesianWorkspaceLimit;
		eeros::control::D<AxisVector> refPosDiff;
		eeros::control::Sum<2, AxisVector> posSum;
		
		scara::I<AxisVector> posIntegral;
		eeros::control::Gain<AxisVector, AxisVector, true> posIntegralGain;
		eeros::control::Sum<2, AxisVector> posIntegralSum;
		
		eeros::control::Gain<AxisVector, AxisVector, true> posController; 
		eeros::control::Sum<2, AxisVector> velFfwSum;
		eeros::control::Sum<2, AxisVector> velSum;
		eeros::control::Gain<AxisVector, AxisVector, true> velController; 
		eeros::control::Gain<AxisVector, AxisVector, true> massMatrix;
		eeros::control::Gain<AxisVector, AxisVector, true> invKm;
		eeros::control::Gain<AxisVector, AxisVector, true> invI;
		eeros::control::DeMux<4, double> demuxDac; 
		eeros::control::PeripheralOutput<double> dac0;
		eeros::control::PeripheralOutput<double> dac1;
		eeros::control::PeripheralOutput<double> dac2;
		eeros::control::PeripheralOutput<double> dac3;
		
		// ref. systems
		eeros::math::CoordinateSystem roboter;
 		eeros::math::CoordinateSystem gestell;
		eeros::math::CoordinateSystem spannrahmen;
		eeros::math::CoordinateSystem bauteile;
		eeros::math::CoordinateSystem mesh;
		eeros::math::Frame KG;
		eeros::math::Frame KS;
		eeros::math::Frame KB;
		eeros::math::Frame KM;

		AxisVector readyPositionCartesian;
		AxisVector toolOffset;
		double angleOffset;
		AxisVector cameraRefPoint;
		AxisVector tipRefPoint; 
		AxisVector crossRefPoint; 
		
		double cameraX, cameraY, cameraZ, cameraAlpha;  
		double kameraX, kameraY, kameraZ, kameraAlpha;
		double greiferX, greiferY, greiferZ, greiferAlpha;  
		double dispencerX, dispencerY, dispencerZ, dispencerAlpha; 
		double plasmaX, plasmaY, plasmaZ, plasmaAlpha;

		eeros::math::Matrix<13, 1, double> mesh_cartesianLimitation_parameters;
		eeros::math::Matrix<13, 1, double> parts_cartesianLimitation_parameters;
		
	}; // END class
}; // END namespace

#endif // CH_NTB_SCARA_SCARACONTROLSYSTEM_HPP_