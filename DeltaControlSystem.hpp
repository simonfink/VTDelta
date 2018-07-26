#ifndef DeltaCONTROLSYSTEM_HPP_
#define DeltaCONTROLSYSTEM_HPP_

#include <eeros/control/Sum.hpp>
#include <eeros/control/D.hpp>
#include <eeros/control/Gain.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/control/PeripheralInput.hpp>
#include <eeros/control/PeripheralOutput.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/control/MouseInput.hpp>
#include <eeros/control/Saturation.hpp>
#include <eeros/control/Switch.hpp>

#include <eeros/control/Mux.hpp>
#include <eeros/control/DeMux.hpp>

#include "../control/types.hpp"
#include "../control/Kinematic.hpp"
#include "../control/Jacobian.hpp"
#include "../control/DirectKinematic.hpp"
#include "../control/Jacobi.hpp"
#include "../control/Inertia.hpp"
#include "../control/MotorModel.hpp"
//#include "PathPlanner.hpp"
#include "../control/constants.hpp"

class DeltaControlSystem {

public:
	DeltaControlSystem(double td);
	~DeltaControlSystem();
	
	void start();
	void stop();
			
	// Methods for the sequencer
	void enableAxis();
	void disableAxis();
	void initBoard();
	void setVoltageForInitializing(AxisVector u);
	bool switchToPosControl();
	bool axisHomed();
	AxisVector getTcpPos();
	AxisVector getAxisPos();
	void goToPos(double x, double y, double z, double phi);
	
	AxisVector i;
	AxisVector kM;
	AxisVector RA;
	Kinematic kinematic;
	Jacobian jacobian;
	
	eeros::control::MouseInput mouse;
	
	eeros::control::PeripheralInput<double> enc1;
	eeros::control::PeripheralInput<double> enc2;
	eeros::control::PeripheralInput<double> enc3;
	eeros::control::PeripheralInput<double> enc4;
	
	eeros::control::PeripheralOutput<double> mot1;
	eeros::control::PeripheralOutput<double> mot2;
	eeros::control::PeripheralOutput<double> mot3;
	eeros::control::PeripheralOutput<double> mot4;
	
	eeros::control::Mux<4, double, AxisVector> muxEnc;
	eeros::control::DeMux<4, double, AxisVector> demuxMot;
	
	
	eeros::control::D<AxisVector> posDiff;
	eeros::control::Sum<2, AxisVector> posSum;
	eeros::control::Gain<AxisVector> posController;

	eeros::control::Sum<2, AxisVector> speedSum;
	eeros::control::Saturation<AxisVector> speedLimitation;
	eeros::control::Gain<AxisVector> speedController;
	
	Inertia inertia;
	Jacobi jacobi;
	
	eeros::control::Saturation<AxisVector> forceLimitation;
	
	eeros::control::Saturation<AxisVector> torqueLimitation;
	eeros::control::Gain<AxisVector, AxisVector, true> torqueGear;
	eeros::control::Gain<AxisVector, AxisVector, true> angleGear;
	
	MotorModel motorModel;
	
	eeros::control::Switch<2, AxisVector> voltageSwitch;
	eeros::control::Constant<AxisVector> voltageSetPoint;
	
	eeros::control::Gain<> gainMot4;

	
	DirectKinematic directKin;	


	eeros::control::TimeDomain timedomain;
	
	bool homed;
	
  private:
	bool allAxisStopped(double maxSpeed = 0.001);
};

#endif // DeltaCONTROLSYSTEM_HPP_