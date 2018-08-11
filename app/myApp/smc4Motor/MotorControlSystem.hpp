#ifndef MOTORCONTROLSYSTEM_HPP_
#define MOTORCONTROLSYSTEM_HPP_

#include <eeros/control/Sum.hpp>
#include <eeros/control/D.hpp>
#include <eeros/control/Gain.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/control/PeripheralInput.hpp>
#include <eeros/control/PeripheralOutput.hpp>
#include <eeros/control/TimeDomain.hpp>

#include <eeros/control/Mux.hpp>
#include <eeros/control/DeMux.hpp>

#include "../control/types.hpp"

class MotorControlSystem {

public:
	MotorControlSystem(double ts);
	~MotorControlSystem();
	
	eeros::control::Constant<AxisVector> setpoint;
	
	eeros::control::PeripheralInput<double> enc1;
	eeros::control::PeripheralInput<double> enc2;
	eeros::control::PeripheralInput<double> enc3;
	eeros::control::PeripheralInput<double> enc4;
	
	/*eeros::control::D<AxisVector> diff1;
	eeros::control::Sum<2, AxisVector> sum1;
	eeros::control::Gain<AxisVector> posController;
	eeros::control::D<AxisVector> diff2;
	eeros::control::Sum<3, AxisVector> sum2;
	eeros::control::Gain<AxisVector> speedController;
	eeros::control::Gain<AxisVector> inertia;
	eeros::control::Gain<AxisVector> invMotConst;*/
	
	eeros::control::PeripheralOutput<double> mot1;
	eeros::control::PeripheralOutput<double> mot2;
	eeros::control::PeripheralOutput<double> mot3;
	eeros::control::PeripheralOutput<double> mot4;
	
	eeros::control::Mux<4> muxEnc;
	eeros::control::DeMux<4> demuxMot;

	eeros::control::TimeDomain timedomain;
};

#endif // MOTORCONTROLSYSTEM_HPP_