#ifndef MOUSECONTROLSYSTEM_HPP_
#define MOUSECONTROLSYSTEM_HPP_

#include <eeros/control/Sum.hpp>
#include <eeros/control/D.hpp>
#include <eeros/control/Gain.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/control/PeripheralInput.hpp>
#include <eeros/control/PeripheralOutput.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/control/MouseInput.hpp>
#include <eeros/control/Saturation.hpp>
#include <eeros/control/DeMux.hpp>
#include "control/types.hpp"


class MotorAxisVectorTestControlSystem {
  public:
    MotorAxisVectorTestControlSystem(double dt):
    enc1("enc1"),
    motor1("motor1"),
    enc2("enc2"),
    motor2("motor2"),
    enc3("enc3"),
    motor3("motor3"),
    enc4("enc4"),
    motor4("motor4"),
    constants({0.0, 1.0, 2.0, 4.0}),
    timedomain("Main time domain", dt, true)
    {
      motor1.getIn().connect(demux.getOut(0));
      motor2.getIn().connect(demux.getOut(1));
      motor3.getIn().connect(demux.getOut(2));
      motor4.getIn().connect(demux.getOut(3));
      
      demux.getIn().connect(constants.getOut());

      timedomain.addBlock(enc1);
      timedomain.addBlock(motor1);
      timedomain.addBlock(enc2);
      timedomain.addBlock(motor2);
      timedomain.addBlock(enc3);
      timedomain.addBlock(motor3);
      timedomain.addBlock(enc4);
      timedomain.addBlock(motor4);
      timedomain.addBlock(demux);
      timedomain.addBlock(constants);
    };

    eeros::control::Signal<double>& getEncoderSignal(){
      return enc1.getOut().getSignal();
    }

    eeros::control::Constant<AxisVector> constants;

    eeros::control::PeripheralInput<double> enc1;
    eeros::control::PeripheralOutput<double> motor1;
    eeros::control::PeripheralInput<double> enc2;
    eeros::control::PeripheralOutput<double> motor2;
    eeros::control::PeripheralInput<double> enc3;
    eeros::control::PeripheralOutput<double> motor3;
    eeros::control::PeripheralInput<double> enc4;
    eeros::control::PeripheralOutput<double> motor4;
    
    eeros::control::DeMux<4, double, AxisVector> demux;
    
    eeros::control::TimeDomain timedomain;

};


#endif // MOUSECONTROLSYSTEM_HPP_