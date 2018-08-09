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


class MotorAxisControlSystem {
  public:
    MotorAxisControlSystem(double dt):
    constant({0.0,1.0,2.0,4.0}),
    mot1("motor1"),
    mot2("motor2"),
    mot3("motor3"),
    mot4("motor4"),
    timedomain("Main time domain", dt, true)
    {
      mot1.getIn().connect(demux.getOut(0));
      mot2.getIn().connect(demux.getOut(1));
      mot3.getIn().connect(demux.getOut(2));
      mot4.getIn().connect(demux.getOut(3));
      
      demux.getIn().connect(constant.getOut());
      

      timedomain.addBlock(mot1);
      timedomain.addBlock(mot2);
      timedomain.addBlock(mot3);
      timedomain.addBlock(mot4);
      timedomain.addBlock(demux);
      timedomain.addBlock(constant);
    };

    eeros::control::Constant<AxisVector> constant;
    
    eeros::control::PeripheralOutput<double> mot1;
    eeros::control::PeripheralOutput<double> mot2;
    eeros::control::PeripheralOutput<double> mot3;
    eeros::control::PeripheralOutput<double> mot4;
    eeros::control::DeMux<4,double, AxisVector> demux;

    eeros::control::TimeDomain timedomain;
};


#endif // MOUSECONTROLSYSTEM_HPP_

