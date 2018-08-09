#pragma once
#include <eeros/sequencer/Condition.hpp>
#include "../control/DeltaControlSystem.hpp"

#include <iostream>


namespace eeduro{
  namespace delta{
    
class MoveMouseCondition : public eeros::sequencer::Condition{
public:
  MoveMouseCondition(DeltaControlSystem& controlSys) : 
      controlSys(controlSys)
      {
//  	    mouseOld = controlSys.mouse.getOut().getSignal().getValue();
      }
      
  bool validate() {
    bool retVal = false;
     mouseNew = controlSys.mouse.getOut().getSignal().getValue();
     std::cout << mouseNew << std::endl;
     if(controlSys.mouse.getButtonOut().getSignal().getValue()[0]
       || controlSys.mouse.getButtonOut().getSignal().getValue()[1]
       || controlSys.mouse.getButtonOut().getSignal().getValue()[2])
     {
//        std::cout << "mouse button pressed" << std::endl; 
       retVal = true;
     }
     if(mouseNew!= mouseOld){
//        std::cout << "mouse moved" << std::endl;
       retVal = true; 
     }
     
     mouseOld = mouseNew;
    
    return retVal;
    
  }
  
  DeltaControlSystem& controlSys;
  AxisVector mouseOld = {0.0,0.0,-0.015,0.0};
  AxisVector mouseNew = {0.0,0.0,-0.015,0.0};
  
};

  }
}