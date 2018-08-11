#pragma once
#include <eeros/sequencer/Condition.hpp>
#include <eeros/logger/Logger.hpp>


namespace eeduro{
  namespace delta{
    
class MissingBlockCondition : public eeros::sequencer::Condition{
public:
  MissingBlockCondition(){ }
  bool validate() {
    return true;
  }
  
};

  }
}