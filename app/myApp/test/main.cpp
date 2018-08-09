#include <iostream>
#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
 
int main() {
  using namespace eeros::logger;
 
  StreamLogWriter w(std::cout);
  Logger log;
  log.set(w);
 
  log.info() << "Hello, EEROS";
 
  return 0;
}
