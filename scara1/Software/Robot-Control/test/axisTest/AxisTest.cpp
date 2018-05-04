#include "AxisTest.hpp"

#include <iostream>

using namespace scara;
using namespace eeros::control;
using namespace eeros::logger;

namespace scara {
	
	AxisTest::AxisTest() : l('C'), w(std::cout), enc0("q0"), enc1("q1"), enc2("q2"), enc3("q3") {
		l.set(w);
	}
	
	void AxisTest::run() {
		enc0.run();
		enc1.run();
		enc2.run();
		enc3.run();
		
// 		l.info() << "q0 = " << enc0.getOut() << "\t q1 = " << enc1.getOut() << "\t q2 = " << enc2.getOut() << "\t q3 = " << enc3.getOut() << endl;
	}
	
	AxisTest& AxisTest::instance() {
			static AxisTest axisTestInstance;
			return axisTestInstance;
	}
};


int main(int argc, char* argv[]) {
	AxisTest::AxisTest<> tester;
	if (argc == 2) {
		return tester.run(argv[1]);
	}
	else {
		std::cout << "illegal number of arguments" << std::endl;
	}
	return -3;
}