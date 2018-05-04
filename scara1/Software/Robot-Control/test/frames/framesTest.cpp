#include <eeros/core/Runnable.hpp>
#include <eeros/math/Frame.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/math/CoordinateSystem.hpp>
#include "../Utils.hpp"

#include <iostream>
#include <fstream>

using namespace eeros; 
using namespace eeros::math;

template <typename T = double>
class FrameBlockTest {
	public:
		FrameBlockTest() : a("a"), b("b"), c("c"), frame1(a,b), frame2(b,c) {
		}
		
		int run(const char* filepath) {
			std::ifstream file(filepath);
			if (!file.is_open()) return -2;
			
			int line = 0;
			int error = 0;
			uint64_t timestamp = 0;
			Matrix<4,4,double> in;
			Matrix<4,4,double> out;
			Matrix<4,4,double> calcOut;
			
			Matrix<4, 4, double> q; 
			q << cos(M_PI/3), -sin(M_PI/3), 0.0, 0.2,
				 sin(M_PI/3),  cos(M_PI/3), 0.0, 0.1,
						0.0,		0.0, 	1.0, 0.0,
						0.0,		0.0, 	0.0, 1.0;
			frame1.set(q);

			while (!file.eof()) {
				line++;
				
				// read input data
				for(int i = 0; i<32; i++){
					if(i < 16)
						file >> in(i);
					else
						file >> out(i-16);
				}
				frame2.set(in);
				
				// compute output
				calcOut = frame1.get()*frame2.get();
				
				if (file.eof()) break;

				for(int i = 0; i<16; i++){
					if(!Utils::compareApprox(out(i), calcOut(i), 0.001)) {
						error++;
						std::cout << "line " << line << "; index " << i << "; input " << in(i) << "; expecting  output" << out(i) << "; calculated " << calcOut(i) << std::endl;
					}
				}
			}
			file.close();
			return error;
		}
		
	protected:
		CoordinateSystem a;
		CoordinateSystem b;
		CoordinateSystem c;
		Frame frame1;
		Frame frame2;
};

int main(int argc, char* argv[]) {
	FrameBlockTest<> tester;
	if (argc == 2) {
		return tester.run(argv[1]);
	}
	else {
		std::cout << "illegal number of arguments" << std::endl;
	}
	return -3;
}