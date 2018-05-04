#include <eeros/core/Runnable.hpp>
#include <eeros/math/Frame.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/math/CoordinateSystem.hpp>
#include <cmath>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include "../Utils.hpp"

using namespace eeros; 
using namespace eeros::math;

class SavePointsBlockTest {
	public:
		SavePointsBlockTest() : a("a"), b("b"), frame1(a,b), frame2(a,b) { }
		
		int run(const char* filepath) {
			
			std::ofstream file(filepath);
			if (!file.is_open())
				return -2;
		
			Vector3 A, B, C, D, E, F;
			char m = 0;
			Matrix<6,3,double> savePoints;
			Matrix<6,3,double> inputPoints;
			
			inputPoints << 0, 0, 0,
						   1, 1, 1,
						   2, 2, 2,
						   3, 3, 3, 
						   4, 4, 4, 
						   5, 5, 5;
						   
			// write points to variables A, ...
			while(i<6) {
				std::cin >> m;
				switch(m) {
					case 's':
						// save point
						for(int j = 0; j<3; j++)
							savePoints(i,j) = inputPoints(i,j);
						i++;
						break;
					default:
						// nothing to do
						break;
				}
			}
			// write input data in A, B, ...
			for(int j = 0; j<3; j++){
				A = savePoints(0,j);
				B = savePoints(1,j);
				C = savePoints(2,j);
				D = savePoints(3,j);
				E = savePoints(4,j);
				F = savePoints(5,j);
			}
			
			// save points (later frames) into a file
			for(int j = 0; j<3; j++){
				file << A(j);
				file << "\t";
			}
			for(int j = 0; j<3; j++){
				file << B(j);
				file << "\t";
			}
			for(int j = 0; j<3; j++){
				file << C(j);
				file << "\t";
			}
			for(int j = 0; j<3; j++){
				file << D(j);
				file << "\t";
			}
			for(int j = 0; j<3; j++){
				file << E(j);
				file << "\t";
			}
			for(int j = 0; j<3; j++){
				file << F(j);
				file << "\t";
			}
			file << "\n";
			
			file.close();
			return 0;
		}
		
	protected:
 		CoordinateSystem a;
 		CoordinateSystem b;
		Frame frame1;
		Frame frame2; 
		int i = 0;
};

int main(int argc, char* argv[]) {
	SavePointsBlockTest tester;
	
	if (argc == 2) {
		return tester.run(argv[1]);
	}
	else {
		std::cout << "illegal number of arguments" << std::endl;
	}
	return -3;
}
