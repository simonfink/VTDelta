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

class RefSysBlockTest {
	public:
		RefSysBlockTest() : a("a"), b("b"), frame1(a,b), frame2(a,b) { }
		
		int run(const char* filepath) {
			std::ifstream file(filepath);
			if (!file.is_open())
				return -2;
				
			int line = 0;
			int error = 0;
			uint64_t timestamp = 0;
			
			Vector3 A, B, C, D, E, F;
			Vector3 AB, AC, zABC, DE, k, l, D_1, E_1, F_1;
			double modzABC, modl, aABC, bABC, cABC, dABC, k_D, k_E, k_F;
			Matrix<3, 2, double> aMatrix; Vector3 bMatrix; Vector2 lambda; 
			Matrix<2, 3, double> aMatrixT; 
			Matrix<2, 2, double> aABCmatrix, bABCmatrix, cABCmatrix;
			Vector3 e1, e2, e3, O; Matrix<4,4,double> Tr;

			Matrix<1,16,double> out;
			Matrix<1,16,double> calcOut;
			
			while (!file.eof()) {
				line++;
				
				// read input data
				for(int i = 0; i<34; i++){
					if(i < 3)
						file >> A(i);		// point A
					else if(i < 6)
						file >> B(i-3);		// point B
					else if(i < 9)
						file >> C(i-6);		// point C
					else if(i < 12)
						file >> D(i-9);		// point D
					else if(i < 15)
						file >> E(i-12);	// point E
					else if(i < 18)
						file >> F(i-15);	// point F
					else
						file >> out(i-18);	// out: e1, e2, e3, O
				}
				
				// calculate unity vectors
				AB = B - A;
				AC = C - A;
				zABC = Matrix<3,1>::crossProduct(AB, AC);
				modzABC = sqrt(zABC(0) * zABC(0) + zABC(1) * zABC(1) + zABC(2) * zABC(2)); 
				e3 = zABC/modzABC;
				DE = E - D;
				k = (DE(0)*e3(0)+DE(1)*e3(1)+DE(2)*e3(2))*e3;
				l =  DE - k;
				modl = sqrt(l(0)*l(0)+l(1)*l(1)+l(2)*l(2));
				e2 = (l/modl);
				e1 = Matrix<3,1>::crossProduct(e2, e3);
				
				// calculate plane coefficients
				aABCmatrix << B(1)-A(1), B(2)-A(2), C(1)-A(1), C(2)-A(2);
				bABCmatrix << B(0)-A(0), B(2)-A(2), C(0)-A(0), C(2)-A(2);
				cABCmatrix << B(0)-A(0), B(1)-A(1), C(0)-A(0), C(1)-A(1);
				aABC =  aABCmatrix.det();
				bABC = -bABCmatrix.det();
				cABC =  cABCmatrix.det();   
				dABC =  aABC*(-A(0)) + bABC*(-A(1)) + cABC*(-A(2));
				
				// find projections of points (D, E, F) on the plane ABC
				k_D = -(D(0)*aABC+D(1)*bABC+D(2)*cABC+dABC)/(e3(0)*aABC+e3(1)*bABC+e3(2)*cABC);
				k_E = -(E(0)*aABC+E(1)*bABC+E(2)*cABC+dABC)/(e3(0)*aABC+e3(1)*bABC+e3(2)*cABC);
				k_F = -(F(0)*aABC+F(1)*bABC+F(2)*cABC+dABC)/(e3(0)*aABC+e3(1)*bABC+e3(2)*cABC);
				D_1 << D(0)+k_D*e3(0), D(1)+k_D*e3(1), D(2)+k_D*e3(2);
				E_1 << E(0)+k_E*e3(0), E(1)+k_E*e3(1), E(2)+k_E*e3(2);
				F_1 << F(0)+k_F*e3(0), F(1)+k_F*e3(1), F(2)+k_F*e3(2);

				// find origin
				aMatrix << e1(0), -e2(0), e1(1), -e2(1), e1(2), -e2(2);
				bMatrix << D_1(0)-F_1(0), D_1(1)-F_1(1), D_1(2)-F_1(2);
					
				aMatrixT = aMatrix.transpose(); 
				lambda = !(aMatrixT*aMatrix) * (aMatrixT*bMatrix); 
				O << F_1(0)+lambda(0)*e1(0), F_1(1)+lambda(0)*e1(1), F_1(2)+lambda(0)*e1(2);

				// find transformation matrix for frame1 
				for(int i = 0; i<16; i++){
					if(i<3) 				Tr(i) = e1(i);
					else if(i>=4  && i<7) 	Tr(i) = e2(i-4);
					else if(i>=8  && i<11)	Tr(i) = e3(i-8);
					else if(i>=12 && i<15)	Tr(i) = O(i-12);
					else if(i == 15)		Tr(i) = 1;
					else					Tr(i) = 0;
				} 
				frame1.set(Tr);
				
				// compute output
				for(int i = 0;i<16;i++)
					calcOut(i) = frame1.get()(i); 
			
				if (file.eof()) break;

				for(int i = 0; i<16; i++){
					if(!Utils::compareApprox(out(i), calcOut(i), 0.001)) {
						error++;
						std::cout << "line: " << line << "; index: " << i << "; expecting  out: " << out(i) << "; calculated: " << calcOut(i) << std::endl;
					}
				}
			}
			file.close();
			return error;
		}
		
	protected:
 		CoordinateSystem a;
 		CoordinateSystem b;
		Frame frame1;
		Frame frame2; 
};

int main(int argc, char* argv[]) {
	RefSysBlockTest tester;
	
	if (argc == 2) {
		return tester.run(argv[1]);
	}
	else {
		std::cout << "illegal number of arguments" << std::endl;
	}
	return -3;
}
