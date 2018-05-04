#ifndef CH_NTB_PARALLELSCARA_HALLPHIROTATIONMATRIX_HPP_
#define CH_NTB_PARALLELSCARA_HALLPHIROTATIONMATRIX_HPP_

#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>
#include "../../types.hpp"

#include <cmath>

using namespace eeros::math;
using namespace eeros::control;

namespace parallelscara {

		class HallPhiRotationMatrix: public Block {
			
		public:
			HallPhiRotationMatrix() { }
			virtual ~HallPhiRotationMatrix() { }
			
			virtual void run() {
				AxisVector out;
				AxisVector encPos = encPosIn.getSignal().getValue();
				AxisVector phi = phiIn.getSignal().getValue();
				
				out(0) = cos(encPos(0)) * phi(0) - sin(encPos(0)) * phi(1);
				out(1) = sin(encPos(0)) * phi(0) + cos(encPos(0)) * phi(1);
				
				phiOut.getSignal().setValue(out);
				phiOut.getSignal().setTimestamp(phiIn.getSignal().getTimestamp());
			}

			virtual Input<AxisVector>& getInPhi() {return phiIn;}
			virtual Input<AxisVector>& getInEncPos() {return encPosIn;}
			virtual Output<AxisVector>& getOutPhi() {return phiOut;}

		protected:
			Input<AxisVector> phiIn, encPosIn;
			Output<AxisVector> phiOut;
		};
	};
#endif /* CH_NTB_PARALLELSCARA_HALLPHIROTATIONMATRIX_HPP_ */