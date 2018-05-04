#ifndef CH_NTB_PARALLELSCARA_DIRECTKINEMATIC_HPP_
#define CH_NTB_PARALLELSCARA_DIRECTKINEMATIC_HPP_

#include <eeros/types.hpp>
#include <eeros/control/Block1i1o.hpp>
#include <eeros/math/Matrix.hpp>
#include "../../types.hpp"

using namespace eeros::math;
using namespace eeros::control;

namespace parallelscara{
	
	class DirectKinematic : public Block1i1o<AxisVector> {
	public:
		DirectKinematic(double l1, double l2) : l1(l1), l2(l2) { }
		virtual ~DirectKinematic() { }
		
		virtual void run() {
			AxisVector jointCoords = in.getSignal().getValue();
			AxisVector cartesianCoords;

			cartesianCoords[0] = l1 * cos(jointCoords[0]) + l2 * cos(jointCoords[1]); 
			cartesianCoords[1] = l1 * sin(jointCoords[0]) + l2 * sin(jointCoords[1]);  
			
			out.getSignal().setValue(cartesianCoords);
			out.getSignal().setTimestamp(in.getSignal().getTimestamp());
		}
	
	private:
		double l1, l2;
	};
}; // END namespace

#endif /* CH_NTB_PARALLELSCARA_DIRECTKINEMATIC_HPP_ */
