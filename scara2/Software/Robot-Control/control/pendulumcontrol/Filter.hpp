#ifndef CH_NTB_PARALLELSCARA_FILTER_HPP_
#define CH_NTB_PARALLELSCARA_FILTER_HPP_

#include <eeros/control/Block1i1o.hpp>
#include "../../types.hpp"
#include "../../constants.hpp"

#include <stdlib.h>
#include <cmath>

using namespace eeros::control;
using namespace eeros::math;

namespace parallelscara {
	class Filter: public Block1i1o<Vector2> {
		
	public:
		Filter(double k) : k(k) { }
		virtual ~Filter() { }
		virtual void run() {
			AxisVector filtered; 
			
			filtered = prev * (1.0 - k) + in.getSignal().getValue() * k;
			prev = filtered;
			
			out.getSignal().setValue(filtered);
			out.getSignal().setTimestamp(in.getSignal().getTimestamp());
		}
	
	protected:
		double k;
		AxisVector prev;
	};
};
#endif /* CH_NTB_PARALLELSCARA_FILTER_HPP_ */