#ifndef CH_NTB_PARALLELSCARA_TIPACCTOANGLE_HPP_
#define CH_NTB_PARALLELSCARA_TIPACCTOANGLE_HPP_

#include <eeros/control/Block1i1o.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>
#include "../../types.hpp"
#include "../../constants.hpp"

#include <stdlib.h>
#include <cmath>

namespace parallelscara {

	class TipAccToAngle: public eeros::control::Block1i1o<AxisVector> {
		
	public:
		TipAccToAngle() { }
		virtual ~TipAccToAngle() { }
		
		virtual void run() { 
			AxisVector phi;
			
			phi(0) = atan2(in.getSignal().getValue()(0), g);
			phi(1) = atan2(in.getSignal().getValue()(1), g);
			
			out.getSignal().setValue(phi);
			out.getSignal().setTimestamp(in.getSignal().getTimestamp());
		}
	};
};
#endif /* CH_NTB_PARALLELSCARA_TIPACCTOANGLE_HPP_ */