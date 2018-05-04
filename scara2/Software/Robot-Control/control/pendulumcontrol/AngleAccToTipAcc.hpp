#ifndef CH_NTB_PARALLELSCARA_ANGLEACCTOTIPACC_HPP_
#define CH_NTB_PARALLELSCARA_ANGLEACCTOTIPACC_HPP_

#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>
#include "../../types.hpp"

#include <iostream>
#include <stdlib.h>
#include <cmath>

#include "../../constants.hpp"

namespace parallelscara {

	class AngleAccToTipAcc: public eeros::control::Block {
	
	public:
		AngleAccToTipAcc(double l, double cog, double J, double m) : l (l), cog(cog), J(J), m(m) { }
		virtual ~AngleAccToTipAcc() { }
		
		virtual void run() {
			AxisVector xy_out_temp;
			
			xy_out_temp(0) = g * tan(phi_in.getSignal().getValue()(0)) - ((cog * cog+r_pendulum * r_pendulum) * dd_phi_in.getSignal().getValue()(0))/(cog * cos(phi_in.getSignal().getValue()(0)));
			xy_out_temp(1) = g * tan(phi_in.getSignal().getValue()(1)) - ((cog * cog+r_pendulum * r_pendulum) * dd_phi_in.getSignal().getValue()(1))/(cog * cos(phi_in.getSignal().getValue()(1)));
			
			xy_out.getSignal().setValue(xy_out_temp);
			xy_out.getSignal().setTimestamp(dd_phi_in.getSignal().getTimestamp());
		}

		virtual eeros::control::Input<AxisVector>& getIn_ddphi() {return dd_phi_in;}
		virtual eeros::control::Input<AxisVector>& getIn_phi() {return phi_in;}
		virtual eeros::control::Output<AxisVector>& getOut() {return xy_out;}

	protected:
		double l, cog, J, m;
		eeros::control::Input<AxisVector> dd_phi_in;
		eeros::control::Input<AxisVector> phi_in;
		eeros::control::Output<AxisVector> xy_out;
	};
};
#endif /* CH_NTB_PARALLELSCARA_ANGLEACCTOTIPACC_HPP_ */