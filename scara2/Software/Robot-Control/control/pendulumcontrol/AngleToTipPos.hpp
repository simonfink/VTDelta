#ifndef CH_NTB_PARALLELSCARA_ANGLETOTIPPOS_HPP_
#define CH_NTB_PARALLELSCARA_ANGLETOTIPPOS_HPP_

#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>
#include "../../types.hpp"
#include "../../constants.hpp"

#include <stdlib.h>
#include <cmath>

namespace parallelscara {

	class AngleToTipPos: public eeros::control::Block {
		
	public:
		AngleToTipPos(double l) : l (l) { }
		virtual ~AngleToTipPos() { }
		
		virtual void run() {
			AxisVector xy_out;
			
			xy_out(0) = xy_robot_in.getSignal().getValue()(0) + l * sin(phi_in.getSignal().getValue()(0));
			xy_out(1) = xy_robot_in.getSignal().getValue()(1) + l * sin(phi_in.getSignal().getValue()(1));
			
			xy_pendulum_out.getSignal().setValue(xy_out);
			xy_pendulum_out.getSignal().setTimestamp(phi_in.getSignal().getTimestamp());
		}
		
		virtual eeros::control::Input<AxisVector>& getIn_phi() {return phi_in;}
		virtual eeros::control::Input<AxisVector>& getIn_xyRobot() {return xy_robot_in;}
		virtual eeros::control::Output<AxisVector>& getOut_xyPendulum() {return xy_pendulum_out;}

	protected:
		double l;
		eeros::control::Input<AxisVector> phi_in;
		eeros::control::Input<AxisVector> xy_robot_in;
		eeros::control::Output<AxisVector> xy_pendulum_out;
	};
};
#endif /* CH_NTB_PARALLELSCARA_ANGLETOTIPPOS_HPP_ */