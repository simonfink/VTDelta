#ifndef CH_NTB_SCARA_CARTESIANWORKSPACELIMITATION_HPP_
#define CH_NTB_SCARA_CARTESIANWORKSPACELIMITATION_HPP_

#include <eeros/control/Block1i1o.hpp>
#include <mutex>
#include "../../types.hpp"

namespace scara {
		class CartesianWorkspaceLimitation: public eeros::control::Block1i1o<eeros::math::Vector4> {
			
		public:
			CartesianWorkspaceLimitation();
			
			virtual void enable();
			virtual void disable();
			virtual void setParameters(eeros::math::Matrix<13,1,double> limit_par, AxisVector toolOffset);
			virtual void run();
			
			bool in_workspace = 0;
			AxisVector toolOffset; // TODO shift to protected
			AxisVector input_robot, input, prev_out; // TODO shift to protected
			
		protected:
			bool enabled = 0;
			
			double a_ab, b_ab, c_ab, a_bc, b_bc, c_bc, a_cd, b_cd, c_cd, a_da, b_da, c_da, z;
			
		};
	}
#endif /* CH_NTB_SCARA_CARTESIANWORKSPACELIMITATION_HPP_ */