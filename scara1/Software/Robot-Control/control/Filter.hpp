#ifndef CH_NTB_SCARA_Filter_HPP_
#define CH_NTB_SCARA_Filter_HPP_

#include <eeros/types.hpp>
#include <vector>
#include <eeros/control/Block.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/math/Matrix.hpp>
#include <math.h>

namespace scara{
	
	class Filter : public eeros::control::Block{
			
	public:
		Filter();
		virtual ~Filter();
		virtual void run();
		
		virtual eeros::control::Output< eeros::math::Vector4 >& getOut() {
			return out;
		};
		
		virtual eeros::control::Input< eeros::math::Vector4 >& getIn() {
			return in;
		};
			
			
	private:
		
		eeros::control::Output<eeros::math::Vector4> out;
		eeros::control::Input<eeros::math::Vector4> in;
		//eeros::control::Input<eeros::math::Vector4> prev;
		};
};

#endif /* CH_NTB_Filter_HPP_ */