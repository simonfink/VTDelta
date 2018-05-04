#ifndef CH_NTB_SCARA_DynModell_HPP_
#define CH_NTB_SCARA_DynModell_HPP_

#include <eeros/types.hpp>
#include <vector>
#include <eeros/control/Block.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/math/Matrix.hpp>
#include <math.h>

namespace scara{
	
	class DynModell : public eeros::control::Block{
	
	public:
		DynModell(double m,double c);
 		virtual ~DynModell();
		virtual void run();
		
		virtual eeros::control::Output< eeros::math::Vector4 >& getOut() {
			return out;
		};
		
		virtual eeros::control::Input< eeros::math::Vector4 >& getInForce() {
			return inForce;
		};
		
		virtual eeros::control::Input< eeros::math::Vector4 >& getInCartPos() {
			return inCartPos;
		};
		
		virtual eeros::control::Input< eeros::math::Vector4 >& getInCartVel() {
			return inCartVel;
		};
		
		eeros::math::Vector4 cartesianCoords;
	private:
		double m;
		double c;
		
		eeros::control::Output<eeros::math::Vector4> out;
		eeros::control::Input<eeros::math::Vector4> inForce;
		eeros::control::Input<eeros::math::Vector4> inCartPos;
		eeros::control::Input<eeros::math::Vector4> inCartVel;
	};
}; // END namespace scara

#endif /* CH_NTB_DynModell_HPP_ */