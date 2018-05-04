#ifndef CH_NTB_SCARA_SCARADIRECTKINEMATIC_HPP_
#define CH_NTB_SCARA_SCARADIRECTKINEMATIC_HPP_

#include <eeros/types.hpp>
#include <vector>
#include <eeros/control/Block1i1o.hpp>
#include <eeros/math/Matrix.hpp>

namespace scara{
	
	class ScaraDirectKinematic : public eeros::control::Block1i1o<eeros::math::Vector4> {
		
	public:
		ScaraDirectKinematic(double l1, double l2);
		virtual ~ScaraDirectKinematic();
		
		virtual void run();
		
	private:
		double l1;
		double l2;
	};
}; // END namespace scara

#endif /* CH_NTB_SCARA_SCARADIRECTKINEMATIC_HPP_ */
