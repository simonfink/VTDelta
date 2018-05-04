#ifndef CH_NTB_SCARA_SCARAINVERSEKINEMATIC_HPP_
#define CH_NTB_SCARA_SCARAINVERSEKINEMATIC_HPP_

#include <eeros/types.hpp>
#include <vector>
#include <eeros/control/Block1i1o.hpp>
#include <eeros/math/Matrix.hpp>
#include <math.h>

namespace scara{
	
	class ScaraInverseKinematic : public eeros::control::Block1i1o<eeros::math::Vector4> {
	
	public:
		ScaraInverseKinematic(double l1, double l2);
		virtual ~ScaraInverseKinematic();
		virtual void run();
		
		eeros::math::Vector4 cartesianCoords;
	private:
		double l1, l2;
		double r_min, R_max;
		static constexpr double q1_init = 2.60; // q1  initialization angle 2.60743134- safety properties
		static constexpr double alpha = M_PI-q1_init;
		static constexpr double z_min = -0.40;
		static constexpr double z_max = -0.10;
		static constexpr double alpha_min = -2*M_PI;
		static constexpr double alpha_max = 0;
	};
}; // END namespace scara

#endif /* CH_NTB_SCARA_SCARAINVERSEKINEMATIC_HPP_ */