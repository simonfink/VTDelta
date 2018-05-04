#ifndef CH_NTB_SCARA_SCARATRANSPOSEJACOBI_HPP_
#define CH_NTB_SCARA_SCARATRANSPOSEJACOBI_HPP_

#include <eeros/types.hpp>
#include <vector>
#include <eeros/control/Block.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/math/Matrix.hpp>
#include <math.h>

namespace scara{
	
	class ScaraTransposeJacobi : public eeros::control::Block{
	
	public:
		ScaraTransposeJacobi(double l1, double l2);
 		virtual ~ScaraTransposeJacobi();
		virtual void run();
		
		virtual eeros::control::Output< eeros::math::Vector4 >& getOutJacobi() {
			return outJacobi;
		};
		
		virtual eeros::control::Input< eeros::math::Vector4 >& getInForce() {
			return inForce;
		};
		
		virtual eeros::control::Input< eeros::math::Vector4 >& getInJointPos() {
			return inJointPos;
		};
		
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
		
		eeros::control::Output<eeros::math::Vector4> outJacobi;
		eeros::control::Input<eeros::math::Vector4> inForce;
		eeros::control::Input<eeros::math::Vector4> inJointPos;
	};
}; // END namespace scara

#endif /* CH_NTB_SCARATRANSPOSEJACOBI_HPP_ */