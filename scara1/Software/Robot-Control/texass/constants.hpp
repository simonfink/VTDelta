#ifndef CH_NTB_SCARA_CONSTANTS_HPP_
#define CH_NTB_SCARA_CONSTANTS_HPP_

#include<cmath>

namespace scara {
	
	// Math constants
	constexpr double pi = 3.14159265359;
	// General constants
	constexpr unsigned int nofAxis = 4;
	
	// *** Electrical and mechanical parameters *** //
	// arms length
	static constexpr double l1 = 0.25;
	static constexpr double l2 = 0.25;
	// gear ratio
	static constexpr double i0 = 100.0;
	static constexpr double i1 = 100.0;
	static constexpr double i2 = -1.5;
	static constexpr double i3 = -16.2022;
	// radToM conversion axis 2
	static constexpr double radToM = 0.025/6.28318530718;
	// inertia
	static constexpr double J0 = 1.696484*pow(10.0, -4.0);
	static constexpr double J1 = 6.747575*pow(10.0, -5.0);
	static constexpr double J2 = 7*pow(10.0, -2.0);           // original: 7*pow(10.0, -5.0)
	static constexpr double J3 = 3*pow(10.0, -5.0);
	// motor constants
	static constexpr double km0 = 0.5;
	static constexpr double km1 = 0.47;
	static constexpr double km2 = 0.47;
	static constexpr double km3 = 0.31;
		
	// Controller parameters
	static constexpr double dt = 0.001;
	
	// *** Tools parameters *** //
	static constexpr double tasterX = 0.00; //-0.002;
	static constexpr double tasterY = 0.00; // 0.002;
	static constexpr double tasterZ = -0.172;
	
	static constexpr double kfx = 0.00002; // [m]
	static constexpr double kfy = 0.00001; // [m]
	
}
#endif /* CH_NTB_SCARA_CONSTANTS_HPP_ */