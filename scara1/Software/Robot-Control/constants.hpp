#ifndef CH_NTB_SCARA_CONSTANTS_HPP_
#define CH_NTB_SCARA_CONSTANTS_HPP_

#include<cmath>
#include<eeros/math/Matrix.hpp>

namespace scara {
	
	// Math constants
	constexpr double pi = 3.14159265359;
	// General constants
	constexpr unsigned int nofAxis = 4;
	
	// arms length
	static constexpr double l1 = 0.25;
	static constexpr double l2 = 0.25;
	// gear ratio
	static constexpr double i0 = 100.0;
	static constexpr double i1 = 100.0;
	static constexpr double i2 = -1.5;
	static constexpr double i3 = -16.2022;
	
	static constexpr double ifx = 21.15*(-1);
	static constexpr double ify = 21.15*(1);
	static constexpr double ifz = 7.458*(-1);
	static constexpr double imz = 2.5/10;
	
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
	
// 	static constexpr double kfx = 0.00002; // [m]
// 	static constexpr double kfy = 0.00001; // [m]
	
	static eeros::math::Vector2 Safety_q0_limit  = {-0.1, 0.1}; 
	static eeros::math::Vector2 Safety_q1_limit  = {-0.1, 0.1}; 
	
	static constexpr double vmax = 2.0; // [rad/s]
	
	// homing
	static constexpr double home_speed0 = -0.1; // -0.2;
	static constexpr double home_speed1 = -0.1; // -0.2;
	static constexpr double home_speed2 =  0.2; //  0.4;
	static constexpr double home_speed3 =  0.4; //  0.6;
	
	static constexpr double home_check_angle0 = 2.6; // inputs of muxPosEnc
	static constexpr double home_check_angle1 = 2.4; // inputs of muxPosEnc
	static constexpr double home_check_angle2 = 0.1; // inputs of muxPosEnc
	static constexpr double home_check_angle3 = 0.0; // inputs of muxPosEnc
	
	static constexpr double home_check_error  = 0.2;
	
	// ready
	static eeros::math::Vector4 readyPos_joints = {0.78, -1.57,  0.105,  1.57};
	static eeros::math::Vector4 readyPos_cartes = {0.35, -0.05, -0.105, -0.78};
	
	
	
}
#endif /* CH_NTB_SCARA_CONSTANTS_HPP_ */