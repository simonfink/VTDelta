#ifndef CH_NTB_PARALLELSCARA_CONSTANTS_HPP_
#define CH_NTB_PARALLELSCARA_CONSTANTS_HPP_

#include<cmath>
#include<eeros/math/Matrix.hpp>

namespace parallelscara {
	
	// Math
	static constexpr double pi = 3.14159265359;                         // Pi
	static constexpr double g = 9.80665;                                // Gravity acc    [m/s²]
	
	// Robot
	static constexpr unsigned int nofAxis = 2;                          // No. of axis
	static constexpr double l1 = 0.25;                                  // Arm 1 length   [m]
	static constexpr double l2 = 0.25;                                  // Arm 2 length   [m]
	static constexpr double i = 6.0 * 72.0 / 14.0;                      // Gear ratio
	static constexpr double Jmot = 137.0*pow(10.0, -3.0)*pow(10.0, -4.0); // Motor inertia  [kgm²]
	static constexpr double J = Jmot * i * i;                           // Motor inertia gear side [kgm²]
	static constexpr double km = 60.3*pow(10.0, -3.0);                  // Motor constant [Nm/A]
	
	// Pendulum 300
	static constexpr double l_pendulum = 0.324;                     // length pendulum [m]
	static constexpr double J_pendulum = 0.00038;                     // inertia pendulum [kgm²]
	static constexpr double m_pendulum = 0.037;                       // mass pendulum [kg]
	static constexpr double r_pendulum = sqrt(J_pendulum/m_pendulum); // [m] Silvan: sqrt(1.0/12.0*l_pendulum*l_pendulum)
	static constexpr double cog_pendulum = 0.08;                      // centre of gravity [m]
	
	// Pendulum 76 
	static constexpr double l_pendulum_s = 0.1;                             // length pendulum2 [m]
// 	static constexpr double J_pendulum_s = 0.00038;                         // inertia pendulum [kgm²]
// 	static constexpr double m_pendulum_s = 0.037;                           // mass pendulum [kg]
// 	static constexpr double r_pendulum_s = sqrt(J_pendulum_s/m_pendulum_s); // [m]
// 	static constexpr double cog_pendulum_s = 0.08;                          // centre of gravity [m]
	
	// Robot Controller 
	static constexpr double S = 2.0;
	static constexpr double fctrl_robot = 1000.0; // 500.0;                 // (Stefan: 400)
	static constexpr double dt = 1.0 / fctrl_robot;                         // Sample time    [s]
	static constexpr double D_robot = 1.5; //1.0;                           // Damping factor
	static constexpr double f0_robot = fctrl_robot/(4.0*pi*D_robot*S);      // (Stefan: 28.43) 
	static constexpr double omega_robot = 85.0; //75.0; //2.0 * pi * f0_robot;      // Stefan /2.0 for balancing ; original Claudia = 75.0
// 	static constexpr double omega_robot = ((1/dt)/20)*2*pi;                 // Eigenfrequency [rad/s]
	static eeros::math::Vector2 kp_robCtrl = omega_robot / (2.0 * D_robot); // Pos ctrl gain    // 27.667 (Stefan: 51.0)
	static eeros::math::Vector2 kv_robCtrl = 2.0 * D_robot * omega_robot;   // Vel ctrl gain    // 249    (Stefan: 99.9)
	
	// Pendulum controller
	static constexpr double D_tip = 0.7;                                    // Damping     
	static constexpr double f_tip = 5.0; // 5.6;                            // Stefan = 7.7
	static constexpr double f0_tip = f_tip / (4.0 * pi * D_tip * S);                                   
	static constexpr double omega_tip = 2.0 * pi * f0_tip;                  // Eigenfrequency tip controller [rad/s]   // 2.0; Silvan = 2.75;
	static eeros::math::Vector2 kp_tipCtrl = omega_tip * omega_tip;         // P gain tip controller (Stefan / 5)
	static eeros::math::Vector2 kv_tipCtrl = 2.0 * D_tip * omega_tip;       // D gain tip controller (Stefan / 5)
	
	static constexpr double D_phi = 0.7;                                    // Damping
	static constexpr double f_phi = 20.0; // 25.0;                          // Stefan = 42.0
	static constexpr double f0_phi = f_phi / (4.0 * pi * D_phi * S);                                   
	static constexpr double omega_phi = 2.0 * pi * f0_phi;                  // Eigenfrequency tip controller [rad/s]  // 9.0; Silvan = 15.0;  
	static eeros::math::Vector2 kp_phiCtrl = omega_phi * omega_phi * 0.5;   // P gain angle controller (Stefan / 4)  
	static eeros::math::Vector2 kv_phiCtrl = 2.0 * D_phi * omega_phi * 0.5; // D gain angle controller (Stefan / 4)
	
	// Angle observer
	static constexpr double omega_obs = 7.5;                            	// omega
	static constexpr double D_obs = 1.0;                                	// D
	static eeros::math::Vector2 kp_obs = omega_obs * omega_obs;				// P gain observer 
	static eeros::math::Vector2 kv_obs = 2.0 * D_obs * omega_obs;			// D gain observer 
	
	// Path Planner 
	static eeros::math::Vector2 velMax = 1.0;                       	    // Max velocity   [rad/s]
	static eeros::math::Vector2 accMax = 2.0;                       	    // Max accel      [rad/s²]
	static eeros::math::Vector2 decMax = -2.0;                          	// Max accel      [rad/s²]
	
// 	// DACs offsets
// 	static constexpr double dacResolution = 65535.0;
// 	static constexpr double dac0min = -9.60;
// 	static constexpr double dac0max =  9.69;
// 	static constexpr double dac0lsbs = (dac0max - dac0min) / (37000.0 - 1000.0);
// 	static constexpr double x0min = 1000.0 - (dac0min + 10.0) / dac0lsbs;
// 	static constexpr double x0max = 1000.0 - (dac0min - 10.0) / dac0lsbs;
// 	static constexpr double m0 = (x0max - x0min) / (2.0 * 15.0);
// 	static constexpr double dac0Scale  = 1.0 / m0;
// 	static constexpr double dac0Offset = - (15.0 + x0min / m0);
// 	
// 	static constexpr double dac1min = -9.52;
// 	static constexpr double dac1max =  9.94;
// 	static constexpr double dac1lsbs = (dac1max - dac1min) / (37000.0 - 1000.0);
// 	static constexpr double x1min = 1000.0 - (dac1min + 10.0) / dac1lsbs;
// 	static constexpr double x1max = 1000.0 - (dac1min - 10.0) / dac1lsbs;
// 	static constexpr double m1 = (x1max - x1min) / (2.0 * 15.0);
// 	static constexpr double dac1Scale  = 1.0 / m1;
// 	static constexpr double dac1Offset = - (15.0 + x1min / m1);
// 	
// 	// Encoder scale factors
// 	static constexpr double enc0Scale =        6.28318530718 / (4.0 * 1024.0);
//	static constexpr double enc1Scale = (-1.0) * 6.28318530718 / (4.0 * 1024.0);
	
	// Hall sensors calibration scale and offset
	static constexpr double hallBitScale = 4095.0;   
	static eeros::math::Vector4 hallMeanOffset = {2074, 2059, 2080, 2062}; // {2077, 2059, 2080, 2062}; 
// 	static eeros::math::Vector2 hallCalibrationScale  = { 0.00175, 0.00175};   
// 	static eeros::math::Vector2 hallCalibrationOffset = {-0.01925, 0.014  }; 
	static eeros::math::Vector2 hallCalibrationScale  = { 0.00175*4.0/3.0,       0.00175*4.0/3.0    }; // 2 magnets
	static eeros::math::Vector2 hallCalibrationOffset = {-0.01925*4.0/3.0-0.007, 0.014*4.0/3.0+0.005}; // 2 magnets	
	static constexpr double minSensorOutWithBar = 840.0; // 2 magnets + hole for pendulum
	
	// Kinematic limits
	static constexpr double deltaQ_limMin =  35.0*pi/180.0;						// 30*pi/180;    
	static constexpr double deltaQ_limMax = 145.0*pi/180.0;						//150*pi/180;   
	static eeros::math::Vector2 q0_limit  = {-45.0*pi/180.0, 195.0*pi/180.0};			//{-45*pi/180, 195*pi/180};
	static eeros::math::Vector2 q1_limit  = {-15.0*pi/180.0, 225.0*pi/180.0};			//{-15*pi/180, 225*pi/180}; 
// 	static eeros::math::Vector2 Safety_q0_limit  = {q0_limit(0)+0.01, q0_limit(1)+0.01};		//{-45*pi/180, 195*pi/180};
// 	static eeros::math::Vector2 Safety_q1_limit  = {q1_limit(0)+0.01, q1_limit(1)+0.01};		//{-15*pi/180, 225*pi/180}; 
	static eeros::math::Vector2 posLimitLower  = {-45.0*pi/180.0, -15.0*pi/180.0};			// lower limit for position;
	static eeros::math::Vector2 safPosLimitLower  = {posLimitLower(0)+0.01, posLimitLower(1)+0.01};	// add small margin;
	static eeros::math::Vector2 posLimitUpper  = {195.0*pi/180.0, 225.0*pi/180.0};			// upper limit for position;
	static eeros::math::Vector2 safPosLimitUpper  = {posLimitUpper(0)+0.01, posLimitUpper(1)+0.01};	// add small margin;
	static constexpr double vmax = 5.0; //24.44; 								// max velo [rad/s]
	static eeros::math::Vector2 velLimitLower  = {-vmax, -vmax};					// lower limit for velocity;
	static eeros::math::Vector2 velLimitUpper  = {vmax, vmax};					// upper limit for velocity;
	
	// Constant position data
	static constexpr double maxTorque_mechLimit = 1.0; //1.4;                          // homing: max torque, at mechanical limit
	static eeros::math::Vector2 angles_mechLimit  = {-46.0*pi/180.0, -16.0*pi/180.0};   // homing: angles at mechanical limit
	static eeros::math::Vector2 ready_pos = { 0.0,  0.3};                       // ready : ready position
	
	static constexpr double a_ellipse = 0.35;
	static constexpr  double b_ellipse = 0.10;
	
	// Velocity saturation
	static constexpr double amax = 480.0; // max acceleration [rad/s²]
	static constexpr double BrakingDistance = (vmax*vmax)/(2.0*amax); // [rad]
	// Mot0
	static constexpr double upperDangerzone0 = 195.0*pi/180.0 - BrakingDistance;  // q0_limit[1] - BrakingDistance;
	static constexpr double underDangerzone0 = -45.0*pi/180.0 + BrakingDistance;  // q0_limit[0] + BrakingDistance;
	// Mot1
	static constexpr double upperDangerzone1 = 225.0*pi/180.0 - BrakingDistance;  // q1_limit(1) - BrakingDistance;
	static constexpr double underDangerzone1 = BrakingDistance + -15.0*pi/180.0;  // BrakingDistance + q1_limit(0);
	
	// Trace length for debugging
	static constexpr double traceLen = 4096;

	
}

#endif /* CH_NTB_PARALLELSCARA_CONSTANTS_HPP_ */