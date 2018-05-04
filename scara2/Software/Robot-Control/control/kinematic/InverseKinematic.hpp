#ifndef CH_NTB_PARALLELSCARA_INVERSEKINEMATIC_HPP_
#define CH_NTB_PARALLELSCARA_INVERSEKINEMATIC_HPP_

#include <eeros/types.hpp>
#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/safety/SafetySystem.hpp>

#include "../../types.hpp"
#include "../../constants.hpp"
#include <cmath>

using namespace eeros;
using namespace eeros::control;
using namespace eeros::math;
using namespace parallelscara;

namespace parallelscara {
	
	class InverseKinematic : public Block {
	public:
		InverseKinematic(double l1, double l2, AxisVector q0_lim, AxisVector q1_lim, double deltaQ_min, double deltaQ_max) : 
			l1(l1), 
			l2(l2), 
			q0_lim(q0_lim), 
			q1_lim(q1_lim), 
			deltaQ_min(deltaQ_min),
			deltaQ_max(deltaQ_max) 
		{
			R_max = l1 * cos(deltaQ_limMin / 2.0) + l2 * cos(deltaQ_limMin / 2.0);
			r_min = l1 * cos(deltaQ_limMax / 2.0) + l2 * cos(deltaQ_limMax / 2.0);	
		}
		virtual ~InverseKinematic() { }
		virtual void run() {
			// Cartesian workspace limitation
			double R2 = inActPosXY.getSignal().getValue()(0) * inActPosXY.getSignal().getValue()(0) + inActPosXY.getSignal().getValue()(1) * inActPosXY.getSignal().getValue()(1);
			double R = sqrt(R2);
			
			AxisVector cartesianCoords, jointCoords;
			
			if(enabled) {

				if(R2 < r_min * r_min){
					cartesianCoords = inActPosXY.getSignal().getValue() * r_min / R;
					outOfRange = 1;
				}
				else if(R2 > R_max * R_max){
					cartesianCoords = inActPosXY.getSignal().getValue() * R_max / R;
					outOfRange = 1;
				}
				else{
					cartesianCoords = inActPosXY.getSignal().getValue();
					outOfRange = 0;
				}
				
				// Compute inverse kinematic
				alpha = acos((l1 * l1 + l2 * l2 - cartesianCoords[0] * cartesianCoords[0] - cartesianCoords[1] * cartesianCoords[1]) / (2.0 * l1 * l2));
				delta_q = pi - alpha; 

				// Check delta Q
				if(delta_q < deltaQ_min || delta_q > deltaQ_max){
					outOfRange = 1;
				}

				beta = atan2(cartesianCoords[1], cartesianCoords[0]);
				gamma = asin((l2 * sin(alpha)) / (sqrt(cartesianCoords[0] * cartesianCoords[0] + cartesianCoords[1] * cartesianCoords[1]))); 
				
				jointCoords[0] = beta - gamma;
				jointCoords[1] = beta + gamma;
				
				// Check q0, q1
				if(jointCoords[0] < q0_lim(0) || jointCoords[0] > q0_lim(1)) {
					outOfRange = 1;
				}
				else if(jointCoords[1] < q1_lim(0) || jointCoords[1] > q1_lim(1)) {
					outOfRange = 1;
				}
			}
			else { // disabled: ref = enc
				jointCoords[0] = inActPosEnc.getSignal().getValue()(0);
				jointCoords[1] = inActPosEnc.getSignal().getValue()(1);
			}
			
			// Write data out
			out.getSignal().setValue(jointCoords);
			out.getSignal().setTimestamp(this->inActPosXY.getSignal().getTimestamp());
			
			oor.getSignal().setValue(outOfRange);
			oor.getSignal().setTimestamp(this->inActPosXY.getSignal().getTimestamp());
		}
		
		virtual void enable() {enabled = true;}
		virtual void disable() {enabled = false;}
		
		virtual Input<AxisVector>& getInActPosXY() {return inActPosXY;}
		virtual Input<AxisVector>& getInActPosEnc() {return inActPosEnc;}
		virtual Output<AxisVector>& getOut() {return out;}
		virtual Output<bool>& getOutOfRange() {return oor;}
		virtual bool isOutOfRange() {return outOfRange;}
		virtual void resetOutOfRange() {outOfRange = false;}
		
	private:
		bool enabled = false;
		bool outOfRange = false;
		double l1, l2, deltaQ_min, deltaQ_max, R_max, r_min;
		double alpha, beta, gamma, delta_q;
		AxisVector q0_lim, q1_lim;
		
	protected:
		Input<AxisVector> inActPosXY;
		Input<AxisVector> inActPosEnc;
		Output<AxisVector> out;
		Output<bool> oor;
	};
}; // END namespace

#endif /* CH_NTB_PARALLELSCARA_INVERSEKINEMATIC_HPP_ */