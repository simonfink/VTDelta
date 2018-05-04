#ifndef CH_NTB_PARALLELSCARA_HALLSENSORSDATAACQUISITION_HPP_
#define CH_NTB_PARALLELSCARA_HALLSENSORSDATAACQUISITION_HPP_

#include <eeros/types.hpp>
#include <vector>
#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/math/Matrix.hpp>
#include "../../types.hpp"

using namespace eeros::math;
using namespace eeros::control;

namespace parallelscara{
	
	class HallSensorsDataAcquisition : public Block {
		
	public:
		HallSensorsDataAcquisition() {sensorsInBit.zero();}
		virtual ~HallSensorsDataAcquisition() { }
		
		virtual void run() {
			Vector4 sensorIn = hallIn.getSignal().getValue();
			AxisVector phi_xy;
		
			sensorsInBit = (sensorIn * hallBitScale) - hallMeanOffset;
			
			double sum = sensorsInBit(0) + sensorsInBit(1) + sensorsInBit(2) + sensorsInBit(3);
			
			if(sum > minSensorOutWithBar) barIsOn = true;	// long bar
			else if(sum < -minSensorOutWithBar) barIsOn = true;	// short bar
			else barIsOn = false; 
			
			phi_xy(0) = (sensorsInBit(0) - sensorsInBit(2)) * hallCalibrationScale(0) + hallCalibrationOffset(0) + 0.010;
			phi_xy(1) = (sensorsInBit(1) - sensorsInBit(3)) * hallCalibrationScale(1) + hallCalibrationOffset(1) - 0.024;
			// 0.010 and -0.024 = measured data, balancing at one fix point - correction factor
		
			if(barIsOn) {
				barAngleOut.getSignal().setValue(phi_xy);
				barAngleOut.getSignal().setTimestamp(hallIn.getSignal().getTimestamp());
				barAngleOutX.getSignal().setValue(phi_xy(0));
				barAngleOutY.getSignal().setValue(phi_xy(1));
			} else {
				barAngleOut.getSignal().setValue({0.0, 0.0});
				barAngleOut.getSignal().setTimestamp(hallIn.getSignal().getTimestamp());
			}
			
			barOn.getSignal().setValue(isBarOn());
			barOn.getSignal().setTimestamp(hallIn.getSignal().getTimestamp());
		}
		virtual bool isBarOn() {return barIsOn;}
		virtual Input<Vector4>& getIn() {return hallIn;}
		virtual Output<AxisVector>& getOut() {return barAngleOut;}		
		virtual Output<double>& getOutX() {return barAngleOutX;}
		virtual Output<double>& getOutY() {return barAngleOutY;}		
		virtual Output<bool>& getBarOn() {return barOn;}

	private:
		Vector4 sensorsInBit; 
		Matrix<4,1,int> sensorOffset;
		bool barIsOn = false;
		
		
	protected:
		Input<Vector4> hallIn;
		Output<AxisVector> barAngleOut;
		Output<double> barAngleOutX, barAngleOutY;
		Output<bool> barOn;
	};
}; // END namespace

#endif /* CH_NTB_PARALLELSCARA_HALLSENSORSDATAACQUISITION_HPP_ */
