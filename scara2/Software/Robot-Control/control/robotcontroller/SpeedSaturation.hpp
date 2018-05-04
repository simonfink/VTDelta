#ifndef CH_NTB_PARALLELSCARA_SPEEDSATURATION_HPP_
#define CH_NTB_PARALLELSCARA_SPEEDSATURATION_HPP_

#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>
#include "../../types.hpp"

#include <stdlib.h>
#include <cmath>

namespace parallelscara {

	class SpeedSaturation: public eeros::control::Block {
	
	public:
		SpeedSaturation() : enabled(false) { 
			veloSaturationTmp.zero();
			posTmp.zero();
			veloDesTmp.zero();
		}

		virtual ~SpeedSaturation() { }
		
		virtual void run() {
			posTmp = posActualIn.getSignal().getValue();
			veloDesTmp = veloDesIn.getSignal().getValue();
			
			if(enabled) {
				for ( int i = 0; i < 2; i++ ) {
				
					if (i == 0) {  // Mot0
						if (posTmp(i) < q0_limit(0) || posTmp(i) > q0_limit(1)) {
							veloSaturationTmp(i) = 0.0;
							veloDesTmp(i) = veloSaturationTmp(i);
						}
						else if (veloDesTmp(i) < 0.0 ) { // negativ velocity
							if (posTmp(i) >= upperDangerzone0) {  // in oberer Gefahrenzohne
								veloSaturationTmp(i) = -sqrt(2.0 * -amax * (posTmp(i) - q0_limit(1))); 
							}
							else {
								veloSaturationTmp(i) = -vmax;
							}
							
							if (veloDesTmp(i) < veloSaturationTmp(i)) { // max negativ Velocity
								veloDesTmp(i) = veloSaturationTmp(i);
							}
						}
						else {
							if (posTmp(i) <= underDangerzone0) {  // in unterer Gefahrenzohne
								veloSaturationTmp(i) = sqrt(2.0 * amax * (posTmp(i) - q0_limit(0))); 
							}
							else {
								veloSaturationTmp(i) = vmax;
							}
							
							if (veloDesTmp(i) > veloSaturationTmp(i)) { // max Velocity
								veloDesTmp(i) = veloSaturationTmp(i);
							}
						}
					}
					else { // Mot1
						if (posTmp(i) < q1_limit(0) || posTmp(i) > q1_limit(1)) {
							veloSaturationTmp(i) = 0.0;
							veloDesTmp(i) = veloSaturationTmp(i);
						}
						else if (veloDesTmp(i) < 0.0 ) { // negativ velocity
							if (posTmp(i) >= upperDangerzone1) {  // in oberer Gefahrenzohne
								veloSaturationTmp(i) = -sqrt(2.0 * -amax * (posTmp(i) - q1_limit(1))); 
							}
							else {
								veloSaturationTmp(i) = -vmax;
							}
							
							if (veloDesTmp(i) < veloSaturationTmp(i)) { // max negativ Velocity
								veloDesTmp(i) = veloSaturationTmp(i);
							}
						}
						else {
							if (posTmp(i) <= underDangerzone1) {  // in unterer Gefahrenzohne
								veloSaturationTmp(i) = sqrt(2.0 * amax * (posTmp(i) - q1_limit(0))); 
							}
							else {
								veloSaturationTmp(i) = vmax;
							}
											
							if (veloDesTmp(i) > veloSaturationTmp(i)) { // max Velocity
								veloDesTmp(i) = veloSaturationTmp(i);
							}
						}
					}
				}
			}

			veloOut.getSignal().setValue(veloDesTmp);
			veloOut.getSignal().setTimestamp(veloDesIn.getSignal().getTimestamp());
		}
		
		virtual eeros::control::Input<AxisVector>& getIn_posActual() {return posActualIn;}
		virtual eeros::control::Input<AxisVector>& getIn_veloDes() {return veloDesIn;}
		virtual eeros::control::Output<AxisVector>& getOut_veloDesSaturation() {return veloOut;}
		virtual void enable() {enabled = true;}
		virtual void disable() {enabled = false;}

	protected:
		AxisVector veloSaturationTmp;
		AxisVector posTmp;
		AxisVector veloDesTmp;
		bool enabled;
		
		eeros::control::Input<AxisVector> posActualIn;
		eeros::control::Input<AxisVector> veloDesIn;
		eeros::control::Output<AxisVector> veloOut;
	};
};
#endif /* CH_NTB_PARALLELSCARA_SPEEDSATURATION_HPP_ */