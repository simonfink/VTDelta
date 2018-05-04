#ifndef CH_NTB_PARALLELSCARA_PENDULUMCONTROLLER_HPP_
#define CH_NTB_PARALLELSCARA_PENDULUMCONTROLLER_HPP_

#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/control/D.hpp>
#include <eeros/control/I.hpp>
#include <eeros/control/Sum.hpp>
#include <eeros/control/Gain.hpp>

#include "../../types.hpp"
#include "AngleAccToTipAcc.hpp"
#include "AngleToTipPos.hpp"
#include "TipAccToAngle.hpp"
#include "Filter.hpp"

using namespace eeros;
using namespace eeros::control;
using namespace parallelscara;

namespace parallelscara {
	class PendulumController : public Block {

	public: 
		PendulumController();
		virtual ~PendulumController();
	
		virtual void run();
		virtual Input<AxisVector>& getIn_refPos_xy();
		virtual Input<AxisVector>& getIn_actPos_xy();
		virtual Input<AxisVector>& getIn_phiHall();
// 			virtual Input<bool>& getEnableIntegral();
		virtual Output<AxisVector>& getOutRefPos();
		virtual Output<AxisVector>& getOut_refVel();
		virtual Output<AxisVector>& getOut_refAcc(); 
		
// 		private:
		Filter phiFilter, accFilter; 
		AngleToTipPos angleToTipPos;
		Sum<2,AxisVector> tipSum, phiSum;
		Gain<AxisVector,double> xy_refPos_in, xy_actPos_in, phi_hallSensor_in;
		D<AxisVector> diff_tip;
		Gain<AxisVector,AxisVector,true> Pgain_tip, Dgain_tip; 
		Sum<2,AxisVector> sumPD_tip;
		TipAccToAngle tipAccToAngle;
		D<AxisVector> diff_angle;
		Gain<AxisVector,AxisVector,true> Pgain_angle, Dgain_angle; 
		Sum<2,AxisVector> sumPD_angle;
		AngleAccToTipAcc angleAccToTipAcc;
		I<AxisVector> I_accToVel, I_velToPos;
// 			Gain<bool, bool> enableIntegral;	
	};
};

#endif /* CH_NTB_PARALLELSCARA_PENDULUMCONTROLLER_HPP_ */