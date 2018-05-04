#ifndef CH_NTB_SCARA_SCARASEQUENCECALCULATEREFSYSTEM_HPP_
#define CH_NTB_SCARA_SCARASEQUENCECALCULATEREFSYSTEM_HPP_

#include "../../ScaraControlSystem.hpp"
#include "../../ScaraSafetyProperties.hpp"
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/math/CoordinateSystem.hpp>
#include <eeros/safety/SafetySystem.hpp>

namespace scara{
	class ScaraSequenceCalculateRefSystem : public eeros::sequencer::Sequence<void, eeros::math::Frame*> {

	public:
		ScaraSequenceCalculateRefSystem(eeros::sequencer::Sequencer* sequencer, scara::ScaraControlSystem* controlSys, eeros::safety::SafetySystem* safetySys);
		
		virtual void init();
		virtual bool checkPreCondition();
		virtual void run(eeros::math::Frame*);
		virtual bool checkPostCondition();
		virtual void exit();
		
	private:
		eeros::math::Frame* frame;
		eeros::math::Vector3 offset;
		eeros::math::Matrix <6, 3, double> savePoints;
		eeros::math::Vector3 A, B, C, D, E, F;
		eeros::math::Vector3 AB, AC, zABC, DE, k, l, D_1, E_1, F_1;
		double modzABC, modl, aABC, bABC, cABC, dABC, k_D, k_E, k_F;
		eeros::math::Matrix<3, 2, double> aMatrix; eeros::math::Vector3 bMatrix; eeros::math::Vector2 lambda; 
		eeros::math::Matrix<2, 3, double> aMatrixT; 
		eeros::math::Matrix<2, 2, double> aABCmatrix, bABCmatrix, cABCmatrix;
		eeros::math::Vector3 e1, e2, e3, O; eeros::math::Matrix<4,4,double> Tr;
		double inputScale = 1.0; 
		
		scara::ScaraControlSystem* controlSys;
		eeros::safety::SafetySystem* safetySys;
	};
};

#endif // CH_NTB_SCARA_SCARASEQUENCECALCULATEREFSYSTEM_HPP_