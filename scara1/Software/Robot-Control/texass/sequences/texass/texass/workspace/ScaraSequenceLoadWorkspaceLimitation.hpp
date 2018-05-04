#ifndef CH_NTB_SCARA_SCARASEQUENCELOADWORKSPACELIMITATION_HPP_
#define CH_NTB_SCARA_SCARASEQUENCELOADWORKSPACELIMITATION_HPP_

#include "../../ScaraControlSystem.hpp"
#include "../../ScaraSafetyProperties.hpp"
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/safety/SafetySystem.hpp>

namespace scara{
	class ScaraSequenceLoadWorkspaceLimitation : public eeros::sequencer::Sequence<> {

	public:
		ScaraSequenceLoadWorkspaceLimitation(eeros::sequencer::Sequencer* sequencer, scara::ScaraControlSystem* controlSys, eeros::safety::SafetySystem* safetySys);
		
		virtual void init();
		virtual void run();
		virtual void exit();
	
	private:
		scara::ScaraControlSystem* controlSys;
		eeros::safety::SafetySystem* safetySys;
		
		eeros::math::Matrix<9, 1, double> limitPoints;
		AxisVector A, B, C,  D, A_r, B_r, C_r,  D_r;
		double a_ab, b_ab, c_ab, a_bc, b_bc, c_bc, a_cd, b_cd, c_cd, a_da, b_da, c_da;
	};
};

#endif // CH_NTB_SCARA_SCARASEQUENCELOADWORKSPACELIMITATION_HPP_