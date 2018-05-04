#ifndef CH_NTB_SCARA_SCARASEQUENCECHECKTOOL_HPP_
#define CH_NTB_SCARA_SCARASEQUENCECHECKTOOL_HPP_

#include "../../ScaraControlSystem.hpp"
#include "../../ScaraSafetyProperties.hpp"
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/math/CoordinateSystem.hpp>
#include <eeros/math/Frame.hpp>
#include <eeros/safety/SafetySystem.hpp>

namespace scara{
	class ScaraSequenceCheckTool : public eeros::sequencer::Sequence<void,char,AxisVector> {

	public:
		ScaraSequenceCheckTool(eeros::sequencer::Sequencer* sequencer, scara::ScaraControlSystem* controlSys, eeros::safety::SafetySystem* safetySys);
		
		virtual void init();
		virtual bool checkPreCondition();
		virtual void run(char tool_in, AxisVector checkPos_in);
		virtual bool checkPostCondition();
		virtual void exit();
		
	private:
		eeros::math::Frame* frame;
		eeros::math::Vector3 offset;
		double inputScale = 1.0; 
		AxisVector checkPoint;
		char tool;
		
		scara::ScaraControlSystem* controlSys;
		eeros::safety::SafetySystem* safetySys;	
	};
};

#endif // CH_NTB_SCARA_SCARASEQUENCECHECKTOOL_HPP_