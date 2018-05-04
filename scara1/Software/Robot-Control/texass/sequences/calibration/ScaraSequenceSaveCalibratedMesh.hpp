#ifndef CH_NTB_SCARA_SCARASEQUENCESAVECALIBRATEDMESH_HPP_
#define CH_NTB_SCARA_SCARASEQUENCESAVECALIBRATEDMESH_HPP_

#include <eeros/safety/SafetySystem.hpp>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>

#include "../../ScaraControlSystem.hpp"
#include "../../ScaraSafetyProperties.hpp"
#include "../tools/ScaraSequenceCheckTool.hpp"
#include "../goingToReady/ScaraSequenceGoToReady.hpp"
#include "../tools/ScaraSequenceGoToToolChange.hpp"

#include <eeros/sequencer/Sequence.hpp>

namespace scara{
	class ScaraSequenceSaveCalibratedMesh : public eeros::sequencer::Sequence<> {

	public:
		ScaraSequenceSaveCalibratedMesh(eeros::sequencer::Sequencer* sequencer, scara::ScaraControlSystem* controlSys, eeros::safety::SafetySystem* safetySys);
		
		virtual void init();
		virtual bool checkPreCondition();
		virtual void run();
		virtual bool checkPostCondition();
		virtual void exit();
	
	private: 
		eeros::math::Matrix<9, 4, double> mesh_referencePoints;
		double inputScale = 1.0; 
		
		ScaraSequenceGoToReady goToReady;
		ScaraSequenceCheckTool checkTool; 
		ScaraSequenceGoToToolChange goToTool;
		
		scara::ScaraControlSystem* controlSys;
		eeros::safety::SafetySystem* safetySys;
	};
}; 

#endif // CH_NTB_SCARA_SCARASEQUENCESAVECALIBRATEDMESH_HPP_ 
