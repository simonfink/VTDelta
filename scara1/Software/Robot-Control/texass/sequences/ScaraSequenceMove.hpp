#ifndef CH_NTB_SCARA_SCARASEQUENCEMOVE_HPP_
#define CH_NTB_SCARA_SCARASEQUENCEMOVE_HPP_

#include <eeros/sequencer/Sequence.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include "../ScaraControlSystem.hpp"

namespace scara {

	class ScaraSequenceMove : public eeros::sequencer::Sequence<void, int, int> {
	
		public:
			ScaraSequenceMove(eeros::sequencer::Sequencer* sequencer, scara::ScaraControlSystem* controlSys, eeros::safety::SafetySystem* safetySys);
			virtual void run(int from, int to);
			virtual void run(int to);
			virtual bool checkPreCondition();
			virtual void init();
			virtual void exit();
		
		private:
			virtual void up();
			virtual void down();
			virtual void grab();
			virtual void release();
			virtual void move(int position);
			
			scara::ScaraControlSystem* controlSys;
			eeros::safety::SafetySystem* safetySys;
			
			int state;
			int from;
			int to;
		};
}

#endif // CH_NTB_SCARA_SCARASEQUENCEMOVE_HPP_