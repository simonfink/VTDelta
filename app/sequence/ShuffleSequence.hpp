#pragma once

#include <eeros/sequencer/Sequence.hpp>
#include <eeros/safety/SafetySystem.hpp>

#include "../control/DeltaControlSystem.hpp"
#include "step/move.hpp"
#include "step/detect.hpp"
#include "MoveBlockSequence.hpp"

#include <array>

namespace eeduro {
	namespace delta {
		class ShuffleSequence : public eeros::sequencer::Sequence {
		public:
			ShuffleSequence(std::string name, eeros::sequencer::Sequencer& sequencer,eeros::sequencer::BaseSequence* caller, DeltaControlSystem& controlSys, eeros::safety::SafetySystem& safetySys, Calibration& calibration, eeros::sequencer::Monitor& mouseMove);
			
			int action();
			
			bool checkPreCondition();
			
		private:
			virtual void shuffleBlocks(std::array<int,4> blocks);
			virtual int find(const std::array<int,4> &blocks, int block);
			
			DeltaControlSystem& controlSys;
			eeros::safety::SafetySystem& safetySys;
			Move move;
			Detect detect;
			MoveBlockSequence moveBlock;
		};
	}
}

