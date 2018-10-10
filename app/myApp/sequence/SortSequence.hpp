#pragma once

#include <eeros/sequencer/Sequence.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include "../control/DeltaControlSystem.hpp"
#include "../step/move.hpp"
#include "../step/detect.hpp"
#include "MoveBlockSequence.hpp"
#include <eeros/sequencer/Monitor.hpp>
#include "../conditions/MoveMouse.hpp"
#include <array>

namespace eeduro {
	namespace delta {
		class SortSequence : public eeros::sequencer::Sequence {
		public:
			SortSequence(std::string name, eeros::sequencer::Sequencer& sequencer,eeros::sequencer::BaseSequence* caller, DeltaControlSystem& controlSys, eeros::safety::SafetySystem& safetySys, Calibration& calibration, eeros::sequencer::Monitor& mouseMove);
			
			int action();
			
			bool checkPreCondition();
			
		private:
			virtual void sortBlocks(std::array<int,4> blocks);
			virtual int find(const std::array<int,4> &blocks, int block);
			
			DeltaControlSystem& controlSys;
			eeros::safety::SafetySystem& safetySys;
			Move move;
			Detect detect;
			MoveBlockSequence moveBlock;
		};
	}
}

