#pragma once

#include <eeros/sequencer/Sequence.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include "../control/DeltaControlSystem.hpp"
#include "../safety/DeltaSafetyProperties.hpp"
#include <eeros/sequencer/Monitor.hpp>
#include "../conditions/MoveMouse.hpp"
#include "CalibrateSequence.hpp"
#include "SortSequence.hpp"
#include "ShuffleSequence.hpp"
#include "ExceptionSequence.hpp"
#include <array>

namespace eeduro {
	namespace delta {
		class MainSequence : public eeros::sequencer::Sequence {
		public:
			MainSequence(std::string name, eeros::sequencer::Sequencer& sequencer, DeltaControlSystem& controlSys, eeros::safety::SafetySystem& safetySys, DeltaSafetyProperties properties, Calibration& calibration);
			
			int action();

			
		private:
			
			DeltaControlSystem& controlSys;
			eeros::safety::SafetySystem& safetySys;
			DeltaSafetyProperties& properties;
			
 			CalibrateSequence calibSeq;
			SortSequence sortSeq;
			ShuffleSequence shuffSeq;
			MouseSequence mouseSeq;
			
			MouseExceptionSequence mexSeq;
// 			AutoMoveExceptionSequence amexSeq;
			
			Calibration& calibration;
			
			eeros::sequencer::Monitor mouseMove;
			MoveMouseCondition mmc;
			
			eeros::sequencer::Sequencer& sequencer;
		};
	}
}

