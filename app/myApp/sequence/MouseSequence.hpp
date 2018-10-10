#pragma once

#include <eeros/sequencer/Sequence.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include "../control/DeltaControlSystem.hpp"
#include "../step/move.hpp"
#include "../step/detect.hpp"
#include "MoveBlockSequence.hpp"
#include "ExceptionSequence.hpp"
#include "../step/emag.hpp"
#include <array>

namespace eeduro {
	namespace delta {
		class MouseSequence : public eeros::sequencer::Sequence {
		public:
			MouseSequence(std::string name, eeros::sequencer::Sequencer& sequencer, eeros::sequencer::BaseSequence* caller, DeltaControlSystem& controlSys, eeros::safety::SafetySystem& safetySys, Calibration& calibration, DeltaSafetyProperties& properties);
			
			int action();
			
			bool checkPreCondition();

			
		private:
			int count;
			DeltaControlSystem& controlSys;
			eeros::safety::SafetySystem& safetySys;
			bool buttonPressed = false;
			AxisVector mouseNew;
			AxisVector mouseOld;
			MouseTimeOutExceptionSequence timeoutSeq;
			Emag emag;
		};
	}
}
