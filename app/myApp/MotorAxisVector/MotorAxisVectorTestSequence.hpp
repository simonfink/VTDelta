#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/sequencer/Step.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include "MotorAxisVectorTestControlSystem.hpp"
#include "MotorAxisVectorTestSafetyProperties.hpp"
#include <unistd.h>

using namespace eeros::sequencer;
using namespace eeros::safety;
using namespace eeros::logger;

class Move : public Step {
public:
	Move(std::string name, Sequencer& sequencer, BaseSequence* caller, MotorAxisVectorTestControlSystem& cs) : Step(name, sequencer, caller), cs(cs) { }
	int operator() (double pos) {this->pos = pos; return Step::start();}
	int action() {
		//cs.const1.setValue(pos);
	}
	double pos;
	MotorAxisVectorTestControlSystem& cs;
};

class MotorAxisVectorTestSequence : public Sequence {
public:
	MotorAxisVectorTestSequence(std::string name, Sequencer& seq, SafetySystem& safetySys, MotorAxisVectorTestSafetyProperties& safetyProp, MotorAxisVectorTestControlSystem& cs, double angle) : 
					Sequence(name, seq), safetySys(safetySys), safetyProp(safetyProp), angle(angle), controlSys(cs), move("move", seq, this, cs) {
		log.info() << "Sequence created: " << name;
		setNonBlocking();
	}
	int action() {
	
		angle = 0;
		while (Sequencer::running) {
			//angle += 6.28 / 10;
			//move(angle);
			//sleep(1);
			//log.info() << "enc =  " << controlSys.getEncoderSignal().getValue();
			//log.info() << "mouse= " << controlSys.mouse.getButtonOut().getSignal().getValue();
			//log.info() << "mouse pos= " << controlSys.mouse.getOut().getSignal().getValue();
		}
	}
private:
	Move move;
	double angle;
	SafetySystem& safetySys;
	MotorAxisVectorTestControlSystem& controlSys;
	MotorAxisVectorTestSafetyProperties& safetyProp;
};