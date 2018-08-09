#ifndef ORG_EEROS_EXAMPLES_SEQUENCER_MAINSEQUENCE_HPP_
#define ORG_EEROS_EXAMPLES_SEQUENCER_MAINSEQUENCE_HPP_

#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/sequencer/Step.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include "MotorSafetyProperties.hpp"
#include "MotorControlSystem.hpp"
#include <unistd.h>

using namespace eeros::sequencer;
using namespace eeros::safety;
using namespace eeros::logger;

class Move : public Step {
public:
	Move(std::string name, Sequencer& sequencer, BaseSequence* caller, MotorControlSystem& cs) : Step(name, sequencer, caller), cs(cs) { }
	int operator() (double pos) {this->pos = pos; return Step::start();}
	int action() {
		log.info() << "writing " << pos << " to motors";
		cs.setpoint.setValue({pos, pos, pos, pos});
	}
	double pos;
	MotorControlSystem& cs;
};

class MotorSequence : public Sequence {
public:
	MotorSequence(std::string name, Sequencer& seq, SafetySystem& safetySys, MotorSafetyProperties& safetyProp, MotorControlSystem& cs, double angle) : 
					Sequence(name, seq), safetySys(safetySys), safetyProp(safetyProp), angle(angle), controlSys(cs), move("move", seq, this, cs) {
		log.info() << "Sequence created: " << name;
		setNonBlocking();
	}
	int action() {
		while(safetySys.getCurrentLevel() < safetyProp.slMoving);
	
		angle = 0;
		while (Sequencer::running) {
			if(safetySys.getCurrentLevel() == safetyProp.slEmergency){
			 controlSys.setpoint.setValue({0, 0, 0, 0}); continue; 
			}
			angle += 1.;
			if(angle >8.0) angle = 0;
			move(angle);
			sleep(1);
			log.info() << "enc =  " << controlSys.muxEnc.getOut().getSignal().getValue();
		}
	}
private:
	Move move;
	double angle;
	SafetySystem& safetySys;
	MotorControlSystem& controlSys;
	MotorSafetyProperties& safetyProp;
};

#endif // ORG_EEROS_EXAMPLES_SEQUENCER_MAINSEQUENCE_HPP_ 