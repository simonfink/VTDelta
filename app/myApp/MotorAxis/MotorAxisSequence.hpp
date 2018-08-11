#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/sequencer/Step.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include "MotorAxisSafetyProperties.hpp"
#include "MotorAxisControlSystem.hpp"
#include <unistd.h>

using namespace eeros::sequencer;
using namespace eeros::safety;
using namespace eeros::logger;

class Move : public Step {
public:
	Move(std::string name, Sequencer& sequencer, BaseSequence* caller, MotorAxisControlSystem& cs) : Step(name, sequencer, caller), cs(cs) { }
	int operator() (double pos) {this->pos = pos; return Step::start();}
	int action() {
		//cs.const1.setValue(pos);
	}
	double pos;
	MotorAxisControlSystem& cs;
};

class MotorAxisSequence : public Sequence {
public:
	MotorAxisSequence(std::string name, Sequencer& seq, SafetySystem& safetySys, MotorAxisSafetyProperties& safetyProp, MotorAxisControlSystem& cs, double angle) : 
					Sequence(name, seq), safetySys(safetySys), safetyProp(safetyProp), angle(angle), controlSys(cs), move("move", seq, this, cs) {
		log.info() << "Sequence created: " << name;
		setNonBlocking();
	}
	int action() {
	
		angle = 0;
		while (Sequencer::running) {
			if(safetySys.getCurrentLevel() == safetyProp.slStop){
			  controlSys.constant.setValue({0.0,0.0,0.0,0.0});
			}else{
			  controlSys.constant.setValue({1.0,2.0,4.0,8.0});
			}
			/*controlSys.encInput.getSignal().setValue<AxisVector>({controlSys.enc1.getOut().getSignal().getValue(), controlSys.enc2.getOut().getSignal().getValue(), controlSys.enc3.getOut().getSignal().getValue(), controlSys.enc4.getOut().getSignal().getValue()});
			log.info() << controlSys.encInput.getSignal();*/
			
;
		}
	}
private:
	Move move;
	double angle;
	SafetySystem& safetySys;
	MotorAxisControlSystem& controlSys;
	MotorAxisSafetyProperties& safetyProp;
};