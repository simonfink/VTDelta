#include "ScaraSequenceMove.hpp"

using namespace scara;
using namespace eeros::sequencer;
using namespace eeros::safety;

enum {
	idle,
	move_from_to,
	move_to
};

ScaraSequenceMove::ScaraSequenceMove(Sequencer* sequencer, ScaraControlSystem* controlSys, SafetySystem* safetySys) : Sequence<void, int, int>("main", sequencer), controlSys(controlSys), safetySys(safetySys), state(idle) {
	// nothing to do
}

bool ScaraSequenceMove::checkPreCondition() {
	return (state != idle);
}

void ScaraSequenceMove::run(int from, int to) {
	if (state == idle) {
		this->from = from;
		this->to = to;
		state = move_from_to;
	}
	else {
		log.warn() << "[" << name << "]: another operation is currently executing";
	}
}

void ScaraSequenceMove::run(int to) {
	if (state == idle) {
		this->from = (-1);
		this->to = to;
		state = move_to;
	}
	else {
		log.warn() << "[" << name << "]: another operation is currently executing";
	}
}

void ScaraSequenceMove::init() {
	std::bind(&ScaraSequenceMove::init, *this);
	
	// if (state == move_from_to) {
	// addStep([&] () { up(); });
	// addStep([&] () { move(from); });
	// addStep([&] () { down(); });
	// addStep([&] () { grab(); });

	// }
	//
	// addStep([&] () { up(); });
	// addStep([&] () { move(to); });
	// addStep([&] () { down(); });
	// addStep([&] () { release(); });
	// addStep([&] () { up(); });
}

void ScaraSequenceMove::exit() {
	state = idle;
}

void ScaraSequenceMove::up() {
	log.trace() << "move to transportation height";
}

void ScaraSequenceMove::down() {
	log.trace() << "move down";
}

void ScaraSequenceMove::grab() {
	log.trace() << "grab block";
}

void ScaraSequenceMove::release() {
	log.trace() << "release block";
}

void ScaraSequenceMove::move(int position) {
	log.trace() << "move to position " << position;
}