#include "ScaraSequenceLoadRefSystem.hpp"
#include "../../ScaraControlSystem.hpp"
#include "../../ScaraSafetyProperties.hpp"
#include <eeros/safety/SafetySystem.hpp>
#include <unistd.h>
#include <iostream>
#include <cmath>
#include <fstream>

using namespace scara;
using namespace eeros::control;
using namespace eeros::sequencer;
using namespace eeros::safety;
using namespace eeros::math;

ScaraSequenceLoadRefSystem::ScaraSequenceLoadRefSystem(Sequencer* sequencer, ScaraControlSystem* controlSys, SafetySystem* safetySys) : 
													Sequence<>("main", sequencer), controlSys(controlSys), safetySys(safetySys) {
	// nothing to do
}

void ScaraSequenceLoadRefSystem::init() {
	std::bind(&ScaraSequenceLoadRefSystem::init, *this);
}

void ScaraSequenceLoadRefSystem::run() {
	log.info() << "[" << name << "] " << "started";

	int line = 0;
	std::string ida, idb;
	CoordinateSystem* a;
	CoordinateSystem* b;
	eeros::math::Matrix <4,4,double> T;
	
	std::fstream file;
	file.open("referenceSystems.txt", std::fstream::in);
	if(!file.is_open()) throw EEROSException("File for loading ref. systems is not open!");
	
	int j = 0; 
	while (!file.eof() && j < 4) {
		file >> ida; 
		a = CoordinateSystem::getCoordinateSystem(ida);
		if(a == nullptr) throw EEROSException("Coordinate System not found!");
		
		file >> idb;
		b = CoordinateSystem::getCoordinateSystem(idb);
		if(b == nullptr) throw EEROSException("Coordinate System not found!");
		
		frame = Frame::getFrame(*a, *b);
		if(frame == nullptr) throw EEROSException("Frame not found!");
		
		for(int i = 0; i<16; i++)
				file >> T(i);

		frame->set(T);
		j++;
	}
	file.close();
}

void ScaraSequenceLoadRefSystem::exit() {
	log.info() << "[ Load Ref System ] exit done";
}
