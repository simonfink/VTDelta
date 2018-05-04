#include "ScaraDirectKinematic.hpp"

using namespace scara;
using namespace eeros::control;
using namespace eeros::math;

ScaraDirectKinematic::ScaraDirectKinematic(double l1, double l2) : l1(l1), l2(l2) {
	// nothing to do...
}

ScaraDirectKinematic::~ScaraDirectKinematic() { 
	// nothing to do...
}

void ScaraDirectKinematic::run() {
	
	Vector4 cartesianCoords;
	Vector4 jointCoords = in.getSignal().getValue();
	
	cartesianCoords[0] = l1 * cos(-jointCoords[0]) + l2 * cos(-jointCoords[0]-jointCoords[1]);
	cartesianCoords[1] = l1 * sin(-jointCoords[0]) + l2 * sin(-jointCoords[0]-jointCoords[1]);
	cartesianCoords[2] = -jointCoords[2];
	cartesianCoords[3] = -jointCoords[0] - jointCoords[1] - jointCoords[3];
	
	out.getSignal().setValue(cartesianCoords);
	out.getSignal().setTimestamp(in.getSignal().getTimestamp());
}
