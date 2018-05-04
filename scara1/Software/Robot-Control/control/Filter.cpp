#include "Filter.hpp"
#include <math.h>
#include <iostream>

using namespace scara;
using namespace eeros::control;
using namespace eeros::math;

Filter::Filter() { 
	// nothing to do...
}

Filter::~Filter() { 
	// nothing to do...
}

void Filter::run() {
	Vector4 filtered;
	Vector4 prev;
	
	filtered = (prev*75+in.getSignal().getValue()*25)/100;
	prev = filtered;
	
	out.getSignal().setValue(filtered);
	out.getSignal().setTimestamp(in.getSignal().getTimestamp());
}
