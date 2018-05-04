#include "XBoxInput.hpp"
#include "XBoxController.hpp"

using namespace scara;
using namespace remote;
using namespace eeros::control;
using namespace eeros::math;

XBoxInput::XBoxInput(std::string dev) {
	j.open(dev.c_str());
	t = new std::thread([this](){ this->j.loop(); });
	axisScale << xScale,      0,      0,      0,
	                  0, yScale,      0,      0,
	                  0,      0, zScale,      0,
	                  0,      0,      0, rScale;
}

XBoxInput::~XBoxInput() {
	delete t;
	j.close();
}

void XBoxInput::run() {
	Vector4 v;
// 	// original
// 	v << j.current.axis[XBoxController::Axis::RX],
// 	     j.current.axis[XBoxController::Axis::RY],
// 	     j.current.axis[XBoxController::Axis::LY],
// 	     j.current.axis[XBoxController::Axis::RT] - j.current.axis[XBoxController::Axis::LT];
	
	v <<  j.current.axis[XBoxController::Axis::RY],
	      j.current.axis[XBoxController::Axis::RX],
	     -j.current.axis[XBoxController::Axis::LY],
	      j.current.axis[XBoxController::Axis::RT] - j.current.axis[XBoxController::Axis::LT];
	
	for(int i = 0; i < v.getNofRows(); i++) {
		if(v(i) > -0.2 && v(i) < 0.2) v(i) = 0;
	}
	
	out.getSignal().setValue(out.getSignal().getValue() + axisScale * speedScaleFactor * v);
	
	uint64_t ts = eeros::System::getTimeNs();
	out.getSignal().setTimestamp(ts);
}

void XBoxInput::setInitPos(Vector4 initPos) {
	out.getSignal().setValue(initPos);
}

void XBoxInput::setSpeedScaleFactor(double speedScale) {
	if(speedScale >= 0 && speedScale < 5)
		speedScaleFactor = speedScale;
}

