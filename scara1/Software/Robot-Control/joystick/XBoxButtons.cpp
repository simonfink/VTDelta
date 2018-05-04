#include "XBoxButtons.hpp"
#include "XBoxController.hpp"

#include <iostream>

using namespace scara;
using namespace remote;
using namespace eeros::control;
using namespace eeros::math;

XBoxButtons::XBoxButtons(std::string dev) {
	j.open(dev.c_str());
	t = new std::thread([this](){ this->j.loop(); });
}

XBoxButtons::~XBoxButtons() {
	delete t;
	j.close();
}

void XBoxButtons::run() {
	Vector4 b;

	b << j.current.button_down[remote::XBoxController::Button::A],
	     j.current.button_down[remote::XBoxController::Button::B],
	     j.current.button_down[remote::XBoxController::Button::X],
	     j.current.button_down[remote::XBoxController::Button::Y];
	
	out.getSignal().setValue(b);
		
	uint64_t ts = eeros::System::getTimeNs();
	out.getSignal().setTimestamp(ts);
}

