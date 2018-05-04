#ifndef CH_NTB_SCARA_XBOXBUTTONS_HPP_
#define CH_NTB_SCARA_XBOXBUTTONS_HPP_

#include <string>
#include <thread>
#include <eeros/control/Output.hpp>
#include <eeros/control/Block1o.hpp>
#include <eeros/core/System.hpp>
#include "Joystick.hpp"

#include <eeros/math/Matrix.hpp>

namespace scara {

	class XBoxButtons: public eeros::control::Block1o<eeros::math::Vector4> {
	public:
		XBoxButtons(std::string dev);
		virtual ~XBoxButtons();
		
		virtual void run();
		
		
	protected:
		remote::Joystick j;
		std::thread* t;
	};
};

#endif /* CH_NTB_SCARA_XBOXBUTTONS_HPP_ */
