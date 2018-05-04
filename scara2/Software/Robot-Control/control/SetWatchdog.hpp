#ifndef CH_NTB_PARALLELSCARA_SETWATCHDOG_HPP_
#define CH_NTB_PARALLELSCARA_SETWATCHDOG_HPP_

#include <eeros/control/Block1o.hpp>
#include <eeros/control/PeripheralOutput.hpp>
#include <eeros/hal/HAL.hpp>

using namespace eeros::control;
using namespace eeros::hal;

namespace parallelscara {

	class SetWatchdog: public Block {
	public:
		SetWatchdog(std::string id) : hal(HAL::instance()) { 
			wdt = hal.getLogicOutput("Wdt", false);
		}

		virtual ~SetWatchdog() { }
		
		virtual void run() { 
			static bool first = true;
			if (first) {
				hal.callOutputFeature(wdt, "setWatchdogTimeout", 30000.0);
				hal.callOutputFeature(wdt, "resetWatchdog");
				first = false;
			}
			wdt->set(true);
		}
	private:
		HAL& hal;
		eeros::hal::Output<bool>* wdt;
	};
};
#endif /* CH_NTB_PARALLELSCARA_SETWATCHDOG_HPP_ */ 
