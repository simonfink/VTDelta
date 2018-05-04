#ifndef CH_NTB_SCARA_AXISTEST_HPP_
#define CH_NTB_SCARA_AXISTEST_HPP_

#include <eeros/core/Runnable.hpp>
#include <eeros/control/PeripheralInput.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/logger/LogWriter.hpp>
#include <eeros/logger/StreamLogWriter.hpp>

namespace scara {

	class AxisTest : public eeros::Runnable {
	
	public:
		void run();
		
		static AxisTest& instance();
		
		eeros::control::PeripheralInput<double> enc0;
		eeros::control::PeripheralInput<double> enc1;
		eeros::control::PeripheralInput<double> enc2;
		eeros::control::PeripheralInput<double> enc3;
		
	private:
		AxisTest();
		AxisTest(const AxisTest&);
		AxisTest& operator=(const AxisTest&);
		
		eeros::logger::Logger<eeros::logger::LogWriter> l;
		eeros::logger::StreamLogWriter w;
		
	}; // END class
}; // END namespace

#endif // CH_NTB_SCARA_AXISTEST_HPP_