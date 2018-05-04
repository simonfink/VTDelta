#ifndef CH_NTB_PARALLELSCARA_HARDWARE_HPP_
#define CH_NTB_PARALLELSCARA_HARDWARE_HPP_

#include <eeros/hal/HAL.hpp>
// #include <eeros/hal/FlinkDevice.hpp>
// #include <eeros/hal/FlinkFqd.hpp>
// #include <eeros/hal/FlinkDigIn.hpp>
// #include <eeros/hal/FlinkDigOut.hpp>
// #include <eeros/hal/FlinkAnalogIn.hpp>
// #include <eeros/hal/FlinkAnalogOut.hpp>
// #include <eeros/hal/FlinkWatchdog.hpp>

class Hardware {
	
	public:
		Hardware();
		
	private:
		// Flink device
		eeros::hal::FlinkDevice d;

		// Motors - AnalogOut / EncoderIn
		eeros::hal::FlinkAnalogOut dacOut0; 
		eeros::hal::FlinkAnalogOut dacOut1;
		eeros::hal::FlinkFqd enc0;
		eeros::hal::FlinkFqd enc1;

		// Digital Outputs
		eeros::hal::FlinkDigOut blueLed_balancingButton; 
		eeros::hal::FlinkDigOut greenLed_highSpeedButton; 
		eeros::hal::FlinkDigOut redLed_stopButton;
		eeros::hal::FlinkDigOut redLed_approvalButton;
		eeros::hal::FlinkDigOut testToggle;		// toggles at each run of the safety system

		// Digital Inputs
		eeros::hal::FlinkDigIn emergencyButton;
		eeros::hal::FlinkDigIn approvalButton;
		eeros::hal::FlinkDigIn balancingButton; 
		eeros::hal::FlinkDigIn highSpeedButton; 
		eeros::hal::FlinkDigIn stopButton; 

		// Analog Inputs (HALL sensors)
		eeros::hal::FlinkAnalogIn hall0;
		eeros::hal::FlinkAnalogIn hall1;
		eeros::hal::FlinkAnalogIn hall2;
		eeros::hal::FlinkAnalogIn hall3;

		// Analog Outputs (HALL sensors)
		eeros::hal::FlinkAnalogOut hall0out;
		eeros::hal::FlinkAnalogOut hall1out;

public:
		eeros::hal::FlinkWatchdog watchdog; 
		eeros::hal::FlinkDigOut enable;    
};

#endif /* CH_NTB_PARALLELSCARA_HARDWARE_HPP_ */