#include "Hardware.hpp"

#include<eeros/hal/PeripheralInput.hpp>
#include<eeros/hal/PeripheralOutput.hpp>
#include<eeros/hal/ScalablePeripheralInput.hpp>

#include "constants.hpp"

using namespace eeros::hal;
using namespace parallelscara;

Hardware::Hardware() :
	d("/dev/flink0"),
	
	dacOut0("dacOut0", &d, 3, 0, dac0Scale, dac0Offset),
	dacOut1("dacOut1", &d, 3, 1, dac1Scale, dac1Offset),
	
	enc0("enc0", &d, 1, 0, enc0Scale, 0),
	enc1("enc1", &d, 1, 1, enc1Scale, 0),
	
	watchdog("watchdog", &d, 2, 0.005),
	enable  ("enable"  , &d, 5, 5    ),
	
	blueLed_balancingButton ("blueLed_balancingButton" , &d, 5, 0), 
	greenLed_highSpeedButton("greenLed_highSpeedButton", &d, 5, 2), 
	redLed_stopButton       ("redLed_stopButton"       , &d, 5, 6), 
	redLed_approvalButton   ("redLed_approvalButton"   , &d, 5, 3), 
	
	testToggle("testToggle"   , &d, 5, 1), 
	
	emergencyButton("emergencyButton", &d, 6, 6      ), 
	approvalButton ("approvalButton" , &d, 6, 3, true),  
	balancingButton("balancingButton", &d, 6, 0, true),   
	highSpeedButton("highSpeedButton", &d, 6, 2, true),     
	stopButton     ("stopButton"     , &d, 6, 1, true),
	
// 	tempSensor0("tempSensor0", &d, 6, 5),
// 	tempSensor1("tempSensor1", &d, 6, 7),
	
	hall0("hall0", &d, 4, 1),   /*0),*/
	hall1("hall1", &d, 4, 0),   /*1),*/
	hall2("hall2", &d, 4, 3),   /*2),*/
	hall3("hall3", &d, 4, 2),    /*3) */
	hall0out("hall0out", &d, 3, 2, dac0Scale / 50, dac0Offset / 50),   
	hall1out("hall1out", &d, 3, 3, dac0Scale / 50, dac0Offset / 50)

{
	auto &hal = HAL::instance();
	
	hal.addPeripheralOutput(&dacOut0);
	hal.addPeripheralOutput(&dacOut1);
	
	hal.addPeripheralInput(&enc0);
	hal.addPeripheralInput(&enc1);
	
	hal.addPeripheralOutput(&watchdog);
	hal.addPeripheralOutput(&enable);
	
	hal.addPeripheralOutput(&blueLed_balancingButton); 
	hal.addPeripheralOutput(&greenLed_highSpeedButton);
	hal.addPeripheralOutput(&redLed_stopButton);
	hal.addPeripheralOutput(&redLed_approvalButton);
	
	hal.addPeripheralOutput(&testToggle);
	
	hal.addPeripheralInput(&emergencyButton);
	hal.addPeripheralInput(&approvalButton);
	
	hal.addPeripheralInput(&balancingButton);
	hal.addPeripheralInput(&highSpeedButton);
	hal.addPeripheralInput(&stopButton);
	
	hal.addPeripheralInput(&hall0);
	hal.addPeripheralInput(&hall1);
	hal.addPeripheralInput(&hall2);
	hal.addPeripheralInput(&hall3);
	
	hal.addPeripheralOutput(&hall0out);
	hal.addPeripheralOutput(&hall1out);


}