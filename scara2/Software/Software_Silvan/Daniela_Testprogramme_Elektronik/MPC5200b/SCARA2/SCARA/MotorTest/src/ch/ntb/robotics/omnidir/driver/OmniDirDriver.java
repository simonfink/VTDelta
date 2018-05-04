/*
 * Copyright (c) 2013 NTB Interstate University of Applied Sciences of Technology Buchs.
 * All rights reserved.
 *
 * http://www.ntb.ch/inf
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * 
 * Neither the name of the project's author nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

package ch.ntb.robotics.omnidir.driver;

import ch.ntb.inf.deep.runtime.mpc5200.*;
import ch.ntb.inf.deep.unsafe.US;
import ch.ntb.robotics.omnidir.savety.ErrorHandler;

/**********************************************************
 * File:     OmniDirDriver.java                              
 * Created:  2013-01-31 dfrommelt    
 * Changes:	 none                        
 * ------------------------------------------------------ 
 * Description:                                           
 * concludes all methods for communicate with the periphery
 * on the SPI-bus
 * the method "transceive" must be called every 1ms
 * 
 **********************************************************/
public class OmniDirDriver implements IphyCoreMpc5200tiny{

	// SPI is connected on PSC1
	private static final int PSCBase = PSC1Base;
	
	// digital registers
	private static short regTX00_DIO = 0x2000;
	private static short regRX00_DIO = 0x0000;
	
	// servo transmit registers
	private static char regTX01_ServoConf 	= 0x3400;
	private static char regTX02_Servo0A 	= 0x202F;
	private static char regTX03_Servo0B 	= 0x2000;
	private static char regTX04_Servo1A 	= 0x202F;
	private static char regTX05_Servo1B 	= 0x2000;
	private static char regTX06_Servo2A 	= 0x202F;
	private static char regTX07_Servo2B 	= 0x2000;
	private static char regTX08_Servo3A 	= 0x202F;
	private static char regTX09_Servo3B 	= 0x2000;
	private static char regTX10_Servo4A 	= 0x202F;
	private static char regTX11_Servo4B 	= 0x2000;
	private static char regTX12_Servo5A 	= 0x202F;
	private static char regTX13_Servo5B 	= 0x4000;
	
	// hall sensor transmit registers
	private static char regTX14_Hall0 		= 0x4800;
	private static char regTX15_Hall1 		= 0x5000;
	private static char regTX16_Hall2 		= 0x5800;
	private static char regTX17_Hall3 		= 0x6000;
	
	// acceleration transmit registers
	private static char regTX18_AccA 		= 0x6001;
	private static char regTX19_AccB 		= 0x6002;
	private static char regTX20_AccC 		= 0x6003;
	private static char regTX21_AccD		= 0x6004;
	private static char regTX22_AccE 		= 0x6005;
	private static char regTX23_AccF 		= 0x8000;
	
	// ultrasonic sensor transmit registers
	private static char regTX24_Ultras0 	= 0x8800;
	private static char regTX25_Ultras1 	= 0x9000;
	private static char regTX26_Ultras2 	= 0x9800;
	private static char regTX27_Ultras3 	= 0xA000;
	private static char regTX28_Ultras4 	= 0xA800;
	private static char regTX29_Ultras5 	= 0xB800;
	
	// battery voltage transmit register
	private static char regTX30_Batt 		= 0xC000;
	
	// joystick transmit register
	private static char regTX31_JoyA 		= 0xC800;
	private static char regTX32_JoyB 		= 0x0000;
	
	// servo receive register
	private static short regRX01_ServoConf  = 0x0000;
	private static short regRX02_Servo0A 	= 0x0000;
	private static short regRX03_Servo0B 	= 0x0000;
	private static short regRX04_Servo1A 	= 0x0000;
	private static short regRX05_Servo1B 	= 0x0000;
	private static short regRX06_Servo2A 	= 0x0000;
	private static short regRX07_Servo2B 	= 0x0000;
	private static short regRX08_Servo3A 	= 0x0000;
	private static short regRX09_Servo3B 	= 0x0000;
	private static short regRX10_Servo4A 	= 0x0000;
	private static short regRX11_Servo4B 	= 0x0000;
	private static short regRX12_Servo5A 	= 0x0000;
	private static short regRX13_Servo5B 	= 0x0000;
	
	// hall sensor receive register
	private static short regRX14_Hall0 		= 0x0000;
	private static short regRX15_Hall1 		= 0x0000;
	private static short regRX16_Hall2 		= 0x0000;
	private static short regRX17_Hall3 		= 0x0000;
	
	// acceleration sensor receive register
	private static short regRX18_AccA 		= 0x0000;
	private static short regRX19_AccB 		= 0x0000;
	private static short regRX20_AccC 		= 0x0000;
	private static short regRX21_AccD 		= 0x0000;
	private static short regRX22_AccE 		= 0x0000;
	private static short regRX23_AccF 		= 0x0000;
	
	// ultrasonic receive register
	private static short regRX24_Ultras0 	= 0x0000;
	private static short regRX25_Ultras1 	= 0x0000;
	private static short regRX26_Ultras2 	= 0x0000;
	private static short regRX27_Ultras3 	= 0x0000;
	private static short regRX28_Ultras4 	= 0x0000;
	private static short regRX29_Ultras5 	= 0x0000;
	
	// battery receive register
	private static short regRX30_Batt 		= 0x0000;
	
	// joystick receive register
	private static short regRX31_JoyA 		= 0x0000;
	private static short regRX32_JoyB 		= 0x0000;
	
	//joystick values (limited acceleration)
	private static int val_joyA = 0;
	private static int val_joyB = 0;
	private static final int maxDelta_joy = 3;
	
	//counter for timeouts
	private static int cntTimeout = 0;
	
	/*(non-Javadoc)
	*	class initialize, select the spi mode
	*/
	public static void init(){
		US.PUT1(PSCBase + PSCCR, 0xa); // disable Tx, Rx
		US.PUT1(PSCBase + PSCCR, 0x20); // reset receiver, clears fifo
		US.PUT1(PSCBase + PSCCR, 0x30); // reset transmitter, clears fifo
		US.PUT4(PSCBase + PSCSICR, 0x0280F000); // select SPI mode, master, 16 bit, msb first
		US.PUT4(CDMPSC1MCLKCR, 0x0008);	// Mclk = 56MHz
		US.PUT4(CDMPSC1MCLKCR, 0x8008);	// Mclk = 56MHz
		US.PUT4(CDMCER, US.GET4(CDMCER) | 0x20);	// enable Mclk for PSC1
        US.PUT4(PSCBase + PSCCCR, 0x00080000); // DSCKL = 60ns, SCK = 14MHz
		US.PUT1(PSCBase + PSCCTUR, 0); // set DTL to 150ns
		US.PUT1(PSCBase + PSCCTLR, 0xA); 
		US.PUT1(PSCBase + PSCTFCNTL, 0x1); // no frames
		US.PUT1(PSCBase + PSCRFCNTL, 0x1); // no frames
		US.PUT4(GPSPCR, US.GET4(GPSPCR) | 0x7);	// use pins on PCS1 for SPI
		US.PUT1(PSCBase + PSCCR, 0x5); // enable Tx, Rx	
	}

	/*(non-Javadoc)
	*  method for transmit and receive all data on spi bus
	*  this method can be blocking if there is an fault on init
	*/
	public static void transceive() {
		// clear the receive buffer
		//while (US.GET2(PSCBase + PSCRFNUM) > 0){
		//	US.GET2(PSCBase + PSCRxBuf);
		//}
		US.PUT2(PSCBase + PSCTxBuf, regTX00_DIO); // start transfer
		US.PUT2(PSCBase + PSCTxBuf, regTX01_ServoConf); 
		US.PUT2(PSCBase + PSCTxBuf, regTX02_Servo0A); 
		US.PUT2(PSCBase + PSCTxBuf, regTX03_Servo0B); 
		US.PUT2(PSCBase + PSCTxBuf, regTX04_Servo1A); 
		US.PUT2(PSCBase + PSCTxBuf, regTX05_Servo1B); 
		US.PUT2(PSCBase + PSCTxBuf, regTX06_Servo2A); 
		US.PUT2(PSCBase + PSCTxBuf, regTX07_Servo2B); 
		US.PUT2(PSCBase + PSCTxBuf, regTX08_Servo3A); 
		US.PUT2(PSCBase + PSCTxBuf, regTX09_Servo3B); 
		US.PUT2(PSCBase + PSCTxBuf, regTX10_Servo4A); 
		US.PUT2(PSCBase + PSCTxBuf, regTX11_Servo4B); 
		US.PUT2(PSCBase + PSCTxBuf, regTX12_Servo5A); 
		US.PUT2(PSCBase + PSCTxBuf, regTX13_Servo5B); 
		US.PUT2(PSCBase + PSCTxBuf, regTX14_Hall0); 
		US.PUT2(PSCBase + PSCTxBuf, regTX15_Hall1); 
		US.PUT2(PSCBase + PSCTxBuf, regTX16_Hall2); 
		US.PUT2(PSCBase + PSCTxBuf, regTX17_Hall3); 
		US.PUT2(PSCBase + PSCTxBuf, regTX18_AccA); 
		US.PUT2(PSCBase + PSCTxBuf, regTX19_AccB); 
		US.PUT2(PSCBase + PSCTxBuf, regTX20_AccC); 
		US.PUT2(PSCBase + PSCTxBuf, regTX21_AccD); 
		US.PUT2(PSCBase + PSCTxBuf, regTX22_AccE); 
		US.PUT2(PSCBase + PSCTxBuf, regTX23_AccF); 
		US.PUT2(PSCBase + PSCTxBuf, regTX24_Ultras0); 
		US.PUT2(PSCBase + PSCTxBuf, regTX25_Ultras1); 
		US.PUT2(PSCBase + PSCTxBuf, regTX26_Ultras2); 
		US.PUT2(PSCBase + PSCTxBuf, regTX27_Ultras3); 
		US.PUT2(PSCBase + PSCTxBuf, regTX28_Ultras4); 
		US.PUT2(PSCBase + PSCTxBuf, regTX29_Ultras5); 
		US.PUT2(PSCBase + PSCTxBuf, regTX30_Batt); 
		US.PUT2(PSCBase + PSCTxBuf, regTX31_JoyA); 
		US.PUT2(PSCBase + PSCTxBuf, regTX32_JoyB); 

		// wait for receiving the data and checking for data
		while (US.GET2(PSCBase + PSCRFNUM) < 64){
			cntTimeout ++;
			if(cntTimeout > 100000){
				ErrorHandler.setError(2000);
				cntTimeout = 0;
				return;
			}
		}
		cntTimeout = 0;
		
		// save the received data to the local values
		regRX00_DIO = US.GET2(PSCBase + PSCRxBuf);
		regRX01_ServoConf = US.GET2(PSCBase + PSCRxBuf);
		regRX02_Servo0A = US.GET2(PSCBase + PSCRxBuf);
		regRX03_Servo0B = US.GET2(PSCBase + PSCRxBuf);
		regRX04_Servo1A = US.GET2(PSCBase + PSCRxBuf);
		regRX05_Servo1B = US.GET2(PSCBase + PSCRxBuf);
		regRX06_Servo2A = US.GET2(PSCBase + PSCRxBuf);
		regRX07_Servo2B = US.GET2(PSCBase + PSCRxBuf);
		regRX08_Servo3A = US.GET2(PSCBase + PSCRxBuf);
		regRX09_Servo3B = US.GET2(PSCBase + PSCRxBuf);
		regRX10_Servo4A = US.GET2(PSCBase + PSCRxBuf);
		regRX11_Servo4B = US.GET2(PSCBase + PSCRxBuf);
		regRX12_Servo5A = US.GET2(PSCBase + PSCRxBuf);
		regRX13_Servo5B = US.GET2(PSCBase + PSCRxBuf);
		regRX14_Hall0 = US.GET2(PSCBase + PSCRxBuf);
		regRX15_Hall1 = US.GET2(PSCBase + PSCRxBuf);
		regRX16_Hall2 = US.GET2(PSCBase + PSCRxBuf);
		regRX17_Hall3 = US.GET2(PSCBase + PSCRxBuf);
		regRX18_AccA = US.GET2(PSCBase + PSCRxBuf);
		regRX19_AccB = US.GET2(PSCBase + PSCRxBuf);
		regRX20_AccC = US.GET2(PSCBase + PSCRxBuf);
		regRX21_AccD = US.GET2(PSCBase + PSCRxBuf);
		regRX22_AccE = US.GET2(PSCBase + PSCRxBuf);
		regRX23_AccF = US.GET2(PSCBase + PSCRxBuf);
		regRX24_Ultras0 = US.GET2(PSCBase + PSCRxBuf);
		regRX25_Ultras1 = US.GET2(PSCBase + PSCRxBuf);
		regRX26_Ultras2 = US.GET2(PSCBase + PSCRxBuf);
		regRX27_Ultras3 = US.GET2(PSCBase + PSCRxBuf);
		regRX28_Ultras4 = US.GET2(PSCBase + PSCRxBuf);
		regRX29_Ultras5 = US.GET2(PSCBase + PSCRxBuf);
		regRX30_Batt = US.GET2(PSCBase + PSCRxBuf);
		regRX31_JoyA = US.GET2(PSCBase + PSCRxBuf);
		regRX32_JoyB = US.GET2(PSCBase + PSCRxBuf);
		
		// limit the rate of change of the values of the joystick
		int deltaJoyA = ((int)regRX31_JoyA) - val_joyA;
		if(deltaJoyA>maxDelta_joy) val_joyA+=maxDelta_joy;
		else if(deltaJoyA<-maxDelta_joy) val_joyA -= maxDelta_joy;
		else val_joyA += deltaJoyA;
		
		int deltaJoyB = ((int)regRX32_JoyB) - val_joyB;
		if(deltaJoyB>maxDelta_joy) val_joyB+=maxDelta_joy;
		else if(deltaJoyB<-maxDelta_joy) val_joyB -= maxDelta_joy;
		else val_joyB += deltaJoyB;
		
		// check the received data
		if(regRX01_ServoConf == 0 || regRX01_ServoConf == 0xFFFF) ErrorHandler.setError(2100);
		if(regRX14_Hall0 == 0 || regRX14_Hall0 == 0xFFFF) ErrorHandler.setError(6200);

	}
	
	/*(non-Javadoc)
	*	getting the states of the digital inputs of the HMI
	*@param idx
	*	index of the taster (0 to 3)
	*@return
	*	state of the taster, true means pressed
	*/
	public static boolean getDIButton(int idx){
		return (regRX00_DIO & (1<<(idx+1))) ==0;
	}
	
	/*(non-Javadoc)
	*	getting the state of the digital inputs of the homing sensors
	*@param idx
	*	index of the desired sensor (0 to 2)
	*@return
	*	state of the homing Taster, true means active or not connected
	*/
	public static boolean getDIHoming(int idx){
		return (regRX00_DIO & (1<<(5+idx))) !=0;
	}
	
	/*(non-Javadoc)
	*	getting the battery voltage
	*@return
	*	battery voltage in volts (fits ~0.1V)
	*/
	public static double getBattVoltage(){
		return (double)regRX30_Batt*0.012635206786850;
	
	}
	
	/*(non-Javadoc)
	*	getting the adc value of the ultrasonic sensors
	*@param idx
	*	index of the desired ultrasonic sensor (0 to 5)
	*@return
	*	integer value of the adc value (0 to 2^12-1)
	*/
	public static int getADCUltras(int idx){
		switch(idx){
		case 0: return regRX24_Ultras0;
		case 1: return regRX25_Ultras1;
		case 2: return regRX26_Ultras2;
		case 3: return regRX27_Ultras3;
		case 4: return regRX28_Ultras4;
		case 5: return regRX29_Ultras5;
		default: return 0;
		}
	
	}
	
	/*(non-Javadoc)
	*	getting the adc value of the joystick
	*@param CH_X
	*	if true  -> get channel a
	*   if false -> get channel b
	*@return
	*	integer value (0 -> middle, range ~ -1500 to 1500)
	*/
	public static int getADCJoystick(boolean CH_X){
		int temp_val;
		if(CH_X)
			temp_val = val_joyA - 2176;
		else
			temp_val = val_joyB - 2049;
		
		if(temp_val > 10) temp_val -= 10;
		else if(temp_val < -10) temp_val += 10;
		else temp_val = 0;
		
		if(temp_val < -1500) temp_val = -1500;
		if(temp_val > 1500) temp_val = 1500;
		return temp_val;
	}
	
	/*(non-Javadoc)
	*	method for getting the states of the digital inputs of the joystick
	*@param CH0
	*	if true  -> getting channel C
	*	if false -> getting channel D
	*/
	public static boolean getDIJoystick(boolean CHC){
		if(CHC) return (regRX00_DIO & (1<<9)) ==0;
		else return (regRX00_DIO & (1<<10)) ==0;
	}
	
	/*(non-Javadoc)
	*	method for getting the state of the laser output
	*@return
	*	boolean value of the laser output
	*/
	public static boolean getLaserOutput(){
		return (regRX00_DIO & (1<<8)) != 0;
		
	}
	
	/*(non-Javadoc)
	*	method for getting the position of the motor
	*@param motorNr
	*	index of the motor (0 to 5)
	*@return
	*	integer of counted flags of the encoder
	*/
	public static short getPosShort(int motorNr){
		switch(motorNr){
		case 0:
			return regRX02_Servo0A;
		case 1:
			return regRX04_Servo1A;
		case 2:
			return regRX06_Servo2A;
		case 3:
			return regRX08_Servo3A;
		case 4:
			return regRX10_Servo4A;
		case 5:
			return regRX12_Servo5A;
		default:
			return 0;
		}
	}
	
	/*(non-Javadoc)
	*	method for getting the velocity of the motor
	*@param motorNr
	*	index of the motor (0 to 5)
	*@return
	*	integer of counted clocks of until the the encoder reaches the next flag
	*/
	public static short getVeloShort(int motorNr){
		switch(motorNr){
		case 0:
			return regRX03_Servo0B;
		case 1:
			return regRX05_Servo1B;
		case 2:
			return regRX07_Servo2B;
		case 3:
			return regRX09_Servo3B;
		case 4:
			return regRX11_Servo4B;
		case 5:
			return regRX13_Servo5B;
		default:
				return 0;
		}
	}
	
	/*(non-Javadoc)
	*	method for getting the returned value from the acceleration sensor
	*@param idx
	*	index for the register of the acceleration sensor
	*@return
	*	integer value from the register
	*/
	public static int getAccValue(int idx){
		switch(idx){
		case 0: return regRX18_AccA;
		case 1: return regRX19_AccB;
		case 2: return regRX20_AccC;
		case 3: return regRX21_AccD;
		case 4: return regRX22_AccE;
		case 5: return regRX23_AccF;
		default: return 0;	
		}
		
	}
	
	/*(non-Javadoc)
	*	method for getting the adc value of the inverted pendulum sensor
	*@param motorNr
	*	index of the hall sensor (0 to 3)
	*@return
	*	integer of the adc subtracted with 2048
	*/
	public static int getADCHallSens(int a){
		switch(a){
		case 0: return regRX14_Hall0 - 2048;
		case 1:	return regRX15_Hall1 - 2048;
		case 2:	return regRX16_Hall2 - 2048;
		case 3:	return regRX17_Hall3 - 2048;
		default: return 0;
		}
	}
	
	/*(non-Javadoc)
	*	method for setting the LED of the HMI
	*@param motorNr
	*	index of the motor (0 to 5)
	*/
	public static void setLED(int idx, int mode){
		regTX00_DIO = (short)((regTX00_DIO & ~(0x3<<(1+idx+idx))) | (mode<<(1+idx+idx)));
	}
	
	/*(non-Javadoc)
	*	method for setting the current
	*@param motorNr
	*	index of the motor (0 to 5)
	*@param a
	*	integer number for the current 
	*	see the documentation of the servo controller
	*/
	public static void setCurrentInt(int motorNr, int a){
		if(a<-0x800) a= -0x800;
		if(a>0x7FF) a= 0x7FF;
		
		switch(motorNr){
		case 0:
			regTX03_Servo0B = (char)(0x2000 | (0x0FFF & a));
			break;
		case 1:
			regTX05_Servo1B = (char)(0x2000 | (0x0FFF & a));
			break;
		case 2:
			regTX07_Servo2B = (char)(0x2000 | (0x0FFF & a));
			break;
		case 3:
			regTX09_Servo3B = (char)(0x2000 | (0x0FFF & a));
			break;
		case 4:
			regTX11_Servo4B = (char)(0x2000 | (0x0FFF & a));
			break;
		case 5:
			regTX13_Servo5B = (char)(0x2000 | (0x0FFF & a));
			break;
		default:
			break;
		}
	}
	
	/*(non-Javadoc)
	*	method for setting the servo controller active
	*@param motorNr
	*	index of the motor (0 to 5)
	*@param state
	*	boolean state, true if activate, false if deactivate
	*/
	public static void activateServo(int motorNr, boolean state){
		if(motorNr >=0 && motorNr <6){
			if(state) regTX01_ServoConf = (char)(regTX01_ServoConf | (1<<motorNr));
			else regTX01_ServoConf = (char)(regTX01_ServoConf & (~(1<<motorNr)));
		}
	}
	
	/*(non-Javadoc)
	*	method for getting info to the voltage limit
	*@param motorNr
	*	index of the motor (0 to 5)
	*@return
	*	true if motor is at voltage limit
	*/
	public static boolean getVoltageLimit(int motor){
		return (regRX01_ServoConf & (1<<(1+motor))) !=0;
	}
	
	/*(non-Javadoc)
	*	method for resetting the watchdog of the servo controller
	*/
	private static boolean val = false;
	private static boolean valHighOK = true;
	private static boolean valLowOK = true;
	private static int cntInit = 0;
	public static void resetWatchdog(){
		
		// checking input
		if(val)
			valHighOK = (regRX00_DIO & 1) !=0;
		else
			valLowOK = (regRX00_DIO & 1) == 0;
		
		// set new output
		if(val)
			regTX00_DIO |= 0x0001;
		else
			regTX00_DIO &= 0xFFFE;

		// toggling output
		val = ! val;
		
		// setting error if not ok, -> use a delay for start up
		if(cntInit<20){
			cntInit++;
		} else if(!(valHighOK && valLowOK)) ErrorHandler.setError(1000);
	}
	
	/*(non-Javadoc)
	*	method for checking the emergency button state
	*@return 
	*		true if emergency button is pressed
	*/
	public static boolean getResetState(){
		return !(valHighOK && valLowOK);
	}
}