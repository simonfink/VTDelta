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

package ch.ntb.robotics.omnidir.savety;

import ch.ntb.robotics.omnidir.driver.OmniDirDriver;

/**********************************************************
 * File:     ErrorHandler.java                              
 * Created:  2013-01-20 dfrommelt       
 * Changes:	 none                  
 * ------------------------------------------------------ 
 * Description:                                           
 * Handles all errors of the omnidir project          
 * checks if the error is hard (faild init), normal or
 *    the error stops driving of the platform                      
 *                                                                	  
 *          
 **********************************************************/
public class ErrorHandler{
	private static boolean errorActive = false;
	private static boolean errorActiveDrive = false;
	private static boolean errorActiveHard = false;
	private static int[] errorNrs = new int[20];
	private static int errorIdx = 0;
	
	/*(non-Javadoc)
	*	method for setting an error
	*@param errorNr
	*			error code of the active error
	*			refer to the printError method
	*/
	public static void setError(int errorNr){
		for(int i=0; i<errorIdx; i++){
			if(errorNrs[i] == errorNr) return;
		}
		if(errorIdx<20){
			errorNrs[errorIdx] = errorNr;
			errorIdx ++;
		}
		if(errorNr <5000)  errorActive = true;
		errorActiveDrive = true;
		if(errorNr >7000) errorActiveHard = true;
		
	}
	
	/*(non-Javadoc)
	*	clears all errors if no hard error is active (not wrong init)
	*/
	public static void clearError(){
		if(!errorActiveHard){
			errorIdx = 0;
			errorActive = false;
			errorActiveDrive = false;
		}
	}
	
	/*(non-Javadoc)
	*	check if an error is active
	*@return
	*	true, if an error is active
	*/
	public static boolean isErrorActive(){
		return errorActive;
	}
	
	/*(non-Javadoc)
	*	check if an error that restricts the platform to drive
	*/
	public static boolean isErrorActiveDrive(){
		return errorActiveDrive;
	}
	
	/*(non-Javadoc)
	*	checks if an error on the init of the laser is pending
	*/
	public static boolean isErrorOnLaser(){
		boolean val = false;
		for(int i=0; i<errorIdx; i++){
			if(errorNrs[i]>=7000 && errorNrs[i]<7000) val = true;
		}
		return val;
	}
	
	/*(non-Javadoc)
	*	check the inputs of wrong values
	*/
	public static void checkInputs(){
		for(int i=0; i<6; i++){
			if(OmniDirDriver.getADCUltras(i) < 100) setError(6100 + i);
			else if(OmniDirDriver.getADCUltras(i) < 3700) setError(5100 + i);
		}
	
		//if(OmniDirDriver.getBattVoltage()<23.0) setError(1001);
	}
	
	/*(non-Javadoc)
	*	print all pending errors
	*/
	public static void printError(){
		System.out.print('\n');
		if(!errorActiveDrive) System.out.print("     NO ERROR PENDING");
		else{
			for(int i=0; i<errorIdx; i++){
				System.out.print("     ERROR ");
				System.out.print(errorNrs[i]);
				System.out.print(": ");
				switch(errorNrs[i]){
				case 1000:
					System.out.print("EMERGENCY STOP PRESSED");
					break;
				case 1001:
					System.out.print("BATTERY VOLTAGE TOO LOW: ");
					System.out.print(OmniDirDriver.getBattVoltage());
					System.out.print("Volt");
					break;
				case 2000:
					System.out.print("TIMEOUT ON TRANSCEIVING ON SPI");
					break;
				case 2100:
					System.out.print("FAULT IN TRANSCEIVING DATA FROM SERVO DRIVE");
					break;
				case 5000:
					System.out.print("OBJECT FOUND WITH LASER SENSOR TO NEAR TO DRIVE");
					break;
				case 5100:
					System.out.print("OBJECT FOUND WITH ULTRASONIC 0 TO NEAR TO DRIVE");
					break;
				case 5101:
					System.out.print("OBJECT FOUND WITH ULTRASONIC 1 TO NEAR TO DRIVE");
					break;
				case 5102:
					System.out.print("OBJECT FOUND WITH ULTRASONIC 2 TO NEAR TO DRIVE");
					break;
				case 5103:
					System.out.print("OBJECT FOUND WITH ULTRASONIC 3 TO NEAR TO DRIVE");
					break;
				case 5104:
					System.out.print("OBJECT FOUND WITH ULTRASONIC 4 TO NEAR TO DRIVE");
					break;
				case 5105:
					System.out.print("OBJECT FOUND WITH ULTRASONIC 5 TO NEAR TO DRIVE");
					break;
				case 6000:
					System.out.print("TIMEOUT TRANSCEIVING LASER");
					break;
				case 6100:
					System.out.print("ULTRASONIC SENSOR 0 NOT CONNECTED!");
					break;
				case 6101:
					System.out.print("ULTRASONIC SENSOR 1 NOT CONNECTED!");
					break;
				case 6102:
					System.out.print("ULTRASONIC SENSOR 2 NOT CONNECTED!");
					break;
				case 6103:
					System.out.print("ULTRASONIC SENSOR 3 NOT CONNECTED!");
					break;
				case 6104:
					System.out.print("ULTRASONIC SENSOR 4 NOT CONNECTED!");
					break;
				case 6105:
					System.out.print("ULTRASONIC SENSOR 5 NOT CONNECTED!");
					break;
				case 6200:
					System.out.print("FAULT IN TRANSCEIVING WITH HALL SENSOR");
					break;
				case 6300:
					System.out.print("LASER SENSOR DID NOT ANSWER ");
					break;
				case 6301:
					System.out.print("WRONG ANSWER FROM LASER SENSOR");
					break;
				case 6302:
					System.out.print("WRONG ANSWER FROM LASER SENSOR");
					break;
				case 6303:
					System.out.print("WRONG ANSWER FROM LASER SENSOR");
					break;
				case 7000:
					System.out.print("INITIALITION OF LASER FAILED");
					break;
				case 7001:
					System.out.print("INITIALITION OF LASER FAILED");
					break;
				case 7002:
					System.out.print("INITIALITION OF LASER FAILED");
					break;
				case 7003:
					System.out.print("INITIALITION OF LASER FAILED");
					break;
				default:
					System.out.print("UNKNOWN ERROR!!!");
				}
				System.out.print('\n');
			}
		}
	}
}