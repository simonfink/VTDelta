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

package ch.ntb.robotics.scara.motorcontrol;

import ch.ntb.robotics.scara.driver.Input;
import ch.ntb.robotics.scara.driver.ScaraDriver;


/**********************************************************
 * File:     CurrentOut.java                              
 * Created:  2011-10-07 dfrommelt                         

 * ------------------------------------------------------ 
 * Description:                                           
 * driver for OmniDir to set the current of the motor                                  
 *                                                        
 *       		   +-----------------------+		  	  
 *       		   |					   |         	  
 *         	       >- in              	   |        	  
 *     		       |                       |         	 
 *          	   +-----------------------+         	  
 *                                                        
 **********************************************************/
public class CurrentOut {
	public Input in = new Input();
	
	private int motNr;
	double k;

	/**
	*	object constructor
	*@param motNr
	*				index of the motornr (0 to 1)
	*@parm k
	*				gain for calculating the integer value from the current
	*/
	public CurrentOut(int motNr, double k) {
		this.motNr=motNr;
		this.k = k;
	}

	/**
	*	running-method without using the input
	*@param value
	*			current value in Ampere
	*/
	public void setCurrent(double val){
		
		ScaraDriver.setCurrentInt(this.motNr,(int)(val*this.k));
		
	}
	
	/**
	*	running-method with using the input
	*/
	public void run() {
		int val = (int)(in.getValue() * this.k);
		
		ScaraDriver.setCurrentInt(this.motNr,val);
		
	}
}
