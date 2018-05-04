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

package ch.ntb.robotics.controleng.functions;

import ch.ntb.robotics.scara.driver.Input;
import ch.ntb.robotics.scara.driver.Output;
/**********************************************************
 * File:     Integrator.java                              
 * Created:  2012-10-07 dfrommelt                         
 * Changes:	 none    
 * ------------------------------------------------------ 
 * Description:                                           
 * all-purpose integrator block                           
 *                                                        
 *       		   +-----------------------+		  	 
 *       		   |					   |         	  
 *         	       >- in              out ->         	  
 *     		       |                       |         	  
 *          	   +-----------------------+         	 
 *                                                        
 **********************************************************/
public class Integrator {
	public Input in = new Input();
	public Output out;
	
	private double lastIn, lastOut;
	private double k;
	
	/*(non-Javadoc)
	*	object constructor
	*@param Ts
	*		Sampling time, the frequency of calling the run method must fit with this sampling time
	*/
	public Integrator(double Ts, double val0) {
		out = new Output();
		out.setValue(val0);
		lastOut = val0;
		k = 0.5*Ts;
	}

	/*(non-Javadoc)
	*	method for setting the actual value of the integrator
	*@param val
	*			value to be taken as actual value for the integrator
	*/
	public void setValue(double val){
		lastOut = val;
	}
	
	/*(non-Javadoc)
	*	running method without using the Input and Output 
	*@param in
	*			value to be integrated
	*@return
	*			integrated value
	*/
	public double getIntegral(double in){
		lastOut += (in + lastIn) * k;
		lastIn = in;
		return lastOut;
	}
	
	/*(non-Javadoc)
	*	running method -> calculates the integral value from the input and saves it in the output
	*/
	public void run() {
			if(Math.abs(in.getValue())<1000.0){
			lastOut += (in.getValue() + lastIn)*k;
			out.setValue(lastOut);
			lastIn = in.getValue();
			}
	}
}
