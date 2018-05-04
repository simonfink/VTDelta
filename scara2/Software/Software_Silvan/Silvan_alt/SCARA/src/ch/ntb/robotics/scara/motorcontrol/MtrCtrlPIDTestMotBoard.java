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

import ch.ntb.inf.deep.runtime.mpc5200.Task;
import ch.ntb.robotics.controleng.functions.Integrator;

/**********************************************************
 * File:     MtrCtrlPID.java                              
 * Created:  2012-12-06 dfrommelt                         
 * Changes:	 none
 * ------------------------------------------------------ 
 * Description:                                           
 * PID controller for current controlled servomotors                                                                  
 **********************************************************/
public class MtrCtrlPIDTestMotBoard extends MtrCtrlPDTestMotBoard{

	
	double kAcc, kMI;
	
	double diffAWU;
	Integrator intDiffVelo;	

	double maxVal, minVal;
	
	double prevPos=0.0;
	
	private double lastIn, lastOut;
	
	double k;
	
	
	/*(non-Javadoc)
	*	object constructor
	*@param Ts
	*		sampling time, must fit to the frequency of calling the run method
	*@param posCtrl
	*		true ->  the position is controlled 
	*		false -> the velocity is controlled
	*@param mot
	*		number of motor (0 to 5)
	*@param omega
	*		frequency of the control system
	*@param i
	*		gear ratio to the desired variable
	*@param J
	*		Inertia 
	*@param km
	*		motor constant
	*@param maxI
	*		maximal possible current
	*@param gainIout
	*		factor for the current for calculating the integer value for the servo drive
	*@param encTicks
	*		number of encoder ticks
	*/
	public MtrCtrlPIDTestMotBoard(double Ts, boolean posCtrl, int mot, double omega, double i, double J, double km, double maxU, double gainIout, int encTicks, double R) {

		// inputs and outputs
		enc = new Encoder(mot, i, encTicks,4,2000000);
		iOut = new CurrentOut(mot,gainIout);
		
		// control values
		kp = omega*0.5;
		kd = omega*2.0;
		kAcc = 0.5/omega;
		
		//factor for the current
		this.km = km;
		this.i = i;
		this.J = J;
		this.R = R;
		
		this.J = J;
		
		this.k =  0.5*Ts;
		
		//factor for the current
		kMI = i/km;
		
		// limitations 
		this.maxU = maxU;
		this.minU = - maxU;
		this.maxVal = 6/J/kMI/kd;
		this.minVal = -6/J/kMI/kd;
		
		// integrator to control the velocity 
		intDiffVelo = new Integrator(Ts,0.0);
		
		// factor to calculate the speed
		kDiff = 1.0/Ts;

		this.posCtrl = posCtrl;
		
		timeOld = Task.time()/1000f;
	}

	/*(non-Javadoc)
	*	running method -> controls the motor with proportional, derivative and integral controller
	*@param pos
	*		set value for the position of the motor
	*		this array must fit to the documentation:
	*		[p, v, a]
	*/
	public void run(double[] pos) {
			
		newtime = Task.time()/1000f;							// Time in Second
		double dt = newtime - timeOld;
		timeOld = newtime;
		
		prevPos = actPos;	
		actPos = enc.getPos();
		double actVelo = (actPos - prevPos) / dt;
		
		tempOut1.setValue(actPos);
		tempOut2.setValue(actVelo);
			
		if(this.active){
		
			double val = 0;
			
			// proportional controller
			if(this.posCtrl){
				val = pos[0] - actPos;
				val *= kp;
			}
			
			// adding the velocity
			val += pos[1];
		
			// limit the velocity
			if(val>maxVelo) val=maxVelo;
			if(val<minVelo) val=minVelo;
				
			// control the velocity
			val += (pos[2] * kAcc);
			val -= actVelo;
			val *= kd;
			
						
			// integral controller with limit of the value
			double intVal = intDiffVelo.getIntegral(val);
			double valNext = val + intVal - actVelo;
			
			current = intVal;
			
			if(valNext>maxVal) {
				intDiffVelo.setValue(maxVal-val+actVelo);
				tempOut1.setValue(valNext - maxVal);
				valNext = maxVal;
				
			} else if(valNext < minVal){
				intDiffVelo.setValue(minVal-val+actVelo);
				tempOut1.setValue(valNext - minVal);
				valNext = minVal;
			} else tempOut1.setValue(0.0);
			
			
			// setting the output
			val = valNext * kd;
			tempOut3.setValue(val);
			
			val *= J;
			val *= 1/i;
			
			// calculate the current 
			val *= 1/km;
					
			// calculate the voltage
			val = val*R +  pos[1]*i*km;
				
			// limit the voltage
			if(val>maxU) val = maxU;
			if(val<minU) val = minU;
			
			// calculate dutycycle
			val = val/maxU;
			
			// calculate outputvalue between +/- 1791
			val *= 1791;
			
			// setting the current to the output
			iOut.setCurrent(val);

			
		} else {
			iOut.setCurrent(0.0);
			intDiffVelo.setValue(0.0);
			current = 0;
		}
		
	}
}
