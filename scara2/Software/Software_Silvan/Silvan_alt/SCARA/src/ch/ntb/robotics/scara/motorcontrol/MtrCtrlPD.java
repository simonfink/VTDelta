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

import ch.ntb.robotics.scara.driver.Output;
import ch.ntb.robotics.scara.driver.ScaraDriver;


/**********************************************************
 * File:     MtrCtrlPD.java                              
 * Created:  2012-12-06 dfrommelt                         
 * Changes:	 none
 * ------------------------------------------------------ 
 * Description:                                           
 * PD controller for current controlled servomotors                                                                  
 **********************************************************/
public class MtrCtrlPD {
	
	public Encoder enc;
	public CurrentOut iOut;
	
	double kd, kMI, kp, maxU, minU, maxVelo, minVelo, actVelo, motTorque;
	double kDiff, Ts, dt, val, controlledSpeed;
	double J, i, R, km, current;
	boolean posCtrl;
	
//	double val = 0;
//	private double ctr = 1000;
	
	
	int motNr;
	boolean active, flag;
	
	double actPos=0, prevPos=0;
	final static double k1 = 90000.0;
	final static double k2 = 600.0;
	
	public Output tempOut1 = new Output();
	public Output tempOut2 = new Output();
	public Output tempOut3 = new Output();
	
	double newtime = 0;
	double timeOld = 0; 
	
	double e, epunkt, integratedPos, prevIntegratedPos, desiredPos, desiredSpeed, desiredAcc;
	
	
	/*(non-Javadoc)
	*	empty object constructor (only for subclasses)
	*/
	public MtrCtrlPD(){
	}
	
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
	public MtrCtrlPD(double Ts, boolean posCtrl, int mot, double omega, double i, 
			double J, double km, double maxU, double gainIout, int encTicks, double R) {
		this.Ts = Ts;
		// in and outputs
		enc  = new Encoder(mot, i, encTicks, 4, 2000000);
		iOut = new CurrentOut(mot,gainIout);
		
		// setting the boolean value to also control the position
		this.posCtrl = posCtrl;
		
		// setting the control factors
		this.kp = omega*0.5;
		this.kd = omega*2.0;
		
		// setting the acceleration factor
		this.km = km;
		this.i = i;
		this.J = J;
		this.R = R;
		
		// setting the factor for calculating the velocity out of the position
		kDiff = 1.0/Ts;
		
		// setting the limits
		this.maxU = maxU;
		this.minU = -maxU;
		this.maxVelo = 0.0;
		this.minVelo = 0.0;
		
		// setting the motor number (used for activate the motor)
		this.motNr = mot;
			
	}
	
	/*(non-Javadoc)
	*	method for activating the motor
	*@param state
	*			true  -> motor is set active
	*			false -> motor is set non-active
	*/
	public void activate(boolean state){
		ScaraDriver.activateServo(this.motNr, state);
		this.active = state;
	}
	
	/*(non-Javadoc)
	*	method for setting the velocity maximum and minimum
	*@param maximum
	*			maximal possible velocity
	*/
	public void setVelocity(double maximum){
		this.maxVelo = maximum;
		this.minVelo = -maximum;
	}
	
	/*(non-Javadoc)
	*	method for setting the encoder position
	*@param pos
	*		new position for the encoder
	*/
	public void setEncPos(double pos){
		
		// setting the position for the encoder
		this.enc.setPos(pos);
		
		// setting the actual position for the difference for the velocity
		// otherwise we could get a huge velocity
		this.actPos = pos;
		prevPos = actPos;
		integratedPos = actPos;
		prevIntegratedPos = actPos;
	}

//	public void resetEncPos(){
//		this.enc.reset();
//		this.actPos = 0;
//	}
	
	public void setCtrlValue(double[] pos){
		this.desiredPos = pos[0]; this.desiredSpeed = pos[1]; this.desiredAcc = pos[2];
	}
	
	public void setPosCtrl(){
		this.posCtrl = true;
	}
	
	public void setVeloCtrl(){
//		integratedPos = 0;
		prevIntegratedPos = 0;
		this.posCtrl = false;
	}
	
	
	/*(non-Javadoc)
	*	running method -> controls the motor with proportional and derivative controller
	*@param pos
	*		set value for the position of the motor
	*		this array must fit to the documentation:
	*		[p, v, a]
	*/
	public void run() {
//		if (flag){
//			newtime = Task.time()/1000.0;
//			timeOld = newtime;
//			flag = false;
//		}	
		
//		else{	
//			newtime = Task.time()/1000.0;							// Time in Second
//			dt = newtime - timeOld;
//			timeOld = newtime;
			
			// calculate the velocity
			prevPos = actPos;	
			actPos = enc.getPos();
			actVelo = (actPos - prevPos) * kDiff;
			
			// save the velocity and position to the temporary outputs
	//		tempOut1.setValue(actPos);
	//		tempOut2.setValue(actVelo);
				
			// only control if motor is active
			if(this.active){
				val = 0;
//				current = 0;
				
				// position control
				if(this.posCtrl){		
					val = desiredPos - actPos;
					val *= kp;
					
					// adding the velocity
					val += desiredSpeed;			
					
					// saturate the velocity
					if(val>this.maxVelo) val = this.maxVelo;
					if(val<this.minVelo) val = this.minVelo;
							
					// velocity control
					val -= actVelo;
					val *= kd;
				}
				
				else{				
	//				// adding the velocity
	//				val += desiredSpeed;			
	//				
	//				// saturate the velocity
	//				if(val>this.maxVelo) val = this.maxVelo;
	//				if(val<this.minVelo) val = this.minVelo;
	//						
	//				// velocity control
	//				val -= actVelo;
	//				val *= kd;
					
					// saturate the velocity
					if(desiredSpeed>this.maxVelo) desiredSpeed = this.maxVelo;
					if(desiredSpeed<this.minVelo) desiredSpeed = this.minVelo;
					
					// Integrator
					integratedPos = prevIntegratedPos + desiredSpeed / kDiff;
					
					// Calculate control value
					e = (integratedPos - actPos);					// deltaPos at gear exit
					epunkt = desiredSpeed - actVelo;						// deltaSpeed at gear exit
					
					val = (kp*e + kd*epunkt);
				}
				
				
				// adding the acceleration against contouring error
	//			val += pos[2];
				
				// set the acceleration
	//			tempOut3.setValue(val);
				
				// calculate robot torque
				val *= J;
				
				// calculate motor torque
				val /= i;
				motTorque = val;
				
				// calculate the current 
				val /= km;
							
	//			// limit the current
	//			if(val>maxI) val = maxI;
	//			if(val<minI) val = minI;
				
				// calculate the voltage
				val *= R; 
				val += actVelo*i*km;
				
				current = val;
				
				// limit the voltage
				if(val>this.maxU) val = this.maxU;
				if(val<this.minU) val = this.minU;
				
				// calculate dutycycle
				val /= maxU;
				
				// calculate outputvalue between +/- 1791
				val *= 1791.0;
				
				// setting the current to the output
				iOut.setCurrent(val);
//				current = val;
				prevIntegratedPos = integratedPos;
				prevPos = actPos;
				
			} else {
				// set the current to zero
				iOut.setCurrent(0.0);
				current = 0;
			}	
//		}
	}
	
	


	public double getActVelo(){
//			return current;
			return actVelo;
	}
		
	public double getMotTorque(){
			return motTorque;
//			return actVelo;
	}
		
	public double getMaxVelo(){
			return maxVelo;
	}
				
}

	


	
	
//	public Encoder enc;
//	public CurrentOut iOut;
//	
//	double kd, kMI, kp, maxU, minU, maxVelo, minVelo, maxM, minM, val;
//	double kDiff;
//	double J, km, i, R;
//	double phi1punktpunkt = 0;
//	double phi2punktpunkt = 0;
//	boolean posCtrl;
//	
//	double current, motTorque;
//	
//	int motNr;
//	boolean active;
//	
//	double[] pos = new double[3];
//	
//	double actPos, prevPos;
//	
//	public Output tempOut1 = new Output();
//	public Output tempOut2 = new Output();
//	public Output tempOut3 = new Output();
//	
//	float newtime = 0f;
//	float timeOld = 0f; 
//	double e, epunkt, desiredPos, prevDesiredPos, desiredSpeed;
//		
//	public Dynamic dyn = new Dynamic();
//	InversKinematic invKin = new InversKinematic();
//	InversJakobian invJakob = new InversJakobian();
//		
//	
//	/**
//	*	empty object constructor (only for subclasses)
//	*/
//	public MtrCtrlPD(){
//		
//	}
//	
//	/**
//	*	object constructor
//	*@param Ts
//	*		sampling time, must fit to the frequency of calling the run method
//	*@param posCtrl
//	*		true ->  the position is controlled 
//	*		false -> the velocity is controlled
//	*@param mot
//	*		number of motor (0 to 1)
//	*@param omega
//	*		frequency of the control system
//	*@param i
//	*		gear ratio to the desired variable
//	*@param J
//	*		Inertia 
//	*@param km
//	*		motor constant
//	*@param maxI
//	*		maximal possible current
//	*@param gainIout
//	*		factor for the current for calculating the integer value for the servo drive (S.64 Doku David)
//	*@param encTicks
//	*		number of encoder ticks
//	*@param R
//	*		internal motor resistor
//	*/
//	public MtrCtrlPD(double Ts, boolean posCtrl, int mot, double omega, double i, 
//			double km, double maxU, double gainIout, int encTicks, double R) {
//		
//		// in and outputs
//		enc  = new Encoder(mot, i, encTicks, 4, 160000000);
//		iOut = new CurrentOut(mot,gainIout);
//		
//		// setting the boolean value to also control the position
//		this.posCtrl = posCtrl;
//		
//		// setting the control factors
//		this.kp = omega*0.5;						// kp kaskadiert = omega/(2*D) ; D=1 --> omega*0.5
//		this.kd = omega*2.0;						// kd kaskadiert = omega*2*D; D=1 --> omega*2
//		
//		// setting the acceleration factor
////		this.kMI =  1/(km*i);						// Fehler von David: vorher 1/km*i
//		this.km = km;
//		this.i = i;
////		this.J = J;
//		this.R = R;
//		
//		// setting the factor for calculating the velocity out of the position
//		kDiff = 1.0/Ts;
//		
//		// setting the limits
//		this.maxU = maxU;
//		this.minU = -maxU;
//		this.maxVelo = 0.0;
//		this.minVelo = 0.0;
////		this.maxM = maxI*km;
////		this.minM = -maxM;
//		
//		// setting the motor number (used for activate the motor)
//		this.motNr = mot;
//		
//		timeOld = Task.time()/1000f;
//	}
//	
//	/**
//	*	method for activating the motor
//	*@param state
//	*			true  -> motor is set active
//	*			false -> motor is set non-active
//	*/
//	public void activate(boolean state){
//		ScaraDriver.activateServo(this.motNr, state);
//		this.active = state;
//	}
//	
//	/**
//	*	method for setting the velocity maximum and minimum
//	*@param maximum
//	*			maximal possible velocity
//	*@param minimum
//	*			minimal possible velocity
//	*/
//	public void setVelocity(double maximum, double minimum){
//		this.maxVelo = maximum;
//		this.minVelo = minimum;
//	}
//	
//	
//	/**
//	*	method for setting the encoder position
//	*@param pos
//	*		new position for the encoder
//	*/
//	public void setEncPos(double pos){
//		
//		// setting the position for the encoder
//		this.enc.setPos(pos);
//		
//		// setting the actual position for the difference for the velocity
//		// otherwise we could get a huge velocity
//		this.actPos = pos;
//	}
//
//	public void resetEncPos(){
//		this.enc.reset();
//		this.actPos = 0;
//	}
//	
//	/**
//	*	running method -> controls the motor with proportional and derivative controller
//	*@param pos
//	*		set value for the position of the motor. pos1 for mot1 and pos2 for mot2
//	*		this array must fit to the documentation:
//	*		[q, qpunkt, qpunktpunkt]
//	*@param mot
//	*		which motor should be controlled (different dynamic of mot1 and mot2)
//	*/
//	public void run(double[] pos1, double[] pos2) {
//				
//		// motor controller with Radiant values
//		// differ mot1 and mot2
//		if(this.motNr == 0){
//		pos = pos1;
//		}
//		if(this.motNr == 1){
//		pos = pos2;
//		}
//		
//		newtime = Task.time()/1000f;							// Time in Second
//		double dt = newtime - timeOld;
//		timeOld = newtime;
//		
//		// calculate the velocity
//		prevPos = actPos;	
//		actPos = enc.getPos();
//		double actVelo = (actPos - prevPos)  / dt;
//		
//		// save the velocity and position to the temporary outputs
//		tempOut1.setValue(actPos);
//		tempOut2.setValue(actVelo);
//			
//		// only control if motor is active
//		if(this.active){
//			val = 0;
//			current = 0;
//			
//			// position control
//			if(this.posCtrl){
//				val = pos[0] - actPos;
//				val *= kp;
//				
//				// adding the velocity
//				val += pos[1];
//				
//				// saturate the velocity
//				if(val>this.maxVelo) val = this.maxVelo;
//				if(val<this.minVelo) val = this.minVelo;
//				
//				// velocity control
//				val -= actVelo;
//				val *= kd;
//			}
//			else{	
//				desiredSpeed = pos[1];
//				// saturate the velocity
//				if(desiredSpeed>this.maxVelo) desiredSpeed = this.maxVelo;
//				if(desiredSpeed<this.minVelo) desiredSpeed = this.minVelo;
//				
//				// Integrator
//				desiredPos = prevDesiredPos + desiredSpeed * dt;
//				
//				// Calculate control value
//				e = (desiredPos - actPos);					// deltaPos at gear exit
//				epunkt = desiredSpeed - actVelo;						// deltaSpeed at gear exit
//				
//				val = (kp*e + kd*epunkt);
//			}
//				
//	
//			// adding the acceleration against contouring error
////			val += pos[2];
//			
//			// set the acceleration
//			tempOut3.setValue(val);
//	
//			// calculate the robot torque  (by Silvan)
//			if(this.motNr == 0){
//				phi1punktpunkt = val;
//				val = dyn.getQ1(pos1[0], pos2[0], pos1[1], pos2[1], phi1punktpunkt, pos2[2]);
//			}
//			if(this.motNr == 1){
//				phi2punktpunkt = val;
//				val = dyn.getQ2(pos1[0], pos2[0], pos1[1], pos2[1], pos1[2], phi2punktpunkt);
//			}
//						
//			// calculate motor torque
//			val *= 1/i;
//			motTorque = val;
//			
//			// limit the motor torque
////			if(val>maxM) val = maxM;
////			if(val<minM) val = minM;
//			
//			// calculate the current 
//			val *= 1/km;
//
////			// limit the current
////			if(val>maxI) val = maxI;
////			if(val<minI) val = minI;
//			
//			// calculate the voltage
//			val = val*R +  pos[1]*i*km;
//			
//			// limit the voltage
//			if(val>maxU) val = maxU;
//			if(val<minU) val = minU;
//			
//			// calculate dutycycle
//			val = val/maxU;
//			
//			// calculate outputvalue between +/- 1791
//			val *= 1791;		
//			
//			// setting the current to the output
//			iOut.setCurrent(val);
//			current = val;
//			prevDesiredPos = desiredPos;
//			
//		} else {
//			// set the current to zero
//			iOut.setCurrent(0.0);
//			current = 0;
//		}
//		
//	}
//	
//	public void setPosCtrl(){
//		val = 0;
//		prevDesiredPos = 0;
//		
//		posCtrl = true;
//	}
//	
//
//	
//	public double getPhi1punktpunkt(){
//		return this.phi1punktpunkt;
//	}
//	
//	public double getPhi2punktpunkt(){
//		return this.phi2punktpunkt;
//	}
//	
//	public double getCurrent(){
//		return current;
//	}
//	
//	public double getMotTorque(){
//		return motTorque;
////		return actVelo;
//	}
//}
