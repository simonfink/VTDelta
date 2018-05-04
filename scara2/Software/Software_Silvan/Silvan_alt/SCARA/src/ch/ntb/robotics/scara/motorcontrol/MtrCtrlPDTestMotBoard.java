package ch.ntb.robotics.scara.motorcontrol;

import ch.ntb.inf.deep.runtime.mpc5200.Task;
import ch.ntb.robotics.scara.driver.Output;
import ch.ntb.robotics.scara.driver.ScaraDriver;


public class MtrCtrlPDTestMotBoard {
	
	
	public Encoder enc;
	public CurrentOut iOut;
	
	double kd, kMI, kp, maxU, minU, maxVelo, minVelo, actVelo, motTorque;
	double kDiff, Ts;
	double J, i, R, km, current;
	boolean posCtrl;
	
	double val = 0;
	double oldVal = 0;
//	private double ctr = 1000;
	
	
	int motNr;
	boolean active;
	
	double actPos=0, prevPos=0;
	final static double k1 = 90000.0;
	final static double k2 = 600.0;
	
	public Output tempOut1 = new Output();
	public Output tempOut2 = new Output();
	public Output tempOut3 = new Output();
	
	float newtime = 0f;
	float timeOld = 0f; 
	
	double e, epunkt, integratedPos, prevIntegratedPos, desiredPos, desiredSpeed, desiredAcc;
	
	
	/*(non-Javadoc)
	*	empty object constructor (only for subclasses)
	*/
	public MtrCtrlPDTestMotBoard(){
		
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
	public MtrCtrlPDTestMotBoard(double Ts, boolean posCtrl, int mot, double omega, double i, 
			double J, double km, double maxU, double gainIout, int encTicks, double R) {
		
		// in and outputs
		enc  = new Encoder(mot, i, encTicks, 4, 2000000);
		iOut = new CurrentOut(mot,gainIout);
		
		// setting the boolean value to also control the position
		this.posCtrl = posCtrl;
		
		// setting the control factors
		this.kp = omega*0.5f;
		this.kd = omega*2.0f;
		
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
		
		timeOld = Task.time()/1000f;
		
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
	*@param minimum
	*			minimal possible velocity
	*/
	public void setVelocity(double maximum, double minimum){
		this.maxVelo = maximum;
		this.minVelo = minimum;
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
		integratedPos = actPos;
		prevIntegratedPos = actPos;
	}

	public void resetEncPos(){
		this.enc.reset();
		this.actPos = 0;
	}
	
	/*(non-Javadoc)
	*	running method -> controls the motor with proportional and derivative controller
	*@param pos
	*		set value for the position of the motor
	*		this array must fit to the documentation:
	*		[p, v, a]
	*/
	public void run(double[] pos) {
	
		desiredPos = pos[0]; desiredSpeed = pos[1]; desiredAcc = pos[2];
		
		newtime = Task.time()/1000f;							// Time in Second
		double dt = newtime - timeOld;
		timeOld = newtime;
		
		// calculate the velocity
		prevPos = actPos;	
		actPos = enc.getPos();
		actVelo = (actPos - prevPos) / dt;
		
		// save the velocity and position to the temporary outputs
		tempOut1.setValue(actPos);
		tempOut2.setValue(actVelo);
			
		// only control if motor is active
		if(this.active){
			val = 0;
			current = 0;
			
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
				// saturate the velocity
				if(desiredSpeed>this.maxVelo) desiredSpeed = this.maxVelo;
				if(desiredSpeed<this.minVelo) desiredSpeed = this.minVelo;
				
				// Integrator
				integratedPos = prevIntegratedPos + desiredSpeed * dt;
				
				// Calculate control value
				e = (integratedPos - actPos);					// deltaPos at gear exit
				epunkt = desiredSpeed - actVelo;						// deltaSpeed at gear exit
				
				val = (kp*e + kd*epunkt);
			}
			
			
			// adding the acceleration against contouring error
//			val += pos[2];
			
			// set the acceleration
			tempOut3.setValue(val);
			
			// calculate robot torque
			val *= J;
			
			// calculate motor torque
			val *= 1/i;
			motTorque = val;
			
			// calculate the current 
			val *= 1/km;
						
//			// limit the current
//			if(val>maxI) val = maxI;
//			if(val<minI) val = minI;
			
			// calculate the voltage
			val = val*R +  desiredSpeed*i*km;
			
			// limit the voltage
			if(val>maxU) val = maxU;
			if(val<minU) val = minU;
			
			// calculate dutycycle
			val = val/maxU;
			
			// calculate outputvalue between +/- 1791
			val *= 1791;
			
			// setting the current to the output
			iOut.setCurrent(val);
			current = val;
			prevIntegratedPos = integratedPos;
			
		} else {
			// set the current to zero
			iOut.setCurrent(0.0);
			current = 0;
		}	
	}
	
		public void setPosCtrl(){
			this.posCtrl = true;
		}
		
		public double getActVelo(){
//			return current;
			return actVelo;
		}
	
		public double getCurrent(){
			return current;
//			return actVelo;
		}
		
		public double getMotTorque(){
			return motTorque;
//			return actVelo;
		}
		
		public double getMaxVelo(){
			return maxVelo;
		}
				
}

	

