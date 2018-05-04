package ch.ntb.robotics.scara.dynamic;

import ch.ntb.robotics.scara.motorcontrol.MtrCtrlPD;

public class InversJakobian {
	private double omega1, omega2;
	double[] omega = new double[2];
	private final double l = 0.25;
	
	public MtrCtrlPD mot1;
	public MtrCtrlPD mot2;
	
	/**
	*	object constructor
	*/
	public InversJakobian(){
	}
	
	/**
	*	running method to get the values calculated with the invers Jakobian maxtrix
	*@param vx
	*			speed in x-direction (Cartesian)
	*@param vy
	*			speed in y-direction (Cartesian)
	*/
	public void run(double vx, double vy){
		this.omega1 = (-Math.cos(mot2.enc.getPos())/(l*Math.sin(mot1.enc.getPos() - mot2.enc.getPos())))*vx 
					+ (-Math.sin(mot2.enc.getPos())/(l*Math.sin(mot1.enc.getPos() - mot2.enc.getPos())))*vy;
		
		this.omega2 = (Math.cos(mot1.enc.getPos())/(l*Math.sin(mot1.enc.getPos() - mot2.enc.getPos())))*vx 
					+ (Math.sin(mot1.enc.getPos())/(l*Math.sin(mot1.enc.getPos() - mot2.enc.getPos())))*vy;
		
	}
	
	/**
	*@return qPunkt = omega of axis 1 and axis 2
	*/
	public double[] getOmega(){
		omega[0] = this.omega1;
		omega[1] = this.omega2;
		
		return omega;
	}
}
