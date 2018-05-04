package ch.ntb.robotics.scara.kinematic;

import ch.ntb.robotics.scara.motorcontrol.MtrCtrlPD;

public class Kinematic {

	private final double l = 0.25;
	double[] posXY = new double[2];
	
	public MtrCtrlPD mot1;
	public MtrCtrlPD mot2;

	
	/**
	*	object constructor
	*/
	public Kinematic() {
	}
	
	
	/**
	*	method to get the actual position of TCP, calculated with the kinematic of SCARA robot
	*@return	position of TCP in Cartesian coordinates
	*/
	public double[] getActualPosXY(){
		double phi1 = mot1.enc.getPos();
		double phi2 = mot2.enc.getPos();
		
		double posX = l*(Math.cos(phi1) + Math.cos(phi2));
		double posY = l*(Math.sin(phi1) + Math.sin(phi2));
		
		posXY[0] = posX;
		posXY[1] = posY;
		
		return posXY;
	}
	
}
