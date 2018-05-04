package ch.ntb.ems.ass.scara.blocks.Ctrl;

import ch.ntb.ems.ass.scara.blocks.inOutputs.PWMOut;
import ch.ntb.ems.ass.scara.framework.Input;

public class ColisionProtection {
	public Input actualAngle0;
	public Input actualAngle1;
	
	private PWMOut pwm[];
	private int nrOfPWM;
	private boolean hasFired;
	
	static final double protDist = 0.05;
	static final double maxPhi1 = 225*2*Math.PI/360.0;	// [rad] 
	static final double minPhi1 = -15*2*Math.PI/360.0;	// [rad] 
	static final double maxPhi2 = 195*2*Math.PI/360.0;
	static final double minPhi2 = -45*2*Math.PI/360.0;
	
	public ColisionProtection(PWMOut pwm[], int nrOfPWM){
		actualAngle0 = new Input();
		actualAngle1 = new Input();
		this.nrOfPWM = nrOfPWM;
		this.pwm = pwm;
		this.hasFired = false;
	}
	
	public boolean hasFired(){
		return hasFired;
	}
	
	public void reset(){
		this.hasFired = false;
	}
	public void run(){
		if(	actualAngle0.getValue() >= maxPhi1-protDist |
			actualAngle1.getValue()  >=  maxPhi2-protDist |	
			actualAngle0.getValue() <= minPhi1+protDist	 |
			actualAngle1.getValue()  <= minPhi2+protDist |	
			actualAngle0.getValue() - actualAngle1.getValue()  <= 30*2*Math.PI/360+protDist |	
			actualAngle0.getValue() - actualAngle1.getValue()  >= 150*2*Math.PI/360-protDist
			){
				hasFired = true;
				for(int i= 0;i<nrOfPWM;i++){
					pwm[i].disable();
				}
		}
	}
	
}
