package ch.ntb.ems.ass.scara.blocks.Ctrl;

import ch.ntb.ems.ass.scara.framework.Input;
import ch.ntb.ems.ass.scara.framework.Output;

public class PIController {
	public Input actualValue = new Input();
	public Input desiredValue = new Input();
	
	public Output controlVariable = new Output();
	
	private double kp,ki,dt;
	private double eInt;
	
	public PIController(double kp, double ki, double dt){
		this.kp = kp;
		this.ki = ki;
		this.dt = dt;
		this.eInt = 0;
	}
	
	public void run(){
		double e = desiredValue.getValue() - actualValue.getValue();
		eInt = eInt + e*dt;
		controlVariable.setValue(e*kp+eInt*ki);
		
	}
	
}
