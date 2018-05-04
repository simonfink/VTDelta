package ch.ntb.ems.ass.scara.blocks.Ctrl;

import ch.ntb.ems.ass.scara.framework.Input;
import ch.ntb.ems.ass.scara.framework.Output;

public class InvJacobi {
	public Input armAngle0;
	public Input armAngle1;
	public Input robotVelocityX;
	public Input robotVelocityY;
	public Output armVelocity0;
	public Output armVelocity1;
	
	double l = 0.25;
	
	public InvJacobi(){
		armAngle0 = new Input();
		armAngle1 = new Input();
		robotVelocityX = new Input();
		robotVelocityY = new Input();
		armVelocity0 = new Output();
		armVelocity1 = new Output();
	}
	
	
	public void run(){
		double velocity0 = (-Math.cos(armAngle1.getValue())/(l*Math.sin(armAngle1.getValue() - armAngle1.getValue())))*robotVelocityX.getValue() 
				+ (-Math.sin(armAngle1.getValue())/(l*Math.sin(armAngle1.getValue() - armAngle1.getValue())))*robotVelocityY.getValue();
	
		double velocity1 = (Math.cos(armAngle1.getValue())/(l*Math.sin(armAngle1.getValue() - armAngle1.getValue())))*robotVelocityX.getValue() 
				+ (Math.sin(armAngle1.getValue())/(l*Math.sin(armAngle1.getValue() - armAngle1.getValue())))*robotVelocityY.getValue();
	
		armVelocity0.setValue(velocity0);
		armVelocity1.setValue(velocity1);
		
	}
	
}
