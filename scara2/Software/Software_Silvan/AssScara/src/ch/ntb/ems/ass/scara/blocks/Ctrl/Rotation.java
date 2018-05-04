package ch.ntb.ems.ass.scara.blocks.Ctrl;

import ch.ntb.ems.ass.scara.framework.Input;
import ch.ntb.ems.ass.scara.framework.Output;

public class Rotation {
	public Input rotAngle;
		
	public Input x_in;
	public Input y_in;
		
	public Output x_out;
	public Output y_out;
		
	public Rotation(){
		rotAngle = new Input();
		x_in = new Input();
		y_in = new Input();
		x_out = new Output();
		y_out = new Output();
	}
	
	public void run(){
		double phiX = x_in.getValue() * Math.cos(-rotAngle.getValue()) - y_in.getValue() * Math.sin(-rotAngle.getValue());		// angel of pendulum in Cartesian coordinates in X-direction
		double phiY = x_in.getValue() * Math.sin(-rotAngle.getValue()) + y_in.getValue() * Math.cos(-rotAngle.getValue());		// angel of pendulum in Cartesian coordinates in Y-direction
		x_out.setValue(phiX);
		y_out.setValue(-phiY);
		
	}
}
