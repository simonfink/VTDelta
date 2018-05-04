package ch.ntb.ems.ass.scara.blocks.Ctrl;

import ch.ntb.ems.ass.scara.Robot;
import ch.ntb.ems.ass.scara.framework.Input;
import ch.ntb.ems.ass.scara.framework.Output;

public class BarTipPosEstimation {
	private Robot robot;
	
	public Input robotPos;
	
	public Input barPhi;
	
	public Output barTip;
	
	double lstabTwo = 0.324;
	double lstabOne = 0.1;
	
	
	public BarTipPosEstimation(Robot robot){
		this.robot = robot;
		robotPos = new Input();
		
		barPhi = new Input();
		
		barTip = new Output();
	}
	
	public void run(){
		if(robot.barPos.barOneIsOn()){
			barTip.setValue(robotPos.getValue() + lstabOne*Math.sin(barPhi.getValue()));
		}
		if(robot.barPos.barTwoIsOn()){
			barTip.setValue(robotPos.getValue() + lstabTwo*Math.sin(barPhi.getValue()));
		}
	}
}
