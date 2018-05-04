package ch.ntb.robotics.scara.main;

import ch.ntb.robotics.scara.driver.ScaraDriver;
import ch.ntb.robotics.scara.motorcontrol.MtrCtrlPD;

public class Stop {
	// values for pos[0-2] for PD-controller
	private double[] pos = {0, 0, 0};		
	
	MtrCtrlPD mot1;
	MtrCtrlPD mot2;
	
	/**
	*	object constructor
	*@param	Ts
	*			sampling time
	*/
	public Stop(){		
		}
	
	/**
	*	running stop method
	*@param	max1
	*			maximal values for axis 1 [phiMax, omegaMax, alphaMax]
	*@param	max2
	*			maximal values for axis 2 [phiMax, omegaMax, alphaMax]
	*/
	public boolean run(double maxOmega){
		ScaraDriver.setLED(0,2);				// toggle all LEDs fast --> attention please, keep away
		ScaraDriver.setLED(1,3);
		ScaraDriver.setLED(2,2);
		ScaraDriver.setLED(3,3);
		
		mot1.activate(false);
		mot2.activate(false);
		
//		mot1.setVelocity(maxOmega);
//		mot2.setVelocity(maxOmega);
//				
//		mot1.setCtrlValue(pos);
//		mot2.setCtrlValue(pos);
		
		if(mot1.getActVelo() == 0 && mot2.getActVelo() == 0){ // wait for button push when robot stoped
			return true;
		}
		else{
			return false;
		}
	}
}
