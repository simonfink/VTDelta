package ch.ntb.ems.ass.scara.blocks.Ctrl;

import ch.ntb.ems.ass.scara.framework.Input;
import ch.ntb.ems.ass.scara.framework.Output;

public class InvKinematic {
	public Input armPosX;
	public Input armPosY;
	
	public Output phyArmX;
	public Output phyArmY;
	
	private Atan2 atan2;
	private Acos acos;
	
	double l = 0.25;
	
	public InvKinematic(){
		armPosX = new Input();
		armPosY = new Input();

		phyArmX = new Output();
		phyArmY = new Output();
		atan2 = new Atan2();
		acos = new Acos();
	}
	
	public void run(){
		double x = armPosX.getValue();
		double y = armPosY.getValue();
		double phi1 = atan2.run(y,x) + 1/2.0*acos.run((x*x+y*y)/(2.0*l*l)-1);		
		double phi2 =	atan2.run(y,x) - 1/2.0*acos.run((x*x+y*y)/(2.0*l*l)-1);	
		
		if(phi1 < -15*2*Math.PI/360){		// um die x-Achse führen sehr kleine y-Positionen zu 2*PI gedrehten Winkeln
			phi1 += Math.PI;
		}
		if(phi2 < -45*2*Math.PI/360){
			phi2 += Math.PI;
		}
		
		phyArmX.setValue(phi1);
		phyArmY.setValue(phi2);
	}
	
	

}
