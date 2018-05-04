package ch.ntb.robotics.scara.kinematic;

import ch.ntb.robotics.controleng.functions.Acos;
import ch.ntb.robotics.controleng.functions.Atan2;

public class InversKinematic {
	private double phi1, phi2;
	private final double l = 0.25;
	
	Acos acos = new Acos();
	Atan2 atan2 = new Atan2();
	
	
	public InversKinematic() {
	}
	
	
	/**
	*	running method to get the values calculated with the inverse kinematic of SCARA robot
	*@param x
	*			position in x-direction (Cartesian)
	*@param y
	*			position in y-direction (Cartesian)
	*/
	public void run(double x, double y){
		this.phi1 = atan2.run(y,x) + 1/2.0*acos.run((x*x+y*y)/(2.0*l*l)-1);		
		this.phi2 =	atan2.run(y,x) - 1/2.0*acos.run((x*x+y*y)/(2.0*l*l)-1);	
		
		if(phi1 < -15*2*Math.PI/360){		// um die x-Achse führen sehr kleine y-Positionen zu 2*PI gedrehten Winkeln
			phi1 += Math.PI;
		}
		if(phi2 < -45*2*Math.PI/360){
			phi2 += Math.PI;
		}
	}
	
	
	/**
	*@return q = phi of axis 1 and axis 2
	*/
	double[] phi = {this.phi1, this.phi2};
	
	public double[] getPhi(){
		phi[0] = phi1;
		phi[1] = phi2;
		
		return phi;
	}
}
