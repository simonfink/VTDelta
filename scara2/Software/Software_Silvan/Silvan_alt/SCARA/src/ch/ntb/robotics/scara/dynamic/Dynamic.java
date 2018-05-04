package ch.ntb.robotics.scara.dynamic;

public class Dynamic {

	private final double l = 0.25;
	private final double m11 = 0.44;
	private final double rm11 = 0.025;
	private final double m12 = 0.1;
	private final double rm12 = 0.17;
	private final double m21 = m11;
	private final double rm21 = rm11;
	private final double m22 = 0.063;
	private final double rm22 = 0.145;
	private final double I11z = 0.00211;
	private final double I12z = 0.00085;
	private final double I21z = I11z;
	private final double I22z = 0.00074;
	private final double Jredgetr = 0.017;
	
	/**
	*	dynamic of scara roboter to calculate the desired torque of axis 1
	*@param phi1
	*			position of axis 1
	*@param pos2
	*			position of axis 2
	*@param	phi1punkt
	*			speed of axis 1
	*@param	phi2punkt
	*			speed of axis 2
	*@param	phi1punktpunkt
	*			acceleration of axis 1
	*@param	phi2punktpunkt
	*			acceleration of axis 2
	*@return	calculated torque of axis 1
	*/
	public double getQ1(double phi1, double phi2, double phi1punkt, double phi2punkt, double phi1punktpunkt, double phi2punktpunkt){
		double Q1 = phi1punktpunkt*(m12*l*l + m11*rm11*rm11 + m22*rm22*rm22 + I11z + I22z + Jredgetr) 
		+ l*(phi2punktpunkt*Math.cos(phi1 - phi2) - phi2punkt*Math.sin(phi1 - phi2)*(phi1punkt - phi2punkt))*(m12*rm12 + m22*rm22) 
		+ l*phi1punkt*phi2punkt*Math.sin(phi1 - phi2)*(m12*rm12 + m22*rm22);
		return Q1;
	}
	
	/**
	*	dynamic of scara roboter to calculate the desired torque of axis 2
	*@param phi1
	*			position of axis 1
	*@param pos2
	*			position of axis 2
	*@param	phi1punkt
	*			speed of axis 1
	*@param	phi2punkt
	*			speed of axis 2
	*@param	phi1punktpunkt
	*			acceleration of axis 1
	*@param	phi2punktpunkt
	*			acceleration of axis 2
	*@return	calculated torque of axis 2
	*/
	public double getQ2(double phi1, double phi2, double phi1punkt, double phi2punkt, double phi1punktpunkt, double phi2punktpunkt){
		double Q2 = phi2punktpunkt*(m22*l*l + m12*rm12*rm12 + m21*rm21*rm21 + I12z + I21z + Jredgetr) 
		+ l*(phi1punktpunkt*Math.cos(phi1 - phi2) - phi1punkt*Math.sin(phi1 - phi2)*(phi1punkt - phi2punkt))*(m12*rm12 + m22*rm22) 
		- l*phi1punkt*phi2punkt*Math.sin(phi1 - phi2)*(m12*rm12 + m22*rm22);
		return Q2;
	}
}
