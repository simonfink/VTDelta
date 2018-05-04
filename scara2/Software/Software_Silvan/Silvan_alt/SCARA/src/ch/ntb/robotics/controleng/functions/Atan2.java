package ch.ntb.robotics.controleng.functions;

public class Atan2 {
	
	/**
	*	calculates the arcustangens2 
	*@param arg1
	*		Y-argument
	*@param arg2
	*		X-argument
	*@return	a real number for 360° and not only for 90°
	*/
	public double run(double arg1, double arg2) {
        if(arg1+arg2 == arg1) {
            if(arg1 > 0)
            return Math.PI/2;
            if(arg1 == 0)
            return 0;
                return -Math.PI/2;
        }
        arg1 = Math.atan(arg1/arg2);
        if(arg2 < 0)
       {
            if(arg1 <= 0)
                return arg1 + Math.PI;
            return arg1 - Math.PI;
        }
        return arg1;
	    
	 }
}
