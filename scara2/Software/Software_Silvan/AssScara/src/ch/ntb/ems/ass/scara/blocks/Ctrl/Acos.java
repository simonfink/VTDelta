package ch.ntb.ems.ass.scara.blocks.Ctrl;

public class Acos {

	/**
	*	calculates the arcuscosinus
	*@param x
	*		angel in radiant
	**@return	acos
	*/
	 public double run(double x){
	    	//http://www.visiblevisual.com/jupgrade/index.php/autocad-vb-vba/75-arctan-arccos-arcsin
	    	
	    	double ac = 0;
	    	
	    	if (x == 1){
	    		ac = 0;
	    	}
	    	if (x == 0){
	    		ac = Math.PI/2;
	    	}
	    	
	    	else{
	    		ac = Math.atan(-x / Math.sqrt(-x*x+1)) + Math.PI/2;
	    	}
	    	
	    	return ac;
	    }
	
}
