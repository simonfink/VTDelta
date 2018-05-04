package ch.ntb.robotics.scara.bahnplaner;

public class BahnplanerPrecalc {

	private final double[] amax = {25, 25};				// max linear acceleration 
	private final double[] vmax = {5, 5};				// max linear velocity
	
	private int nt;
	private double dt, t_slow, ta_slow, tc_slow, tend_slow;
	private int ind_slow;
	private double[] saCalc = new double[2];
	private double[] vc = new double[2];
	private double[] tc = new double[2];
	private double[] ta = new double[2];
	private double[] tend = new double[2];
	private double[] a = new double[2];
	private double[][] dv = new double[2][1];
	private double[][] vv = new double[2][nt];
	private double[][] sv = new double[2][nt];
	private double[][] av = new double[2][nt];
			

	/**
	*	method calculating the positions, velocities and accelerations
	*	this method is for pre calculate a whole path and not for calculating periodical
	*@param start
	*			start position [X, Y]
	*@param ende
	*			end position [X, Y]
	*@param Ts
	*			Sampling time
	*/
	public void run(double[] start, double[] ende, double Ts){
		dt = Ts;
		
		double[] ds = {ende[0]-start[0], ende[1]-start[1]};
		
		for (int i=0; i<1; i++){
			saCalc[i] = vmax[i]*vmax[i]/amax[i];					// distanc to accelerate and break
			
			// vmax reached or not
			if (Math.abs(saCalc[i]) >= Math.abs(ds[i])){			// vmax won't be reached
				vc[i] = Math.sqrt(Math.abs(ds[i])*amax[i]);			// reached velocity
				tc[i] = 0;											// time with constant velocity
			}
			else{
				vc[i] = vmax[i];									// reached constant velocity
				tc[i] = Math.ceil((float) ((Math.abs(ds[i]) - saCalc[i]) / vc[i] / dt) ) * dt; 	// time with constant velocity rounde to next higher dt
			}

			ta[i] = Math.ceil( (float) ((vc[i] / amax[i]) / dt)) *dt;
			tend[i] = (ta[i] + tc[i] + ta[i]);						// total time
		}
		
		for (int i = 0; i < tend.length; i++) {  					// time and index of slowest axis
			if ( tend[i] > t_slow ) {      
				t_slow = tend[i];      
				ind_slow = i;   
			}
		}
		
		ta_slow = ta[ind_slow];										// time for acceleration for all axis
		tc_slow = tc[ind_slow];										// time for constant velocity for all axis
		tend_slow = ta_slow + tc_slow + ta_slow;
		
		for (int i = 0; i < vc.length; i++){							
			vc[i] = ds[i] / (ta_slow + tc_slow);					// velocity from rounded times
			a[i] = vc[i] / ta_slow;									// acceleration from rounded times
		}
		
		nt = (int) (tend_slow * dt);								// number of dt's in tend_slow
		
		dv[0][0] = a[0]*dt;											// velocity increment for each axis
		dv[1][0] = a[1]*dt;
		
		vv[0][0] = 0;												// start velocity X-axis
		vv[1][0] = 0;												// start velocity Y-axis
		
		sv[0][0] = start[0];										// start position X-axis
		sv[1][0] = start[1];										// start position Y-axis
	
		
		// build the position, velocity and acceleration arrays
		for (int i = 1; i < nt; i++ ){							
			if (i-1 <= ta_slow/dt){									// acceleration phase
				vv[0][i] = vv[0][i-1] + dv[0][0];
				vv[1][i] = vv[1][i-1] + dv[1][0];
				
				sv[0][i] = sv[0][i-1] + (vv[0][i] + vv[0][i-1])*dt/2;
				sv[1][i] = sv[1][i-1] + (vv[1][i] + vv[1][i-1])*dt/2;
			}
			
			else if (i-1 <= (ta_slow+tc_slow)/dt){					// phase with constant velocity
				vv[0][i] = vc[0];
				vv[1][i] = vc[1];
			}
			
			else if (i-1 <= tend_slow/dt){							// break phase
				vv[0][i] = vv[0][i-1]-dv[0][0];
				vv[1][i] = vv[1][i-1]-dv[1][0];
				
				sv[0][i] = sv[0][i-1] + (vv[0][i] + vv[0][i-1])*dt/2;
				sv[1][i] = sv[1][i-1] + (vv[1][i] + vv[1][i-1])*dt/2;
			}
			
			av[0][i] = (vv[0][i]-vv[0][i-1])/dt;					// acceleration differentiated by velocity
		}
	}
	
	
	/**
	*@return	array with positions of both axis
	*/
	public double[][] getPosArray(){
		return sv;
	}
	
	/**
	*@return	array with velocities of both axis
	*/
	public double[][] getVelArray(){
		return vv;
	}
	
	/**
	*@return	array with accelerations of both axis
	*/
	public double[][] getAccArray(){
		return av;
	}

}
