package ch.ntb.robotics.scara.bahnplaner;


public class Bahnplaner2 {

	// vorgaben Bahnplaner
	private final double[] vmax = {20, 20};				// max linear velocity		5 m/s
	private final double[] amax = {100, 100};			// max linear acceleration		25 m/s^2
	
	
	private int z = 0;
	private double dt, t_slow, ta_slow, tc_slow, tend_slow;
	private int ind_slow;
	private double[] saCalc = new double[2];
	private double[] vc = new double[2];
	private double[] tc = new double[2];
	private double[] ta = new double[2];
	private double[] tend = new double[2];
	private double[] a = new double[2];
	private double[] dv = new double[2];
	private double[] av = new double[2];
	private double[] vv = new double[2];
	private double[] sv = new double[2];
	private double[] v0 = new double[2];
	private double[] s0 = new double[2];
	private double[] pos1 = new double[3];
	private double[] pos2 = new double[3];
	private double[] start = new double[2];
	private double[] ende = new double[2];
	
//	private double[] phi = new double[2];
//	private double[] omega = new double[2];
//	private double[] val1 = new double[3];
//	private double[] val2 = new double[3];

	private double[] ds = new double[2];
	
	int timer;
	double n;
	
	/**
	*	calculates the times for the Bahnplaner once
	*@param start
	*		start position [X, Y] 
	*@param ende
	*		end position [X, Y] 
	*@param	Ts
	*		sampling time (must fit with the task-sampling-time)
	*/
	public Bahnplaner2(final double[] start, final double[] ende, double Ts) {
		this.dt = Ts;
		this.ende[0] = ende[0];
		this.ende[1] = ende[1];
		this.start[0] = start[0];
		this.start[1] = start[1];
		
		ds[0] = ende[0]-start[0];
		ds[1] = ende[1]-start[1];
		
		
		
		for (int i=0; i<2; i++){
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
		
		dv[0] = a[0]*dt;											// velocity increment for each axis / dvx
		dv[1] = a[1]*dt;											// dvy
		
		vv[0] = 0;												// start velocity X-axis
		vv[1] = 0;												// start velocity Y-axis
		
		sv[0] = start[0];										// start position X-axis
		sv[1] = start[1];										// start position Y-axis
	
		s0 = sv;
		v0 = vv;
	}

	
	
	/**
	*	running method calculates the actual position and speed periodically
	*/
	public void run(){
		
		if (z == 0){
			s0[0] = start[0];										// start position X-axis
			s0[1] = start[1];
			vv[0] = 0;
			vv[1] = 0;
		}
		else if (z-1 < ta_slow/dt){	
			vv[0] = v0[0] + dv[0];
			vv[1] = v0[1] + dv[1];
			
			sv[0] = s0[0] + (vv[0] + v0[0])*dt/2;
			sv[1] = s0[1] + (vv[1] + v0[1])*dt/2;
			
			av[0] = a[0];
			av[1] = a[1];
		}
		else if (z-1 < (ta_slow + tc_slow)/dt){
			vv = vc;
			sv[0] = s0[0] + vv[0]*dt;
			sv[1] = s0[1] + vv[1]*dt;
			
			av[0] = 0;
			av[1] = 0;
			
		}
		else if (z-1 < tend_slow/dt){
			vv[0] = v0[0] - dv[0];
			vv[1] = v0[1] - dv[1];
			
			sv[0] = s0[0] + (vv[0] + v0[0])*dt/2;
			sv[1] = s0[1] + (vv[1] + v0[1])*dt/2;
			
			av[0] = -a[0];
			av[1] = -a[1];
		}
		else {
			sv = ende;
			vv[0] = 0;	vv[1] = 0;
			av[0] = 0;	av[1] = 0;
			z =  (int)(tend_slow/dt) +1;
		}
		
		v0 = vv;
		s0 = sv;
		
		pos1[0] = sv[0];		pos1[1] = vv[0];		pos1[2] = av[0];
		pos2[0] = sv[1];		pos2[1] = vv[1];		pos2[2] = av[1];
	
		
//		//transform from Cartesian to Radiant
//		invKin.run(pos1[0], pos2[0]);					// calculate phi1,2 with inverted Kinematic
//		phi = invKin.getPhi();							// returns phi1 and phi2
//		
//		invJakob.run(pos1[1], pos2[1]);							// calculate omega1, 2 with the inverted Jakobian
//		omega = invJakob.getOmega();					// returns omega1 and omega2
//		
//		val1[0] = phi[0];		val1[1] = omega[0];		val1[2] = 0;
//		val2[0] = phi[1];		val2[1] = omega[1];		val2[2] = 0;
		
		z++;
	}
	
	public void reset(){
		z = 0;
	}
	
	public void setMaxValues(double vmax, double amax){
		this.vmax[0] = vmax;
		this.vmax[1] = vmax;
		
		this.amax[0] = amax;
		this.amax[1] = amax;
	}
	
	/**
	*@return	array for axis 1 with [position, velocity, acceleration] which is needed by the motor controller
	*/
	public double[] getVal1(){
		return pos1;
	}
	
	/**
	*@return 	array for axis 2 with [position, velocity, acceleration] which is needed by the motor controller
	*/
	public double[] getVal2(){
		return pos2;
	}
	
	public double[] getDv(){
		return dv;
	}
	public double getTslow(){
//		return ta_slow;
//		return saCalc[1];
		return n;
	}
	
}
