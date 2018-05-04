package ch.ntb.ems.ass.scara.blocks.Ctrl;

import ch.ntb.ems.ass.scara.framework.Output;

public class PathPlaner {

	public Output desiredPositionX;
	public Output desiredPositionY;
	public Output desiredVelocityX;
	public Output desiredVelocityY;
	
	
	// vorgaben Bahnplaner
	private final double[] vmax = {0.1, 0.1};				// max linear velocity		5 m/s
	private final double[] amax = {0.5, 0.5};			// max linear acceleration		25 m/s^2
	
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
	private double[] startPos = new double[2];
	private double[] endPos = new double[2];
	private double[] ds = new double[2];
	
	int timer;
	double n;
	
	
	public PathPlaner(double dt){
		desiredPositionX = new Output();
		desiredPositionY = new Output();
		desiredVelocityX = new Output();
		desiredVelocityY = new Output();
		
		this.dt = dt;	

	}

	
	public void precalcs(double[] startPos, double[] endPos){
	
		t_slow = 0;
		
		this.startPos[0] = startPos[0];
		this.startPos[1] = startPos[1];
		this.endPos[0] = endPos[0];
		this.endPos[1] = endPos[1];
		
		ds[0] = endPos[0]-startPos[0];
		ds[1] = endPos[1]-startPos[1];

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
				
		for (int i = 0; i < 2; i++) {  					// time and index of slowest axis
			if (tend[i] > t_slow) {      
				t_slow = tend[i];      
				ind_slow = i;   
			}
		}
		
		ta_slow = ta[ind_slow];										// time for acceleration for all axis
		tc_slow = tc[ind_slow];										// time for constant velocity for all axis
		tend_slow = ta_slow + tc_slow + ta_slow;
		
		for (int i = 0; i < 2; i++){							
			vc[i] = ds[i] / (ta_slow + tc_slow);					// velocity from rounded times
			a[i] = vc[i] / ta_slow;									// acceleration from rounded times
		}
		
		dv[0] = a[0]*dt;											// velocity increment for each axis / dvx
		dv[1] = a[1]*dt;											// dvy
		
		vv[0] = 0;													// start velocity X-axis
		vv[1] = 0;													// start velocity Y-axis
		
		sv[0] = startPos[0];										// start position X-axis
		sv[1] = startPos[1];										// start position Y-axis
	
		s0 = sv;
		v0 = vv;
		
		desiredPositionX.setValue(sv[0]);		desiredVelocityX.setValue(vv[0]);
		desiredPositionY.setValue(sv[1]);		desiredVelocityY.setValue(vv[1]);
		
		z = 0;
	}
	
	
	
	
	/**
	*	running method calculates the actual position and speed periodically
	*/
	public void run(){
		
		if (z == 0){
			s0[0] = startPos[0];										// start position X-axis
			s0[1] = startPos[1];
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
			sv = endPos;
			vv[0] = 0;	vv[1] = 0;
			av[0] = 0;	av[1] = 0;
			z =  (int)(tend_slow/dt) +1;
		}
		
		v0 = vv;
		s0 = sv;
		
		desiredPositionX.setValue(sv[0]);		desiredVelocityX.setValue(vv[0]);
		desiredPositionY.setValue(sv[1]);		desiredVelocityY.setValue(vv[1]);
			
		z++;
		
//		System.out.println(z);

	}
		
	
	public void setMaxValues(double vmax, double amax){
		this.vmax[0] = vmax;
		this.vmax[1] = vmax;
		
		this.amax[0] = amax;
		this.amax[1] = amax;
	}
	
	public void reset(){
		z = 0;
	}
	
}



