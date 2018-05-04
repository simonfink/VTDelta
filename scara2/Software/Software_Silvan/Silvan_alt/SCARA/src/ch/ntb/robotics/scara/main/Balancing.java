package ch.ntb.robotics.scara.main;

import ch.ntb.robotics.scara.driver.ScaraDriver;
import ch.ntb.robotics.scara.kinematic.Kinematic;
import ch.ntb.robotics.scara.motorcontrol.MtrCtrlPD;

public class Balancing {
	
	MtrCtrlPD mot1;
	MtrCtrlPD mot2;
	Kinematic kin;	
	
	double[] balPosPtip = new double[2];
	double[] valDiff = new double[2];
	double[] valPosAct = new double[2];
	double[] valPosPrev = new double[2];
	double[] valPhiAct = new double[2];
	double[] valPhiPrev = new double[2];
	double[] accAct = new double[2];
	double[] accPrev = new double[2];
	double[] v = new double[2];
	double[] vPrev = new double[2];
	double[] pos = new double[2];
	double[] pPrev = new double[2];
	double[] val1 = new double[3];
	double[] val2 = new double[3];
	double[] phiC = new double[2];
	double[] posPT = new double[2];
	double[] phiPendCart = new double[2];
	double[] posTCP = new double[2];
	double[] phiPcart = new double[2];
	
	double lstab = 0.324;
	double g = 9.81;
	double d = 0.138;
	double Jstab = 0.00038;
	double mstab = 0.037;
	double r = Math.sqrt(Jstab/mstab);
	
	double[] val = {0, 0};
	double dt;
	
	// PD-controller parameters
	double kpPosPtip = 4*4;				// omega0 ^2
	double kdPosPtip = 2*5*1;			// 2*omega0*D
	
	double kpPhiPend = 20*20;
	double kdPhiPend = 2*1*1;
	
	/**
	*	object constructor
	*@param	balPos
	*			position where the pendulum shall stay
	*@param	maxOmega
	*			maximal Speed of axis 1 and axis 2
	*@param	Ts
	*			sampling time
	*/
	public Balancing(double[] balPos, double Ts){
		this.dt = Ts;
		
		this.balPosPtip[0] = balPos[0];
		this.balPosPtip[1] = balPos[1];
	}
	
	
	/**
	*	running method to balance the pendulum
	*/
	public void run(){
		
		phiPendCart[0] = getPhiPendCartcoord()[0];
		phiPendCart[1] = getPhiPendCartcoord()[1];
		
		valPosAct[0] = balPosPtip[0] - getPosPendTip()[0];
		valPosAct[1] = balPosPtip[1] - getPosPendTip()[1];
			
		val[0] = valPosAct[0]*kpPosPtip + (valPosAct[0] - valPosPrev[0]) / dt * kdPosPtip;		// PD-controller for tip-position pendulum
		val[1] = valPosAct[1]*kpPosPtip + (valPosAct[1] - valPosPrev[1]) / dt * kdPosPtip;				
		valPosPrev[0] = valPosAct[0];															// save Prev values for next differentiation / integration
		valPosPrev[1] = valPosAct[1];
		
		val[0] = Math.atan(val[0] / g);
		val[1] = Math.atan(val[1] / g);
		
		valPhiAct[0] = val[0] - phiPendCart[0];
		valPhiAct[1] = val[1] - phiPendCart[1];
		
		val[0] = valPhiAct[0]*kpPhiPend + (valPhiAct[0] - valPhiPrev[0]) / dt * kdPhiPend;			// PD-controller for alpha phi pendulum
		val[1] = valPhiAct[1]*kpPhiPend + (valPhiAct[1] - valPhiPrev[1]) / dt * kdPhiPend;	
		valPhiPrev[0] = valPhiAct[0];
		valPhiPrev[1] = valPhiAct[1];
		
		// Pendulum equations; returns desired position and velocity for robot controller
		accAct[0] = - val[0] * (d*d + r*r) / d / Math.cos(phiPendCart[0]) + g * Math.tan(phiPendCart[0]);		
		accAct[1] = - val[1] * (d*d + r*r) / d / Math.cos(phiPendCart[1]) + g * Math.tan(phiPendCart[1]);
		
		v[0] = vPrev[0] + accAct[0]*dt ; 												// desired robot speed Cartesian by integration
		v[1] = vPrev[1] + accAct[1]*dt ;
		vPrev[0] = v[0];
		vPrev[1] = v[1];
		
		pos[0] = pPrev[0] + v[0]*dt;														// desired robot position Cartesian by integration
		pos[1] = pPrev[1] + v[1]*dt;
		pPrev[0] = pos[0];
		pPrev[1] = pos[1];
		
		val1[0] = pos[0];  val1[1] = v[0]; val1[2] =  0;
		val2[0] = pos[1];  val2[1] = v[1]; val2[2] =  0;
		
		mot1.setCtrlValue(val1);
		mot2.setCtrlValue(val2);
				
	}
	
	/**
	*@return	angel of pendulum in Cartesian coordinates of the Sensor/arm12=arm21=arm2
	*/
	public double[] getPhiPendCartcoord(){
		
		double phiSx = 0.00175 * (ScaraDriver.getADCHallSens(0) - ScaraDriver.getADCHallSens(2));		// angel of pendulum in Sensor coordinates in X-direction
		double phiSy = 0.00175 * (ScaraDriver.getADCHallSens(3) - ScaraDriver.getADCHallSens(1));		// angel of pendulum in Sensor coordinates in y-direction
				
		double phiCx = phiSx * Math.cos(-mot2.enc.getPos()) - phiSy * Math.sin(-mot2.enc.getPos());		// angel of pendulum in Cartesian coordinates in X-direction
		double phiCy = phiSx * Math.sin(-mot2.enc.getPos()) + phiSy * Math.cos(-mot2.enc.getPos());		// angel of pendulum in Cartesian coordinates in Y-direction
				
		phiC[0] = phiCx; phiC[1] = phiCy;			// Cartesian coordinates in 
		
		return phiC;
	}
	
	
	/**
	*@return	position of the tip from the pendulum
	*/
	public double[] getPosPendTip(){
		
		posTCP[0] = kin.getActualPosXY()[0];
		posTCP[1] = kin.getActualPosXY()[1];
		
		phiPcart[0] = getPhiPendCartcoord()[0];
		phiPcart[1] = getPhiPendCartcoord()[1];
		
		double posPTx = posTCP[0] + lstab*Math.sin(phiPcart[0]);
		double posPTy = posTCP[1] + lstab*Math.sin(phiPcart[1]);
		
		posPT[0] = posPTx; posPT[1] = posPTy;
		
		return posPT;
	}
	
	public void reset(){
		valPosPrev[0] = 0;															// save Prev values for next differentiation / integration
		valPosPrev[1] = 0;
		valPhiPrev[0] = 0;
		valPhiPrev[1] = 0;
		vPrev[0] = 0;
		vPrev[1] = 0;
		pPrev[0] = 0;
		pPrev[1] = 0;
		val[0] = 0;
		val[1] = 0;
	}
	
}
