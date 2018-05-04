package ch.ntb.robotics.scara.dynamic;

import ch.ntb.robotics.scara.motorcontrol.MtrCtrlPD;

public class DynVeloLimiter {
	
	double maxSpeed, maxAcc, phiMax1, phiMin1, phiMax2, phiMin2;
	double protDist = 0.05;		// 0.05 rad = 2.9°
	
	public MtrCtrlPD mot1;
	public MtrCtrlPD mot2;
	
	
	public DynVeloLimiter(double[] max1, double[] max2){
		this.maxSpeed = max1[2];		// maxSpeed axis 1 = axis 2
		this.maxAcc = max1[3];		// acceleration axis 1 = axis 2
		
		this.phiMax1 = max1[0];
		this.phiMin1 = max1[1];
		this.phiMax2 = max2[0];
		this.phiMin2 = max2[1];
	}
	
	

	/**
	*	running method for limiting the velocity dynamic to axis positions (square-function)
	*@param max
	*			contains all maximal values [qMax, qMin, maxSpeed, maxAcc] in radiant
	*/
	
	public void run(){	
		
		double omegaMax1 = Math.sqrt(2*maxAcc*(phiMax1-mot1.enc.getPos()));
		double omegaMin1 = -Math.sqrt(2*maxAcc*(mot1.enc.getPos()-phiMin1));
		double omegaMax2 = Math.sqrt(2*maxAcc*(phiMax2-mot2.enc.getPos()));
		double omegaMin2 = -Math.sqrt(2*maxAcc*(mot2.enc.getPos()-phiMin2));
		
		if (mot1.enc.getVelo() >= omegaMax1){
			mot1.setVelocity(omegaMax1);
		}
		else if (mot1.enc.getVelo() < omegaMin1){
			mot1.setVelocity(omegaMax1);
		}
		else{
			mot1.setVelocity(maxSpeed);
		}
		
		if (mot2.enc.getVelo() >= omegaMax2){
			mot2.setVelocity(omegaMax2);
		}
		else if (mot2.enc.getVelo() < omegaMin2){
			mot2.setVelocity(omegaMax2);
		}
		else{
			mot2.setVelocity(maxSpeed);
		}
	}
	
	public void colisionProtect(){
		if(mot1.enc.getPos() >= phiMax1-protDist){
			mot1.activate(false);
			mot2.activate(false);
		}
		if(mot2.enc.getPos() >= phiMax2-protDist){
			mot1.activate(false);
			mot2.activate(false);
		}
		if(mot1.enc.getPos() <= phiMin1+protDist){
			mot1.activate(false);
			mot2.activate(false);
		}
		if(mot2.enc.getPos() <= phiMin2+protDist){
			mot1.activate(false);
			mot2.activate(false);
		}
		if(mot1.enc.getPos() - mot2.enc.getPos() <= 30*2*Math.PI/360+protDist){
			mot1.activate(false);
			mot2.activate(false);
		}
		if(mot1.enc.getPos() - mot2.enc.getPos() >= 150*2*Math.PI/360-protDist){
			mot1.activate(false);
			mot2.activate(false);
		}
	}

}
