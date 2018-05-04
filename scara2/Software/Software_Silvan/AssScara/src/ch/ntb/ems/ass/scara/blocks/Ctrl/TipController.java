package ch.ntb.ems.ass.scara.blocks.Ctrl;

import ch.ntb.ems.ass.scara.Robot;
import ch.ntb.ems.ass.scara.framework.Input;
import ch.ntb.ems.ass.scara.framework.Output;

public class TipController {
	private Robot robot;
	
	public Input actualBarTipPos;
	public Input desiredBarTipPos;
	public Input actualPos;
	
	public Input phiR;
	
	public Output desiredPosition;
	public Output desiredVelocity;
	
	
	static final double DESIRED_ACCELERATION_MAX = 100.0;
	static final double DESIRED_VELOCITY_MAX = 10.0;
	static final double DESIRED_POSITION_MAX = 0.45;
	
	
	double kpTipPosAcc,kdTipPosAcc, dt;
	double kiTipPosAcc = 0.02;
	double kpAccArm,kdAccArm;
	double speed, speedOld = 0;
	double oldTipPosDeviation,phiDeviationOld, oldIntTipPosDeviation = 0.0;
	double oldDesiredPos, desiredPos;
	double acclerationTip, intTipPosDeviation;
	double counter = 0.0;
	double g = 9.81;
	double dOne = 0.138;
	double JstabOne = 0.00038;
	double mstabOne = 0.037;
	double lstabOne = 0.1;
	double rOne = Math.sqrt(1/12.0*lstabOne*lstabOne); //Math.sqrt(JstabOne/mstabOne);
	double dTwo = 0.138;
	double JstabTwo = 0.00038;
	double mstabTwo = 0.037;
	double lstabTwo = 0.32;
	double rTwo = Math.sqrt(1/12.0*lstabTwo*lstabTwo); // Math.sqrt(JstabTwo/mstabTwo);
	boolean first;
	boolean integratorLimiter;
	double ddX;
	
	double timer;
	
	public TipController(Robot robot, double kpAcc, double kdAcc,double kpAccArm, double kdAccArm, double dt){
		this.robot = robot;
		actualBarTipPos = new Input();
		desiredBarTipPos = new Input();
		actualPos = new Input();
		desiredPosition = new Output();
		
		desiredVelocity = new Output();
		
		phiR = new Input();
		this.dt = dt;
		this.kpTipPosAcc = kpAcc;
		this.kdTipPosAcc = kdAcc;
		this.kpAccArm = kpAccArm;
		this.kdAccArm = kdAccArm;
		this.first = true;
		
	}
	
	

public void reset(){
	this.first = true;
}
	
public void disableIntegratorLimit(){
	this.integratorLimiter = false;
}

public void enableIntegratorLimit(){
	this.integratorLimiter = true;
}
	
public void run(){
		if(first){
			first = false;
			integratorLimiter = false;
			counter = 0;
			double tipPosDeviation = desiredBarTipPos.getValue() - actualBarTipPos.getValue();
			double acclerationTip = tipPosDeviation*kpTipPosAcc + (tipPosDeviation - oldTipPosDeviation)/dt*kdTipPosAcc;
			oldTipPosDeviation = tipPosDeviation;
			oldIntTipPosDeviation = 0.0;
			double desiredPhi = Math.atan(acclerationTip/ g);
			double phiDeviation = desiredPhi - phiR.getValue();
			phiDeviationOld = phiDeviation;
			desiredPosition.setValue(actualPos.getValue());
			oldDesiredPos = actualPos.getValue();
			desiredVelocity.setValue(0.0);
			
//			System.out.println("desiredPosition:");
//			System.out.print(desiredBarTipPos.getValue());
//			System.out.print('\t');
//			System.out.println(actualBarTipPos.getValue());	
			
			
		}else{
			
			double tipPosDeviation = desiredBarTipPos.getValue() - actualBarTipPos.getValue();
		
			double speed = speedOld*0.99 + 0.01*(tipPosDeviation - oldTipPosDeviation);
			speedOld = speed;
			
//			 ohne Integrator
//			double acclerationTip = tipPosDeviation*kpTipPosAcc + speed/dt*kdTipPosAcc;
			
			// mit Integrator
			intTipPosDeviation = oldIntTipPosDeviation + tipPosDeviation;
			
			if(integratorLimiter && intTipPosDeviation > 10.0){
				intTipPosDeviation = 10.0;
			}else if(integratorLimiter && intTipPosDeviation < -10.0){
				intTipPosDeviation = -10.0;
			}
			oldIntTipPosDeviation = intTipPosDeviation;
			acclerationTip = tipPosDeviation*kpTipPosAcc + speed/dt*kdTipPosAcc + kiTipPosAcc*intTipPosDeviation;

						
			oldTipPosDeviation = tipPosDeviation;
			
			double desiredPhi = Math.atan(acclerationTip/ g);
			
//			desiredPhi = 0.0;		// for test without tip controller
			
			double phiDeviation = desiredPhi - phiR.getValue();

			
			double ddPhi = phiDeviation*kpAccArm + (phiDeviation - phiDeviationOld)/dt* kdAccArm;
			
			phiDeviationOld = phiDeviation;
			
			//pendel model
			if(robot.barPos.barOneIsOn()){
				ddX = -ddPhi * (dOne*dOne + rOne*rOne) / dOne / Math.cos(phiR.getValue()) + g * Math.tan(phiR.getValue());	
			}
			else if(robot.barPos.barTwoIsOn()){
				ddX = -ddPhi * (dTwo*dTwo + rTwo*rTwo) / dTwo / Math.cos(phiR.getValue()) + g * Math.tan(phiR.getValue());	
			}

			if(counter < 50){
				counter++;
				if(ddX > DESIRED_ACCELERATION_MAX){
	//				System.out.println(ddX);
					ddX = DESIRED_ACCELERATION_MAX;
				}else if(ddX < -DESIRED_ACCELERATION_MAX){
	//				System.out.println(ddX);
					ddX = - DESIRED_ACCELERATION_MAX;
				}
			}
			
			
			desiredVelocity.setValue(desiredVelocity.getValue() + ddX*dt);
			if(desiredVelocity.getValue() > DESIRED_VELOCITY_MAX){
				desiredVelocity.setValue(DESIRED_VELOCITY_MAX);
			}else if(desiredVelocity.getValue() < -DESIRED_VELOCITY_MAX){
				desiredVelocity.setValue(-DESIRED_VELOCITY_MAX);
			}
			
			// Filter by Silvan gegen Rattern
			desiredPos = 0.25*oldDesiredPos + 0.75*(desiredPosition.getValue() + desiredVelocity.getValue()*dt);
			oldDesiredPos = desiredPos;
			desiredPosition.setValue(desiredPos);
			
			// no filter
//			desiredPosition.setValue(desiredPosition.getValue() + desiredVelocity.getValue()*dt);
			
			if(desiredPosition.getValue() > DESIRED_POSITION_MAX){
				desiredPosition.setValue(DESIRED_POSITION_MAX);
			}else if (desiredPosition.getValue() < -DESIRED_POSITION_MAX){
				desiredPosition.setValue(-DESIRED_POSITION_MAX);
			}
			
			
//			timer++;
//			if(timer > 1000){
//				System.out.println(intTipPosDeviation);
//				timer = 0;
//			}
			
		}	
	}
	
	public double getValues(){
		return oldIntTipPosDeviation;
	}
	
	
}
