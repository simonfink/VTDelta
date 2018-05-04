package ch.ntb.ems.ass.scara.blocks.Ctrl;

import ch.ntb.ems.ass.scara.framework.Input;
import ch.ntb.ems.ass.scara.framework.Output;
import ch.ntb.ems.ass.scara.model.Motor;

public class PDMotController {
	public enum ControllerType{POSITION,SPEED};
	public Input actualPosition = new Input();
	public Input desiredPosition = new Input();
	
	public Input desiredVelocity = new Input();
	public Input actualVelocity = new Input();
	
	public Output controlVoltage = new Output();
	
	
	private double kp,kd;
	private double desiredVelocityLimit,maxVoltage,maxMotorTorque, Jred;
	private double motorTorque, armTorque;
	private Motor m;
	private ControllerType type;
	private double integratedfakePos;
	private double dt;
	
	public double getMotorTorque(){
		return motorTorque;
	}
	
	public double getArmTorque(){
		return armTorque;
	}
	
	public void setPosControl(){
		this.type = ControllerType.POSITION;
	}
	
	public void setSpeedControl(){
		this.type = ControllerType.SPEED;
		this.integratedfakePos = 0;
	}
	
	public void setVelocityLimit(double limit){
		this.desiredVelocityLimit = limit;
	}
	
	public PDMotController(Motor m,double omega,double desiredVelocityLimit, double maxVoltage, double maxMotorTorque,ControllerType type, double Jred, double dt){
		this.kp = omega*0.5;
		this.kd = omega*2.0;
		this.m = m;
		this.desiredVelocityLimit = desiredVelocityLimit;
		this.maxVoltage = maxVoltage;
		this.maxMotorTorque = maxMotorTorque;
		this.type = type;
		this.Jred = Jred;
		this.integratedfakePos = 0;
		this.dt = dt;
	}
	

	public void run(){
		double positionDeviation,deisredVelocityComp,velocityDeviation,desiredAcceleration;
		double current,voltage;

		
		if (type == ControllerType.POSITION){
			positionDeviation = desiredPosition.getValue() - actualPosition.getValue();
			
			deisredVelocityComp = positionDeviation * kp + desiredVelocity.getValue();
			
			//limit desired velocity
			if(deisredVelocityComp > desiredVelocityLimit){
				deisredVelocityComp = desiredVelocityLimit;
			}else if(deisredVelocityComp < -desiredVelocityLimit){
				deisredVelocityComp = -desiredVelocityLimit;
			}
			
			velocityDeviation = deisredVelocityComp - actualVelocity.getValue();
			desiredAcceleration = velocityDeviation*kd;
			
		}else{
			
			// integrator
			integratedfakePos = integratedfakePos + desiredVelocity.getValue()*dt;
			
			// Calculate control value
			positionDeviation = integratedfakePos - actualPosition.getValue();// deltaPos at gear exit
			
			
			deisredVelocityComp = actualVelocity.getValue();
			
			velocityDeviation = desiredVelocity.getValue() - actualVelocity.getValue();
			
			desiredAcceleration = positionDeviation*kp + velocityDeviation *kd;
			
		}
		
		

		armTorque = desiredAcceleration*Jred;
		
		motorTorque = armTorque/m.gear.i;

		//limit motor torque
		if(motorTorque > maxMotorTorque ){
			motorTorque = maxMotorTorque;
		}else if (motorTorque < -maxMotorTorque){
			motorTorque = -maxMotorTorque;
		}
		
		
		current = motorTorque/m.km;
		
		voltage = current*m.R + deisredVelocityComp * m.gear.i * m.km;
		
		//limit desired voltage
		if(voltage > maxVoltage){
			voltage = maxVoltage;
		}else if(voltage < -maxVoltage){
			voltage = -maxVoltage;
		}
		
		controlVoltage.setValue(voltage);
		
	}
	

}
