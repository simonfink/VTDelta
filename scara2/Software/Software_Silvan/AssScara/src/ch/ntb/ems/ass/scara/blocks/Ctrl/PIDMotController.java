package ch.ntb.ems.ass.scara.blocks.Ctrl;

import ch.ntb.ems.ass.scara.framework.Input;
import ch.ntb.ems.ass.scara.framework.Output;
import ch.ntb.ems.ass.scara.model.Motor;

public class PIDMotController {
	public enum ControllerType{POSITION,SPEED};
	public Input actualPosition = new Input();
	public Input desiredPosition = new Input();
	
	public Input desiredVelocity = new Input();
	public Input actualVelocity = new Input();
	
	public Output controlVoltage = new Output();
	
	
	private double kp,kd,ki,dt;
	private double desiredVelocityLimit,maxVoltage,maxMotorTorque, Jred,antiWindUpLimit;
	private double motorTorque,velocityDeviationInt;
	private Motor m;
	private ControllerType type;
	
	public double getMotorTorque(){
		return motorTorque;
	}
	
	public void setPosControl(){
		this.type = ControllerType.POSITION;
	}
	
	public void setSpeedControl(){
		this.type = ControllerType.SPEED;
	}
	
	
	public PIDMotController(Motor m,double omega,double desiredVelocityLimit, double maxVoltage, double maxMotorTorque,ControllerType type, double Jred, double antiWindUpLimit, double dt){
		this.kp = omega*0.5;
		this.kd = omega*2.0;
		this.ki = kp;
		this.m = m;
		this.desiredVelocityLimit = desiredVelocityLimit;
		this.maxVoltage = maxVoltage;
		this.maxMotorTorque = maxMotorTorque;
		this.type = type;
		this.Jred = Jred;
		this.antiWindUpLimit = antiWindUpLimit;
		this.dt = dt;
		this.velocityDeviationInt = 0;
	}
	
	

	public void run(){
		double positionDeviation,deisredVelocityComp,velocityDeviation,desiredAcceleration;
		double roboterTorque,current,voltage;

		
		if (type == ControllerType.POSITION){
			positionDeviation = desiredPosition.getValue() - actualPosition.getValue();
		}else{
			positionDeviation = 0.0;
		}
		
		deisredVelocityComp = positionDeviation * kp + desiredVelocity.getValue();
		
		//limit desired velocity
		if(deisredVelocityComp > desiredVelocityLimit){
			deisredVelocityComp = desiredVelocityLimit;
		}else if(deisredVelocityComp < -desiredVelocityLimit){
			deisredVelocityComp = -desiredVelocityLimit;
		}
		
		
		velocityDeviation = deisredVelocityComp - actualVelocity.getValue();
		
		velocityDeviationInt = velocityDeviationInt + velocityDeviation;
		
		if(velocityDeviationInt > antiWindUpLimit){
			velocityDeviationInt = antiWindUpLimit;
		}else if(velocityDeviationInt <-antiWindUpLimit){
			velocityDeviationInt = -antiWindUpLimit;
		}
		
		desiredAcceleration = velocityDeviation*kd + velocityDeviationInt*ki;
		
		
		
		roboterTorque = desiredAcceleration*Jred;
		
		motorTorque = roboterTorque/m.gear.i;

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
