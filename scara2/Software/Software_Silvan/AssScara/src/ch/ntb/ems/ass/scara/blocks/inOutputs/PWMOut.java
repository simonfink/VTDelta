package ch.ntb.ems.ass.scara.blocks.inOutputs;

import ch.ntb.ems.ass.scara.framework.Input;
import ch.ntb.inf.deep.runtime.mpc5200.driver.NtbDriverSPIScaraWorkaround;

public class PWMOut {
	public Input voltage = new Input();
	private static double MAX_PWM_RATIO_REG_VALUE = 1920.0; //See FPGA code for this value
	private double maxVoltage;
	private int motorNr;
	
	public PWMOut(int axisNr, double maxVoltage){
		this.maxVoltage = maxVoltage;
		this.motorNr = axisNr;
		NtbDriverSPIScaraWorkaround.setSetValueAxis(axisNr,0);
		NtbDriverSPIScaraWorkaround.enableAxis(axisNr);
	}
	
	public void run(){
		short dutyCycle = (short) (voltage.getValue()/maxVoltage*MAX_PWM_RATIO_REG_VALUE);
		NtbDriverSPIScaraWorkaround.setSetValueAxis(motorNr,dutyCycle);
	}
	
	public void enable(){
		NtbDriverSPIScaraWorkaround.enableAxis(motorNr);
	}
	
	public void disable(){
		NtbDriverSPIScaraWorkaround.setSetValueAxis(motorNr,0);
		NtbDriverSPIScaraWorkaround.disableAxis(motorNr);
	}
}
