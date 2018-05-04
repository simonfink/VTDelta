package ch.ntb.ems.ass.scara.blocks.inOutputs;

import ch.ntb.ems.ass.scara.framework.Output;
import ch.ntb.ems.ass.scara.model.Motor;
import ch.ntb.inf.deep.runtime.mpc5200.driver.NtbDriverSPIScaraWorkaround;


public class FQD {
	public Output actualPosition = new Output();
	public Output actualVelocity = new Output(); 
	
	private double speedScale,posScale;
	private int motorNumber;
	private short oldTicks;
	private long intTicks;
	
	public FQD (int motorNumber, double i, double encTicks, double encEdges, double dt) {
		this.motorNumber = motorNumber;
		this.speedScale = 2.0*Math.PI/encTicks/encEdges/dt/i;
		this.posScale = 2.0*Math.PI/encTicks/encEdges/i;
		this.oldTicks = NtbDriverSPIScaraWorkaround.getEncoderPosition(motorNumber);
		this.intTicks = 0;
	}
	
	public FQD (int motorNumber, Motor m, double dt) {
		this.motorNumber = motorNumber;
		this.speedScale = 2.0*Math.PI/m.encoder.ticksPerTurn/m.encoder.edgeCount/dt/m.gear.i;
		this.posScale = 2.0*Math.PI/m.encoder.ticksPerTurn/m.encoder.edgeCount/m.gear.i;
		this.oldTicks = NtbDriverSPIScaraWorkaround.getEncoderPosition(motorNumber);
		this.intTicks = 0;
	}
	
	public void reset(){
		this.intTicks = 0;
		actualPosition.setValue(0);
	}
	
	public void setPosition(double newPos){
		this.intTicks = (long) (newPos/posScale);
		actualPosition.setValue(newPos);
	}
	
	
	public void run(){
		short actualTicks = NtbDriverSPIScaraWorkaround.getEncoderPosition(motorNumber);
		short deltaTicks = (short) (actualTicks - oldTicks);
		oldTicks = actualTicks;
		actualVelocity.setValue(deltaTicks*speedScale);
		this.intTicks = this.intTicks + deltaTicks;
		actualPosition.setValue(this.intTicks*posScale);
	}
	
	
	
}
