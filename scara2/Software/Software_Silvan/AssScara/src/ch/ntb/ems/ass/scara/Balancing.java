package ch.ntb.ems.ass.scara;
import ch.ntb.ems.ass.scara.framework.Output;
import ch.ntb.inf.deep.runtime.mpc5200.driver.NtbDriverSPIScaraWorkaround;


public class Balancing {
	
	public Output desiredPositionX;
	public Output desiredPositionY;

	private enum State{INIT, DRIVE_01};
	
	private Robot robot;
	private State state;
	boolean barIsOnFlag = false;
	public int cycleCounter = 0;
	
	double[] pos01 = {0.25, 0.3};
	double[] pos02 = {-0.25, 0.3};
	double[] posCenter = {0.0, 0.3};
	double hystRobot = 0.0005;
	double hystBar = 0.005;
	double posX, posY;
	double ds = 0.07;					// maximal allowed difference between desiredTipPos and actualTipPos
	double t = 0.0, dt = 0.0;
	

	
	public Balancing(Robot robot){
		this.robot = robot;
		this.state = State.INIT;
		
		desiredPositionX = new Output();
		desiredPositionY = new Output();
	}
	
	public void run(){
		switch(state){
		
		case INIT:
			t = Math.PI/2;
			dt = 0.0;
			
			NtbDriverSPIScaraWorkaround.turnLedOn(3);	// set right button
			
			desiredPositionX.setValue(posCenter[0]);
			desiredPositionY.setValue(posCenter[1]);		
			robot.tipController[0].desiredBarTipPos.connect(desiredPositionX);
			robot.tipController[1].desiredBarTipPos.connect(desiredPositionY);
			
			robot.invKinematic.armPosX.connect(robot.tipController[0].desiredPosition);
			robot.invKinematic.armPosY.connect(robot.tipController[1].desiredPosition);
			robot.motorController[0].setVelocityLimit(Robot.VELOCITY_LIMIT);
			robot.motorController[1].setVelocityLimit(Robot.VELOCITY_LIMIT);

			state = State.DRIVE_01;
			break;
		
		case DRIVE_01:
						
			desiredPositionX.setValue(0.2*Math.cos(t));
			desiredPositionY.setValue(posCenter[1]);
			
			if(t <= 0.0){
				t = 2*Math.PI;
			}
			
			posX = robot.tipController[0].actualBarTipPos.getValue();
			posY = robot.tipController[1].actualBarTipPos.getValue();
			
			
			if(Math.abs(desiredPositionX.getValue() - robot.tipController[0].actualBarTipPos.getValue()) < ds  
				&& Math.abs(desiredPositionY.getValue() - robot.tipController[1].actualBarTipPos.getValue()) < ds){
					robot.tipController[0].disableIntegratorLimit();
					robot.tipController[1].disableIntegratorLimit();
					if(dt < 1/1000.0){
						dt = dt + 1/2000000.0;	// 2 sec to dt = 1/1000
					}
					
					t = t - dt;
			}
			else{
				robot.tipController[0].enableIntegratorLimit();
				robot.tipController[1].enableIntegratorLimit();
			}

			break;

		}
		
	}
	
	public void reset(){
		state = State.INIT;
	}
		
}

