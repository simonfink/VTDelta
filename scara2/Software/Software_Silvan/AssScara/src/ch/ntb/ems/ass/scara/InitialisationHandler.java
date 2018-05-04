package ch.ntb.ems.ass.scara;
import ch.ntb.ems.ass.scara.framework.Output;
import ch.ntb.inf.deep.runtime.mpc5200.driver.NtbDriverSPIScaraWorkaround;


public class InitialisationHandler {
	private enum InitState{INIT,DRIVE_TO_RIGHT_LIMIT,SET_ARM_POS, DONE, DRIVE_TO_LEFT_LIMIT, ERROR};
	public final static double INITIALISATION_VELOCITY = Math.PI/4;
	private final static double INITIALISATION_TORQUE = 0.3*6.0*72.0/14.0;
	private final static double TIME_BEVORE_TORQUE_MEASURE = 1.0;
	
	private Robot robot;
	private InitState state;
	private Output desiredVelocity;
	private Output desiredPos[] = new Output[Robot.NUMBER_OF_AXIS];
	private int cylceCounter;
	
	
	
	public InitialisationHandler(Robot robot){
		this.robot = robot;
		this.cylceCounter = 0;
		this.desiredVelocity = new Output();
		this.state = InitState.INIT;
		for(int i = 0; i<Robot.NUMBER_OF_AXIS;i++){
			desiredPos[i] = new Output();
			desiredPos[i].setValue(0);
		}
		
	}
	
	public int run(){
		switch(state){
		case INIT:
			init();
			break;
		case DRIVE_TO_RIGHT_LIMIT:
			driveToRightLimit();
			break;
		case SET_ARM_POS:
			setArmPos();
			break;
		case DRIVE_TO_LEFT_LIMIT:
			driveToLeftLimit();
			break;
		case ERROR:
			error();
			break;
		case DONE:
			break;
		default:
			System.out.println("Error: Unknown state in initialisation handler!");
			return -1;
		}
		
		if(state == InitState.DONE){
			return 1;
		}else{
			return 0;
		}
	}
	
	

	private void driveToRightLimit() {
		desiredVelocity.setValue(-INITIALISATION_VELOCITY);
		cylceCounter++;
		if(cylceCounter*robot.dt >= 1.0){
			if(robot.motorController[0].getArmTorque() <= -INITIALISATION_TORQUE){
				System.out.println("Drive to rigth limit done");
				desiredVelocity.setValue(0);
				state = InitState.SET_ARM_POS;
				cylceCounter = 0;
			}
		}
	}
	
	private void setArmPos() {
		robot.enc[0].setPosition(-16.0*2.0*Math.PI/360.0);
		robot.enc[1].setPosition(-46.0*2.0*Math.PI/360.0);
		double phi1 = robot.enc[0].actualPosition.getValue();
		double phi2 = robot.enc[1].actualPosition.getValue();
		System.out.print("Pos: ");
		System.out.print(phi1);
		System.out.print("\t");
		System.out.println(phi2);
		
		robot.motorController[1].desiredVelocity.connect(desiredVelocity);
		robot.pwm[0].disable();
		robot.pwm[1].enable();
		System.out.println("Set encoder done");
		
		state = InitState.DRIVE_TO_LEFT_LIMIT;
		
	}
	
	private void driveToLeftLimit() {
		desiredVelocity.setValue(INITIALISATION_VELOCITY);
		cylceCounter++;
		if(cylceCounter*robot.dt >= TIME_BEVORE_TORQUE_MEASURE){
			if(robot.motorController[1].getArmTorque() >= INITIALISATION_TORQUE){
				if(robot.enc[1].actualPosition.getValue() >= Math.PI){
					desiredVelocity.setValue(0);
					for(int i = 0; i<Robot.NUMBER_OF_AXIS;i++){
						robot.pwm[i].enable();
						robot.motorController[i].setVelocityLimit(INITIALISATION_VELOCITY);
						robot.motorController[i].setPosControl();
					}
					cylceCounter = 0;
					System.out.println("Drive to left limit done");
					state = InitState.DONE;
				}
				else{
					System.out.println("ERROR");
					System.out.println("remove disturbing part and switch power off");
					state = InitState.ERROR;
				}
			}
		}
	}

	private void init() {
		NtbDriverSPIScaraWorkaround.turnLedOn(3);	// set right button
		
		System.out.println("Init done");
		robot.motorController[0].desiredVelocity.connect(desiredVelocity);
		for(int i = 0; i<Robot.NUMBER_OF_AXIS;i++){
			robot.motorController[i].desiredPosition.connect(desiredPos[i]);
		}
		robot.pwm[0].enable();
		state = InitState.DRIVE_TO_RIGHT_LIMIT;
	}
	
	public void error(){
		robot.pwm[0].disable();
		robot.pwm[1].disable();
	}
	
}
