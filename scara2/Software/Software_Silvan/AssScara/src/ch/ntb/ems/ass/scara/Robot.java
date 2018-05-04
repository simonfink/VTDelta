package ch.ntb.ems.ass.scara;
import ch.ntb.ems.ass.scara.blocks.Ctrl.BarTipPosEstimation;
import ch.ntb.ems.ass.scara.blocks.Ctrl.ColisionProtection;
import ch.ntb.ems.ass.scara.blocks.Ctrl.InvKinematic;
import ch.ntb.ems.ass.scara.blocks.Ctrl.Kinematic;
import ch.ntb.ems.ass.scara.blocks.Ctrl.PDMotController;
import ch.ntb.ems.ass.scara.blocks.Ctrl.PathPlaner;
import ch.ntb.ems.ass.scara.blocks.Ctrl.Rotation;
import ch.ntb.ems.ass.scara.blocks.Ctrl.TipController;
import ch.ntb.ems.ass.scara.blocks.Ctrl.barPosEstimation;
import ch.ntb.ems.ass.scara.blocks.Ctrl.PDMotController.ControllerType;
import ch.ntb.ems.ass.scara.blocks.inOutputs.ADC128S102;
import ch.ntb.ems.ass.scara.blocks.inOutputs.FQD;
import ch.ntb.ems.ass.scara.blocks.inOutputs.PWMOut;
import ch.ntb.ems.ass.scara.framework.Output;
import ch.ntb.ems.ass.scara.model.Motor;
import ch.ntb.inf.deep.runtime.mpc5200.driver.NtbDriverSPIScaraWorkaround;


/**
 * Main program of SCARA-robot for ASS AG
 * The robot is able to balance an inverted pendulum and to accelerate with 120 m/s^2
 * 30.7.2014
 * Silvan Huber
 * NTB / Buchs / Switzerland
 */


public class Robot {
	private static enum State{INIT,BALANCE_BAR, DRIVE_TO_MIDDLE, HIGH_SPEED, STOP};
	
	public int cycleCounter = 0;
	int disableButtonCounter = 0;
	boolean buttonFlag = false;
	
	public final static int NUMBER_OF_AXIS = 2;
	public final static int NUMBER_OF_USED_CHANELS= 7;
	private final double BRIDGE_VOLTAGE = 48.0; 
	public final static double VELOCITY_LIMIT = 24.0;		// 7000/60/31*2*3.14
	private final double OMEGA = 300.0;
	
	private final int MIDDLE_BUTTON = 0;
	private final int MOTOR_LED = 1;
	private final int LEFT_BUTTON = 2;
	private final int RIGHT_BUTTON = 3;
	
	double[] posCenter = {0.0, 0.3};
	double[] pos01 = {0.3, 0.3};
	double[] pos02 = {-0.3, 0.3};
	double hystRobot = 0.0005;
	double hystBar = 0.005;
	double posX, posY;
	
	/**
	 * Motor initialisation
	 *
	 * @param  gear ratio  			
	 * @param  encoder tics				per round of encoder (see motor data sheet)
	 * @param  quadratur detection		factor 4 if with quadratur detection
	 * @param  motor resistance			per round of encoder (see motor data sheet)
	 * @param  km						torque constant
	 * @param  motor inertia			
	 * @param  gear inertia			
	 * @param  encoder inertia			
	 */
	public Motor maxon148877_0 = new Motor(
			6.0*72.0/14.0,				// gear ratio
			1024,						// enc tics per round
			4,							// quadratur detection
			1.13,						// motor resistance
			0.0603,						// km
			137e-7,						// motor inertia
			0,							// gear inertia
			1.7e-7);					// encoder inertia
	
	public Motor maxon148877_1 = new Motor(
		   -6.0*72.0/14.0,				// gear ratio
			1024,						// enc tics per round
			4,							// quadratur detection
			1.13,						// motor resistance
			0.0603,						// km
			137e-7,						// motor inertia
			0,							// gear inertia
			1.7e-7);					// encoder inertia

	
	public FQD[] enc = new FQD[NUMBER_OF_AXIS];
	public PWMOut[] pwm = new PWMOut[NUMBER_OF_AXIS];
	public PDMotController[] motorController = new PDMotController[NUMBER_OF_AXIS];
	public ADC128S102[] adc = new ADC128S102[NUMBER_OF_USED_CHANELS];
	public barPosEstimation barPos;
	public Kinematic kinematic;
	public Rotation rotation;
	public BarTipPosEstimation[] barTipPosEstimation = new BarTipPosEstimation[NUMBER_OF_AXIS];
	public TipController[] tipController = new TipController[NUMBER_OF_AXIS];
	public Output[] desiredPosition = new Output[NUMBER_OF_AXIS];
	public ColisionProtection colisionProtection;
	public InvKinematic invKinematic;
	public PathPlaner pathPlaner;
	public HighSpeed highSpeed;
	
	private InitialisationHandler initHandler;
	private Balancing balancing;
	
	public double dt;
	
	private State state = State.INIT;
	
	public Robot(double dt){
		this.dt = dt;
		enc[0] = new FQD(0,maxon148877_0,dt);
		enc[1] = new FQD(1,maxon148877_1,dt);
		
		
		/**
		 * controller initialisation
		 *
		 * @param  motor  			
		 * @param  omega					max. angle speed
		 * @param  velocitiy limit			in cartesian coordinates
		 * @param  max. voltage				for motors
		 * @param  controller type			speed or position
		 * @param  inertia reduced			inertia of motor, gear, arm reduced to arm			
		 * @param  dt						sampling time					
		 */
		
		motorController[0] = new PDMotController(
				maxon148877_0,			// motor
				OMEGA,					// omega
				VELOCITY_LIMIT,			// velocity limit
				48.0,					// max voltage
				0.6,					// max motor torque
				ControllerType.SPEED,	// controller type (speed/position)
				1e-02,					// Jred to arm
				dt);					// dt
		
		motorController[1] = new PDMotController(
				maxon148877_1,			// motor
				OMEGA,					// omega
				VELOCITY_LIMIT,			// velocity limit
				48.0,					// max voltage
				0.6,					// max motor torque
				ControllerType.SPEED,	// controller type (speed/position)
				1e-02,					// Jred to arm
				dt);					// dt
		
		for(int i = 0; i <NUMBER_OF_USED_CHANELS;i++ ){
			adc[i] = new ADC128S102(i);
		}
		barPos = new barPosEstimation(0.00175);
		barPos.hallSignal0.connect(adc[0].voltage);
		barPos.hallSignal1.connect(adc[1].voltage);
		barPos.hallSignal2.connect(adc[2].voltage);
		barPos.hallSignal3.connect(adc[3].voltage);
		
		for(int i = 0; i<NUMBER_OF_AXIS; i++){
			pwm[i] = new PWMOut(i, BRIDGE_VOLTAGE);
			motorController[i].setSpeedControl();
			motorController[i].actualPosition.connect(enc[i].actualPosition);
			motorController[i].actualVelocity.connect(enc[i].actualVelocity);
			pwm[i].voltage.connect(motorController[i].controlVoltage);
			pwm[i].disable();
			
			double omegaTip = 2.75;
			double DTip = 0.7;						// with 1.0 to nervous
			double KpTip = omegaTip*omegaTip;
			double KdTip = 2*DTip*omegaTip;
			
			double omegaBarAngle = 15.0;
			double DAngleBar = 0.7;					// with 1.0 to nervous
			double KpBarAngle = omegaBarAngle*omegaBarAngle;
			double KdBarAngle = 2*DAngleBar*omegaBarAngle;
			

			tipController[i] = new TipController(this, KpTip,KdTip,KpBarAngle,KdBarAngle,dt);
			desiredPosition[i] = new Output();
			
			barTipPosEstimation[i] = new BarTipPosEstimation(this);
		} 
		
		kinematic = new Kinematic();
		kinematic.actualPos0.connect(enc[0].actualPosition);
		kinematic.actualPos1.connect(enc[1].actualPosition);

		rotation = new Rotation();
		rotation.rotAngle.connect(enc[1].actualPosition);
		rotation.x_in.connect(barPos.phi_s_x);
		rotation.y_in.connect(barPos.phi_s_y);
		
		barTipPosEstimation[0].barPhi.connect(rotation.x_out);
		barTipPosEstimation[1].barPhi.connect(rotation.y_out);
		barTipPosEstimation[0].robotPos.connect(kinematic.xPosition);
		barTipPosEstimation[1].robotPos.connect(kinematic.yPosition);
		
		tipController[0].actualBarTipPos.connect(barTipPosEstimation[0].barTip);
		tipController[1].actualBarTipPos.connect(barTipPosEstimation[1].barTip);
		tipController[0].phiR.connect(rotation.x_out);
		tipController[1].phiR.connect(rotation.y_out);
		
		tipController[0].actualPos.connect(kinematic.xPosition);
		tipController[1].actualPos.connect(kinematic.yPosition);
		
		tipController[0].desiredBarTipPos.connect(desiredPosition[0]);
		tipController[1].desiredBarTipPos.connect(desiredPosition[1]);
		
		colisionProtection = new ColisionProtection(pwm,NUMBER_OF_AXIS);
		
		colisionProtection.actualAngle0.connect(enc[0].actualPosition);
		colisionProtection.actualAngle1.connect(enc[1].actualPosition);
		
		pathPlaner = new PathPlaner(dt);
		invKinematic = new InvKinematic();
		invKinematic.armPosX.connect(tipController[0].desiredPosition);
		invKinematic.armPosY.connect(tipController[1].desiredPosition);
		
		initHandler = new InitialisationHandler(this); 
		balancing = new Balancing(this); 
		highSpeed = new HighSpeed(this);
	}
	
	
	/**
	 * Main program with 4 main parts:
	 * 		1.	Initialisation of robot
	 * 		2.	High speed
	 * 		3.	STOP
	 * 		4.	Balancing				
	 */
	
	public void run(){
		switch (state){
		case INIT:
			NtbDriverSPIScaraWorkaround.turnLedOn(LEFT_BUTTON);
			NtbDriverSPIScaraWorkaround.turnLedOn(MIDDLE_BUTTON);
			NtbDriverSPIScaraWorkaround.turnLedOn(RIGHT_BUTTON);
			NtbDriverSPIScaraWorkaround.turnLedOn(MOTOR_LED);
			NtbDriverSPIScaraWorkaround.turnLedOn(LEFT_BUTTON);
			NtbDriverSPIScaraWorkaround.turnLedOn(MIDDLE_BUTTON);
			NtbDriverSPIScaraWorkaround.turnLedOn(RIGHT_BUTTON);
			NtbDriverSPIScaraWorkaround.turnLedOn(MOTOR_LED);
			
			if(initHandler.run()>0){
				NtbDriverSPIScaraWorkaround.turnLedOn(RIGHT_BUTTON);
				System.out.println("Init Done!");
						
				cycleCounter = 0;
				colisionProtection.reset();
				desiredPosition[0].setValue(posCenter[0]);
				desiredPosition[1].setValue(posCenter[1]);
				motorController[0].setVelocityLimit(InitialisationHandler.INITIALISATION_VELOCITY);
				motorController[1].setVelocityLimit(InitialisationHandler.INITIALISATION_VELOCITY);
				motorController[0].desiredPosition.connect(invKinematic.phyArmX);
				motorController[1].desiredPosition.connect(invKinematic.phyArmY);
				invKinematic.armPosX.connect(desiredPosition[0]);
				invKinematic.armPosY.connect(desiredPosition[1]);
				pwm[0].enable();
				pwm[1].enable();
				System.out.println("goto drive to middle");
				state = State.DRIVE_TO_MIDDLE;
			}
			break;
			
		case STOP:	
			pwm[0].disable();pwm[1].disable();

			if(NtbDriverSPIScaraWorkaround.getSwitchState(LEFT_BUTTON) == false){
				buttonFlag = true;
				cycleCounter = 0;
				colisionProtection.reset();
				tipController[0].reset();	
				tipController[1].reset();
				highSpeed.reset();
				balancing.reset();
				
				desiredPosition[0].setValue(posCenter[0]);
				desiredPosition[1].setValue(posCenter[1]);
				motorController[0].setVelocityLimit(InitialisationHandler.INITIALISATION_VELOCITY);
				motorController[1].setVelocityLimit(InitialisationHandler.INITIALISATION_VELOCITY);
				invKinematic.armPosX.connect(desiredPosition[0]);
				invKinematic.armPosY.connect(desiredPosition[1]);
	
				pwm[0].enable();
				pwm[1].enable();
				state = State.HIGH_SPEED;
			}
			
			if(NtbDriverSPIScaraWorkaround.getSwitchState(RIGHT_BUTTON) == false){
				buttonFlag = true;
				cycleCounter = 0;
				colisionProtection.reset();
				tipController[0].reset();	
				tipController[1].reset();
				highSpeed.reset();
				balancing.reset();
				
				desiredPosition[0].setValue(posCenter[0]);
				desiredPosition[1].setValue(posCenter[1]);
				motorController[0].setVelocityLimit(InitialisationHandler.INITIALISATION_VELOCITY);
				motorController[1].setVelocityLimit(InitialisationHandler.INITIALISATION_VELOCITY);
				invKinematic.armPosX.connect(desiredPosition[0]);
				invKinematic.armPosY.connect(desiredPosition[1]);
	
				pwm[0].enable();
				pwm[1].enable();
				System.out.println("goto drive to middle");
				state = State.DRIVE_TO_MIDDLE;
			}
			break;
		
		case DRIVE_TO_MIDDLE:
			posX = kinematic.xPosition.getValue();
			posY = kinematic.yPosition.getValue();
			if(posX < posCenter[0]+hystRobot && posX > posCenter[0]-hystRobot && posY < posCenter[1]+hystRobot && posY > posCenter[1]-hystRobot){		
				if(NtbDriverSPIScaraWorkaround.getSwitchState(RIGHT_BUTTON) == false && (barPos.barOneIsOn() || barPos.barTwoIsOn())){
					state = State.BALANCE_BAR;
				}
				else if (NtbDriverSPIScaraWorkaround.getSwitchState(LEFT_BUTTON) == false && barPos.barOneIsOn() == false && barPos.barTwoIsOn() == false && buttonFlag == false){
					state = State.HIGH_SPEED;
				}
			}
			break;
			
		case BALANCE_BAR: 	
			if(barPos.barOneIsOn() || barPos.barTwoIsOn()){
				balancing.run();
				
			}else if(colisionProtection.hasFired()){
				System.out.println("colision detected!");
				state = State.STOP;
			}else{
				System.out.println("lost bar!");
				state = State.STOP;
			}

			break;

		case HIGH_SPEED:
			if(highSpeed.run()>0){
				System.out.println("High-Speed mode Done!");
				
				highSpeed.reset();
				invKinematic.armPosX.connect(desiredPosition[0]);
				invKinematic.armPosY.connect(desiredPosition[1]);
				motorController[0].setVelocityLimit(InitialisationHandler.INITIALISATION_VELOCITY);
				motorController[1].setVelocityLimit(InitialisationHandler.INITIALISATION_VELOCITY);
				state = State.DRIVE_TO_MIDDLE;
			}
			break;
			
		default:
			state = State.STOP;
			break;
		}
		
		
		if(NtbDriverSPIScaraWorkaround.getSwitchState(MIDDLE_BUTTON) == false){
			state = State.STOP;
		}
		
		for(int i = 0; i <NUMBER_OF_USED_CHANELS;i++ ){
			adc[i].run();
		}
		barPos.run();
		for(int i = 0; i<NUMBER_OF_AXIS;i++){
			enc[i].run();
			
		}
		kinematic.run();
		invKinematic.run();
		rotation.run();

		
		if(state == State.BALANCE_BAR && (barPos.barOneIsOn() || barPos.barTwoIsOn())){
			for(int i = 0; i<NUMBER_OF_AXIS;i++){
				barTipPosEstimation[i].run();
				tipController[i].run();
			}
			colisionProtection.run();
		}
		
		if(state == State.HIGH_SPEED){
			colisionProtection.run();
		}
		
		for(int i = 0; i<NUMBER_OF_AXIS;i++){
			motorController[i].run();
			pwm[i].run();
		}		
		
		if(buttonFlag){
			disableButtonCounter++;
			if(disableButtonCounter >= 1000){
				buttonFlag = false;
				disableButtonCounter = 0;
			}
		}
		
		cycleCounter++;
		if(cycleCounter == 1000){
			if(barPos.barOneIsOn()){
				System.out.println("Bar 1 is on");
			}
			if(barPos.barTwoIsOn()){
				System.out.println("Bar 2 is on");
			}

			cycleCounter = 0;		
		}	
	}

}
