package ch.ntb.ems.ass.scara;
import ch.ntb.ems.ass.scara.framework.Output;
import ch.ntb.inf.deep.runtime.mpc5200.driver.NtbDriverSPIScaraWorkaround;


public class HighSpeed {
	
	
	public Output desiredPositionX;
	public Output desiredPositionY;
	
	private enum State{INIT, MODIFY_ELLIPSE, DRIVE_ELLIPSE, DRIVE_POS_CENTER, DRIVE_POS_01, DRIVE_POS_02, DRIVE_POS_03, DRIVE_POS_04, DONE};
	
	private Robot robot;
	private State state;
	boolean first = true;
	boolean homing = true;
	public int cycleCounter = 0;
	
	double hystRobot = 0.0005;
	final double a0 = 0.35, b0 = 0.15;		// Ellipse: x = a*cos(t); y = b*sin(t)
	double a = 0.0, b = 0.0, t = 0.0;
	int i, di, roundCounter = 0;
	double n = 8.0, da = a0/n, db = b0/n, dt, dt0;
	double aOld = 0.0, bOld = 0.0;
	double[] posCenter = {0.0, 0.3};
	double[] pos01 = {0.0, posCenter[1]+b0};
	double[] pos04 = {a0, posCenter[1]};
	double[] pos03 = {0.0, posCenter[1]-b0};
	double[] pos02 = {-a0, posCenter[1]};
	double[] pos01n = {0.0, 0.0};
	double[] posAct = {0.0,0.0};
	
	double posX, posY;
	boolean flag = true, secondCircle = false;


	public HighSpeed(Robot robot){
		this.robot = robot;
		this.state = State.INIT;	
		
		desiredPositionX = new Output();
		desiredPositionY = new Output();
	}
	
	public int run(){
		switch(state){
		
		case INIT:
			NtbDriverSPIScaraWorkaround.turnLedOn(3);	// set right button
			
			first = true;
			homing = true;
			flag = true;
			secondCircle = false;
			roundCounter = 0;
			t = Math.PI/2;
			i = 0;
			a = a0;
			aOld = a;
			b = 0.0;
			bOld = 0.0;
			di = 1;
			da = 0.0;
			db = b0/n;
			dt = 1/5000000.0;
			dt0 = 0.001;
						
			robot.motorController[0].setVelocityLimit(Robot.VELOCITY_LIMIT);			// max. 24
			robot.motorController[1].setVelocityLimit(Robot.VELOCITY_LIMIT);
			
			posX = robot.kinematic.xPosition.getValue();
			posY = robot.kinematic.yPosition.getValue();
			posAct[0] = posX;
			posAct[1] = posY;
			robot.invKinematic.armPosX.connect(robot.pathPlaner.desiredPositionX);
			robot.invKinematic.armPosY.connect(robot.pathPlaner.desiredPositionY);
			robot.pathPlaner.setMaxValues(0.2, 1.0);
			robot.pathPlaner.precalcs(posAct, robot.posCenter);
			System.out.println("pathplaner precalcs done");
			state = State.DRIVE_POS_CENTER;

			break;
		
		case MODIFY_ELLIPSE:
			
			System.out.println(i);
			
			if(i < n && i > 0){
				a = aOld + da;
				aOld = a;
				
				b = bOld + db;
				bOld = b;
				
				state = State.DRIVE_ELLIPSE;
			}
			else if(i == n){
				secondCircle = true;
				dt = -dt;
				di = -di;
				da = -a0/n;
				db = 0.0;
				state = State.DRIVE_ELLIPSE;
			}
			else if(i == 0){
				robot.pathPlaner.setMaxValues(10.0, 50.0);				// max. 15/75	
				
				pos01n[0] = robot.kinematic.xPosition.getValue();
				pos01n[1] = robot.kinematic.yPosition.getValue();
				
				robot.pathPlaner.precalcs(pos01n, pos02);
				
				robot.invKinematic.armPosX.connect(robot.pathPlaner.desiredPositionX);
				robot.invKinematic.armPosY.connect(robot.pathPlaner.desiredPositionY);
				
				System.out.println("pathplaner precalcs done");
				state = State.DRIVE_POS_02;
			}
			else{
				break;
			}
			break;
			
						
		case DRIVE_ELLIPSE:
						
			t = t + dt0;
			dt0 = dt0 + dt;
						
			if(secondCircle && dt0 < 0.002){
				dt0 = 0.002;
			}
			
			desiredPositionX.setValue(a*Math.cos(t));
			desiredPositionY.setValue(b*Math.sin(t)+posCenter[1]);
			
			if(secondCircle == false){
				if(t >= Math.PI  && flag == true){
					flag = false;
					i = i + di;
					state = State.MODIFY_ELLIPSE;
				}
			}
			else{
				if(t >= Math.PI/2  && flag == true){
					flag = false;
					i = i + di;
					state = State.MODIFY_ELLIPSE;
				}
			}
			
			if(t >= 2*Math.PI){
				flag = true;
				t = 0.0;
			}
		
			break;
			
			
		case DRIVE_POS_01:
			posX = robot.kinematic.xPosition.getValue();
			posY = robot.kinematic.yPosition.getValue();
			
			if (posX < pos01[0]+hystRobot && posX > pos01[0]-hystRobot && posY < pos01[1]+hystRobot && posY > pos01[1]-hystRobot){
				System.out.println("is on Pos. 01");
				roundCounter++;
				if(roundCounter > 5){
					robot.pathPlaner.precalcs(pos01, robot.posCenter);
					System.out.println("pathplaner precalcs done");
					state = State.DRIVE_POS_CENTER;
				}
				else{
					robot.pathPlaner.precalcs(pos01, pos02);
					System.out.println("pathplaner precalcs done");
					state = State.DRIVE_POS_02;
				}
			}
			robot.pathPlaner.run();
			break;
			
			
		case DRIVE_POS_02:
			posX = robot.kinematic.xPosition.getValue();
			posY = robot.kinematic.yPosition.getValue();
			
			if (posX < pos02[0]+hystRobot && posX > pos02[0]-hystRobot && posY < pos02[1]+hystRobot && posY > pos02[1]-hystRobot){
				System.out.println("is on Pos");

				robot.pathPlaner.precalcs(pos02, pos03);
				System.out.println("pathplaner precalcs done");
				state = State.DRIVE_POS_03;
			}
			robot.pathPlaner.run();
			break;
			
			
		case DRIVE_POS_03:
			posX = robot.kinematic.xPosition.getValue();
			posY = robot.kinematic.yPosition.getValue();
			
			if (posX < pos03[0]+hystRobot && posX > pos03[0]-hystRobot && posY < pos03[1]+hystRobot && posY > pos03[1]-hystRobot){
				System.out.println("is on Pos");

				robot.pathPlaner.precalcs(pos03, pos04);
				System.out.println("pathplaner precalcs done");
				state = State.DRIVE_POS_04;
			}
			robot.pathPlaner.run();
			break;
	
			
		case DRIVE_POS_04:
			posX = robot.kinematic.xPosition.getValue();
			posY = robot.kinematic.yPosition.getValue();
			
			if (posX < pos04[0]+hystRobot && posX > pos04[0]-hystRobot && posY < pos04[1]+hystRobot && posY > pos04[1]-hystRobot){
				System.out.println("is on Pos");

				robot.pathPlaner.precalcs(pos04, pos01);
				System.out.println("pathplaner precalcs done");
				state = State.DRIVE_POS_01;
			}
			robot.pathPlaner.run();
			break;
			
			
		case DRIVE_POS_CENTER:
			posX = robot.kinematic.xPosition.getValue();
			posY = robot.kinematic.yPosition.getValue();
			
			if (posX < robot.posCenter[0]+hystRobot && posX > robot.posCenter[0]-hystRobot && posY < robot.posCenter[1]+hystRobot && posY > robot.posCenter[1]-hystRobot){
				System.out.println("is on Pos");
				
				if(homing){
					homing = false;
					desiredPositionX.setValue(posCenter[0]);
					desiredPositionY.setValue(posCenter[1]);			
					robot.invKinematic.armPosX.connect(desiredPositionX);
					robot.invKinematic.armPosY.connect(desiredPositionY);
					state = State.DRIVE_ELLIPSE;
				}
				else{
					state = State.DONE;
				}
			}
			robot.pathPlaner.run();
			break;
		
			
		case DONE:
			break;
			
		default:
			break;
		}
		
		if(state == State.DONE){
			return 1;
		}else{
			return 0;
		}
	}
	
	
	public void reset(){
		state = State.INIT;
	}
		
}


