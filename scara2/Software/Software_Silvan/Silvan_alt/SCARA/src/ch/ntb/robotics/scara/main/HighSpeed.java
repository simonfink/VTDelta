package ch.ntb.robotics.scara.main;

import ch.ntb.robotics.scara.bahnplaner.Bahnplaner2;
import ch.ntb.robotics.scara.driver.ScaraDriver;
import ch.ntb.robotics.scara.dynamic.InversJakobian;
import ch.ntb.robotics.scara.kinematic.InversKinematic;
import ch.ntb.robotics.scara.kinematic.Kinematic;
import ch.ntb.robotics.scara.motorcontrol.MtrCtrlPD;

public class HighSpeed {

	MtrCtrlPD mot1;
	MtrCtrlPD mot2;
	Kinematic kin;
	InversKinematic invKin;
	InversJakobian invJakob;

	private int m = 0;
	private int timer = 0;
	
	private double[] posCenter = {0.0, 0.25};
	private static final double[] pos01 = {-0.45, 0.0};
	private double[] pos02 = {-0.22, 0.38};
	private double[] pos03 = { 0.22, 0.38};
	private double[] pos04 = { 0.45, 0.0};
	
	private double[] omega = {0.0, 0.0};
	private double[] phi = {0.0, 0.0};
	
	Bahnplaner2 path01;
	Bahnplaner2 path12;
	Bahnplaner2 path23;
	Bahnplaner2 path34;
	Bahnplaner2 path40;

	
	double[] val1, val2;
	double posX, posY;
	
	private static final double hyst = 0.002;					// hysteresis for detection if desired Position reached
	
	private final int INIT_HIGHSPEED_STATE = 0;
	private final int WAIT_TIME_STATE = 1;
	private final int GO_POS_01_STATE = 2;
	private final int GO_POS_02_STATE = 3;
	private final int GO_POS_03_STATE = 4;
	private final int GO_POS_04_STATE = 5;
	private final int GO_CENTER_STATE = 6;
	private final int TURN_FINISHED_STATE = 7;
	private final int STOP_STATE = 8;
	
	private int state = INIT_HIGHSPEED_STATE; 
	

	/**
	*	object constructor
	*@param	Ts
	*			sampling time
	*/
	public HighSpeed(double Ts){	
		path01 = new Bahnplaner2(posCenter, pos01, Ts);
		path12 = new Bahnplaner2(pos01, pos02, Ts);
		path23 = new Bahnplaner2(pos02, pos03, Ts);
		path34 = new Bahnplaner2(pos03, pos04, Ts);
		path40 = new Bahnplaner2(pos04, posCenter, Ts);
	}
	
	/**
	*	running method for the high speed program
	*@param	n
	*			number of how many times the turn shall be executed
	*@return	true if turn is n times executed; else: false
	*/
	public boolean run(int n){
		
		switch(state){
		
		case INIT_HIGHSPEED_STATE:
			ScaraDriver.setLED(0,1);				// toggle LEDs
			ScaraDriver.setLED(1,1);
			ScaraDriver.setLED(2,1);
			ScaraDriver.setLED(3,1);
			
			path01.reset();
			path12.reset();
			path23.reset();
			path34.reset();
			path40.reset();
			
			mot1.setVelocity(100);
			mot2.setVelocity(100);
			mot1.setPosCtrl();				// set controller to position controller
			mot2.setPosCtrl();
			
			mot1.activate(true);
			mot2.activate(true);
			
			System.out.print("init high-speed finish");
			System.out.print("\r\n");
			
			state = WAIT_TIME_STATE;
			break;
		
		case WAIT_TIME_STATE:
			timer++;
			if (timer >= 2000){
				timer = 0;
				ScaraDriver.setLED(0,3);				// set all LEDs on
				ScaraDriver.setLED(1,3);
				ScaraDriver.setLED(2,3);
				ScaraDriver.setLED(3,3);
				
				System.out.print("wait high-speed finish");
				System.out.print("\r\n");
				
				state = GO_POS_01_STATE;
			}
			break;
			
						
		case GO_POS_01_STATE:
			path01.run();									// calculate positions and velocities periodically
			
			val1 = path01.getVal1();						// posC1[0] = x1, posC1[1] = v1; posC1[2] = a1
			val2 = path01.getVal2();
						
			invKin.run(val1[0], val2[0]);					// calculate phi1,2 with inverted Kinematic
			phi = invKin.getPhi();							// returns phi1 and phi2
		
			invJakob.run(val1[1], val2[1]);				    // calculate omega1, 2 with the inverted Jakobian
			omega = invJakob.getOmega();					// returns omega1 and omega2
		
			val1[0] = phi[0]; val1[1] = omega[0]; val1[2] = 0;
			val2[0] = phi[1]; val2[1] = omega[1]; val2[2] = 0;
			
			mot1.setCtrlValue(val1);
			mot2.setCtrlValue(val2);
			
			posX = kin.getActualPosXY()[0];						// returns actual position of TCP
			posY = kin.getActualPosXY()[1];
			
			if (posX < pos01[0]+hyst && posX > pos01[0]-hyst && posY < pos01[1]+hyst && posY > pos01[1]-hyst){
				path01.reset();
				state = GO_POS_02_STATE;
			}
			
			break;
				
			
		case GO_POS_02_STATE:
			path12.run();									// calculate positions and velocities periodically
			
			val1 = path12.getVal1();						// posC1[0] = x1, posC1[1] = v1; posC1[2] = a1
			val2 = path12.getVal2();
			
			invKin.run(val1[0], val2[0]);					// calculate phi1,2 with inverted Kinematic
			phi = invKin.getPhi();							// returns phi1 and phi2
			
			invJakob.run(val1[1], val2[1]);				// calculate omega1, 2 with the inverted Jakobian
			omega = invJakob.getOmega();					// returns omega1 and omega2
			
			val1[0] = phi[0]; val1[1] = omega[0]; val1[2] = 0;
			val2[0] = phi[1]; val2[1] = omega[1]; val2[2] = 0;
			
			mot1.setCtrlValue(val1);
			mot2.setCtrlValue(val2);
			
			posX = kin.getActualPosXY()[0];						// returns actual position of TCP
			posY = kin.getActualPosXY()[1];
			
			if (posX < pos02[0]+hyst && posX > pos02[0]-hyst && posY < pos02[1]+hyst && posY > pos02[1]-hyst){
				path12.reset();
				state = GO_POS_03_STATE;
			}
			break;

			
		case GO_POS_03_STATE:
			path23.run();									// calculate positions and velocities periodically
			
			val1 = path23.getVal1();						// posC1[0] = x1, posC1[1] = v1; posC1[2] = a1
			val2 = path23.getVal2();
			
			invKin.run(val1[0], val2[0]);					// calculate phi1,2 with inverted Kinematic
			phi = invKin.getPhi();							// returns phi1 and phi2
			
			invJakob.run(val1[1], val2[1]);				// calculate omega1, 2 with the inverted Jakobian
			omega = invJakob.getOmega();					// returns omega1 and omega2
			
			val1[0] = phi[0]; val1[1] = omega[0]; val1[2] = 0;
			val2[0] = phi[1]; val2[1] = omega[1]; val2[2] = 0;
			
			mot1.setCtrlValue(val1);
			mot2.setCtrlValue(val2);
			
			posX = kin.getActualPosXY()[0];						// returns actual position of TCP
			posY = kin.getActualPosXY()[1];
			
			if (posX < pos03[0]+hyst && posX > pos03[0]-hyst && posY < pos03[1]+hyst && posY > pos03[1]-hyst){
				path23.reset();
				state = GO_POS_04_STATE;
			}
			break;

		
		case GO_POS_04_STATE:
			path34.run();									// calculate positions and velocities periodically
			
			val1 = path34.getVal1();						// posC1[0] = x1, posC1[1] = v1; posC1[2] = a1
			val2 = path34.getVal2();
			
			invKin.run(val1[0], val2[0]);					// calculate phi1,2 with inverted Kinematic
			phi = invKin.getPhi();							// returns phi1 and phi2
			
			invJakob.run(val1[1], val2[1]);				// calculate omega1, 2 with the inverted Jakobian
			omega = invJakob.getOmega();					// returns omega1 and omega2
			
			val1[0] = phi[0]; val1[1] = omega[0]; val1[2] = 0;
			val2[0] = phi[1]; val2[1] = omega[1]; val2[2] = 0;
			
			mot1.setCtrlValue(val1);
			mot2.setCtrlValue(val2);
			
			posX = kin.getActualPosXY()[0];						// returns actual position of TCP
			posY = kin.getActualPosXY()[1];
			
			if (posX < pos04[0]+hyst && posX > pos04[0]-hyst && posY < pos04[1]+hyst && posY > pos04[1]-hyst){
				path34.reset();
				state = GO_CENTER_STATE;
			}
			
			break;

			
		case GO_CENTER_STATE:
			path40.run();									// calculate positions and velocities periodically
			
			val1 = path40.getVal1();						// posC1[0] = x1, posC1[1] = v1; posC1[2] = a1
			val2 = path40.getVal2();
			
			invKin.run(val1[0], val2[0]);					// calculate phi1,2 with inverted Kinematic
			phi = invKin.getPhi();							// returns phi1 and phi2
			
			invJakob.run(val1[1], val2[1]);				// calculate omega1, 2 with the inverted Jakobian
			omega = invJakob.getOmega();					// returns omega1 and omega2
			
			
//			if(timer >= 250){
//				timer = 0;
//				System.out.print(val1[0]);	
//				System.out.print('\t');
//				System.out.print(val2[0]);	
//				System.out.print('\t');
//				System.out.print(phi[0]);
//				System.out.print('\t');
//				System.out.print(phi[1]);
//	//			System.out.print('\t');
//	//			System.out.print(val2[2]);	
//	//			System.out.print('\t');
//				System.out.print("\r\n");
//			}
//			else{timer++;}
			
			
			val1[0] = phi[0]; val1[1] = omega[0]; val1[2] = 0;
			val2[0] = phi[1]; val2[1] = omega[1]; val2[2] = 0;
			
			mot1.setCtrlValue(val1);
			mot2.setCtrlValue(val2);
			
			posX = kin.getActualPosXY()[0];						// returns actual position of TCP
			posY = kin.getActualPosXY()[1];
									
			if (posX < posCenter[0]+hyst && posX > posCenter[0]-hyst && posY < posCenter[1]+hyst && posY > posCenter[1]-hyst){
				path40.reset();
				state = TURN_FINISHED_STATE;
			}
			
			break;
			
			
		case TURN_FINISHED_STATE:
			m++;
			if (m >= n){
				state =  STOP_STATE;
				break;
			}
			else{
				System.out.print(m);
				System.out.print(" finished");
				System.out.print("\r\n");
				state =  WAIT_TIME_STATE;
			}
			break;
		
		case STOP_STATE:
			
			break;
			
			
		default:
			System.out.print("Failure in HighSpeed");
			mot1.activate(false); mot2.activate(false);
			break;
		}

		
		if (m >= n){
			System.out.print(m);
			System.out.print(" turns finished");
			System.out.print("\r\n");
			m = 0;
			return true;
		}
		else{
			return false;
		}
	}
	
	public void resetHighSpeed(){
		m = 0;
		state = INIT_HIGHSPEED_STATE;
	}
	
	public double[] getVal1(){
		return val1;
	}
}
