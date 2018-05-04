package ch.ntb.robotics.scara.main;

import ch.ntb.robotics.scara.driver.ScaraDriver;
import ch.ntb.robotics.scara.motorcontrol.MtrCtrlPD;

public class InitRobotPos {
	
	double[] pos1 = {0, -1, 0};			// pos mot 0 = 0; vel = 3.14
	double[] pos2 = {0, 1, 0};			// pos mot 0 = 0; vel = 3.14

	
	double hyst;					// hysteresis for detection if desired Position reached
	private final double initTorque = 0.2;		// defined motor torque at end stop
	
	private int cutPeakControlValue = 0;
	private final int ignoreTimePCV = 1000;		// time (ms) to ignore Peak Control Values
	
	int waitTimer = 0;
	
	MtrCtrlPD mot1;
	MtrCtrlPD mot2;

	private final int INIT_STATE = 0;
	private final int WAIT_STATE = 1;
	private final int TURN_RIGHT_STATE = 2;
	private final int TURN_LEFT_STATE = 3;
	private final int SET_ENC_STATE = 4;
	private final int INIT_FINISHED_STATE = 5;
	private final int STOP_STATE = 6;
	
	private int state = INIT_STATE;
	
	/**
	*	object constructor
	*	!! motor controller is set to velocity control instead of unknown axis positions !!
	*/
	public InitRobotPos(double hyst){
		this.hyst = hyst;
	}
	
	/**
	*	running method for initializing the robot position
	*@param	max1
	*			maximal values for axis 1 [phiMax, omegaMax, alphaMax]
	*@param	max2
	*			maximal values for axis 2 [phiMax, omegaMax, alphaMax]
	*/
	public boolean run(){
//		System.out.print("test1");
//		return false;
//		mot1.enc.getPos();
		
		switch (state) {
		
		// initialising
		case INIT_STATE:
			doInitState();	
			break;
			
		// wait 5 sec
		case WAIT_STATE:
			doWaitState();
			break;
			
		// turn to right stop until current 1 and 2 is lower than initTorque
		case TURN_RIGHT_STATE:
			doTurnRightState();
			break;
			
		// set encoder1 to -15° and enc2 to -45°
		case SET_ENC_STATE:
			doSetEncState();
			break;
			
		case TURN_LEFT_STATE:
			doTurnLeftState();
			break;
			
		// flag that initialising is finished
		case INIT_FINISHED_STATE:
			break;
			
		case STOP_STATE:
			mot1.activate(false);
			mot2.activate(false);
			break;
			
		default:
			System.out.print("Failure in InitRobotPos");
			System.out.print("\r\n");
			break;
		}
		
		if (state == INIT_FINISHED_STATE){
			return true;
		}
		else{
			return false;
		}
		
	}

	/**
	*	initialize program
	*/
	public void doInitState(){
		mot1.enc.reset(); mot2.enc.reset();
		
		mot1.setVeloCtrl();
		mot2.setVeloCtrl();
		
		mot1.setVelocity(Math.PI/2);
		mot2.setVelocity(Math.PI/2);
		
		ScaraDriver.setLED(0,1);				// toggle all LEDs fast --> attention please, keep away
		ScaraDriver.setLED(1,1);
		ScaraDriver.setLED(2,1);
		ScaraDriver.setLED(3,1);
		
		System.out.print("init state done!!");
		System.out.print("\r\n");
		
		state = WAIT_STATE;
	}
	
	/**
	*	wait for 5 seconds to ensure every person has kept away
	*/
	public void doWaitState(){
		waitTimer++;
		if(waitTimer >= 2000){					// wait for 5 seconds
			waitTimer = 0;
			
			System.out.print("wait state done!!");
			System.out.print("\r\n");
			
			mot1.activate(true);
			mot2.activate(false);
			state = TURN_RIGHT_STATE;
		}
	}
	
	/**
	*	turn right to end stop with axis 1 and axis 2
	*/
	public void doTurnRightState(){
		mot1.setCtrlValue(pos1);
		mot2.setCtrlValue(pos1);
					
		if(mot1.getMotTorque() >= initTorque){
			cutPeakControlValue++;
			if (cutPeakControlValue >= ignoreTimePCV) {		// Peak-ControlValue ignored for xx ms
				cutPeakControlValue = 0;
				state = SET_ENC_STATE;
			}
		}
		else{
			cutPeakControlValue = 0;
		}
	}
	
	/**
	*	set the encoders to end stop positions
	*/
	public void doSetEncState(){
		mot1.setEncPos(-16.0*2.0*Math.PI/360.0);
		mot2.setEncPos(-46.0*2.0*Math.PI/360.0);
		
		System.out.print("set enc state done");
		System.out.print('\t');
		System.out.print(mot1.enc.getPos()); 
		System.out.print("\r\n");
		
		mot1.activate(false);
		mot2.activate(true);
		state = TURN_LEFT_STATE;
	}
	
	public void doTurnLeftState(){
//		System.out.print("do turn right state");

		mot1.setCtrlValue(pos2);
		mot2.setCtrlValue(pos2);
					
		if(mot2.getMotTorque() >= initTorque){
			cutPeakControlValue++;
			if (cutPeakControlValue >= ignoreTimePCV) {		// Peak-ControlValue ignored for xx ms
				cutPeakControlValue = 0;
				
				if(mot1.enc.getPos() < 220*2*Math.PI/360){
					System.out.print("Hindernis im Weg");
					System.out.print('\t');
					System.out.print(mot1.enc.getPos()); 
					System.out.print("\r\n");
					state = STOP_STATE;
				}
				else{		
					System.out.print(mot1.enc.getPos()); 
					System.out.print('\t');
					System.out.print("turn left done");
					System.out.print("\r\n");
					state = INIT_FINISHED_STATE;
				}
			}
		}
		else{
			cutPeakControlValue = 0;
		}
	}
	
	public void resetInitRobotPos(){
		state = INIT_STATE;
	}
	
	public int getState(){
		return state;
	}
	
}
