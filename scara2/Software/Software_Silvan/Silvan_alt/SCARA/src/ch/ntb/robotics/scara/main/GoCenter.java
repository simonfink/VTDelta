package ch.ntb.robotics.scara.main;

import ch.ntb.robotics.scara.driver.ScaraDriver;
import ch.ntb.robotics.scara.kinematic.InversKinematic;
import ch.ntb.robotics.scara.motorcontrol.MtrCtrlPD;

public class GoCenter {

	private double hystRad = 0.01;					// hysteresis for detection if desired Position reached
	double[] posCenter = new double[2];
	double[] phi = new double[2];
	double[] p1 = new double[3];
	double[] p2 = new double[3];
	
	MtrCtrlPD mot1;
	MtrCtrlPD mot2;
	InversKinematic invKin;
	
	
	private final int INIT_GOCENTER_STATE = 0;
	private final int GO_CENTER_STATE = 1;
	private final int GOCENTER_FINISHED_STATE = 2;
	
	private int state = INIT_GOCENTER_STATE; 
	
	
	/**
	*	object constructor
	*@param	posCenter
	*			main position from where all sub programs start
	*@param	maxOmega
	*			maximal Speed of axis 1 and axis 2
	*@param	Ts
	*			sampling time
	*/
	public GoCenter(double[] posCenter){
		this.posCenter[0] = posCenter[0];
		this.posCenter[1] = posCenter[1];
	}

	/**
	*	running method to go to center position
	*@return	true; when finished; else false
	*/
	public boolean run(){
		
		switch(state){
		
		case INIT_GOCENTER_STATE:
			ScaraDriver.setLED(0,1);				// toggle all LEDs fast --> attention please, keep away
			ScaraDriver.setLED(1,1);
			ScaraDriver.setLED(2,1);
			ScaraDriver.setLED(3,1);
			
			mot1.activate(true);
			mot2.activate(true);
			
			mot1.setVelocity(1);
			mot2.setVelocity(1);
			
			mot1.setPosCtrl();				// set controller to position controller
			mot2.setPosCtrl();
	
			invKin.run(posCenter[0], posCenter[1]);					// calculate phi1,2 with inverted Kinematic
			phi[0] = invKin.getPhi()[0];	
			phi[1] = invKin.getPhi()[1];	
			
			p1[0] = phi[0]; p1[1] = 0; p1[2] = 0;
			p2[0] = phi[1]; p2[1] = 0; p2[2] = 0;
			
			System.out.print("init go center done");
			System.out.print("\t");
			System.out.print(phi[0]);
			System.out.print("\t");
			System.out.print(phi[1]);
			System.out.print("\r\n");
			state = GO_CENTER_STATE;

				
		case GO_CENTER_STATE:
			mot1.setCtrlValue(p1);
			mot2.setCtrlValue(p2);
			
			double phi1 = mot1.enc.getPos();						// returns actual position of TCP
			double phi2 = mot2.enc.getPos();
			
			if (phi1 < phi[0]+hystRad && phi1 > phi[0]-hystRad && phi2 < phi[1]+hystRad && phi2 > phi[1]-hystRad){
				System.out.print("robot in center");
				System.out.print("\r\n");
				state = GOCENTER_FINISHED_STATE;
			}
			
			break;
			
		case GOCENTER_FINISHED_STATE:
			break;
			
		default:
			break;
		}
		
		
		if (state == GOCENTER_FINISHED_STATE){
			return true;
		}
		else{
			return false;
		}
	}
	
	public void reset(){
		state = INIT_GOCENTER_STATE;
	}
	
}
