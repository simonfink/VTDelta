package ch.ntb.robotics.scara.main;

import java.io.PrintStream;

import ch.ntb.inf.deep.runtime.mpc5200.IphyCoreMpc5200tiny;
import ch.ntb.inf.deep.runtime.mpc5200.Task;
import ch.ntb.inf.deep.runtime.mpc5200.driver.UART3;
import ch.ntb.inf.deep.unsafe.US;
import ch.ntb.robotics.scara.driver.ScaraDriver;
import ch.ntb.robotics.scara.dynamic.DynVeloLimiter;
import ch.ntb.robotics.scara.dynamic.InversJakobian;
import ch.ntb.robotics.scara.kinematic.InversKinematic;
import ch.ntb.robotics.scara.kinematic.Kinematic;
import ch.ntb.robotics.scara.motorcontrol.MtrCtrlPD;

public class TestMotBoard extends Task implements IphyCoreMpc5200tiny {

	double Ts;

	final double[] posCenter = {0.0, 0.25};
	private double hyst = 0.002;					// hysteresis for detection if desired Position reached

	
	int cntMottemp = 0;
	int butCnt = 0;
	int cntHall = 0;
	boolean but = true;
	boolean initFinished = false;
	boolean goCenterFinished = false;
	boolean goCenterFirstTime = false;
	
	// Vorgaben dynVeloLimiter
	final double maxPhi1 = 225*2*Math.PI/360.0;	// [rad] 
	final double minPhi1 = -15*2*Math.PI/360.0;	// [rad] 
	final double maxPhi2 = 195*2*Math.PI/360.0;
	final double minPhi2 = -45*2*Math.PI/360.0;
	final double Jmax = 0.05;		// 0.017 of the gear + 0.028 with 0.15 kg at 0.5 m; only approx. because unknown axis positions
	double maxAlpha = 0.6*6*5.14/Jmax;	// [rad/s^2] approx. 400 rad/s^2;
	final double maxOmega = 25;				// [rad/s] maximal 7000/60/(6*5.14)*2*PI = 25 rad/s
	// maximal and minimal values; minimal = - maximal (expected the position)
	double[] max1 = {maxPhi1, minPhi1, maxOmega, maxAlpha};
	double[] max2 = {maxPhi2, minPhi2, maxOmega, maxAlpha};

	double magFluxMin = -50;				// wenn kein Magnet erkannt wird --> minimalem Wert (MUSS NACHGEPRUEFT WERDEN WAS FUER EIN WERT!!!)
	
	
	double[] phi = new double[2];
	double[] omega = {0.0, 0.0};
	double[] val1 = new double[3];
	double[] val2 = new double[3];
	double[] posC1 = new double[3];
	double[] posC2 = new double[3];
	
	private int ctr = 1000;
	private int timer0 = 0;
	private boolean flag = true;
	
	private int rampcounter = 0;
	private int increment = 100;
	
	
	// Objektinitialisierung
	InitRobotPos irp = new InitRobotPos(hyst);
	GoCenter goCent = new GoCenter(posCenter);
	Stop stop = new Stop();
	Kinematic kin = new Kinematic();	
	InversKinematic invKin = new InversKinematic();
	InversJakobian invJakob = new InversJakobian();
	DynVeloLimiter dynVeloLimiter = new DynVeloLimiter(max1, max2);
	
	HighSpeed highSpeed;
	Balancing balance;
	MtrCtrlPD mot1;
	MtrCtrlPD mot2;
	
	
	// Statesinitialisierung
	private final int INIT_ROBOTPOS_STATE = 0;
	private final int STOP_STATE = 1;
	private final int GO_CENTER_STATE = 2;
	private final int HIGHSPEED_STATE = 3;
	private final int WAIT_FOR_PENDULUM_STATE = 4;
	private final int BALANCING_STATE = 5;
	private int state = INIT_ROBOTPOS_STATE; 
	

	
	public TestMotBoard(float Ts) {
		this.Ts = Ts;
		// Initialize UART3 (9600 8N1) and use it for stdout and stderr
		UART3.start(9600, UART3.NO_PARITY, (short)8);
		System.out = new PrintStream(UART3.out);
		System.err = System.out;
		System.out.print("Motor Test started...\n\r");
		
		// Initialize SPI
		ScaraDriver.init();
				
		// Initialize GPIO7 as digital output for inverted led
		US.PUT4(GPWER, US.GET4(GPWER) | 0x80000000); // enable GPIO use
		US.PUT4(GPWDDR, US.GET4(GPWDDR) | 0x80000000); // make output

		
		// velocity control because of unknown axis positions	
		mot1 = new MtrCtrlPD(
				Ts, 		// Sampling time
				false, 		// use position control
				0, 			// motor nr
				100.0, 		// omega regel
				-6.0*72.0/14.0,	// gear i	(- to change direction of arm1)			RE 25:	14
				3e-02,	// J	(0.00000108+0.00000052)*i*i 			RE 25 20 Watt + GP 26 B = 3.136e-04
				0.0603, 	// motor constant
				24.0, 		// maximal voltage 
				1.0,			// gain current to output-value  (S.64 Doku David)
				1024,		// encoder ticks								RE 25: 1000		
				1.13		// internal motor resistor						RE 25:	2.32
		);	
		// limit the velocity, because the possible current is to low to accelerate fast
		mot1.setVelocity(maxOmega);
		
		mot2 = new MtrCtrlPD(
				Ts, 		// Sampling time
				false, 		// use position control
				1, 			// motor nr
				100.0, 		// omega regel
				6.0*72.0/14.0,	// gear i										RE 25:	14
				3e-02,	// J	(0.00000108+0.00000052)*i*i 			RE 25 20 Watt + GP 26 B = 3.136e-04
				0.0603, 	// motor constant
				24.0, 		// maximal voltage 
				1.0,			// gain current to output-value  (S.64 Doku David)
				1024,		// encoder ticks								RE 25: 1000		
				1.13		// internal motor resistor						RE 25:	2.32
		);	
		// limit the velocity, because the possible current is to low to accelerate fast
		mot2.setVelocity(maxOmega);
		
		highSpeed = new HighSpeed(Ts);
		balance = new Balancing(posCenter, Ts);

		irp.mot1 = mot1; irp.mot2 = mot2;
		goCent.mot1 = mot1; goCent.mot2 = mot2; goCent.invKin = invKin;
		kin.mot1 = mot1; kin.mot2 = mot2;
		invJakob.mot1 = mot1; invJakob.mot2 = mot2;
		highSpeed.mot1 = mot1; highSpeed.mot2 = mot2; highSpeed.kin = kin; highSpeed.invKin = invKin; highSpeed.invJakob = invJakob;
		stop.mot1 = mot1; stop.mot2 = mot2;
		dynVeloLimiter.mot1 = mot1; dynVeloLimiter.mot2 = mot2;
		balance.mot1 = mot1; balance.mot2 = mot2; balance.kin = kin;

		
		ScaraDriver.activateServo(0, true);	
	}
	
	public void action(){
		ScaraDriver.transceive();
		
		
		// timeout für SPI-Initialisierung
		if(flag && timer0 < 500){timer0++; }		//mot1.enc.reset(); mot2.enc.reset();
		else{flag = false; timer0 = 0; }
		
		if(flag == false){
			
			// ignore button pushes after a push for 1 second
			if (but == false){butCnt++;}
			if (butCnt >= 200){but = true;	butCnt = 0;}
//			
////			if(ScaraDriver.getResetState()){ScaraDriver.setCurrentInt(0, (int)(0)); ScaraDriver.setCurrentInt(1, (int)(0)); mot1.setVelocity(0); mot2.setVelocity(0);}
//			mot1.run();
//			mot2.run();
//			
//			if(goCenterFirstTime){
//				dynVeloLimiter.colisionProtect();
//			}
//			
			if(ScaraDriver.getDIButton(0) && but){				// button 2 (middle, red) for Stop
				but = false;
				System.out.print("STOP");
				System.out.print("\r\n");
				state = STOP_STATE;
			}
			
			

			

//			if(ScaraDriver.getDIButton(3) == false){		// Temperatur protection for motors
////				but = false;
//				System.out.print("High Motor Temperatur");
//				cntMottemp++;
//				
//				if (cntMottemp > 10*1000){					// if Mot temp after 10 s is still too high --> stop
//					System.out.print("Abort: Motor Temperatur to high");
//					cntMottemp = 0;
//					state = STOP_STATE;
//				}
//			}
//			else{
//				cntMottemp = 0;
//			}

		
			switch(state){
			
			// Initial the robot position at right end stop
			case INIT_ROBOTPOS_STATE:	
				if(irp.run()){		// wait for return true = finished
					initFinished = true;
					System.out.print("Initialising Done");
					System.out.print("\r\n");
					state = GO_CENTER_STATE;
				}
				else{
					initFinished = false;
				}
				break;
			
				
			// go to main position and wait for button push
			case GO_CENTER_STATE: 
				if(goCent.run()){		// wait for return true = finished
					ScaraDriver.setLED(0,2);
					ScaraDriver.setLED(1,3);
					ScaraDriver.setLED(2,2);
					ScaraDriver.setLED(3,3);
					goCenterFinished = true;
					goCenterFirstTime = true;
					
					if(ScaraDriver.getDIButton(0) && but){						// button 1(left, green) High-Speed Demo 
						but = false;
//						if(ScaraDriver.getADCHallSens(0) >= 50){
							System.out.print("High-Speed Mode");
							System.out.print("\r\n");
							highSpeed.resetHighSpeed();
							state = HIGHSPEED_STATE;
//						}
					}
					if(ScaraDriver.getDIButton(2)  && but){	// button 3 (right, blue) balancing and pendulum detected
						but = false;
//						if(ScaraDriver.getADCHallSens(0) >= 50){
							System.out.print("Balancing Mode");
							System.out.print("\r\n");
//							balance.reset();
//							mot1.setVelocity(maxOmega);
//							mot2.setVelocity(maxOmega);
//							cntHall = 0;
							highSpeed.resetHighSpeed();
							state = HIGHSPEED_STATE;
//						}
//						else{	
//							float hs0 = ScaraDriver.getADCHallSens(0);
//							System.out.print(hs0);
//							System.out.print("\t\t");
//						}
					}
				}
				else{
					goCenterFinished = false;
				}
				break;
				
				
			//	if switch 1 pressed: do high speed demo
			case HIGHSPEED_STATE: 
				if (highSpeed.run(5)){ 			// passes 5x / has to return true if done 5x
					goCent.reset();
					state = GO_CENTER_STATE;
				}
				break;
				
			
				
			case BALANCING_STATE:
			
//				double phiSx = 0.00175 * (ScaraDriver.getADCHallSens(0) - ScaraDriver.getADCHallSens(2));		// angel of pendulum in Sensor coordinates in X-direction
//				double phiSy = 0.00175 * (ScaraDriver.getADCHallSens(3) - ScaraDriver.getADCHallSens(1));		// angel of pendulum in Sensor coordinates in y-direction

				if (cntHall >= 1000) {
					balance.run();
					
//					System.out.print(phiSx);
//					System.out.print("\t\t");
//					System.out.print(phiSy);
//					System.out.print("\t\t");
					
				
//					float hs0 = ScaraDriver.getADCHallSens(0);
//					System.out.print(hs0);
//					System.out.print("\t\t");
//					float hs1 = ScaraDriver.getADCHallSens(1);
//					System.out.print(hs1);
//					System.out.print("\t\t");
//					float hs2 = ScaraDriver.getADCHallSens(2);
//					System.out.print(hs2);
//					System.out.print("\t\t");
//					float hs3 = ScaraDriver.getADCHallSens(3);
//					System.out.print(hs3);
//					System.out.print("\n\r");
//
//					cntHall = 0;
				}
				else {cntHall++;}
				
				if (ScaraDriver.getADCHallSens(0) <= 50){
					System.out.print("Balancing Programm: Wait for Pendulum\n\r");
					System.out.print("\r\n");
					state = STOP_STATE;
				}

				break;
				
			// if switch 2 pressed: STOP as fast as possible
			case STOP_STATE:
				if(stop.run(maxOmega)){
					
					if(initFinished){								// if initialising is finished
						ScaraDriver.setLED(1,3);
						ScaraDriver.setLED(3,3);
						if(ScaraDriver.getDIButton(0) && but){			// button 1(left, green) go to center 
							but = false;
							System.out.print("Init RobotPos Done");
							System.out.print("\r\n");
							goCent.reset();
							state = GO_CENTER_STATE;
						}
						if(ScaraDriver.getDIButton(2) && but){			// button 3 (right, blue) go to center
							but = false;
							System.out.print("Init RobotPos Done");
							System.out.print("\r\n");
							goCent.reset();
							state = GO_CENTER_STATE;
						}
					}
					else{											// if initialising wasn't finished
						if(ScaraDriver.getDIButton(0) && but){	
							but = false;
							irp.resetInitRobotPos();
							state = INIT_ROBOTPOS_STATE;
						}
	
						if(ScaraDriver.getDIButton(2) && but){	
							but = false;
							irp.resetInitRobotPos();
							state = INIT_ROBOTPOS_STATE;
						}
					}		
				}
				break;	
				
			default:
				mot1.setVeloCtrl();
				mot2.setVeloCtrl();
				
				state = STOP_STATE;
				break;
			}
			

			
			
			if(ctr >= 500) {
				
				rampcounter = rampcounter + increment;
				ScaraDriver.setCurrentInt(0, rampcounter);
				if(rampcounter >= 1500  || rampcounter <= -1500){
					increment = -increment;
				}
				
				
				System.out.print(rampcounter);
				System.out.print('\t');
//					
//				int hs0 = ScaraDriver.getADCHallSens(0);
//				System.out.print(hs0);
//				System.out.print("\t\t");
//				int hs1 = ScaraDriver.getADCHallSens(1);
//				System.out.print(hs1);
//				System.out.print("\t\t");
//				int hs2 = ScaraDriver.getADCHallSens(2);
//				System.out.print(hs2);
//				System.out.print("\t\t");
//				int hs3 = ScaraDriver.getADCHallSens(3);
//				System.out.print(hs3);
//				System.out.print("\t\t");
//				
//				
////				pos[0] = -pos[0];
////				pos[1] = -pos[1];
//				
////				if(ScaraDriver.getResetState()){System.out.print("stop");}
////				System.out.print(ScaraDriver.getPosShort(0)); // read encoder
////				System.out.print('\t');
//				
////				System.out.print(irp.getState()); // read encoder
////				System.out.print('\t');
////				
////				float hs3 = ScaraDriver.getADCHallSens(3);
////				System.out.print(hs3);
////				System.out.print(ScaraDriver.getVeloShort(n)); 
//				
////				System.out.print(start[0]); 
////				System.out.print('\t');
////				System.out.print(start[1]); 
////				System.out.print('\t');
//				
////				val1 = bapla.getVal1();							// Val1[0] = x1, Val1[1] = v1; Val1[2] = a1
////				val2 = bapla.getVal2();
////				System.out.print(bapla.getDv()[0]);
////				System.out.print('\t');
////				System.out.print(bapla.getDv()[1]);
////				System.out.print('\t');
////				System.out.print(bapla.getTslow());	
//////				System.out.print('\t');
////				System.out.print("\r\n");
//				
////				val1 = highSpeed.getVal1();							// Val1[0] = x1, Val1[1] = v1; Val1[2] = a1
//////				val2 = bapla.getVal2();
////				System.out.print(val1[0]);
////				System.out.print('\t');
////				System.out.print(val1[1]);
////				System.out.print('\t');
////				System.out.print(val1[2]);	
////				System.out.print('\t');
//				
////				System.out.print(posC1[0]); 
////				System.out.print('\t');
////				System.out.print(posC2[0]); 
////				System.out.print('\t');
////				System.out.print((float)phi[0]); 
////				System.out.print('\t');
////				System.out.print((float)phi[1]); 
////				System.out.print('\t');
////				System.out.print(omega[0]); 
////				System.out.print('\t');
////				System.out.print(omega[1]); 
////				System.out.print('\t');
//				
////				System.out.print('\t');
////				System.out.print(val1[1]); 
////				System.out.print('\t');
////				System.out.print(val1[2]); 
////				System.out.print('\t');
//				
//				
////				System.out.print(val2[1]); 
////				System.out.print('\t');
////				System.out.print(val2[2]); 
////				System.out.print('\t');
//
//
////				System.out.print(ScaraDriver.getVeloShort(0)); 
////				System.out.print(ScaraDriver.getPosShort(0)); 
////				System.out.print((float)mot1.enc.getPos()); 
////				System.out.print('\t');
//////				System.out.print((float)mot1.getActVelo()); 
//////				System.out.print('\t');
//////				System.out.print(mot1.getMotTorque()); 
//////				System.out.print('\t');
//////				
////////				System.out.print(ScaraDriver.getPosShort(1)); 
////				System.out.print((float)mot2.enc.getPos()); 
////				System.out.print('\t');
////				System.out.print((float)mot2.getActVelo()); 
////				System.out.print('\t');
////				System.out.print(mot2.getMotTorque()); 
////				System.out.print('\t');
//				
//				
////				System.out.print(i); 
////				System.out.print('\t');
////				System.out.print(mot0.getMaxVelo()); // read encoder
////				System.out.print('\t');
//				
				System.out.print("\r\n");
//////				
////				i += di;		
////				if(i<= -1790f || i>=1790f){di = -di;}
//				
					ctr = 0;
			}

			ctr++;
		}
		ScaraDriver.resetWatchdog();

	}
	
}





