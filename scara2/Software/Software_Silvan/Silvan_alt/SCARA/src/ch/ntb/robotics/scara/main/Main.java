package ch.ntb.robotics.scara.main;

import java.io.PrintStream;

import ch.ntb.inf.deep.runtime.mpc5200.IphyCoreMpc5200tiny;
import ch.ntb.inf.deep.runtime.mpc5200.Task;
import ch.ntb.inf.deep.runtime.mpc5200.driver.UART3;
import ch.ntb.inf.deep.unsafe.US;
import ch.ntb.robotics.scara.driver.ErrorHandler;
import ch.ntb.robotics.scara.driver.ScaraDriver;
import ch.ntb.robotics.scara.dynamic.DynVeloLimiter;
import ch.ntb.robotics.scara.motorcontrol.MtrCtrlPD;

public class Main extends Task implements IphyCoreMpc5200tiny{
	double Jmax = 0.05;		// 0.017 of the gear + 0.028 with 0.15 kg at 0.5 m; only approx. because unknown axis positions
	double maxAlpha = 0.6*6*5.14/Jmax;	// [rad/s^2] approx. 400 rad/s^2;
	double maxOmega = 1;				// [rad/s] maximal 7000/60/(6*5.14)*2*PI = 25 rad/s
	double magFluxMin = 10000000;				// wenn kein Magnet erkannt wird --> minimalem Wert (MUSS NACHGEPRUEFT WERDEN WAS FUER EIN WERT!!!)
	
	final double maxPhi1 = 225*2*Math.PI/360;	// [rad] 
	final double minPhi1 = -15*2*Math.PI/360;	// [rad] 
	final double maxPhi2 = 195*2*Math.PI/360;
	final double minPhi2 = -45*2*Math.PI/360;
	
	final double[] posCenter = {0, 0.25};
	private double hyst = 0.002;					// hysteresis for detection if desired Position reached

	// maximal and minimal values; minimal = - maximal (expected the position)
	double[] max1 = {maxPhi1, minPhi1, maxOmega, maxAlpha};
	double[] max2 = {maxPhi2, minPhi2, maxOmega, maxAlpha};
	
	MtrCtrlPD mot1;
	MtrCtrlPD mot2;
	
	private double Ts;
	
	public InitRobotPos irp = new InitRobotPos(hyst);
	public Stop stop = new Stop();
	public GoCenter goCent = new GoCenter(posCenter);
	public DynVeloLimiter dynVelLim = new DynVeloLimiter(max1, max2);
	public HighSpeed highSpeed = new HighSpeed(Ts);
	public Balancing bal = new Balancing(posCenter, Ts);
	
	int cnt=0;
	int cntMottemp = 0;
		
	private final int INIT_ROBOTPOS_STATE = 0;
	private final int STOP_STATE = 1;
	private final int GO_CENTER_STATE = 2;
	private final int HIGHSPEED_STATE = 3;
	private final int BALANCING_STATE = 4;
	private int state = INIT_ROBOTPOS_STATE; 
	
	boolean initFinished = false;
	boolean goCenterFinished = false;
	
	
	
	/**
	*	object constructor
	*@param	Ts
	*			sampling time
	*/
	public Main(final double Ts){
		
		this.Ts = Ts;
		
		// Initialize UART (9600 8N1)
		// Use the UART3 for stdout and stderr
		// Print a string to the stdout
		UART3.start(9600, UART3.NO_PARITY, (short)8);
		System.out = new PrintStream(UART3.out);
		System.err = System.out;
		System.out.print("\n\n\nPROJEKT SCARA\n\r");
		
		// Initialize SPI
		System.out.print("INITIALIZE DRIVER ... ");
		ScaraDriver.init();
		System.out.print("DONE\n");
		
		// INIT GPIO7 as digital output for inverted led
		System.out.print("INITIALIZE GPIO LED ...");
		US.PUT4(GPWER, US.GET4(GPWER) | 0x80000000);	// enable GPIO use
		US.PUT4(GPWDDR, US.GET4(GPWDDR) | 0x80000000);	// make output
		System.out.print("DONE\n");
			
		mot1 = new MtrCtrlPD(
				Ts, 		// Sampling time
				false, 		// use velocity control
				0, 			// motor nr
				100.0, 		// omega regel
				6*72/14, 	// gear i
				1.37e-05,	// J						// approx. because positions of axis unknown
				0.0603, 	// motor constant
				24, 		// maximal current  maxM = maxI*km
				1,		// gain current to output-value  (S.64 Doku David)
				1024,		// encoder ticks
				1.13		// internal motor resistor
		);	
		// limit the velocity, because the possible current is to low to accelerate fast
		mot1.setVelocity(maxOmega);
		
		
		// velocity control because of unknown axis positions	
		mot2 = new MtrCtrlPD(
				Ts, 		// Sampling time
				false, 		// use velocity control
				1, 			// motor nr
				100.0, 		// omega regel
				6*72/14, 	// gear i
				1.37e-05,	// J						// approx. because positions of axis unknown
				0.0603, 	// motor constant
				24, 		// maximal current  maxM = maxI*km
				1,		// gain current to output-value  (S.64 Doku David)
				1024,		// encoder ticks
				1.13		// internal motor resistor
		);	
		// limit the velocity, because the possible current is to low to accelerate fast
		mot2.setVelocity(maxOmega);
		
	}
		

	/**
	*	running main method
	*/
	public void action(){
		
		ScaraDriver.transceive();
		if(ScaraDriver.getResetState()){state = STOP_STATE;}
		ErrorHandler.checkInputs();
		dynVelLim.run();
		

		if(ScaraDriver.getDIButton(1)){				// button 2 (middle, red) for Stop
			System.out.print("STOP");
			state = STOP_STATE;
		}

		if(ScaraDriver.getDIButton(3) == false){		// Temperatur protection for motors
			System.out.print("High Motor Temperatur");
			cntMottemp++;
			if (cntMottemp > 10*1000){					// if Mot temp after 10 s is still too high --> stop
				System.out.print("Abort: Motor Temperatur to high");
				cntMottemp = 0;
				state = STOP_STATE;
			}
		}
		else{
			cntMottemp = 0;
		}

		
		
		switch(state){
		// Initial the robot position at right end stop
		case INIT_ROBOTPOS_STATE:	
			if(irp.run()){		// wait for return true = finished
				initFinished = true;
				System.out.print("Initialising Done");
				state = GO_CENTER_STATE;
			}
			else{
				initFinished = false;
			}
			break;
			
		// if switch 2 pressed: STOP as fast as possible
		case STOP_STATE:
			stop.run(maxOmega);
			
			if(mot1.getActVelo()== 0 && mot2.getActVelo()== 0){		// wait for button push when robot stoped
				if(initFinished){								// if initialising is finished
					ScaraDriver.setLED(1,3);
					ScaraDriver.setLED(3,3);
					if(ScaraDriver.getDIButton(0)){			// button 1(left, green) go to center 
						System.out.print("Init RobotPos Done");
						state = GO_CENTER_STATE;
					}
					if(ScaraDriver.getDIButton(2)){			// button 3 (right, blue) go to center
						System.out.print("Init RobotPos Done");
						state = GO_CENTER_STATE;
					}
				}
				else{											// if initialising wasn't finished
					if(ScaraDriver.getDIButton(0)){				
						state = INIT_ROBOTPOS_STATE;
					}
					if(ScaraDriver.getDIButton(1)){				
						state = INIT_ROBOTPOS_STATE;
					}
					if(ScaraDriver.getDIButton(2)){				
						state = INIT_ROBOTPOS_STATE;
					}
				}		
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
				
				double hallSens0 = ScaraDriver.getADCHallSens(0);
				double hallSens1 = ScaraDriver.getADCHallSens(1);
				double hallSens2 = ScaraDriver.getADCHallSens(2);
				double hallSens3 = ScaraDriver.getADCHallSens(3);
				
				if(ScaraDriver.getDIButton(0)){				// button 1(left, green) High-Speed Demo 
					System.out.print("Go to Center Done");
					state = HIGHSPEED_STATE;
				}
				if(ScaraDriver.getDIButton(2) && hallSens0+hallSens1+hallSens2+hallSens3 >= magFluxMin){	// button 3 (right, blue) balancing and pendulum detected
					System.out.print("Go to Center Done");
					state = BALANCING_STATE;
				}
			}
			else{
				goCenterFinished = false;
			}
			break;
			
		//	if switch 1 pressed: do high speed demo
		case HIGHSPEED_STATE: 
			highSpeed.run(5);						// passes 5x / has to return true if done 5x
			
			if (highSpeed.run(5)){
				state = STOP_STATE;
			}
			break;

			
		// if switch 3 pressed: do balancing
		case BALANCING_STATE:

			double hallSens0 = ScaraDriver.getADCHallSens(0);
			double hallSens1 = ScaraDriver.getADCHallSens(1);
			double hallSens2 = ScaraDriver.getADCHallSens(2);
			double hallSens3 = ScaraDriver.getADCHallSens(3);
			
			if(hallSens0+hallSens1+hallSens2+hallSens3 <= magFluxMin ){			// if no pendelum (magnet) is detected --> Stop
				state = STOP_STATE;
			}
			else{
				bal.run();
			}

			break;
			
		default:
			state = STOP_STATE;
			break;
		}
		
	}	
}
	


