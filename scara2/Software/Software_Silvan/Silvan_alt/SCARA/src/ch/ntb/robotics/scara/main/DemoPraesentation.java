package ch.ntb.robotics.scara.main;
	
import java.io.PrintStream;

import ch.ntb.inf.deep.runtime.mpc5200.IphyCoreMpc5200tiny;
import ch.ntb.inf.deep.runtime.mpc5200.Task;
import ch.ntb.inf.deep.runtime.mpc5200.driver.UART3;
import ch.ntb.inf.deep.unsafe.US;
import ch.ntb.robotics.scara.bahnplaner.Bahnplaner;
import ch.ntb.robotics.scara.driver.ScaraDriver;
import ch.ntb.robotics.scara.kinematic.Kinematic;

public class DemoPraesentation extends Task implements IphyCoreMpc5200tiny{

	double[] start = {-0.25, 0.25};
	double[] ende = {0.25, 0.25};
	double[] val;
	double Ts;
	
	int cnt = 0;
	int cntPrintVal = 100;
	int cntInit = 0;
	int cntHall = 0;
	boolean f = true;
	boolean initFinished = false;
	
	private double hyst = 0.003;					// hysteresis for detection if desired Position reached
	
	Bahnplaner bapla;
	Kinematic kin;
	
	private final int INIT_STATE = 0;
	private final int MAIN_STATE = 1;
	private final int STOP_STATE = 2;
	private final int CALC_PATH_STATE = 3;
	private final int HIGHSPEED_STATE = 4;
	private final int WAIT_FOR_PENDULUM_STATE = 5;
	private final int BALANCING_STATE = 6;

	private int state = INIT_STATE; 
	
	
	public DemoPraesentation(float Ts){
		this.Ts = Ts;
		// Initialize UART (9600 8N1)
		// Use the UART3 for stdout and stderr
		// Print a string to the stdout
		UART3.start(9600, UART3.NO_PARITY, (short)8);
		System.out = new PrintStream(UART3.out);
		System.err = System.out;
		System.out.print("PROJEKT SCARA\n\r");
		
		// Initialize SPI
		System.out.print("INITIALIZE DRIVER ... ");
		ScaraDriver.init();
		System.out.print("DONE\n\r");
		
		// INIT GPIO7 as digital output for inverted led
		System.out.print("INITIALIZE GPIO LED ...");
		US.PUT4(GPWER, US.GET4(GPWER) | 0x80000000);	// enable GPIO use
		US.PUT4(GPWDDR, US.GET4(GPWDDR) | 0x80000000);	// make output
		System.out.print("DONE\n\r");
		
//		bapla = new Bahnplaner(start, ende, Ts);
	}
	

	
	/**
	 *	test program for microcontroller board
	*/
	
	public void action(){
		ScaraDriver.transceive();
		
		// ignore button pushes after a push for 1 second
		if (f == false){cnt++;}
		if (cnt >= 200){f = true;	cnt = 0;}
		
		// check continuously if stop button is pushed
		if(ScaraDriver.getDIButton(1) && f){
			System.out.print("\n");
			System.out.print("STOP\n\r");
			f = false;
			state = STOP_STATE;
		}
		
		
		switch(state){
		
		case INIT_STATE:
			// init LEDs
			ScaraDriver.setLED(0,1);
			ScaraDriver.setLED(1,1);
			ScaraDriver.setLED(2,1);
			ScaraDriver.setLED(3,3);
			
			if(cntInit==3000){
				initFinished = true;
				cntInit = 0;
				System.out.print("Initialize Robot ...DONE\n\r");
				state = MAIN_STATE;
			}
			else{
				initFinished = false;
				cntInit++;
			}
			break;
			
			
		case MAIN_STATE:
			
			// set LEDs 1+3 toggeling, set other LEDs on
			ScaraDriver.setLED(0,2);
			ScaraDriver.setLED(1,3);
			ScaraDriver.setLED(2,2);
			ScaraDriver.setLED(3,3);

			if(ScaraDriver.getDIButton(0) && f){
				System.out.print("\n");
				System.out.print("High-Speed Programm\n\r");
				f = false;
				state = CALC_PATH_STATE;
			}
			
			if(ScaraDriver.getDIButton(2) && f){
				System.out.print("\n");
				System.out.print("Balancing Programm: Wait for Pendulum\n\r");
				f = false;
				state = WAIT_FOR_PENDULUM_STATE;
			}
			break;
			

		case STOP_STATE:
			if(initFinished){
				System.out.print("Program stoped --> Main state\n\r");
				state = MAIN_STATE;
			}
			else{
				System.out.print("Program stoped without initializing was finish --> Initialize\n\r");
				cntInit = 0;
				state = INIT_STATE;
			}
			break;
			
		case CALC_PATH_STATE:
			bapla.calcPath(start, ende, Ts);
			state = HIGHSPEED_STATE;
			break;
			
		case HIGHSPEED_STATE:
			ScaraDriver.setLED(0,3);
			ScaraDriver.setLED(1,3);
			ScaraDriver.setLED(2,3);
			ScaraDriver.setLED(3,3);
		
			bapla.run();
			
			if(cntPrintVal >= 39){				// print every 0.01 second a value
				val = bapla.getVal1();			// [phi1, phi2]
				System.out.print(val[0]);
				System.out.print("\t\t");
				System.out.print(val[1]);
				System.out.print("\t\t");
				System.out.print(val[2]);
				System.out.print("\n\r");
				cntPrintVal = 0;
				
				double posX = kin.getActualPosXY()[0];
				
				if (posX < ende[0]+hyst && posX > ende[0]-hyst){
				System.out.print("High-Speed Programm finished\n\r");
					
					cntPrintVal = 100;
					state = MAIN_STATE;
				}
			}
			else{
				cntPrintVal++;
			}

			break;
		
		case WAIT_FOR_PENDULUM_STATE:	
			ScaraDriver.setLED(0,3);
			ScaraDriver.setLED(1,2);
			ScaraDriver.setLED(2,3);
			ScaraDriver.setLED(3,3);
			
			if(ScaraDriver.getADCHallSens(0) < -50){
				System.out.print("Balancing\n\r");
				cntHall = 2000;
				state = BALANCING_STATE;
			}
			break;
			
		case BALANCING_STATE:
			
			if (cntHall >= 2000) {
				float hs0 = ScaraDriver.getADCHallSens(0);
				System.out.print(hs0);
				System.out.print("\t\t");
				float hs1 = ScaraDriver.getADCHallSens(1);
				System.out.print(hs1);
				System.out.print("\t\t");
				float hs2 = ScaraDriver.getADCHallSens(2);
				System.out.print(hs2);
				System.out.print("\t\t");
				float hs3 = ScaraDriver.getADCHallSens(3);
				System.out.print(hs3);
				System.out.print("\n\r");

				cntHall=0;
			}
			if (ScaraDriver.getADCHallSens(0) >= -50){
				System.out.print("Balancing Programm: Wait for Pendulum\n\r");
				state = WAIT_FOR_PENDULUM_STATE;
			}
			
			else {cntHall++;}
			break;
			
		default:
			state = STOP_STATE;
		}
		
	}		
}
