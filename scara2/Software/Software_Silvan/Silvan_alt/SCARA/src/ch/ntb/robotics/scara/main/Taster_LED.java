package ch.ntb.robotics.scara.main;

import java.io.PrintStream;

import ch.ntb.inf.deep.runtime.mpc5200.IphyCoreMpc5200tiny;
import ch.ntb.inf.deep.runtime.mpc5200.Task;
import ch.ntb.inf.deep.runtime.mpc5200.driver.UART3;
import ch.ntb.inf.deep.unsafe.US;
import ch.ntb.robotics.scara.driver.ScaraDriver;

public class Taster_LED extends Task implements IphyCoreMpc5200tiny{

	public Taster_LED(float Ts){
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
		System.out.print("DONE\n\r");
		
		// INIT GPIO7 as digital output for inverted led
		System.out.print("INITIALIZE GPIO LED ...");
		US.PUT4(GPWER, US.GET4(GPWER) | 0x80000000);	// enable GPIO use
		US.PUT4(GPWDDR, US.GET4(GPWDDR) | 0x80000000);	// make output
		System.out.print("DONE\n\r");
		
	}
	
	int cnt=0;
	int cnt2=0;
	boolean f = true;
	
	/**
	 *	test program for microcontroller board
	*/
	
	public void action(){
		ScaraDriver.transceive();
//		ErrorHandler.checkInputs();
//		StateMachine.run();
		
		
		// LEDs
		ScaraDriver.setLED(0,2);
		ScaraDriver.setLED(1,2);
		ScaraDriver.setLED(2,2);
		ScaraDriver.setLED(3,3);
		
		
		// Motor
//		OmniDirDriver.activateServo(1,true);
//		OmniDirDriver.setCurrentInt(1,0);		//82 1/A * 0.25 A =20
		
		
		// Test Hall Sensoren
//		if (cnt2==1000) {
//			float hs0 = OmniDirDriver.getADCHallSens(0);
//			System.out.print(hs0);
//			System.out.print("\t\r");
//			float hs1 = OmniDirDriver.getADCHallSens(1);
//			System.out.print(hs1);
//			System.out.print("\t");
//			float hs2 = OmniDirDriver.getADCHallSens(2);
//			System.out.print(hs2);
//			System.out.print("\t");
//			float hs3 = OmniDirDriver.getADCHallSens(3);
//			System.out.print(hs3);
//			System.out.print("\t");
			
//			float acc0 = OmniDirDriver.getAccValue(0);
//			System.out.print(acc0);
			
//			System.out.print("\n");
				
//			System.out.print(OmniDirDriver.getPosShort(1));
//			System.out.print("\r\n");
//		
//			
//			cnt2=0;
//		}
//		
//		else {
//			cnt2++;
//		}
		

		// Taster
		if(ScaraDriver.getDIButton(0) && f ){
			System.out.print("Taster1\n\r");
			f = false;
		}
		if (f == false){
			cnt++;
		}
		if (cnt == 1000){
			f = true;
			cnt = 0;
		}

		if(ScaraDriver.getDIButton(1) && f ){
			System.out.print("Taster2\n\r");
			f = false;
		}
		if (f == false){
			cnt++;
		}
		if (cnt == 1000){
			f = true;
			cnt = 0;
		}

		if(ScaraDriver.getDIButton(2) && f ){
			System.out.print("Taster3\n\r");
			f = false;
		}
		if (f == false){
			cnt++;
		}
		if (cnt == 1000){
			f = true;
			cnt = 0;
		}
		
	}
	
	
}
	


