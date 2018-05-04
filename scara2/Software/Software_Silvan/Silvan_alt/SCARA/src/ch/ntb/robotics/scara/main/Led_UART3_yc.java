package ch.ntb.robotics.scara.main;

import ch.ntb.inf.deep.runtime.mpc5200.IphyCoreMpc5200tiny;
import ch.ntb.inf.deep.runtime.mpc5200.Task;
import ch.ntb.inf.deep.unsafe.US;

public class Led_UART3_yc extends Task implements IphyCoreMpc5200tiny {

////protected static final float ts = 0.001f;
//	
//	public void action() {
//		OmniDirDriver.transceive();
////			
////		
/////*		if (OmniDirDriver.getDIButton(1)) {
////			OmniDirDriver.setLED(2,1);
////		}
////		else {
////			OmniDirDriver.setLED(2,0);
////		}
////*/	}
////	
////	static {
////	
////		OmniDirDriver.init();
////		
////		OmniDirDriver.setLED(3,1);
////		
////		// initialize task
////		Task t = new Led();
////		t.period = (int) (ts * 1000);
////		Task.install(t);
////	}
//
////public void action(){
////	US.PUT4(GPWOUT, US.GET4(GPWOUT) ^ 0x80000000);
//	
//}
//
//static {
////	US.PUT4(GPWER, US.GET4(GPWER) | 0x80000000);	// enable GPIO use
////	US.PUT4(GPWDDR, US.GET4(GPWDDR) | 0x80000000);	// make output
//	
//	OmniDirDriver.init();
//	
//	OmniDirDriver.setLED(1,1);
//	
//	// Create and install the task
//	Led t = new Led();
//	t.period = 1;
//	Task.install(t);
//}
	
//	public void action() {
//		// Write a single character to the stdout
//		System.out.print("Helloworld\n");
//	}
//
//	static {
//		
//		// Initialize UART (9600 8N1)
//		UART3.start(9600, UART3.NO_PARITY, (short)8);
//		
//		// Use the UART3 for stdout and stderr
//		System.out = new PrintStream(UART3.out);
//		System.err = System.out;
//		
//		// Print a string to the stdout
//		System.out.print("System.out demo (UART3)\n");
//		
//		// Create and install the demo task
//		Task t = new Led();
//		t.period = 500;
//		Task.install(t);
//	}
	
	public void action(){
		US.PUT4(GPWOUT, US.GET4(GPWOUT) ^ 0x80000000);
	}
	
	static {
		US.PUT4(GPWER, US.GET4(GPWER) | 0x80000000);	// enable GPIO use
		US.PUT4(GPWDDR, US.GET4(GPWDDR) | 0x80000000);	// make output
		
		// Create and install the task
		Led_UART3_yc t = new Led_UART3_yc();
		t.period = 300;
		Task.install(t);
	}
	
}
