package ch.ntb.ems.ass.scara;
import java.io.PrintStream;


import ch.ntb.inf.deep.runtime.mpc5200.Task;
import ch.ntb.inf.deep.runtime.mpc5200.driver.NtbDriverSPIScaraWorkaround;
import ch.ntb.inf.deep.runtime.mpc5200.driver.UART3;

/**
 * Task generating			
 */

public class AssScara extends Task {
	private final static double CYCLE_TIME = 0.001; //sec
	private static Robot robot;
	
	static{
		UART3.start(9600, UART3.NO_PARITY, (short) 8);
		
		System.out = new PrintStream(UART3.out);
		System.out.println("Ass Scara #");
		System.out.println("###########");
		
		NtbDriverSPIScaraWorkaround.init();
		NtbDriverSPIScaraWorkaround.reset();
		NtbDriverSPIScaraWorkaround.globalEnable();
		System.out.println("Starting Task");
		robot = new Robot(CYCLE_TIME);
		AssScara t = new AssScara();
		t.period = (int) (CYCLE_TIME*1000);
		t.time = 100; 	// delay
		Task.install(t);
		System.out.println("###########");
	}
	
	
	public void action(){
		robot.run();
	}
}
