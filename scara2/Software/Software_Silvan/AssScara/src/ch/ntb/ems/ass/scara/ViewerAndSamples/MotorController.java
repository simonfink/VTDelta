package ch.ntb.ems.ass.scara.ViewerAndSamples;
import java.io.PrintStream;

import ch.ntb.ems.ass.scara.blocks.Ctrl.PIController;
import ch.ntb.ems.ass.scara.blocks.inOutputs.ADC128S102;
import ch.ntb.ems.ass.scara.blocks.inOutputs.FQD;
import ch.ntb.ems.ass.scara.blocks.inOutputs.PWMOut;
import ch.ntb.ems.ass.scara.framework.Output;
import ch.ntb.ems.ass.scara.model.Motor;
import ch.ntb.inf.deep.runtime.mpc5200.Task;
import ch.ntb.inf.deep.runtime.mpc5200.driver.NtbDriverSPIScaraWorkaround;
import ch.ntb.inf.deep.runtime.mpc5200.driver.UART3;


public class MotorController extends Task {
	private final static double cycleTime = 0.1; //sec
	private static FQD enc;
	private static PWMOut pwm;
	private static PIController controller;
	private static ADC128S102 adc;
	private static Motor maxon309702 = new Motor(14,1000,4,2.32,0.0234,0.00108,0.00005,0.00007);
	
	
	
	
	private static Output testOut = new Output();
	private int count = 0;
	
	static{
		UART3.start(9600, UART3.NO_PARITY, (short) 8);
		System.out = new PrintStream(UART3.out);
		System.out.println("Motor controller started #");
		System.out.println("##########################");
		System.out.println();
		
		
		NtbDriverSPIScaraWorkaround.init();
		NtbDriverSPIScaraWorkaround.reset();
		NtbDriverSPIScaraWorkaround.globalEnable();

		
		
		enc = new FQD(1,maxon309702,cycleTime);
		pwm = new PWMOut(1,24.0);
		controller = new PIController(1.0,0.01,cycleTime);
		adc = new ADC128S102(6);
		
		
		enc.reset();
		pwm.enable();
		
		testOut.setValue(0.0);
		
		
		pwm.voltage.connect(controller.controlVariable);
		controller.actualValue.connect(enc.actualPosition);
		controller.desiredValue.connect(testOut);
		
		MotorController t = new MotorController();
		t.period = (int) (cycleTime*1000);
		Task.install(t);

	}


	public void action(){
		enc.run();
		pwm.run();
		controller.run();
		adc.run();
		
		if(count*cycleTime == 1.0){
			System.out.print("Pos: ");
			System.out.println(enc.actualPosition.getValue());
			System.out.print("Velo: ");
			System.out.println(enc.actualVelocity.getValue());
			System.out.print("ADC Voltage: ");
			System.out.println(adc.voltage.getValue());
			count=0;
			testOut.setValue(testOut.getValue() + Math.PI/2);
		}else{
			count++;
		}
		
	}
	
}
