package ch.ntb.ems.ass.scara.ViewerAndSamples;
import java.io.PrintStream;

import ch.ntb.inf.deep.runtime.mpc5200.Task;
import ch.ntb.inf.deep.runtime.mpc5200.driver.NtbDriverSPIScaraWorkaround;
import ch.ntb.inf.deep.runtime.mpc5200.driver.NtbDriversSPI;
import ch.ntb.inf.deep.runtime.mpc5200.driver.UART3;


public class Playground extends Task {
	public char[] sym = { '/', '-', '\\', '|' };
	public int index = 0;
	
	static{
		UART3.start(9600, UART3.NO_PARITY, (short) 8);
		System.out = new PrintStream(UART3.out);
		System.out.println("SPI Playground #");
		System.out.println("################");
		
		NtbDriverSPIScaraWorkaround.init();
		
		NtbDriverSPIScaraWorkaround.reset();
		
		NtbDriverSPIScaraWorkaround.globalEnable();
		
		NtbDriverSPIScaraWorkaround.turnLedOn(0);
		NtbDriverSPIScaraWorkaround.turnLedOn(1);
		NtbDriverSPIScaraWorkaround.turnLedOn(2);
		NtbDriverSPIScaraWorkaround.turnLedOn(3);
		
		Playground t = new Playground();
		t.period = 500;
		t.time = 100;
		Task.install(t);
	}
	
	
	public void action(){
		System.out.print((char)0x0c);
		System.out.println(sym[index]);
		System.out.println();
		if (++index > 3) index = 0;
		
		
		System.out.println("WHO AM I: ");	
		System.out.printHexln(NtbDriverSPIScaraWorkaround.tranceive(NtbDriversSPI.WHO_AM_I, 0, 0));
		
		System.out.print("POS 0: ");
		System.out.print("\t");	
		System.out.println("POS 1: ");
		System.out.print(NtbDriverSPIScaraWorkaround.getEncoderPosition(0));
		System.out.print("\t");	
		System.out.println(NtbDriverSPIScaraWorkaround.getEncoderPosition(1));
		
		System.out.println("HALL SENSORS: ");	
		System.out.print(NtbDriverSPIScaraWorkaround.getADCValue(0));
		System.out.print("\t");	
		System.out.print(NtbDriverSPIScaraWorkaround.getADCValue(1));
		System.out.println("");	
		System.out.print(NtbDriverSPIScaraWorkaround.getADCValue(2));
		System.out.print("\t");	
		System.out.print(NtbDriverSPIScaraWorkaround.getADCValue(3));
		System.out.println();
		
		System.out.println("ACCELERATION SENSOR: ");	
		System.out.print("x: ");	
		System.out.print(NtbDriverSPIScaraWorkaround.getADCValue(4));
		System.out.print("\t");	
		System.out.print("y: ");	
		System.out.print(NtbDriverSPIScaraWorkaround.getADCValue(5));
		System.out.print("\t");	
		System.out.print("z: ");	
		System.out.print(NtbDriverSPIScaraWorkaround.getADCValue(6));
		System.out.print("\t");	
		System.out.println();
		
		System.out.println("SWITCHES: ");
		
		for(int i = 0; i<4; i++){
			System.out.print(NtbDriverSPIScaraWorkaround.getSwitchState(i));
			System.out.print("\t");
		}
		System.out.println();
	}
}
