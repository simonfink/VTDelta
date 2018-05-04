package ch.ntb.ems.ass.scara.blocks.inOutputs;

import ch.ntb.ems.ass.scara.framework.Output;
import ch.ntb.inf.deep.runtime.mpc5200.driver.NtbDriverSPIScaraWorkaround;

public class ADC128S102 {
	public Output voltage = new Output();
	private int chanel;
	
	public ADC128S102 (int chanel){
		this.chanel = chanel;
	}
	
	public void run(){
		voltage.setValue(NtbDriverSPIScaraWorkaround.getADCValue(chanel));
	}	
}
