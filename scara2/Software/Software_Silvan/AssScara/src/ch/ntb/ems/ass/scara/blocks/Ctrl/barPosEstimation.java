package ch.ntb.ems.ass.scara.blocks.Ctrl;

import ch.ntb.ems.ass.scara.framework.Input;
import ch.ntb.ems.ass.scara.framework.Output;

public class barPosEstimation {
	public static final int SENSOR_0_OFFSET = 2024; // 2018;
	public static final int SENSOR_1_OFFSET = 2024; // 2020;
	public static final int SENSOR_2_OFFSET = 2024; // 2008;
	public static final int SENSOR_3_OFFSET = 2024; // 2027;
	public Input hallSignal0;
	public Input hallSignal1;
	public Input hallSignal2;
	public Input hallSignal3;
	
	public Output phi_s_x;
	public Output phi_s_y;
	
	double magicFactor = 0;
	double phiSx, phiSy;
	
	public barPosEstimation(double factor){
		this.magicFactor = factor;
		hallSignal0 = new Input();
		hallSignal1 = new Input();
		hallSignal2 = new Input();
		hallSignal3 = new Input();
		phi_s_x = new Output();
		phi_s_y = new Output();
	}
	
	
	public void run(){
		
		int val0OffCorr = (int) (hallSignal0.getValue() - SENSOR_0_OFFSET);
		int val1OffCorr = (int) (hallSignal1.getValue() - SENSOR_1_OFFSET);
		int val2OffCorr = (int) (hallSignal2.getValue() - SENSOR_2_OFFSET);
		int val3OffCorr = (int) (hallSignal3.getValue() - SENSOR_3_OFFSET);
		
		if(barTwoIsOn()){
			val0OffCorr = -val0OffCorr;
			val1OffCorr = -val1OffCorr;
			val2OffCorr = -val2OffCorr;
			val3OffCorr = -val3OffCorr;
		}
		
		phiSx = magicFactor*(val0OffCorr-val2OffCorr);
		phiSy = magicFactor*(val3OffCorr-val1OffCorr);

		phi_s_x.setValue(phiSx);
		phi_s_y.setValue(phiSy);
	}
	
	
	public boolean barOneIsOn(){
		if(hallSignal0.getValue() - SENSOR_0_OFFSET > 150 || hallSignal1.getValue() - SENSOR_1_OFFSET > 150 || hallSignal2.getValue() - SENSOR_2_OFFSET > 150 || hallSignal3.getValue() - SENSOR_3_OFFSET > 150){
			return true;
		}else{
			return false;
		}
	}
	
	public boolean barTwoIsOn(){
		if(hallSignal0.getValue() - SENSOR_0_OFFSET < -150 || hallSignal1.getValue() - SENSOR_1_OFFSET < -150 || hallSignal2.getValue() - SENSOR_2_OFFSET < -150 || hallSignal3.getValue() - SENSOR_3_OFFSET < -150){
			return true;
		}else{
			return false;
		}
	}
}
