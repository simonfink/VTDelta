package ch.ntb.inf.deep.runtime.mpc5200.driver;

import ch.ntb.inf.deep.runtime.mpc5200.IphyCoreMpc5200tiny;
import ch.ntb.inf.deep.unsafe.US;

public class NtbDriverSPIScaraWorkaround implements IphyCoreMpc5200tiny{
	private static int ledState;
	
	
	public static void init() {
		NtbDriversSPI.init();
		ledState = 0;
		US.PUT4(GPWER, US.GET4(GPWER) | 0x80000000);	// enable GPIO use
		US.PUT4(GPWDDR, US.GET4(GPWDDR) | 0x80000000);	// make output
		
	}
	
	public static short tranceive(int readAddress, int writeAddress, int data) {
		US.PUT4(GPWOUT, US.GET4(GPWOUT) & 0x7FFFFFFF); //set GPIO 7 low
		short readData = NtbDriversSPI.tranceive(readAddress, writeAddress, data);
		return readData;
	}
	
	public static void globalEnable(){
		tranceive(0, NtbDriversSPI.CONF_GLOB, 1);
	}
	
	public static void enableAxis(int axis){
		if(axis<NtbDriversSPI.MAX_NOF_AXIS){
			short reg = getAxisEnableRegister();
			tranceive(0, NtbDriversSPI.ENABLE,reg | (1<<axis));
		}
	}
	
	public static short getAxisEnableRegister(){
		return tranceive(NtbDriversSPI.ENABLE, 0,0);
	}

	public static void reset() {
		tranceive(0,NtbDriversSPI.CONF_GLOB,0x2);
		for(int i = 0; i< 10000; i++){
			
		}
		
	}

	public static void setSetValueAxis(int axis, int value) {
		tranceive(0, NtbDriversSPI.SET_VALUE_0 +axis, value);	
	}
	
	public static short getEncoderPosition(int axis) {
		if(axis >= 0 && axis < NtbDriversSPI.MAX_NOF_AXIS) {
			return tranceive(NtbDriversSPI.POSITION_0 + axis, 0, 0);
		}
		return 0;
	}
	
	
	public static int getADCValue(int channel){
		int transmitData = 0;
		int receiveData = 0;
		
	
		
		transmitData = ((channel & 0x7) << 27);

		
		US.PUT4(GPWOUT, US.GET4(GPWOUT) | 0x80000000); //set GPIO 7 high
		
		US.PUT4(PSC1Base+ PSCTxBuf, transmitData);

		while (US.GET2(PSC1Base + PSCRFNUM) < 4);
		receiveData = US.GET4(PSC1Base + PSCRxBuf);
		
		return (receiveData & 0xFFF) ;
		
	}
	
	public static boolean getSwitchState(int nr){
		if(nr < 4){
			int transmitData = 0;
			int receiveData = 0;
			
			transmitData = (0x3<<30);
	
			
			US.PUT4(GPWOUT, US.GET4(GPWOUT) | 0x80000000); //set GPIO 7 high
			
			US.PUT4(PSC1Base+ PSCTxBuf, transmitData);
	
			while (US.GET2(PSC1Base + PSCRFNUM) < 4);
			receiveData = US.GET4(PSC1Base + PSCRxBuf);
		
			receiveData = receiveData >> 28;
			receiveData = receiveData & 0xF;
			if((receiveData & (0x1<<nr)) > 0){
				return true;
			}else{
				return false;
			}
		}else{
			return false;
		}
	}
	
	public static void turnLedOn(int nr){
		if(nr <4){
			int transmitData = 0;
			ledState = ledState | (0x1<<nr);
			transmitData = (0x1<<30) | (ledState<<16);
			
			US.PUT4(GPWOUT, US.GET4(GPWOUT) | 0x80000000); //set GPIO 7 high
			
			US.PUT4(PSC1Base+ PSCTxBuf, transmitData);
	
			while (US.GET2(PSC1Base + PSCRFNUM) < 4);
			US.GET4(PSC1Base + PSCRxBuf);
		}
	}
	
	public static void turnLedOff(int nr){
		if(nr <4){
			int transmitData = 0;
			ledState = ledState & ~(0x1<<nr);
			ledState = ledState & 0xF;
			transmitData = (0x1<<30) | (ledState<<16);
			
			US.PUT4(GPWOUT, US.GET4(GPWOUT) | 0x80000000); //set GPIO 7 high
			
			US.PUT4(PSC1Base+ PSCTxBuf, transmitData);

			while (US.GET2(PSC1Base + PSCRFNUM) < 4);
			US.GET4(PSC1Base + PSCRxBuf);
		}
	}
	
	
	

	public static void disableAxis(int axis) {
		if(axis<NtbDriversSPI.MAX_NOF_AXIS){
			short reg = getAxisEnableRegister();
			tranceive(0, NtbDriversSPI.ENABLE,reg & ~(1<<axis));
		}
	}
	
}
