package ch.ntb.inf.deep.runtime.mpc5200.driver;

import ch.ntb.inf.deep.runtime.mpc5200.IphyCoreMpc5200tiny;
import ch.ntb.inf.deep.unsafe.US;

public class NtbDriversSPI implements IphyCoreMpc5200tiny {
public static final int MAX_NOF_AXIS = 16;
	
	// Registers:
	public static final int WHO_AM_I = 0x1;
	public static final int STATUS_GLOB = 0x2;
	public static final int CONF_GLOB = 0x3;
	public static final int CONF_MODE_A = 0x4;
	public static final int CONF_MODE_B = 0x5;
	public static final int WD_COUNTER = 0x6;
	public static final int ENABLE = 0x7;
	public static final int GPIO_DIR = 0x8;
	public static final int GPIO_VAL = 0x9;
	public static final int SET_VALUE_0 = 0x10;
	public static final int SET_VALUE_1 = 0x11;
	public static final int SET_VALUE_2 = 0x12;
	public static final int SET_VALUE_3 = 0x13;
	public static final int SET_VALUE_4 = 0x14;
	public static final int SET_VALUE_5 = 0x15;
	public static final int SET_VALUE_6 = 0x16;
	public static final int SET_VALUE_7 = 0x17;
	public static final int SET_VALUE_8 = 0x18;
	public static final int SET_VALUE_9 = 0x19;
	public static final int SET_VALUE_10 = 0x1A;
	public static final int SET_VALUE_11 = 0x1B;
	public static final int SET_VALUE_12 = 0x1C;
	public static final int SET_VALUE_13 = 0x1D;
	public static final int SET_VALUE_14 = 0x1E;
	public static final int SET_VALUE_15 = 0x1F;
	public static final int LIMIT_0 = 0x20;
	public static final int LIMIT_1 = 0x21;
	public static final int LIMIT_2 = 0x22;
	public static final int LIMIT_3 = 0x23;
	public static final int LIMIT_4 = 0x24;
	public static final int LIMIT_5 = 0x25;
	public static final int LIMIT_6 = 0x26;
	public static final int LIMIT_7 = 0x27;
	public static final int LIMIT_8 = 0x28;
	public static final int LIMIT_9 = 0x29;
	public static final int LIMIT_10 = 0x2A;
	public static final int LIMIT_11 = 0x2B;
	public static final int LIMIT_12 = 0x2C;
	public static final int LIMIT_13 = 0x2D;
	public static final int LIMIT_14 = 0x2E;
	public static final int LIMIT_15 = 0x2F;
	public static final int POSITION_0 = 0x30;
	public static final int POSITION_1 = 0x31;
	public static final int POSITION_2 = 0x32;
	public static final int POSITION_3 = 0x33;
	public static final int POSITION_4 = 0x34;
	public static final int POSITION_5 = 0x35;
	public static final int POSITION_6 = 0x36;
	public static final int POSITION_7 = 0x37;
	public static final int POSITION_8 = 0x38;
	public static final int POSITION_9 = 0x39;
	public static final int POSITION_10 = 0x3A;
	public static final int POSITION_11 = 0x3B;
	public static final int POSITION_12 = 0x3C;
	public static final int POSITION_13 = 0x3D;
	public static final int POSITION_14 = 0x3E;
	public static final int POSITION_15 = 0x3F;
	public static final int SPEED_0 = 0x40;
	public static final int SPEED_1 = 0x41;
	public static final int SPEED_2 = 0x42;
	public static final int SPEED_3 = 0x43;
	public static final int SPEED_4 = 0x44;
	public static final int SPEED_5 = 0x45;
	public static final int SPEED_6 = 0x46;
	public static final int SPEED_7 = 0x47;
	public static final int SPEED_8 = 0x48;
	public static final int SPEED_9 = 0x49;
	public static final int SPEED_10 = 0x4A;
	public static final int SPEED_11 = 0x4B;
	public static final int SPEED_12 = 0x4C;
	public static final int SPEED_13 = 0x4D;
	public static final int SPEED_14 = 0x4E;
	public static final int SPEED_15 = 0x4F;
	public static final int CURRENT_0 = 0x50;
	public static final int CURRENT_1 = 0x51;
	public static final int CURRENT_2 = 0x52;
	public static final int CURRENT_3 = 0x53;
	public static final int CURRENT_4 = 0x54;
	public static final int CURRENT_5 = 0x55;
	public static final int CURRENT_6 = 0x56;
	public static final int CURRENT_7 = 0x57;
	public static final int CURRENT_8 = 0x58;
	public static final int CURRENT_9 = 0x59;
	public static final int CURRENT_10 = 0x5A;
	public static final int CURRENT_11 = 0x5B;
	public static final int CURRENT_12 = 0x5C;
	public static final int CURRENT_13 = 0x5D;
	public static final int CURRENT_14 = 0x5E;
	public static final int CURRENT_15 = 0x5F;
	public static final int STATUS_0 = 0x60;
	public static final int STATUS_1 = 0x61;
	public static final int STATUS_2 = 0x62;
	public static final int STATUS_3 = 0x63;
	public static final int STATUS_4 = 0x64;
	public static final int STATUS_5 = 0x65;
	public static final int STATUS_6 = 0x66;
	public static final int STATUS_7 = 0x67;
	public static final int STATUS_8 = 0x68;
	public static final int STATUS_9 = 0x69;
	public static final int STATUS_10 = 0x6A;
	public static final int STATUS_11 = 0x6B;
	public static final int STATUS_12 = 0x6C;
	public static final int STATUS_13 = 0x6D;
	public static final int STATUS_14 = 0x6E;
	public static final int STATUS_15 = 0x6F;
	
	// Drive modes:
	public static final int OFF = 0;
	public static final int VOLTAGE_CONTROL = 1;
	public static final int CURRENT_CONTROL = 2;
	public static final int SPEED_CONTROL = 3;
	
	
	public static void init() {
		US.PUT1(PSC1Base + PSCCR, 0xa); // disable Tx, Rx
		US.PUT1(PSC1Base + PSCCR, 0x20); // reset receiver, clears fifo
		US.PUT1(PSC1Base + PSCCR, 0x30); // reset transmitter, clears fifo
		//US.PUT4(PSC1Base + PSCSICR, 0x0280D000); // select SPI mode, master, 16 bit, msb first,
		US.PUT4(PSC1Base + PSCSICR, 0x0F80D000);
		US.PUT4(CDMPSC1MCLKCR, 0x800f);	// Mclk = 33MHz
		US.PUT4(CDMCER, US.GET4(CDMCER) | 0x20);	// enable Mclk for PSC1
		//US.PUT4(PSC1Base + PSCCCR, 0x00080000);
		
		US.PUT4(PSC1Base + PSCCCR, 0x00030000); // DSCKL = 60ns, SCK = 8MHz
		//US.PUT4(PSC1Base + PSCCCR, 0x001F0000); // DSCKL = 60ns, SCK = 2MHz
		
		
		US.PUT1(PSC1Base + PSCCTUR, 1); // set DTL to 150ns
		//US.PUT1(PSC1Base + PSCCTLR, 2); 
		US.PUT1(PSC1Base + PSCTFCNTL, 0x1); // no frames
		US.PUT1(PSC1Base + PSCRFCNTL, 0x1); // no frames
		US.PUT4(GPSPCR, US.GET4(GPSPCR) | 0x7);	// use pins on PCS1 for SPI
		US.PUT1(PSC1Base + PSCCR, 0x5); // enable Tx, Rx	
	}
	
	public static short tranceive(int readAddress, int writeAddress, int data) {
		int transmitData = 0;
		int receiveData = 0;
		
		transmitData = (0 << 31) |
			((readAddress & 0x7F) << 24 ) |
			((writeAddress & 0x7F) << 17 ) |
			((data & 0x8000) << 1) |
			(1 << 15) |
			(data & 0x7FFF);

		US.PUT4(PSC1Base+ PSCTxBuf, transmitData);
		
		while (US.GET2(PSC1Base + PSCRFNUM) < 4);
		receiveData = US.GET4(PSC1Base + PSCRxBuf);
		
		return (short)(((receiveData >> 1) & 0x8000) | (receiveData & 0x7FFF));
	}
	
	public static void transmit(int writeAddress, int data) {
		tranceive(0, writeAddress, data);
	}
	
	public static int receive(int readAddress) {
		return tranceive(readAddress, 0, 0);
	}
	
	
}
