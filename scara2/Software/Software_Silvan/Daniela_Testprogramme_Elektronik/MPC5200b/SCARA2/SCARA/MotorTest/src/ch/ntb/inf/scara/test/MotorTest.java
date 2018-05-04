package ch.ntb.inf.scara.test;

import java.io.PrintStream;
import java.lang.Math;
//import java.util.Timer;
//import java.util.Date;
//import java.lang.System;

import ch.ntb.inf.deep.runtime.mpc5200.IphyCoreMpc5200tiny;
import ch.ntb.inf.deep.runtime.mpc5200.Task;
import ch.ntb.inf.deep.runtime.mpc5200.driver.UART3;
import ch.ntb.inf.deep.unsafe.US;
import ch.ntb.robotics.omnidir.driver.OmniDirDriver;

public class MotorTest extends Task implements IphyCoreMpc5200tiny {

	private static final float controlClock = 0.001f; // [s] (Regeltakt)
	private float encoderticks = 1024f;	// 65535f;
//	private float sf = 2f;				// Sicherheitsfaktor
	private float sf = 5f;				// Sicherheitsfaktor
	private float pD = 1f;				// Lehr‘sches Dämpfungsmass
	private float pJ = 0.0000139f;		// Rotationsträgheit des Motors [kg*m2]
	private float kM = 0.0603f;			// Motorenkonstante [Nm/A]
	private float pR = 1.13f;			// Ohm’scher Widerstand des Motors [Ohm]
	private float kdk;					// kinematische Differentialverstärkung
	private float omegaCtrl;			// Eigenkreisfrequenz/Eckfrequenz des geschlossenen Regelkreises
	private float kpk;					// kinematische Proportionalverstärkung
	private float kd;					// Differentialverstärkung (Dämpfungskonstante)
	private float kp;					// Proportionalverstärkung (Federkonstante)
	private float dTime;
	private float newtime = 0f;
	private float timeOld = 0f; 
	private float qOld;
	private float qSOld;
	private float qSd;					// zeitliche Ableitung der Sollposition der Roboterkoordinaten
	private float qd;					// zeitliche Ableitung von q (Winkelgeschwindigkeit)
	private float q;					// Position Roboterkoordinaten
	private float torque;				// Drehmoment
	private float U;					// Motorspannung
	private float UMax = 24f;			// Maximale Motorspannung
	private float pwm;
	private float value;
	
	private float qMS = 0f;				// Mittelwert
	private float qAS = 1f;				// Amplitude
	private float fSP = 1f;				// Frequenz
	private float qS;					// Sollpositon der Roboterkoordinaten
	
	private int toggle = 0;
	
	private int ctr = 0;
	
	public MotorTest() {
		// Initialize UART3 (9600 8N1) and use it for stdout and stderr
		UART3.start(9600, UART3.NO_PARITY, (short)8);
		System.out = new PrintStream(UART3.out);
		System.err = System.out;
		System.out.print("Motor Test started...\n\r");
		
		// Initialize SPI
		OmniDirDriver.init();
				
		// Initialize GPIO7 as digital output for inverted led
		US.PUT4(GPWER, US.GET4(GPWER) | 0x80000000); // enable GPIO use
		US.PUT4(GPWDDR, US.GET4(GPWDDR) | 0x80000000); // make output
				
		// Initialize motor
		// All motors active
//		OmniDirDriver.activateServo(0, true);
		OmniDirDriver.activateServo(1, true);	
//		OmniDirDriver.activateServo(2, true);	
//		OmniDirDriver.activateServo(3, true);
//		OmniDirDriver.activateServo(4, true);	
//		OmniDirDriver.activateServo(5, true);
		
		// Parameter (calculated at Initialize):
		kdk = 1/( sf*controlClock);						// kinematische Differentialverstärkung
		omegaCtrl = kdk/(2*pD);							// Eigenkreisfrequenz/Eckfrequenz des geschlossenen Regelkreises
		kpk = omegaCtrl*omegaCtrl;						// kinematische Proportionalverstärkung
		kd = kdk*pJ;									// Differentialverstärkung (Dämpfungskonstante)
		kp = kpk*pJ;									// Proportionalverstärkung (Federkonstante)
		timeOld = time()/1000f;


				
	}
	
	public void action(){
		
		newtime = time()/1000f;							// Time in Second
//		
//		//qS=qMS+qAS*Math.signum(Math.sin(fSP*(2*Math.PI)*newtime));
//		if (Math.sin(fSP*(2*Math.PI)*newtime)> 0)
//			qS=qMS+qAS;
//			//qS=0.5f;
//		else if (Math.sin(fSP*(2*Math.PI)*newtime)< 0)
//			qS=qMS-qAS;
//			//qS=-0.5f;
//		else
//			qS=qMS;
			qS=0;
		
		
		
		// Read the actual value:
		q = (float)(OmniDirDriver.getPosShort(1)/(4*encoderticks)*2*Math.PI);

		// Calculate the velocity by numeric  differentiation:
		dTime= newtime-timeOld;						// Zeit seit letztem Aufruf, in der Regel etwa der Regeltakt
		qd = (q-qOld)/dTime;						// Ist-Geschwindigkeit in rad/s
		qSd = (qS-qSOld)/dTime;						// Soll-Geschwindigkeit in rad/s
		qSOld = qS;									// alte Werte speichern
		qOld = q;									// alte Werte speichern
		timeOld = newtime;							// alte Werte speichern

		// Controller
		torque = (float)(kp*(qS-q) + kd*(qSd-qd));

		// Calculate the output voltage
		U = qSd*kM + torque/kM*pR;

		// Duty Cycle
		pwm = U/UMax;//*100f;

		// value must be between +/-1791
//		if (pwm < -1.2)
//			pwm = -1.2f;
////			value = 0f;
//		else if (pwm > 1.2)
//			value = 0f;
//		else
			value = (float)(pwm * 1492f);
		
			if(value >= 1791){
				value = 1791;
			}
			else if (value < -1791){
				value = -1791;
			}
			else{}
			
			
			if(toggle == 0){
				OmniDirDriver.setCurrentInt(1, (int)(100)); 
				toggle = 1;
			}
			
			else{
				OmniDirDriver.setCurrentInt(1, (int)(-100)); 
				toggle = 0;
			}
//		OmniDirDriver.setCurrentInt(0, (int)(value)); 

//		OmniDirDriver.setCurrentInt(2, (int)(value)); 
//		OmniDirDriver.setCurrentInt(3, (int)(value)); 
//		OmniDirDriver.setCurrentInt(4, (int)(value)); 
//		OmniDirDriver.setCurrentInt(5, (int)(value)); 
		OmniDirDriver.resetWatchdog();
		OmniDirDriver.transceive();
				
		if(ctr > 1000) {
			System.out.print(OmniDirDriver.getPosShort(0)); // read encoder
			System.out.print('\t');
			System.out.print(OmniDirDriver.getPosShort(1)); // read encoder
			System.out.print('\t');
			System.out.print(OmniDirDriver.getPosShort(2)); // read encoder
			System.out.print('\t');
			System.out.print(OmniDirDriver.getPosShort(3)); // read encoder
			System.out.print('\t');
			System.out.print(OmniDirDriver.getPosShort(4)); // read encoder
			System.out.print('\t');
			System.out.print(OmniDirDriver.getPosShort(5)); // read encoder
			System.out.print("\r\n");
			ctr = 0;
		}
		
		ctr++;
	}
	
	static {
		Task t = new MotorTest();
		t.period = (int)(controlClock * 1000);
		Task.install(t);

	}
}
