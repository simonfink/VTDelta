package ch.ntb.ems.ass.scara.blocks.Ctrl;

import ch.ntb.ems.ass.scara.framework.Input;
import ch.ntb.ems.ass.scara.framework.Output;

public class Kinematic {

		public Input actualPos0;
		public Input actualPos1;
		
		private double l = 0.25;
		
		public Output xPosition;
		public Output yPosition;
		
		public Kinematic(){
			actualPos0 = new Input();
			actualPos1 = new Input();
			
			xPosition = new Output();
			yPosition = new Output();
			
		}

		
		public void run(){
			double phi1 = actualPos0.getValue();
			double phi2 = actualPos1.getValue();
			
			double posX = l*(Math.cos(phi1) + Math.cos(phi2));
			double posY = l*(Math.sin(phi1) + Math.sin(phi2));
			
			xPosition.setValue(posX);
			yPosition.setValue(posY);
		}
}
