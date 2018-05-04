package ch.ntb.ems.ass.scara.model;

public class Motor {
		public Gear gear;
		public Encoder encoder;

		public double R; 
		public double km;
		public double J; 
		
		
		public Motor(double i, int encTicksPerTurn, int nrOfEdges, double R, double km, double Jmot, double Jgear,double Jenc){
			this.gear = new Gear(i,Jgear);
			this.encoder = new Encoder(encTicksPerTurn, nrOfEdges,Jenc );
			this.R = R;
			this.km = km;
			this.J = Jmot;
		}

		public Motor(Gear g, Encoder e, double R, double km, double Jmot){
			this.gear = g;
			this.encoder = e;
			this.R = R;
			this.km = km;
			this.J = Jmot;
		}
		
		public double getTotalTorque(){
			return this.J + this.gear.J + this.encoder.J;
		}
}
