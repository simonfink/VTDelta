package ch.ntb.ems.ass.scara.model;

public class Encoder {
	public int ticksPerTurn;
	public int edgeCount;
	public double J;
	
	public Encoder(int ticksPerTurn,int edgeCount, double J){
		this.ticksPerTurn = ticksPerTurn;
		this.edgeCount = edgeCount;
		this.J = J;
	}
	
}
