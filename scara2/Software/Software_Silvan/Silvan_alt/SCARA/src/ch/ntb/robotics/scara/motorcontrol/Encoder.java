/*
 * Copyright (c) 2013 NTB Interstate University of Applied Sciences of Technology Buchs.
 * All rights reserved.
 *
 * http://www.ntb.ch/inf
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * 
 * Neither the name of the project's author nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

package ch.ntb.robotics.scara.motorcontrol;

import ch.ntb.robotics.scara.driver.ScaraDriver;
import ch.ntb.robotics.scara.driver.Output;


/**********************************************************
 * File:     Encoder.java                              
 * Created:  2013-01-04 dfrommelt                         
 * Changes:	 none
 * ------------------------------------------------------ 
 * Description:                                           
 * driver for OmniDir, gets the encoder position and velocity                                       
 *                                                       
 *       		   +-----------------------+		  	  
 *       		   |					   |         	  
 *         	       |	              pos ->         	  
 *     		       |                       |         
 *         	       |	             velo ->         	  
 *     		       |                       | 	  
 *          	   +-----------------------+         	  
 *                                                        
 **********************************************************/
public class Encoder {
	public Output pos = new Output();
	public Output velo = new Output(); 
	
	private int motNr;
	private short lastPos=0;
	private int actPos = 0;
	
	public double kPos, kPosInv;
	private double kVelo, i;
	
	/**
	*	object constructor
	*@param motNr
	*			index of motor (0 to 5)
	*@param i
	*			gear ratio of motor to set value (f.e. linear position)
	*@param encTicks
	*			encoder ticks of motorencoder (f.e. 512)
	*@param encEdges
	*			number of calculated edges of encoder
	*@param fCLK
	*			frequency of counter to measure the time for one encodertick
	*			this is only used for velocity
	*/
	public Encoder(int motNr, double i, int encTicks, int encEdges, int fCLK) {
		this.motNr=motNr;
		this.lastPos = ScaraDriver.getPosShort(motNr);
		this.kPos = 2.0*Math.PI/(double)encTicks/(double)encEdges/i;
		this.kVelo = 2.0*Math.PI*fCLK/i/(double)encTicks;
		this.kPosInv = 1.0 / this.kPos;
		this.i = i;
	}
	
	/**
	*	method for setting the position
	*@param val
	*			position to set
	*/
	public void setPos(double val){
		this.lastPos = ScaraDriver.getPosShort(this.motNr);
		this.actPos = (int)(0.5 + val * this.kPosInv);		
			
	}
	
	/**
	*	method for resetting the position
	*/
	public void reset(){
		this.lastPos = ScaraDriver.getPosShort(motNr);
		this.actPos = 0;
	}
	
	/**
	*	method for getting the position
	*@return
	*	position in the desired unit
	*/
	public double getPos(){
		short pos = ScaraDriver.getPosShort(motNr);
		actPos += (short)(pos-lastPos);
		lastPos = pos;
		
		return kPos * (double)actPos;								
	}
	
	/**
	*	method for resetting the velocity
	*@return
	*	velocity in the desired unit
	*/
	public double getVelo(){
		short velo = ScaraDriver.getVeloShort(motNr);
		
		if((velo>-2 && velo<2) || velo>=0x7FFF || velo<=-0x7FFF) return 0.0;
		return this.kVelo/(double)velo;	
	}
	
	public double getI(){
		return i;
	}
	
	/**
	*	running method, saves the position and velocity in the outputs
	*/
	public void run() {
		
		this.pos.setValue(this.getPos());
		
		this.velo.setValue(this.getVelo());
		
	}
}
