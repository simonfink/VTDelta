/*
 * Copyright (c) 2011 NTB Interstate University of Applied Sciences of Technology Buchs.
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

package ch.ntb.ems.ass.scara.framework;

/*(non-Javadoc)
*	Input:
*		Eingangselement das mit einem Ausgangelement (Output) verbunden werden kann.
*/
public class Input {
	Output connectedOutput;
	private boolean connected=false;

	/*(non-Javadoc)
	*	object constructor
	*/
	public Input(){
		
	}
	
	/*(non-Javadoc)
	*	connect the object to an output
	*@param out
	*            connected output
	*/
	public void connect(Output out) {
		connectedOutput = out;
		connected=true;
	}

	/*(non-Javadoc)
	*	method for getting the saved value in the output
	*@return
	*			saved value in output, if input is not connected return 0.0	
	*/
	public double getValue() {
		if (connected) return connectedOutput.getValue();
		else return 0;
	}
	
	/*(non-Javadoc)
	*	method for checking if the input is connected
	*
	*@return
	*			boolean value, true if input is connected
	*/
	public boolean isConnected() {
		return this.connected;
	}
	
}
