/*
 * Copyright (c) 2012 NTB Interstate University of Applied Sciences of Technology Buchs.
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
*	Output:
*		Ausgangselement auf das Eingänge (Input) verbunden werden können.
*/

public class Output {
	
	// saved value
	private double value;

	/*(non-Javadoc)
	*	object constructor
	*/
	public Output() {
  	}
  	

  	/*(non-Javadoc)
  	 * Function for setting the value in the output
  	 *@param value
	 *            value to be stored in the output
  	 */
	public void setValue(double value) {
		this.value=value;
	}
	
  	/*(non-Javadoc)
  	 * Function for adding a value to the value stored in the output
  	 *@param value
	 *            value to be added
  	 */
	public void addValue(double value) {
		this.value+=value;
	}
	
  	/*(non-Javadoc)
  	 * Function for getting the value stored in the output
  	 *@return
  	 *			value stored in the output
  	 */
	public double getValue() {
		return this.value;
	}

}
