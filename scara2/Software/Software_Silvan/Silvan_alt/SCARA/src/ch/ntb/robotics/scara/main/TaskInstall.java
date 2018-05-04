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



package ch.ntb.robotics.scara.main;

import ch.ntb.inf.deep.runtime.mpc5200.Task;

/**********************************************************
 * File:     TaskInstall.java                              
 * Created:  2011-06-07 mzueger                         
 * Changes:	 2012-11-04 dfrommelt, 2013-08-08 shuber
 * 
 * ------------------------------------------------------ 
 * Description:                                           
 * installs a task                                  
 * changed to set the sampling time in the controller task
 * **********************************************************/
public class TaskInstall {
	
	static {
		// Create and install the task
		int Ts_ms = 1;
		TestMotBoard t = new TestMotBoard((float)Ts_ms*0.001f);
		t.period = Ts_ms;
		Task.install(t);
		
//		UART3.start(9600, UART3.NO_PARITY, (short)8);
//		System.out = new PrintStream(UART3.out);
//		System.err = System.out;
//		System.out.print("Motor Test started...\n\r");

	}

}