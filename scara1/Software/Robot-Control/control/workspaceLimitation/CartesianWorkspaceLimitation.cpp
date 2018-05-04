#include "CartesianWorkspaceLimitation.hpp"
#include <eeros/core/System.hpp>
#include <iostream>
#include <fstream>

using namespace scara;
using namespace eeros;
using namespace eeros::control;
using namespace eeros::math;

CartesianWorkspaceLimitation::CartesianWorkspaceLimitation() {}


void CartesianWorkspaceLimitation::enable(){
	enabled = 1;
};

void CartesianWorkspaceLimitation::disable(){
	enabled = 0;
};

void CartesianWorkspaceLimitation::setParameters (Matrix<13,1,double> limit_par, AxisVector toolOffset) {
	a_ab = limit_par(0);
	b_ab = limit_par(1);
	c_ab = limit_par(2);
	a_bc = limit_par(3);
	b_bc = limit_par(4);
	c_bc = limit_par(5);
	a_cd = limit_par(6);
	b_cd = limit_par(7);
	c_cd = limit_par(8);
	a_da = limit_par(9);
	b_da = limit_par(10); 
	c_da = limit_par(11);
	z = limit_par(12);
	
	this->toolOffset = toolOffset;
}

void CartesianWorkspaceLimitation::run() {
	
	input_robot = in.getSignal().getValue();
	input = input_robot - toolOffset;
	
	if(enabled){
		// Check if inside workspace - xy
		if ((c_ab > c_cd) && (c_da > c_bc)) {
			if ((a_ab*input(0)+b_ab*input(1)+c_ab > 0) && (a_cd*input(0)+b_cd*input(1)+c_cd < 0) && (a_bc*input(0)+b_bc*input(1)+c_bc < 0) && (a_da*input(0)+b_da*input(1)+c_da > 0)) {   
				in_workspace = 1;
			}
			else {
				in_workspace = 0;
			}
		}		
		else if ((c_ab > c_cd) && (c_da < c_bc)) {
			if ((a_ab*input(0)+b_ab*input(1)+c_ab > 0) && (a_cd*input(0)+b_cd*input(1)+c_cd < 0) && (a_bc*input(0)+b_bc*input(1)+c_bc > 0) && (a_da*input(0)+b_da*input(1)+c_da < 0)) {   
				in_workspace = 1;
			}
			else {
				in_workspace = 0;
			}
		}
		else if ((c_ab < c_cd) && (c_da > c_bc)) {
			if ((a_ab*input(0)+b_ab*input(1)+c_ab < 0) && (a_cd*input(0)+b_cd*input(1)+c_cd > 0) && (a_bc*input(0)+b_bc*input(1)+c_bc < 0) && (a_da*input(0)+b_da*input(1)+c_da > 0)) {   
				in_workspace = 1;
			}
			else {
				in_workspace = 0;
			}
		}
		else if ((c_ab < c_cd) && (c_da < c_bc)) {
			if ((a_ab*input(0)+b_ab*input(1)+c_ab < 0) && (a_cd*input(0)+b_cd*input(1)+c_cd > 0) && (a_bc*input(0)+b_bc*input(1)+c_bc > 0) && (a_da*input(0)+b_da*input(1)+c_da < 0)) {   
				in_workspace = 1;
			}
			else {
				in_workspace = 0;
			}
		}
		else {
			in_workspace = 0;
		}
		
		// Check if inside workspace - z
		if(input(2) < z)
			in_workspace = 0;
		
		// Define output
		if (in_workspace == true) {
			out.getSignal().setValue(in.getSignal().getValue());
		}
		else{
			out.getSignal().setValue(prev_out); 
			throw Fault("EMERGENCY! Trajectory out of workspace");
		}
	}
	else
		out.getSignal().setValue(in.getSignal().getValue());
	
	// Set output timestamp
	out.getSignal().setTimestamp(in.getSignal().getTimestamp());;
	
	prev_out = out.getSignal().getValue();
}
