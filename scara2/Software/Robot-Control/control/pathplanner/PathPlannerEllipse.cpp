#include "PathPlannerEllipse.hpp"
#include <eeros/core/System.hpp>
#include <iostream>
#include <stdio.h>

using namespace parallelscara;
using namespace eeros;
using namespace eeros::control;

PathPlannerEllipse::PathPlannerEllipse(double a, double b, double dt, AxisVector centrePos) : a0(a), b0(b), dt(dt), centrePos(centrePos) { }

Output<AxisVector>& PathPlannerEllipse::getPosOut() {
	return posOut;
}
Input<AxisVector>& PathPlannerEllipse::getIn_actPos_xy() {
	return actPos_xy;
}

bool PathPlannerEllipse::posReached() {
	return done;
}

void PathPlannerEllipse::enable() {
	enabled = 1;
	count = 1.0;
	t = pi/2.0;
	first = true;
	done = false;
}
void PathPlannerEllipse::disable() {
	enabled = 0;
}

void PathPlannerEllipse::run() {
	AxisVector x;
	timestamp_t ts = System::getTimeNs();
	
	if(enabled && count <= countLim1+countLim2+1.0){
		if(first){
			x = centrePos;
			t = t + dt; 
			a = a0;
			b = 0.0;
			first = false;
		}
		else{
			x(0) = centrePos(0) + a * cos(t);
			x(1) = centrePos(1) + b * sin(t);
			
			if(fabs(x(0)-posPrev(0))>0.01 || fabs(x(1)-posPrev(1))>0.01 && !first) {
				std::cout << "pos: " << x(0) << "; " << x(1) << "; prev: " << posPrev(0) << "; " << posPrev(1) << "; count: " << count << "; t: " << t << "; a: " << a << "; b: " << b << "; bsint: " << b*sin(t) << "; acost: " << a*cos(t) << std::endl;
				throw Fault("wrong ellipse set point");
			}
	
			t = t + dt;
			
			if(count <= countLim1){
				if(t >= 2.0*pi*count){
					a = a0;
					b = b0/(countLim1 + 1.0 - count);
					count++;
					dt = dt + t_incr; 
				}
			}
			else if(count == countLim1 + 1.0){
				if(t <= 2.0*pi*count-3.0*pi/2.0){
					a = a0;
					b = b0/((countLim1 + 1.0) + 1.0 - count);
					
				}
				else if(t > 2.0*pi*count-3.0*pi/2.0 && t <= 2.0*pi*count + pi/2.0){
					a = a0 - a0 * ((count - 1.0) / (countLim1 + countLim2));
					b = b0;
				}
				else{
					a = a0 - a0 * (count / (countLim1 + countLim2));
					b = b0;
					count++;
					dt = dt - t_incr; 
				}
			}
			else if(count <= countLim1 + countLim2) {
				if(t >= 2.0*pi*count + pi/2.0){
					a = a0 - a0 * (count / (countLim1 + countLim2)); 
					b = b0;
					count++;
					dt = dt - t_incr;
				}
			}
			else if(count <= countLim1 + countLim2 + 1.0) {
				if(t <= 2.0*pi*count){
					a = 0.0; 
					b = b0;
				}
				else{
					a = 0.0; 
					b = 0.0;
					done = true;
					enabled = false; // go out of enabled loop
					count++;         // go out of enabled loop
				}
			}
		}
	}
	else{
		x = actPos_xy.getSignal().getValue();
	}
	
	posPrev = x;
	
	posOut.getSignal().setValue(x);
	posOut.getSignal().setTimestamp(ts);
}