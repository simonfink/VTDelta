#ifndef CH_NTB_SCARA_I_HPP_
#define CH_NTB_SCARA_I_HPP_

#include <eeros/control/Block1i1o.hpp>
#include <eeros/core/System.hpp>
#include <eeros/safety/SafetySystem.hpp>

#include <iostream>
#include <stdlib.h>

namespace scara {

		template < typename T = double >
		class I: public eeros::control::Block1i1o<T> {
			
		public:
			I() : first(true) { }
			
			virtual void run() {
				if(first) {  // first run, no previous value available -> set output to zero
					prev = this->in.getSignal();
					this->out.getSignal().clear();
					this->out.getSignal().setTimestamp(this->in.getSignal().getTimestamp());
					first = false;
				}
				else {
					double tin = this->in.getSignal().getTimestamp() / 1000000000.0;
					double tprev = this->prev.getTimestamp() / 1000000000.0;
					double dt = (tin - tprev);
					T valin = this->in.getSignal().getValue();
					T valprev = this->prev.getValue();
					
					if(enabled){
						// Calculate output
						output = this->out.getSignal().getValue() + valin * dt;
						
						// Check saturation
						for(uint8_t i = 0; i < 4; i++){
							if(output[i] >= outMax[i])
								output[i] = outMax[i];
							else if (output[i] <= outMin[i])
								output[i] = outMin[i];
						}	
						this->out.getSignal().setValue(output);
						this->out.getSignal().setTimestamp(this->in.getSignal().getTimestamp());
	// 					this->out.getSignal().setTimestamp((this->in.getSignal().getTimestamp() + this->prev.getTimestamp()) / 2);
						
						prev = this->in.getSignal();
					}
					else{
						output = this->out.getSignal().getValue();
					}
				}
			}
			
			virtual void enable() {
				this->enabled = true;
			}
			virtual void disable() {
				this->enabled = false;
			}
			virtual void setInitCondition(T pos) {
				this -> prev = pos; 
			}
			
			T outMax = 0.01;  // TODO shift in protected
			T outMin = -0.01; // TODO shift in protected
			Signal<T> prev;
			
		protected:
			bool first;
			bool enabled = false;
			T output; 
// 			T outMax = 1.0;
// 			T outMin = 1.0;
		};
	};
#endif /* CH_NTB_SCARA_I_HPP_ */
