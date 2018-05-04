#ifndef CH_NTB_SCARA_SETPLASMAPEN_HPP_
#define CH_NTB_SCARA_SETPLASMAPEN_HPP_

#include <eeros/control/Block1i.hpp>
#include <mutex>

namespace scara {
		class SetPlasmaPen: public eeros::control::Block1i<eeros::math::Vector4> {
			
		public:
			SetPlasmaPen();
			virtual ~SetPlasmaPen();
			
			virtual void run();
			virtual void enable();
			virtual void disable();
			
		protected:
			bool enabled = false;
		};
	}
#endif /* CH_NTB_SCARA_SETPLASMAPEN_HPP_ */



		