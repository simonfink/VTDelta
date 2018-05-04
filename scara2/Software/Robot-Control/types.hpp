#ifndef CH_NTB_PARALLELSCARA_TYPES_HPP
#define CH_NTB_PARALLELSCARA_TYPES_HPP

#include <eeros/math/Matrix.hpp>
#include "constants.hpp"

namespace parallelscara {
	using AxisVector = eeros::math::Matrix<nofAxis, 1>;
	using AxisSquareMatrix = eeros::math::Matrix<nofAxis, nofAxis>;
	
	using VectorSize = eeros::math::Matrix<1,7,double>;
}

#endif /* CH_NTB_PARALLELSCARA_TYPES_HPP */