#ifndef CH_NTB_SCARA_TYPES_HPP
#define CH_NTB_SCARA_TYPES_HPP

#include <eeros/math/Matrix.hpp>
#include "constants.hpp"

namespace scara {
	using AxisVector = eeros::math::Matrix<nofAxis, 1>;
	using AxisSquareMatrix = eeros::math::Matrix<nofAxis, nofAxis>;
}

#endif /* CH_NTB_SCARA_TYPES_HPP */