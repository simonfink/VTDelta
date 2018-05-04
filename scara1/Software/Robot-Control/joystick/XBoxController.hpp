#ifndef REMOTE_XBOXCONTROLLER_HPP
#define REMOTE_XBOXCONTROLLER_HPP
#include "Joystick.hpp"

namespace remote
{
	struct XBoxController
	{
		struct Axis
		{
			static constexpr int LX = 0;
			static constexpr int LY = 1;
			static constexpr int RX = 2;
			static constexpr int RY = 3;
			static constexpr int RT = 4;
			static constexpr int LT = 5;
			static constexpr int CX = 6;
			static constexpr int CY = 7;
		};
		struct Button
		{
			static constexpr int A = 0;
			static constexpr int B = 1;
			static constexpr int X = 2;
			static constexpr int Y = 3;
			static constexpr int LB = 4;
			static constexpr int RB = 5;
			static constexpr int back = 6;
			static constexpr int start = 7;
			static constexpr int guide = 8;
			static constexpr int L = 9;
			static constexpr int R = 10;
		};
	};
}

#endif // REMOTE_XBOXCONTROLLER_HPP
