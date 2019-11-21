#pragma once

////////////////////////////////////////////////////////////////////////////////
// The MIT License (MIT)
//
// Copyright (c) 2018 Nicholas Frechette & Realtime Math contributors
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
////////////////////////////////////////////////////////////////////////////////

#include "rtm/impl/compiler_utils.h"
#include "rtm/impl/error.h"

RTM_IMPL_FILE_PRAGMA_PUSH

namespace rtm
{
	namespace rtm_impl
	{
		//////////////////////////////////////////////////////////////////////////
		// This is a helper struct to allow a single consistent API between
		// various angle types when the semantics are identical but the return
		// type differs. Implicit coercion is used to return the desired value
		// at the call site.
		//////////////////////////////////////////////////////////////////////////
		struct angle_constant
		{
			constexpr angle_constant operator-() const RTM_NO_EXCEPT { return angle_constant{ -dbl }; }

			// Angle in radians as a double
			double dbl;
		};
	}

	//////////////////////////////////////////////////////////////////////////
	// Add two angle constants.
	//////////////////////////////////////////////////////////////////////////
	constexpr rtm_impl::angle_constant operator+(rtm_impl::angle_constant lhs, rtm_impl::angle_constant rhs) RTM_NO_EXCEPT
	{
		return rtm_impl::angle_constant{ lhs.dbl + rhs.dbl };
	}

	//////////////////////////////////////////////////////////////////////////
	// Various constants
	//////////////////////////////////////////////////////////////////////////

	namespace constants
	{
		//////////////////////////////////////////////////////////////////////////
		// PI
		//////////////////////////////////////////////////////////////////////////
		constexpr rtm_impl::angle_constant pi() RTM_NO_EXCEPT
		{
			return rtm_impl::angle_constant{ 3.141592653589793238462643383279502884 };
		}

		//////////////////////////////////////////////////////////////////////////
		// PI / 2
		//////////////////////////////////////////////////////////////////////////
		constexpr rtm_impl::angle_constant half_pi() RTM_NO_EXCEPT
		{
			return rtm_impl::angle_constant{ 1.570796326794896619231321691639751442 };
		}

		//////////////////////////////////////////////////////////////////////////
		// PI * 2
		//////////////////////////////////////////////////////////////////////////
		constexpr rtm_impl::angle_constant two_pi() RTM_NO_EXCEPT
		{
			return rtm_impl::angle_constant{ 6.283185307179586476925286766559005768 };
		}

		//////////////////////////////////////////////////////////////////////////
		// PI / 180
		//////////////////////////////////////////////////////////////////////////
		constexpr rtm_impl::angle_constant pi_div_one_eighty() RTM_NO_EXCEPT
		{
			return rtm_impl::angle_constant{ pi().dbl / 180.0 };
		}

		//////////////////////////////////////////////////////////////////////////
		// 180 / PI
		//////////////////////////////////////////////////////////////////////////
		constexpr rtm_impl::angle_constant one_eighty_div_pi() RTM_NO_EXCEPT
		{
			return rtm_impl::angle_constant{ 180.0 / pi().dbl };
		}
	}
}

RTM_IMPL_FILE_PRAGMA_POP
