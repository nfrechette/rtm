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
			explicit constexpr angle_constant(double dbl_) RTM_NO_EXCEPT : flt(float(dbl_)), dbl(dbl_) {}
			constexpr angle_constant(float flt_, double dbl_) RTM_NO_EXCEPT : flt(flt_), dbl(dbl_) {}

			constexpr angle_constant operator-() const RTM_NO_EXCEPT { return angle_constant(-flt, -dbl); }

			constexpr operator float() const RTM_NO_EXCEPT { return flt; }
			constexpr operator double() const RTM_NO_EXCEPT { return dbl; }

			// Angle in radians as a float and double
			float flt;
			double dbl;

			friend anglef;
			friend angled;
		};
	}

	//////////////////////////////////////////////////////////////////////////
	// Various constants in radians
	//////////////////////////////////////////////////////////////////////////

	constexpr rtm_impl::angle_constant k_pi(3.14159265358979323846);					// PI
	constexpr rtm_impl::angle_constant k_pi_2(1.57079632679489661923);					// PI / 2
	constexpr rtm_impl::angle_constant k_pi_4(0.785398163397448309616);					// PI / 4
	constexpr rtm_impl::angle_constant k_pi_180((double)k_pi / 180.0);					// PI / 180
	constexpr rtm_impl::angle_constant k_inv_pi_180(180.0 / (double)k_pi);				// 180 / PI
	constexpr rtm_impl::angle_constant k_2_pi(6.283185307179586476925286766559);		// PI * 2
}

RTM_IMPL_FILE_PRAGMA_POP
