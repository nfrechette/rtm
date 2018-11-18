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

#include "rtm/error.h"

namespace rtm
{
	namespace rtm_impl
	{
		// A helper struct to wrap scalar constants and ensure type safe coercion.
		struct scalar_constant
		{
			explicit constexpr scalar_constant(double dbl_) RTM_NO_EXCEPT : flt(float(dbl_)), dbl(dbl_) {}
			constexpr scalar_constant(float flt_, double dbl_) RTM_NO_EXCEPT : flt(flt_), dbl(dbl_) {}

			constexpr operator float() const RTM_NO_EXCEPT { return flt; }
			constexpr operator double() const RTM_NO_EXCEPT { return dbl; }

			float flt;
			double dbl;
		};
	}

	constexpr rtm_impl::scalar_constant k_pi(3.14159265358979323846);
	constexpr rtm_impl::scalar_constant k_pi_2(1.57079632679489661923);
	constexpr rtm_impl::scalar_constant k_pi_4(0.785398163397448309616);
}
