#pragma once

////////////////////////////////////////////////////////////////////////////////
// The MIT License (MIT)
//
// Copyright (c) 2017 Nicholas Frechette & Animation Compression Library contributors
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

#include "rtm/math.h"
#include "rtm/impl/scalar_common.h"

#include <algorithm>
#include <cmath>

namespace rtm
{
	inline double scalar_floor(double input) RTM_NO_EXCEPT
	{
		return std::floor(input);
	}

	inline double scalar_ceil(double input) RTM_NO_EXCEPT
	{
		return std::ceil(input);
	}

	inline double scalar_clamp(double input, double min, double max) RTM_NO_EXCEPT
	{
		return std::min(std::max(input, min), max);
	}

	inline double scalar_abs(double input) RTM_NO_EXCEPT
	{
		return std::fabs(input);
	}

	inline double scalar_sqrt(double input) RTM_NO_EXCEPT
	{
		return std::sqrt(input);
	}

	inline double scalar_sqrt_reciprocal(double input) RTM_NO_EXCEPT
	{
		// TODO: Use recip instruction
		return 1.0 / scalar_sqrt(input);
	}

	inline double scalar_reciprocal(double input) RTM_NO_EXCEPT
	{
		return 1.0 / input;
	}

	inline double scalar_sin(double angle) RTM_NO_EXCEPT
	{
		return std::sin(angle);
	}

	inline double scalar_cos(double angle) RTM_NO_EXCEPT
	{
		return std::cos(angle);
	}

	inline void scalar_sincos(double angle, double& out_sin, double& out_cos) RTM_NO_EXCEPT
	{
		out_sin = scalar_sin(angle);
		out_cos = scalar_cos(angle);
	}

	inline double scalar_acos(double value) RTM_NO_EXCEPT
	{
		return std::acos(value);
	}

	inline double scalar_atan2(double left, double right) RTM_NO_EXCEPT
	{
		return std::atan2(left, right);
	}

	inline double scalar_min(double left, double right) RTM_NO_EXCEPT
	{
		return std::min(left, right);
	}

	inline double scalar_max(double left, double right) RTM_NO_EXCEPT
	{
		return std::max(left, right);
	}

	constexpr double scalar_deg_to_rad(double deg) RTM_NO_EXCEPT
	{
		return deg * ((double)k_pi / 180.0);
	}

	inline bool scalar_near_equal(double lhs, double rhs, double threshold) RTM_NO_EXCEPT
	{
		return scalar_abs(lhs - rhs) < threshold;
	}

	inline bool scalar_is_finite(double input) RTM_NO_EXCEPT
	{
		return std::isfinite(input);
	}

	inline double scalar_symmetric_round(double input) RTM_NO_EXCEPT
	{
		return input >= 0.0 ? scalar_floor(input + 0.5) : scalar_ceil(input - 0.5);
	}

	inline double scalar_fraction(double value) RTM_NO_EXCEPT
	{
		return value - scalar_floor(value);
	}

	template<typename SrcIntegralType>
	inline double scalar_safe_to_double(SrcIntegralType input) RTM_NO_EXCEPT
	{
		double input_f = double(input);
		RTM_ASSERT(SrcIntegralType(input_f) == input, "Conversion to double would result in truncation");
		return input_f;
	}
}
