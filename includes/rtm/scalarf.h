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
	inline float scalar_floor(float input)
	{
		return std::floor(input);
	}

	inline float scalar_ceil(float input)
	{
		return std::ceil(input);
	}

	inline float scalar_clamp(float input, float min, float max)
	{
		return std::min(std::max(input, min), max);
	}

	inline float scalar_abs(float input)
	{
		return std::fabs(input);
	}

	inline float scalar_sqrt(float input)
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_cvtss_f32(_mm_sqrt_ss(_mm_set_ps1(input)));
#else
		return std::sqrt(input);
#endif
	}

	inline float scalar_sqrt_reciprocal(float input)
	{
#if defined(RTM_SSE2_INTRINSICS)
		// Perform two passes of Newton-Raphson iteration on the hardware estimate
		__m128 input_v = _mm_set_ss(input);
		__m128 half = _mm_set_ss(0.5f);
		__m128 input_half_v = _mm_mul_ss(input_v, half);
		__m128 x0 = _mm_rsqrt_ss(input_v);

		// First iteration
		__m128 x1 = _mm_mul_ss(x0, x0);
		x1 = _mm_sub_ss(half, _mm_mul_ss(input_half_v, x1));
		x1 = _mm_add_ss(_mm_mul_ss(x0, x1), x0);

		// Second iteration
		__m128 x2 = _mm_mul_ss(x1, x1);
		x2 = _mm_sub_ss(half, _mm_mul_ss(input_half_v, x2));
		x2 = _mm_add_ss(_mm_mul_ss(x1, x2), x1);

		return _mm_cvtss_f32(x2);
#else
		return 1.0f / scalar_sqrt(input);
#endif
	}

	inline float scalar_reciprocal(float input)
	{
#if defined(RTM_SSE2_INTRINSICS)
		// Perform two passes of Newton-Raphson iteration on the hardware estimate
		__m128 input_v = _mm_set_ps1(input);
		__m128 x0 = _mm_rcp_ss(input_v);

		// First iteration
		__m128 x1 = _mm_sub_ss(_mm_add_ss(x0, x0), _mm_mul_ss(input_v, _mm_mul_ss(x0, x0)));

		// Second iteration
		__m128 x2 = _mm_sub_ss(_mm_add_ss(x1, x1), _mm_mul_ss(input_v, _mm_mul_ss(x1, x1)));

		return _mm_cvtss_f32(x2);
#else
		return 1.0f / input;
#endif
	}

	inline float scalar_sin(float angle)
	{
		return std::sin(angle);
	}

	inline float scalar_cos(float angle)
	{
		return std::cos(angle);
	}

	inline void scalar_sincos(float angle, float& out_sin, float& out_cos)
	{
		out_sin = scalar_sin(angle);
		out_cos = scalar_cos(angle);
	}

	inline float scalar_acos(float value)
	{
		return std::acos(value);
	}

	inline float scalar_atan2(float left, float right)
	{
		return std::atan2(left, right);
	}

	inline float scalar_min(float left, float right)
	{
		return std::min(left, right);
	}

	inline float scalar_max(float left, float right)
	{
		return std::max(left, right);
	}

	constexpr float scalar_deg_to_rad(float deg)
	{
		return deg * ((float)k_pi / 180.0f);
	}

	inline bool scalar_near_equal(float lhs, float rhs, float threshold)
	{
		return scalar_abs(lhs - rhs) < threshold;
	}

	inline bool scalar_is_finite(float input)
	{
		return std::isfinite(input);
	}

	inline float scalar_symmetric_round(float input)
	{
		return input >= 0.0f ? scalar_floor(input + 0.5f) : scalar_ceil(input - 0.5f);
	}

	inline float scalar_fraction(float value)
	{
		return value - scalar_floor(value);
	}

	template<typename SrcIntegralType>
	inline float scalar_safe_to_float(SrcIntegralType input)
	{
		float input_f = float(input);
		RTM_ASSERT(SrcIntegralType(input_f) == input, "Conversion to float would result in truncation");
		return input_f;
	}
}
