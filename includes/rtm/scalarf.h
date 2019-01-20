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
#include "rtm/impl/compiler_utils.h"

#include <algorithm>
#include <cmath>

RTM_IMPL_FILE_PRAGMA_PUSH

namespace rtm
{
	//////////////////////////////////////////////////////////////////////////
	// Creates a scalar from a floating point value.
	//////////////////////////////////////////////////////////////////////////
	inline scalarf RTM_SIMD_CALL scalar_set(float xyzw) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_set_ps1(xyzw);
#else
		return xyzw;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Casts a scalar into a floating point value.
	//////////////////////////////////////////////////////////////////////////
	inline float RTM_SIMD_CALL scalar_cast(scalarf_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_cvtss_f32(input);
#else
		return input;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the largest integer value not greater than the input.
	// scalar_floor(1.8) = 1.0
	// scalar_floor(-1.8) = -2.0
	//////////////////////////////////////////////////////////////////////////
	inline float scalar_floor(float input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE4_INTRINSICS)
		const __m128 value = _mm_set_ps1(input);
		return _mm_cvtss_f32(_mm_round_ss(value, value, 0x9));
#else
		return std::floor(input);
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the smallest integer value not less than the input.
	// scalar_ceil(1.8) = 2.0
	// scalar_ceil(-1.8) = -1.0
	//////////////////////////////////////////////////////////////////////////
	inline float scalar_ceil(float input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE4_INTRINSICS)
		const __m128 value = _mm_set_ps1(input);
		return _mm_cvtss_f32(_mm_round_ss(value, value, 0xA));
#else
		return std::ceil(input);
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the input if it is within the min/max values otherwise the
	// exceeded boundary is returned.
	//////////////////////////////////////////////////////////////////////////
	inline float scalar_clamp(float input, float min, float max) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_cvtss_f32(_mm_min_ss(_mm_max_ss(_mm_set_ps1(input), _mm_set_ps1(min)), _mm_set_ps1(max)));
#else
		return std::min(std::max(input, min), max);
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the absolute value of the input.
	//////////////////////////////////////////////////////////////////////////
	inline float scalar_abs(float input) RTM_NO_EXCEPT
	{
		return std::fabs(input);
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the square root of the input.
	//////////////////////////////////////////////////////////////////////////
	inline float RTM_SIMD_CALL scalar_sqrt(float input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_cvtss_f32(_mm_sqrt_ss(_mm_set_ps1(input)));
#else
		return std::sqrt(input);
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the reciprocal square root of the input.
	//////////////////////////////////////////////////////////////////////////
	inline float RTM_SIMD_CALL scalar_sqrt_reciprocal(float input) RTM_NO_EXCEPT
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

	//////////////////////////////////////////////////////////////////////////
	// Returns the reciprocal of the input.
	//////////////////////////////////////////////////////////////////////////
	inline float RTM_SIMD_CALL scalar_reciprocal(float input) RTM_NO_EXCEPT
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

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Returns the reciprocal of the input.
	//////////////////////////////////////////////////////////////////////////
	inline scalarf RTM_SIMD_CALL scalar_reciprocal(scalarf_arg0 input) RTM_NO_EXCEPT
	{
		// Perform two passes of Newton-Raphson iteration on the hardware estimate
		__m128 x0 = _mm_rcp_ss(input);

		// First iteration
		__m128 x1 = _mm_sub_ss(_mm_add_ss(x0, x0), _mm_mul_ss(input, _mm_mul_ss(x0, x0)));

		// Second iteration
		__m128 x2 = _mm_sub_ss(_mm_add_ss(x1, x1), _mm_mul_ss(input, _mm_mul_ss(x1, x1)));

		return x2;
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Returns the sine of the input angle.
	//////////////////////////////////////////////////////////////////////////
	inline float scalar_sin(float angle) RTM_NO_EXCEPT
	{
		return std::sin(angle);
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the cosine of the input angle.
	//////////////////////////////////////////////////////////////////////////
	inline float scalar_cos(float angle) RTM_NO_EXCEPT
	{
		return std::cos(angle);
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns both sine and cosine of the input angle.
	//////////////////////////////////////////////////////////////////////////
	inline void scalar_sincos(float angle, float& out_sin, float& out_cos) RTM_NO_EXCEPT
	{
		out_sin = scalar_sin(angle);
		out_cos = scalar_cos(angle);
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the arc-cosine of the input.
	//////////////////////////////////////////////////////////////////////////
	inline float scalar_acos(float value) RTM_NO_EXCEPT
	{
		return std::acos(value);
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the arc-tangent of [x/y] using the sign of the arguments to
	// determine the correct quadrant.
	//////////////////////////////////////////////////////////////////////////
	inline float scalar_atan2(float x, float y) RTM_NO_EXCEPT
	{
		return std::atan2(x, y);
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the smallest of the two inputs.
	//////////////////////////////////////////////////////////////////////////
	inline float scalar_min(float left, float right) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_cvtss_f32(_mm_min_ss(_mm_set_ps1(left), _mm_set_ps1(right)));
#else
		return std::min(left, right);
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the largest of the two inputs.
	//////////////////////////////////////////////////////////////////////////
	inline float scalar_max(float left, float right) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_cvtss_f32(_mm_max_ss(_mm_set_ps1(left), _mm_set_ps1(right)));
#else
		return std::max(left, right);
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if both inputs are nearly equal, false otherwise.
	//////////////////////////////////////////////////////////////////////////
	inline bool scalar_near_equal(float lhs, float rhs, float threshold = 0.00001f) RTM_NO_EXCEPT
	{
		return scalar_abs(lhs - rhs) < threshold;
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if the input is finite (not NaN or Inf), false otherwise.
	//////////////////////////////////////////////////////////////////////////
	inline bool scalar_is_finite(float input) RTM_NO_EXCEPT
	{
		return std::isfinite(input);
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the rounded input using a symmetric algorithm.
	// scalar_symmetric_round(1.5) = 2.0
	// scalar_symmetric_round(1.2) = 1.0
	// scalar_symmetric_round(-1.5) = -2.0
	// scalar_symmetric_round(-1.2) = -1.0
	//////////////////////////////////////////////////////////////////////////
	inline float scalar_symmetric_round(float input) RTM_NO_EXCEPT
	{
		return input >= 0.0f ? scalar_floor(input + 0.5f) : scalar_ceil(input - 0.5f);
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the fractional part of the input.
	//////////////////////////////////////////////////////////////////////////
	inline float scalar_fraction(float value) RTM_NO_EXCEPT
	{
		return value - scalar_floor(value);
	}

	//////////////////////////////////////////////////////////////////////////
	// Safely casts an integral input into a float64 output.
	//////////////////////////////////////////////////////////////////////////
	template<typename SrcIntegralType>
	inline float scalar_safe_to_float(SrcIntegralType input) RTM_NO_EXCEPT
	{
		float input_f = float(input);
		RTM_ASSERT(SrcIntegralType(input_f) == input, "Conversion to float would result in truncation");
		return input_f;
	}
}

RTM_IMPL_FILE_PRAGMA_POP
