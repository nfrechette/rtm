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
#include "rtm/impl/scalar_common.h"

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
	// Writes a scalar to memory.
	//////////////////////////////////////////////////////////////////////////
	inline void scalar_store(float input, float* output) RTM_NO_EXCEPT
	{
		*output = input;
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
#if defined(RTM_SSE2_INTRINSICS)
#if defined(_MSC_VER)
		constexpr __m128i masks = { 0xFFU, 0xFFU, 0xFFU, 0x7FU, 0xFFU, 0xFFU, 0xFFU, 0x7FU, 0xFFU, 0xFFU, 0xFFU, 0x7FU, 0xFFU, 0xFFU, 0xFFU, 0x7FU };
#else
		constexpr __m128i masks = { 0x7FFFFFFF7FFFFFFFULL, 0x7FFFFFFF7FFFFFFFULL };
#endif
		return _mm_cvtss_f32(_mm_and_ps(_mm_set_ps1(input), _mm_castsi128_ps(masks)));
#else
		return std::fabs(input);
#endif
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
#if defined(_MSC_VER) && _MSC_VER >= 1920 && defined(_M_X64) && defined(RTM_SSE2_INTRINSICS) && !defined(RTM_AVX_INTRINSICS)
	// HACK!!! Visual Studio 2019 has a code generation bug triggered by the code below, disable optimizations for now
	// Bug only happens with x64 SSE2, not with AVX nor with x86
	#pragma optimize("", off)
#endif
	inline float RTM_SIMD_CALL scalar_sqrt_reciprocal(float input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		// Perform two passes of Newton-Raphson iteration on the hardware estimate
		const __m128 input_v = _mm_set_ss(input);
		const __m128 half = _mm_set_ss(0.5F);
		const __m128 input_half_v = _mm_mul_ss(input_v, half);
		const __m128 x0 = _mm_rsqrt_ss(input_v);

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
		return 1.0F / scalar_sqrt(input);
#endif
	}
#if defined(_MSC_VER) && _MSC_VER >= 1920 && defined(_M_X64) && defined(RTM_SSE2_INTRINSICS) && !defined(RTM_AVX_INTRINSICS)
	// HACK!!! See comment above
	#pragma optimize("", on)
#endif

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

	//////////////////////////////////////////////////////////////////////////
	// Returns the subtraction of the two scalar inputs.
	//////////////////////////////////////////////////////////////////////////
	inline float scalar_sub(float lhs, float rhs) RTM_NO_EXCEPT
	{
		return lhs - rhs;
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the multiplication/addition of the three inputs: s2 + (s0 * s1)
	//////////////////////////////////////////////////////////////////////////
	inline float scalar_mul_add(float s0, float s1, float s2) RTM_NO_EXCEPT
	{
		return (s0 * s1) + s2;
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the linear interpolation of the two inputs at the specified alpha.
	//////////////////////////////////////////////////////////////////////////
	inline float scalar_lerp(float start, float end, float alpha) RTM_NO_EXCEPT
	{
		return ((end - start) * alpha) + start;
	}

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
	// Returns true if both inputs are equal, false otherwise.
	//////////////////////////////////////////////////////////////////////////
	inline bool scalar_is_equal(float lhs, float rhs) RTM_NO_EXCEPT
	{
		return lhs == rhs;
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if lhs < rhs, false otherwise.
	//////////////////////////////////////////////////////////////////////////
	inline bool scalar_is_lower(float lhs, float rhs) RTM_NO_EXCEPT
	{
		return lhs < rhs;
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if lhs <= rhs, false otherwise.
	//////////////////////////////////////////////////////////////////////////
	inline bool scalar_is_lower_equal(float lhs, float rhs) RTM_NO_EXCEPT
	{
		return lhs <= rhs;
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if lhs > rhs, false otherwise.
	//////////////////////////////////////////////////////////////////////////
	inline bool scalar_is_greater(float lhs, float rhs) RTM_NO_EXCEPT
	{
		return lhs > rhs;
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if lhs >= rhs, false otherwise.
	//////////////////////////////////////////////////////////////////////////
	inline bool scalar_is_greater_equal(float lhs, float rhs) RTM_NO_EXCEPT
	{
		return lhs >= rhs;
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if both inputs are nearly equal, false otherwise.
	//////////////////////////////////////////////////////////////////////////
	inline bool scalar_near_equal(float lhs, float rhs, float threshold) RTM_NO_EXCEPT
	{
		return scalar_abs(lhs - rhs) < threshold;
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if both inputs are nearly equal, false otherwise.
	//////////////////////////////////////////////////////////////////////////
	RTM_DEPRECATED("Always specify a threshold explicitly, to be removed in v2.0")
	inline bool scalar_near_equal(float lhs, float rhs) RTM_NO_EXCEPT
	{
		return scalar_abs(lhs - rhs) < 0.00001F;
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
		return input >= 0.0F ? scalar_floor(input + 0.5F) : scalar_ceil(input - 0.5F);
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

	//////////////////////////////////////////////////////////////////////////
	// SSE implementation of scalarf
	//////////////////////////////////////////////////////////////////////////

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Writes a scalar to memory.
	//////////////////////////////////////////////////////////////////////////
	inline void RTM_SIMD_CALL scalar_store(scalarf_arg0 input, float* output) RTM_NO_EXCEPT
	{
		_mm_store_ss(output, input);
	}

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

	//////////////////////////////////////////////////////////////////////////
	// Returns the subtraction of the two scalar inputs.
	//////////////////////////////////////////////////////////////////////////
	inline scalarf RTM_SIMD_CALL scalar_sub(scalarf_arg0 lhs, scalarf_arg1 rhs) RTM_NO_EXCEPT
	{
		return _mm_sub_ss(lhs, rhs);
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the multiplication/addition of the three inputs: s2 + (s0 * s1)
	//////////////////////////////////////////////////////////////////////////
	inline scalarf RTM_SIMD_CALL scalar_mul_add(scalarf_arg0 s0, scalarf_arg1 s1, scalarf_arg2 s2) RTM_NO_EXCEPT
	{
		return _mm_add_ss(_mm_mul_ss(s0, s1), s2);
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the linear interpolation of the two inputs at the specified alpha.
	//////////////////////////////////////////////////////////////////////////
	inline scalarf RTM_SIMD_CALL scalar_lerp(scalarf_arg0 start, scalarf_arg1 end, float alpha) RTM_NO_EXCEPT
	{
		return scalar_mul_add(scalar_sub(end, start), scalar_set(alpha), start);
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the smallest of the two inputs.
	//////////////////////////////////////////////////////////////////////////
	inline scalarf RTM_SIMD_CALL scalar_min(scalarf_arg0 lhs, scalarf_arg1 rhs) RTM_NO_EXCEPT
	{
		return _mm_min_ss(lhs, rhs);
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the largest of the two inputs.
	//////////////////////////////////////////////////////////////////////////
	inline scalarf RTM_SIMD_CALL scalar_max(scalarf_arg0 lhs, scalarf_arg1 rhs) RTM_NO_EXCEPT
	{
		return _mm_max_ss(lhs, rhs);
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the absolute value of the input.
	//////////////////////////////////////////////////////////////////////////
	inline scalarf RTM_SIMD_CALL scalar_abs(scalarf_arg0 input) RTM_NO_EXCEPT
	{
#if defined(_MSC_VER)
		constexpr __m128i masks = { 0xFFU, 0xFFU, 0xFFU, 0x7FU, 0xFFU, 0xFFU, 0xFFU, 0x7FU, 0xFFU, 0xFFU, 0xFFU, 0x7FU, 0xFFU, 0xFFU, 0xFFU, 0x7FU };
#else
		constexpr __m128i masks = { 0x7FFFFFFF7FFFFFFFULL, 0x7FFFFFFF7FFFFFFFULL };
#endif
		return _mm_and_ps(input, _mm_castsi128_ps(masks));
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the square root of the input.
	//////////////////////////////////////////////////////////////////////////
	inline scalarf RTM_SIMD_CALL scalar_sqrt(scalarf_arg0 input) RTM_NO_EXCEPT
	{
		return _mm_sqrt_ss(input);
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if both inputs are equal, false otherwise.
	//////////////////////////////////////////////////////////////////////////
	inline bool RTM_SIMD_CALL scalar_is_equal(scalarf_arg0 lhs, scalarf_arg1 rhs) RTM_NO_EXCEPT
	{
		return _mm_comieq_ss(lhs, rhs) != 0;
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if lhs < rhs, false otherwise.
	//////////////////////////////////////////////////////////////////////////
	inline bool RTM_SIMD_CALL scalar_is_lower(scalarf_arg0 lhs, scalarf_arg1 rhs) RTM_NO_EXCEPT
	{
		return _mm_comilt_ss(lhs, rhs) != 0;
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if lhs <= rhs, false otherwise.
	//////////////////////////////////////////////////////////////////////////
	inline bool RTM_SIMD_CALL scalar_is_lower_equal(scalarf_arg0 lhs, scalarf_arg1 rhs) RTM_NO_EXCEPT
	{
		return _mm_comile_ss(lhs, rhs) != 0;
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if lhs > rhs, false otherwise.
	//////////////////////////////////////////////////////////////////////////
	inline bool RTM_SIMD_CALL scalar_is_greater(scalarf_arg0 lhs, scalarf_arg1 rhs) RTM_NO_EXCEPT
	{
		return _mm_comigt_ss(lhs, rhs) != 0;
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if lhs >= rhs, false otherwise.
	//////////////////////////////////////////////////////////////////////////
	inline bool RTM_SIMD_CALL scalar_is_greater_equal(scalarf_arg0 lhs, scalarf_arg1 rhs) RTM_NO_EXCEPT
	{
		return _mm_comige_ss(lhs, rhs) != 0;
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if both inputs are nearly equal, false otherwise.
	//////////////////////////////////////////////////////////////////////////
	inline bool scalar_near_equal(scalarf_arg0 lhs, scalarf_arg1 rhs, scalarf_arg2 threshold) RTM_NO_EXCEPT
	{
		return scalar_is_lower(scalar_abs(scalar_sub(lhs, rhs)), threshold);
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if both inputs are nearly equal, false otherwise.
	//////////////////////////////////////////////////////////////////////////
	RTM_DEPRECATED("Always specify a threshold explicitly, to be removed in v2.0")
	inline bool scalar_near_equal(scalarf_arg0 lhs, scalarf_arg1 rhs) RTM_NO_EXCEPT
	{
		return scalar_is_lower(scalar_abs(scalar_sub(lhs, rhs)), scalar_set(0.00001F));
	}
#endif
}

RTM_IMPL_FILE_PRAGMA_POP
