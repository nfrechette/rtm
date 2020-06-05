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
		return scalarf{ _mm_set_ps1(xyzw) };
#else
		return xyzw;
#endif
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Writes a scalar to memory.
	//////////////////////////////////////////////////////////////////////////
	inline void RTM_SIMD_CALL scalar_store(scalarf_arg0 input, float* output) RTM_NO_EXCEPT
	{
		_mm_store_ss(output, input.value);
	}
#endif

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
		return _mm_cvtss_f32(input.value);
#else
		return input;
#endif
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Returns the largest integer value not greater than the input.
	// scalar_floor(1.8) = 1.0
	// scalar_floor(-1.8) = -2.0
	//////////////////////////////////////////////////////////////////////////
	inline scalarf RTM_SIMD_CALL scalar_floor(scalarf_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE4_INTRINSICS)
		return scalarf{ _mm_round_ss(input.value, input.value, 0x9) };
#else
		// NaN, +- Infinity, and numbers larger or equal to 2^23 remain unchanged
		// since they have no fractional part.

		const __m128i abs_mask = _mm_set_epi32(0x7FFFFFFFULL, 0x7FFFFFFFULL, 0x7FFFFFFFULL, 0x7FFFFFFFULL);
		const __m128 fractional_limit = _mm_set_ps1(8388608.0F); // 2^23

		// Build our mask, larger values that have no fractional part, and infinities will be true
		// Smaller values and NaN will be false
		__m128 abs_input = _mm_and_ps(input.value, _mm_castsi128_ps(abs_mask));
		__m128 is_input_large = _mm_cmpge_ss(abs_input, fractional_limit);

		// Test if our input is NaN with (value != value), it is only true for NaN
		__m128 is_nan = _mm_cmpneq_ss(input.value, input.value);

		// Combine our masks to determine if we should return the original value
		__m128 use_original_input = _mm_or_ps(is_input_large, is_nan);

		// Convert to an integer and back. This does banker's rounding by default
		__m128 integer_part = _mm_cvtepi32_ps(_mm_cvtps_epi32(input.value));

		// Test if the returned value is greater than the original.
		// A negative input will round towards zero and be greater when we need it to be smaller.
		__m128 is_negative = _mm_cmpgt_ss(integer_part, input.value);

		// Convert our mask to a float, ~0 yields -1.0 since it is a valid signed integer
		// Positive values will yield a 0.0 bias
		__m128 bias = _mm_cvtepi32_ps(_mm_castps_si128(is_negative));

		// Add our bias to properly handle negative values
		integer_part = _mm_add_ss(integer_part, bias);

		__m128 result = _mm_or_ps(_mm_and_ps(use_original_input, input.value), _mm_andnot_ps(use_original_input, integer_part));
		return scalarf{ result };
#endif
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Returns the largest integer value not greater than the input.
	// scalar_floor(1.8) = 1.0
	// scalar_floor(-1.8) = -2.0
	//////////////////////////////////////////////////////////////////////////
	inline float scalar_floor(float input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return scalar_cast(scalar_floor(scalar_set(input)));
#else
		return std::floor(input);
#endif
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Returns the smallest integer value not less than the input.
	// scalar_ceil(1.8) = 2.0
	// scalar_ceil(-1.8) = -1.0
	//////////////////////////////////////////////////////////////////////////
	inline scalarf RTM_SIMD_CALL scalar_ceil(scalarf_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE4_INTRINSICS)
		return scalarf{ _mm_round_ss(input.value, input.value, 0xA) };
#else
		// NaN, +- Infinity, and numbers larger or equal to 2^23 remain unchanged
		// since they have no fractional part.

		const __m128i abs_mask = _mm_set_epi32(0x7FFFFFFFULL, 0x7FFFFFFFULL, 0x7FFFFFFFULL, 0x7FFFFFFFULL);
		const __m128 fractional_limit = _mm_set_ps1(8388608.0F); // 2^23

		// Build our mask, larger values that have no fractional part, and infinities will be true
		// Smaller values and NaN will be false
		__m128 abs_input = _mm_and_ps(input.value, _mm_castsi128_ps(abs_mask));
		__m128 is_input_large = _mm_cmpge_ss(abs_input, fractional_limit);

		// Test if our input is NaN with (value != value), it is only true for NaN
		__m128 is_nan = _mm_cmpneq_ss(input.value, input.value);

		// Combine our masks to determine if we should return the original value
		__m128 use_original_input = _mm_or_ps(is_input_large, is_nan);

		// Convert to an integer and back. This does banker's rounding by default
		__m128 integer_part = _mm_cvtepi32_ps(_mm_cvtps_epi32(input.value));

		// Test if the returned value is smaller than the original.
		// A positive input will round towards zero and be lower when we need it to be greater.
		__m128 is_positive = _mm_cmplt_ss(integer_part, input.value);

		// Convert our mask to a float, ~0 yields -1.0 since it is a valid signed integer
		// Negative values will yield a 0.0 bias
		__m128 bias = _mm_cvtepi32_ps(_mm_castps_si128(is_positive));

		// Subtract our bias to properly handle positive values
		integer_part = _mm_sub_ss(integer_part, bias);

		__m128 result = _mm_or_ps(_mm_and_ps(use_original_input, input.value), _mm_andnot_ps(use_original_input, integer_part));
		return scalarf{ result };
#endif
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Returns the smallest integer value not less than the input.
	// scalar_ceil(1.8) = 2.0
	// scalar_ceil(-1.8) = -1.0
	//////////////////////////////////////////////////////////////////////////
	inline float scalar_ceil(float input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return scalar_cast(scalar_ceil(scalar_set(input)));
#else
		return std::ceil(input);
#endif
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Returns the input if it is within the min/max values otherwise the
	// exceeded boundary is returned.
	//////////////////////////////////////////////////////////////////////////
	inline scalarf RTM_SIMD_CALL scalar_clamp(scalarf_arg0 input, scalarf_arg1 min, scalarf_arg2 max) RTM_NO_EXCEPT
	{
		return scalarf{ _mm_min_ss(_mm_max_ss(input.value, min.value), max.value) };
	}
#endif

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

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Returns the absolute value of the input.
	//////////////////////////////////////////////////////////////////////////
	inline scalarf RTM_SIMD_CALL scalar_abs(scalarf_arg0 input) RTM_NO_EXCEPT
	{
		const __m128i abs_mask = _mm_set_epi32(0x7FFFFFFFULL, 0x7FFFFFFFULL, 0x7FFFFFFFULL, 0x7FFFFFFFULL);
		return scalarf{ _mm_and_ps(input.value, _mm_castsi128_ps(abs_mask)) };
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Returns the absolute value of the input.
	//////////////////////////////////////////////////////////////////////////
	inline float scalar_abs(float input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		const __m128i abs_mask = _mm_set_epi32(0x7FFFFFFFULL, 0x7FFFFFFFULL, 0x7FFFFFFFULL, 0x7FFFFFFFULL);
		return _mm_cvtss_f32(_mm_and_ps(_mm_set_ps1(input), _mm_castsi128_ps(abs_mask)));
#else
		return std::fabs(input);
#endif
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Returns the square root of the input.
	//////////////////////////////////////////////////////////////////////////
	inline scalarf RTM_SIMD_CALL scalar_sqrt(scalarf_arg0 input) RTM_NO_EXCEPT
	{
		return scalarf{ _mm_sqrt_ss(input.value) };
	}
#endif

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

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Returns the reciprocal square root of the input.
	//////////////////////////////////////////////////////////////////////////
	#if defined(_MSC_VER) && !defined(__clang__) && _MSC_VER >= 1920 && _MSC_VER < 1925 && defined(_M_X64) && !defined(RTM_AVX_INTRINSICS)
		// HACK!!! Visual Studio 2019 has a code generation bug triggered by the code below, disable optimizations for now
		// Bug only happens with x64 SSE2, not with AVX nor with x86
		// Fixed in 16.5.4, see https://github.com/nfrechette/rtm/issues/35
		// TODO: Remove this hack sometime in 2022 or later once the fix is old enough that we no longer have to support the hack
		#pragma optimize("", off)
	#endif
	inline scalarf RTM_SIMD_CALL scalar_sqrt_reciprocal(scalarf_arg0 input) RTM_NO_EXCEPT
	{
		// Perform two passes of Newton-Raphson iteration on the hardware estimate
		const __m128 half = _mm_set_ss(0.5F);
		const __m128 input_half = _mm_mul_ss(input.value, half);
		const __m128 x0 = _mm_rsqrt_ss(input.value);

		// First iteration
		__m128 x1 = _mm_mul_ss(x0, x0);
		x1 = _mm_sub_ss(half, _mm_mul_ss(input_half, x1));
		x1 = _mm_add_ss(_mm_mul_ss(x0, x1), x0);

		// Second iteration
		__m128 x2 = _mm_mul_ss(x1, x1);
		x2 = _mm_sub_ss(half, _mm_mul_ss(input_half, x2));
		x2 = _mm_add_ss(_mm_mul_ss(x1, x2), x1);

		return scalarf{ x2 };
	}
	#if defined(_MSC_VER) && !defined(__clang__) && _MSC_VER >= 1920 && _MSC_VER < 1925 && defined(_M_X64) && !defined(RTM_AVX_INTRINSICS)
		// HACK!!! See comment above
		#pragma optimize("", on)
	#endif
#endif

	//////////////////////////////////////////////////////////////////////////
	// Returns the reciprocal square root of the input.
	//////////////////////////////////////////////////////////////////////////
	inline float RTM_SIMD_CALL scalar_sqrt_reciprocal(float input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return scalar_cast(scalar_sqrt_reciprocal(scalar_set(input)));
#else
		return 1.0F / scalar_sqrt(input);
#endif
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Returns the reciprocal of the input.
	//////////////////////////////////////////////////////////////////////////
	inline scalarf RTM_SIMD_CALL scalar_reciprocal(scalarf_arg0 input) RTM_NO_EXCEPT
	{
		// Perform two passes of Newton-Raphson iteration on the hardware estimate
		__m128 x0 = _mm_rcp_ss(input.value);

		// First iteration
		__m128 x1 = _mm_sub_ss(_mm_add_ss(x0, x0), _mm_mul_ss(input.value, _mm_mul_ss(x0, x0)));

		// Second iteration
		__m128 x2 = _mm_sub_ss(_mm_add_ss(x1, x1), _mm_mul_ss(input.value, _mm_mul_ss(x1, x1)));

		return scalarf{ x2 };
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Returns the reciprocal of the input.
	//////////////////////////////////////////////////////////////////////////
	inline float RTM_SIMD_CALL scalar_reciprocal(float input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return scalar_cast(scalar_reciprocal(scalar_set(input)));
#else
		return 1.0f / input;
#endif
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Returns the addition of the two scalar inputs.
	//////////////////////////////////////////////////////////////////////////
	inline scalarf RTM_SIMD_CALL scalar_add(scalarf_arg0 lhs, scalarf_arg1 rhs) RTM_NO_EXCEPT
	{
		return scalarf{ _mm_add_ss(lhs.value, rhs.value) };
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Returns the addition of the two scalar inputs.
	//////////////////////////////////////////////////////////////////////////
	constexpr float scalar_add(float lhs, float rhs) RTM_NO_EXCEPT
	{
		return lhs + rhs;
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Returns the subtraction of the two scalar inputs.
	//////////////////////////////////////////////////////////////////////////
	inline scalarf RTM_SIMD_CALL scalar_sub(scalarf_arg0 lhs, scalarf_arg1 rhs) RTM_NO_EXCEPT
	{
		return scalarf{ _mm_sub_ss(lhs.value, rhs.value) };
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Returns the subtraction of the two scalar inputs.
	//////////////////////////////////////////////////////////////////////////
	constexpr float scalar_sub(float lhs, float rhs) RTM_NO_EXCEPT
	{
		return lhs - rhs;
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Returns the multiplication of the two scalar inputs.
	//////////////////////////////////////////////////////////////////////////
	inline scalarf RTM_SIMD_CALL scalar_mul(scalarf_arg0 lhs, scalarf_arg1 rhs) RTM_NO_EXCEPT
	{
		return scalarf{ _mm_mul_ss(lhs.value, rhs.value) };
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Returns the multiplication of the two scalar inputs.
	//////////////////////////////////////////////////////////////////////////
	constexpr float scalar_mul(float lhs, float rhs) RTM_NO_EXCEPT
	{
		return lhs * rhs;
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Returns the division of the two scalar inputs.
	//////////////////////////////////////////////////////////////////////////
	inline scalarf RTM_SIMD_CALL scalar_div(scalarf_arg0 lhs, scalarf_arg1 rhs) RTM_NO_EXCEPT
	{
		return scalarf{ _mm_div_ss(lhs.value, rhs.value) };
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Returns the division of the two scalar inputs.
	//////////////////////////////////////////////////////////////////////////
	constexpr float scalar_div(float lhs, float rhs) RTM_NO_EXCEPT
	{
		return lhs / rhs;
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Returns the multiplication/addition of the three inputs: s2 + (s0 * s1)
	//////////////////////////////////////////////////////////////////////////
	inline scalarf RTM_SIMD_CALL scalar_mul_add(scalarf_arg0 s0, scalarf_arg1 s1, scalarf_arg2 s2) RTM_NO_EXCEPT
	{
		return scalarf{ _mm_add_ss(_mm_mul_ss(s0.value, s1.value), s2.value) };
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Returns the multiplication/addition of the three inputs: s2 + (s0 * s1)
	//////////////////////////////////////////////////////////////////////////
	constexpr float scalar_mul_add(float s0, float s1, float s2) RTM_NO_EXCEPT
	{
		return (s0 * s1) + s2;
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Returns the negative multiplication/subtraction of the three inputs: -((s0 * s1) - s2)
	// This is mathematically equivalent to: s2 - (s0 * s1)
	//////////////////////////////////////////////////////////////////////////
	inline scalarf RTM_SIMD_CALL scalar_neg_mul_sub(scalarf_arg0 s0, scalarf_arg1 s1, scalarf_arg2 s2) RTM_NO_EXCEPT
	{
		return scalarf{ _mm_sub_ss(s2.value, _mm_mul_ss(s0.value, s1.value)) };
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Returns the negative multiplication/subtraction of the three inputs: -((s0 * s1) - s2)
	// This is mathematically equivalent to: s2 - (s0 * s1)
	//////////////////////////////////////////////////////////////////////////
	constexpr float scalar_neg_mul_sub(float s0, float s1, float s2) RTM_NO_EXCEPT
	{
		return s2 - (s0 * s1);
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Returns the linear interpolation of the two inputs at the specified alpha.
	// The formula used is: ((1.0 - alpha) * start) + (alpha * end).
	// Interpolation is stable and will return 'start' when alpha is 0.0 and 'end' when it is 1.0.
	// This is the same instruction count when FMA is present but it might be slightly slower
	// due to the extra multiplication compared to: start + (alpha * (end - start)).
	//////////////////////////////////////////////////////////////////////////
	RTM_DEPRECATED("Use a scalarf 'alpha', to be removed in v2.0")
	inline scalarf RTM_SIMD_CALL scalar_lerp(scalarf_arg0 start, scalarf_arg1 end, float alpha) RTM_NO_EXCEPT
	{
		// ((1.0 - alpha) * start) + (alpha * end) == (start - alpha * start) + (alpha * end)
		const scalarf alpha_ = scalar_set(alpha);
		return scalar_mul_add(end, alpha_, scalar_neg_mul_sub(start, alpha_, start));
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the linear interpolation of the two inputs at the specified alpha.
	// The formula used is: ((1.0 - alpha) * start) + (alpha * end).
	// Interpolation is stable and will return 'start' when alpha is 0.0 and 'end' when it is 1.0.
	// This is the same instruction count when FMA is present but it might be slightly slower
	// due to the extra multiplication compared to: start + (alpha * (end - start)).
	//////////////////////////////////////////////////////////////////////////
	inline scalarf RTM_SIMD_CALL scalar_lerp(scalarf_arg0 start, scalarf_arg1 end, scalarf_arg2 alpha) RTM_NO_EXCEPT
	{
		// ((1.0 - alpha) * start) + (alpha * end) == (start - alpha * start) + (alpha * end)
		return scalar_mul_add(end, alpha, scalar_neg_mul_sub(start, alpha, start));
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Returns the linear interpolation of the two inputs at the specified alpha.
	// The formula used is: ((1.0 - alpha) * start) + (alpha * end).
	// Interpolation is stable and will return 'start' when alpha is 0.0 and 'end' when it is 1.0.
	// This is the same instruction count when FMA is present but it might be slightly slower
	// due to the extra multiplication compared to: start + (alpha * (end - start)).
	//////////////////////////////////////////////////////////////////////////
	constexpr float scalar_lerp(float start, float end, float alpha) RTM_NO_EXCEPT
	{
		// ((1.0 - alpha) * start) + (alpha * end) == (start - alpha * start) + (alpha * end)
		return (start - (alpha * start)) + (alpha * end);
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Returns the smallest of the two inputs.
	//////////////////////////////////////////////////////////////////////////
	inline scalarf RTM_SIMD_CALL scalar_min(scalarf_arg0 lhs, scalarf_arg1 rhs) RTM_NO_EXCEPT
	{
		return scalarf{ _mm_min_ss(lhs.value, rhs.value) };
	}
#endif

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

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Returns the largest of the two inputs.
	//////////////////////////////////////////////////////////////////////////
	inline scalarf RTM_SIMD_CALL scalar_max(scalarf_arg0 lhs, scalarf_arg1 rhs) RTM_NO_EXCEPT
	{
		return scalarf{ _mm_max_ss(lhs.value, rhs.value) };
	}
#endif

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

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Returns true if both inputs are equal, false otherwise.
	//////////////////////////////////////////////////////////////////////////
	inline bool RTM_SIMD_CALL scalar_is_equal(scalarf_arg0 lhs, scalarf_arg1 rhs) RTM_NO_EXCEPT
	{
		return _mm_comieq_ss(lhs.value, rhs.value) != 0;
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Returns true if both inputs are equal, false otherwise.
	//////////////////////////////////////////////////////////////////////////
	constexpr bool scalar_is_equal(float lhs, float rhs) RTM_NO_EXCEPT
	{
		return lhs == rhs;
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Returns true if lhs < rhs, false otherwise.
	//////////////////////////////////////////////////////////////////////////
	inline bool RTM_SIMD_CALL scalar_is_lower(scalarf_arg0 lhs, scalarf_arg1 rhs) RTM_NO_EXCEPT
	{
		return _mm_comilt_ss(lhs.value, rhs.value) != 0;
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Returns true if lhs < rhs, false otherwise.
	//////////////////////////////////////////////////////////////////////////
	constexpr bool scalar_is_lower(float lhs, float rhs) RTM_NO_EXCEPT
	{
		return lhs < rhs;
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Returns true if lhs <= rhs, false otherwise.
	//////////////////////////////////////////////////////////////////////////
	inline bool RTM_SIMD_CALL scalar_is_lower_equal(scalarf_arg0 lhs, scalarf_arg1 rhs) RTM_NO_EXCEPT
	{
		return _mm_comile_ss(lhs.value, rhs.value) != 0;
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Returns true if lhs <= rhs, false otherwise.
	//////////////////////////////////////////////////////////////////////////
	constexpr bool scalar_is_lower_equal(float lhs, float rhs) RTM_NO_EXCEPT
	{
		return lhs <= rhs;
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Returns true if lhs > rhs, false otherwise.
	//////////////////////////////////////////////////////////////////////////
	inline bool RTM_SIMD_CALL scalar_is_greater(scalarf_arg0 lhs, scalarf_arg1 rhs) RTM_NO_EXCEPT
	{
		return _mm_comigt_ss(lhs.value, rhs.value) != 0;
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Returns true if lhs > rhs, false otherwise.
	//////////////////////////////////////////////////////////////////////////
	constexpr bool scalar_is_greater(float lhs, float rhs) RTM_NO_EXCEPT
	{
		return lhs > rhs;
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Returns true if lhs >= rhs, false otherwise.
	//////////////////////////////////////////////////////////////////////////
	inline bool RTM_SIMD_CALL scalar_is_greater_equal(scalarf_arg0 lhs, scalarf_arg1 rhs) RTM_NO_EXCEPT
	{
		return _mm_comige_ss(lhs.value, rhs.value) != 0;
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Returns true if lhs >= rhs, false otherwise.
	//////////////////////////////////////////////////////////////////////////
	constexpr bool scalar_is_greater_equal(float lhs, float rhs) RTM_NO_EXCEPT
	{
		return lhs >= rhs;
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Returns true if both inputs are nearly equal, otherwise false: abs(lhs - rhs) <= threshold
	//////////////////////////////////////////////////////////////////////////
	inline bool RTM_SIMD_CALL scalar_near_equal(scalarf_arg0 lhs, scalarf_arg1 rhs, scalarf_arg2 threshold) RTM_NO_EXCEPT
	{
		return scalar_is_lower_equal(scalar_abs(scalar_sub(lhs, rhs)), threshold);
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if both inputs are nearly equal, otherwise false: abs(lhs - rhs) <= 0.00001
	//////////////////////////////////////////////////////////////////////////
	RTM_DEPRECATED("Always specify a threshold explicitly, to be removed in v2.0")
	inline bool RTM_SIMD_CALL scalar_near_equal(scalarf_arg0 lhs, scalarf_arg1 rhs) RTM_NO_EXCEPT
	{
		return scalar_is_lower_equal(scalar_abs(scalar_sub(lhs, rhs)), scalar_set(0.00001F));
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Returns true if both inputs are nearly equal, otherwise false: abs(lhs - rhs) <= threshold
	//////////////////////////////////////////////////////////////////////////
	inline bool scalar_near_equal(float lhs, float rhs, float threshold) RTM_NO_EXCEPT
	{
		return scalar_abs(lhs - rhs) <= threshold;
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if both inputs are nearly equal, otherwise false: abs(lhs - rhs) <= 0.00001
	//////////////////////////////////////////////////////////////////////////
	RTM_DEPRECATED("Always specify a threshold explicitly, to be removed in v2.0")
	inline bool scalar_near_equal(float lhs, float rhs) RTM_NO_EXCEPT
	{
		return scalar_abs(lhs - rhs) <= 0.00001F;
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if the input is finite (not NaN or Inf), false otherwise.
	//////////////////////////////////////////////////////////////////////////
	inline bool scalar_is_finite(float input) RTM_NO_EXCEPT
	{
		return std::isfinite(input);
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Returns the rounded input using a symmetric algorithm.
	// scalar_symmetric_round(1.5) = 2.0
	// scalar_symmetric_round(1.2) = 1.0
	// scalar_symmetric_round(-1.5) = -2.0
	// scalar_symmetric_round(-1.2) = -1.0
	// Note: This function relies on the default floating point rounding mode (banker's rounding).
	//////////////////////////////////////////////////////////////////////////
	inline scalarf RTM_SIMD_CALL scalar_round_symmetric(scalarf_arg0 input) RTM_NO_EXCEPT
	{
		// NaN, +- Infinity, and numbers larger or equal to 2^23 remain unchanged
		// since they have no fractional part.

#if defined(RTM_SSE4_INTRINSICS)
		const __m128 sign_mask = _mm_set_ps(-0.0F, -0.0F, -0.0F, -0.0F);
		__m128 sign = _mm_and_ps(input.value, sign_mask);

		// For positive values, we add a bias of 0.5.
		// For negative values, we add a bias of -0.5.
		__m128 bias = _mm_or_ps(sign, _mm_set_ps1(0.5F));
		__m128 biased_input = _mm_add_ss(input.value, bias);

		__m128 floored = _mm_floor_ss(biased_input, biased_input);
		__m128 ceiled = _mm_ceil_ss(biased_input, biased_input);
		__m128 is_positive = _mm_cmpge_ss(input.value, _mm_setzero_ps());

#if defined(RTM_AVX_INTRINSICS)
		__m128 result = _mm_blendv_ps(ceiled, floored, is_positive);
#else
		__m128 result = _mm_or_ps(_mm_and_ps(is_positive, floored), _mm_andnot_ps(is_positive, ceiled));
#endif
		return scalarf{ result };
#else
		const __m128i abs_mask = _mm_set_epi32(0x7FFFFFFFULL, 0x7FFFFFFFULL, 0x7FFFFFFFULL, 0x7FFFFFFFULL);
		const __m128 fractional_limit = _mm_set_ps1(8388608.0F); // 2^23

		// Build our mask, larger values that have no fractional part, and infinities will be true
		// Smaller values and NaN will be false
		__m128 abs_input = _mm_and_ps(input.value, _mm_castsi128_ps(abs_mask));
		__m128 is_input_large = _mm_cmpge_ss(abs_input, fractional_limit);

		// Test if our input is NaN with (value != value), it is only true for NaN
		__m128 is_nan = _mm_cmpneq_ss(input.value, input.value);

		// Combine our masks to determine if we should return the original value
		__m128 use_original_input = _mm_or_ps(is_input_large, is_nan);

		const __m128 sign_mask = _mm_set_ps(-0.0F, -0.0F, -0.0F, -0.0F);
		__m128 sign = _mm_and_ps(input.value, sign_mask);

		// For positive values, we add a bias of 0.5.
		// For negative values, we add a bias of -0.5.
		__m128 bias = _mm_or_ps(sign, _mm_set_ps1(0.5F));
		__m128 biased_input = _mm_add_ss(input.value, bias);

		// Convert to an integer with truncation and back, this rounds towards zero.
		__m128 integer_part = _mm_cvtepi32_ps(_mm_cvttps_epi32(biased_input));

		__m128 result = _mm_or_ps(_mm_and_ps(use_original_input, input.value), _mm_andnot_ps(use_original_input, integer_part));

		return scalarf{ result };
#endif
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Returns the rounded input using a symmetric algorithm.
	// scalar_round_symmetric(1.5) = 2.0
	// scalar_round_symmetric(1.2) = 1.0
	// scalar_round_symmetric(-1.5) = -2.0
	// scalar_round_symmetric(-1.2) = -1.0
	//////////////////////////////////////////////////////////////////////////
	inline float scalar_round_symmetric(float input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return scalar_cast(scalar_round_symmetric(scalar_set(input)));
#else
		return input >= 0.0F ? scalar_floor(input + 0.5F) : scalar_ceil(input - 0.5F);
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the rounded input using a symmetric algorithm.
	// scalar_symmetric_round(1.5) = 2.0
	// scalar_symmetric_round(1.2) = 1.0
	// scalar_symmetric_round(-1.5) = -2.0
	// scalar_symmetric_round(-1.2) = -1.0
	//////////////////////////////////////////////////////////////////////////
	RTM_DEPRECATED("Use scalar_round_symmetric instead, to be removed in v2.0")
	inline float scalar_symmetric_round(float input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return scalar_cast(scalar_round_symmetric(scalar_set(input)));
#else
		return input >= 0.0F ? scalar_floor(input + 0.5F) : scalar_ceil(input - 0.5F);
#endif
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Returns the rounded input using banker's rounding (half to even).
	// scalar_round_bankers(2.5) = 2.0
	// scalar_round_bankers(1.5) = 2.0
	// scalar_round_bankers(1.2) = 1.0
	// scalar_round_bankers(-2.5) = -2.0
	// scalar_round_bankers(-1.5) = -2.0
	// scalar_round_bankers(-1.2) = -1.0
	// Note: This function relies on the default floating point rounding mode (banker's rounding).
	//////////////////////////////////////////////////////////////////////////
	inline scalarf RTM_SIMD_CALL scalar_round_bankers(scalarf_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE4_INTRINSICS)
		return scalarf{ _mm_cvtss_f32(_mm_round_ss(input.value, input.value, _MM_FROUND_TO_NEAREST_INT | _MM_FROUND_NO_EXC)) };
#else
		const __m128 sign_mask = _mm_set_ps(-0.0F, -0.0F, -0.0F, -0.0F);
		__m128 sign = _mm_and_ps(input.value, sign_mask);

		// We add the largest integer that a 32 bit floating point number can represent and subtract it afterwards.
		// This relies on the fact that if we had a fractional part, the new value cannot be represented accurately
		// and IEEE 754 will perform rounding for us. The default rounding mode is Banker's rounding.
		// This has the effect of removing the fractional part while simultaneously rounding.
		// Use the same sign as the input value to make sure we handle positive and negative values.
		const __m128 fractional_limit = _mm_set_ps1(8388608.0F); // 2^23
		__m128 truncating_offset = _mm_or_ps(sign, fractional_limit);
		__m128 integer_part = _mm_sub_ss(_mm_add_ss(input.value, truncating_offset), truncating_offset);

		// If our input was so large that it had no fractional part, return it unchanged
		// Otherwise return our integer part
		const __m128i abs_mask = _mm_set_epi32(0x7FFFFFFFULL, 0x7FFFFFFFULL, 0x7FFFFFFFULL, 0x7FFFFFFFULL);
		__m128 abs_input = _mm_and_ps(input.value, _mm_castsi128_ps(abs_mask));
		__m128 is_input_large = _mm_cmpge_ss(abs_input, fractional_limit);
		__m128 result = _mm_or_ps(_mm_and_ps(is_input_large, input.value), _mm_andnot_ps(is_input_large, integer_part));

		return scalarf{ result };
#endif
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Returns the rounded input using banker's rounding (half to even).
	// scalar_round_bankers(2.5) = 2.0
	// scalar_round_bankers(1.5) = 2.0
	// scalar_round_bankers(1.2) = 1.0
	// scalar_round_bankers(-2.5) = -2.0
	// scalar_round_bankers(-1.5) = -2.0
	// scalar_round_bankers(-1.2) = -1.0
	//////////////////////////////////////////////////////////////////////////
	inline float scalar_round_bankers(float input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return scalar_cast(scalar_round_bankers(scalar_set(input)));
#else
		int32_t whole = static_cast<int32_t>(input);
		float whole_f = static_cast<float>(whole);
		float remainder = scalar_abs(input - whole_f);
		if (remainder < 0.5F)
			return whole_f;
		if (remainder > 0.5F)
			return input >= 0.0F ? (whole_f + 1.0F) : (whole_f - 1.0F);

		if ((whole % 2) == 0)
			return whole_f;
		else
			return input >= 0.0F ? (whole_f + 1.0F) : (whole_f - 1.0F);
#endif
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
	// Trigonometric functions
	//////////////////////////////////////////////////////////////////////////

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
}

RTM_IMPL_FILE_PRAGMA_POP
