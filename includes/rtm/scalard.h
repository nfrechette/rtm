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
	inline scalard RTM_SIMD_CALL scalar_set(double xyzw) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return scalard{ _mm_set1_pd(xyzw) };
#else
		return xyzw;
#endif
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Writes a scalar to memory.
	//////////////////////////////////////////////////////////////////////////
	inline void RTM_SIMD_CALL scalar_store(scalard input, double* output) RTM_NO_EXCEPT
	{
		_mm_store_sd(output, input.value);
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Writes a scalar to memory.
	//////////////////////////////////////////////////////////////////////////
	inline void scalar_store(double input, double* output) RTM_NO_EXCEPT
	{
		*output = input;
	}

	//////////////////////////////////////////////////////////////////////////
	// Casts a scalar into a floating point value.
	//////////////////////////////////////////////////////////////////////////
	inline double RTM_SIMD_CALL scalar_cast(scalard input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_cvtsd_f64(input.value);
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
	inline scalard RTM_SIMD_CALL scalar_floor(scalard input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE4_INTRINSICS)
		return scalard{ _mm_round_sd(input.value, input.value, 0x9) };
#else
		// NaN, +- Infinity, and numbers larger or equal to 2^23 remain unchanged
		// since they have no fractional part.

#if defined(_MSC_VER) && !defined(__clang__)
		constexpr __m128i abs_mask = { 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0x7FU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0x7FU };
#else
		constexpr __m128i abs_mask = { 0x7FFFFFFFFFFFFFFFULL, 0x7FFFFFFFFFFFFFFFULL };
#endif
		const __m128d fractional_limit = _mm_set1_pd(4503599627370496.0); // 2^52

		// Build our mask, larger values that have no fractional part, and infinities will be true
		// Smaller values and NaN will be false
		__m128d abs_input = _mm_and_pd(input.value, _mm_castsi128_pd(abs_mask));
		__m128d is_input_large = _mm_cmpge_sd(abs_input, fractional_limit);

		// Test if our input is NaN with (value != value), it is only true for NaN
		__m128d is_nan = _mm_cmpneq_sd(input.value, input.value);

		// Combine our masks to determine if we should return the original value
		__m128d use_original_input = _mm_or_pd(is_input_large, is_nan);

		// Convert to an integer and back. This does banker's rounding by default
		__m128d integer_part = _mm_cvtepi32_pd(_mm_cvtpd_epi32(input.value));

		// Test if the returned value is greater than the original.
		// A negative input will round towards zero and be greater when we need it to be smaller.
		__m128d is_negative = _mm_cmpgt_sd(integer_part, input.value);

		// Convert our mask to a float, ~0 yields -1.0 since it is a valid signed integer
		// Positive values will yield a 0.0 bias
		__m128d bias = _mm_cvtepi32_pd(_mm_castpd_si128(is_negative));

		// Add our bias to properly handle negative values
		integer_part = _mm_add_sd(integer_part, bias);

		__m128d result = _mm_or_pd(_mm_and_pd(use_original_input, input.value), _mm_andnot_pd(use_original_input, integer_part));
		return scalard{ result };
#endif
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Returns the largest integer value not greater than the input.
	// scalar_floor(1.8) = 1.0
	// scalar_floor(-1.8) = -2.0
	//////////////////////////////////////////////////////////////////////////
	inline double scalar_floor(double input) RTM_NO_EXCEPT
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
	inline scalard RTM_SIMD_CALL scalar_ceil(scalard input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE4_INTRINSICS)
		return scalard{ _mm_round_sd(input.value, input.value, 0xA) };
#else
		// NaN, +- Infinity, and numbers larger or equal to 2^23 remain unchanged
		// since they have no fractional part.

#if defined(_MSC_VER) && !defined(__clang__)
		constexpr __m128i abs_mask = { 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0x7FU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0x7FU };
#else
		constexpr __m128i abs_mask = { 0x7FFFFFFFFFFFFFFFULL, 0x7FFFFFFFFFFFFFFFULL };
#endif
		const __m128d fractional_limit = _mm_set1_pd(4503599627370496.0); // 2^52

		// Build our mask, larger values that have no fractional part, and infinities will be true
		// Smaller values and NaN will be false
		__m128d abs_input = _mm_and_pd(input.value, _mm_castsi128_pd(abs_mask));
		__m128d is_input_large = _mm_cmpge_sd(abs_input, fractional_limit);

		// Test if our input is NaN with (value != value), it is only true for NaN
		__m128d is_nan = _mm_cmpneq_sd(input.value, input.value);

		// Combine our masks to determine if we should return the original value
		__m128d use_original_input = _mm_or_pd(is_input_large, is_nan);

		// Convert to an integer and back. This does banker's rounding by default
		__m128d integer_part = _mm_cvtepi32_pd(_mm_cvtpd_epi32(input.value));

		// Test if the returned value is smaller than the original.
		// A positive input will round towards zero and be lower when we need it to be greater.
		__m128d is_positive = _mm_cmplt_sd(integer_part, input.value);

		// Convert our mask to a float, ~0 yields -1.0 since it is a valid signed integer
		// Negative values will yield a 0.0 bias
		__m128d bias = _mm_cvtepi32_pd(_mm_castpd_si128(is_positive));

		// Subtract our bias to properly handle positive values
		integer_part = _mm_sub_sd(integer_part, bias);

		__m128d result = _mm_or_pd(_mm_and_pd(use_original_input, input.value), _mm_andnot_pd(use_original_input, integer_part));
		return scalard{ result };
#endif
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Returns the smallest integer value not less than the input.
	// scalar_ceil(1.8) = 2.0
	// scalar_ceil(-1.8) = -1.0
	//////////////////////////////////////////////////////////////////////////
	inline double scalar_ceil(double input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return scalar_cast(scalar_ceil(scalar_set(input)));
#else
		return std::ceil(input);
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the input if it is within the min/max values otherwise the
	// exceeded boundary is returned.
	//////////////////////////////////////////////////////////////////////////
	inline double scalar_clamp(double input, double min, double max) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_cvtsd_f64(_mm_min_sd(_mm_max_sd(_mm_set1_pd(input), _mm_set1_pd(min)), _mm_set1_pd(max)));
#else
		return std::min(std::max(input, min), max);
#endif
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Returns the absolute value of the input.
	//////////////////////////////////////////////////////////////////////////
	inline scalard RTM_SIMD_CALL scalar_abs(scalard input) RTM_NO_EXCEPT
	{
#if defined(_MSC_VER) && !defined(__clang__)
		constexpr __m128i abs_mask = { 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0x7FU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0x7FU };
#else
		constexpr __m128i abs_mask = { 0x7FFFFFFFFFFFFFFFULL, 0x7FFFFFFFFFFFFFFFULL };
#endif
		return scalard{ _mm_and_pd(input.value, _mm_castsi128_pd(abs_mask)) };
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Returns the absolute value of the input.
	//////////////////////////////////////////////////////////////////////////
	inline double scalar_abs(double input) RTM_NO_EXCEPT
	{
		return std::fabs(input);
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Returns the square root of the input.
	//////////////////////////////////////////////////////////////////////////
	inline scalard RTM_SIMD_CALL scalar_sqrt(scalard input) RTM_NO_EXCEPT
	{
		return scalard{ _mm_sqrt_sd(input.value, input.value) };
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Returns the square root of the input.
	//////////////////////////////////////////////////////////////////////////
	inline double scalar_sqrt(double input) RTM_NO_EXCEPT
	{
		return std::sqrt(input);
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Returns the reciprocal square root of the input.
	//////////////////////////////////////////////////////////////////////////
	inline scalard RTM_SIMD_CALL scalar_sqrt_reciprocal(scalard input) RTM_NO_EXCEPT
	{
		const __m128d input_sqrt = _mm_sqrt_sd(input.value, input.value);
		const __m128d result = _mm_div_sd(_mm_set_sd(1.0), input_sqrt);
		return scalard{ result };
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Returns the reciprocal square root of the input.
	//////////////////////////////////////////////////////////////////////////
	inline double scalar_sqrt_reciprocal(double input) RTM_NO_EXCEPT
	{
		return 1.0 / scalar_sqrt(input);
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Returns the reciprocal of the input.
	//////////////////////////////////////////////////////////////////////////
	inline scalard RTM_SIMD_CALL scalar_reciprocal(scalard input) RTM_NO_EXCEPT
	{
		return scalard{ _mm_div_sd(_mm_set1_pd(1.0), input.value) };
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Returns the reciprocal of the input.
	//////////////////////////////////////////////////////////////////////////
	constexpr double scalar_reciprocal(double input) RTM_NO_EXCEPT
	{
		return 1.0 / input;
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Returns the addition of the two scalar inputs.
	//////////////////////////////////////////////////////////////////////////
	inline scalard RTM_SIMD_CALL scalar_add(scalard lhs, scalard rhs) RTM_NO_EXCEPT
	{
		return scalard{ _mm_add_sd(lhs.value, rhs.value) };
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Returns the addition of the two scalar inputs.
	//////////////////////////////////////////////////////////////////////////
	constexpr double scalar_add(double lhs, double rhs) RTM_NO_EXCEPT
	{
		return lhs + rhs;
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Returns the subtraction of the two scalar inputs.
	//////////////////////////////////////////////////////////////////////////
	inline scalard RTM_SIMD_CALL scalar_sub(scalard lhs, scalard rhs) RTM_NO_EXCEPT
	{
		return scalard{ _mm_sub_sd(lhs.value, rhs.value) };
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Returns the subtraction of the two scalar inputs.
	//////////////////////////////////////////////////////////////////////////
	constexpr double scalar_sub(double lhs, double rhs) RTM_NO_EXCEPT
	{
		return lhs - rhs;
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Returns the multiplication of the two scalar inputs.
	//////////////////////////////////////////////////////////////////////////
	inline scalard RTM_SIMD_CALL scalar_mul(scalard lhs, scalard rhs) RTM_NO_EXCEPT
	{
		return scalard{ _mm_mul_sd(lhs.value, rhs.value) };
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Returns the multiplication of the two scalar inputs.
	//////////////////////////////////////////////////////////////////////////
	constexpr double scalar_mul(double lhs, double rhs) RTM_NO_EXCEPT
	{
		return lhs * rhs;
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Returns the division of the two scalar inputs.
	//////////////////////////////////////////////////////////////////////////
	inline scalard RTM_SIMD_CALL scalar_div(scalard lhs, scalard rhs) RTM_NO_EXCEPT
	{
		return scalard{ _mm_div_sd(lhs.value, rhs.value) };
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Returns the division of the two scalar inputs.
	//////////////////////////////////////////////////////////////////////////
	constexpr double scalar_div(double lhs, double rhs) RTM_NO_EXCEPT
	{
		return lhs / rhs;
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Returns the multiplication/addition of the three inputs: s2 + (s0 * s1)
	//////////////////////////////////////////////////////////////////////////
	inline scalard RTM_SIMD_CALL scalar_mul_add(scalard s0, scalard s1, scalard s2) RTM_NO_EXCEPT
	{
		return scalard{ _mm_add_sd(_mm_mul_sd(s0.value, s1.value), s2.value) };
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Returns the multiplication/addition of the three inputs: s2 + (s0 * s1)
	//////////////////////////////////////////////////////////////////////////
	constexpr double scalar_mul_add(double s0, double s1, double s2) RTM_NO_EXCEPT
	{
		return (s0 * s1) + s2;
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Returns the negative multiplication/subtraction of the three inputs: -((s0 * s1) - s2)
	// This is mathematically equivalent to: s2 - (s0 * s1)
	//////////////////////////////////////////////////////////////////////////
	inline scalard RTM_SIMD_CALL scalar_neg_mul_sub(scalard s0, scalard s1, scalard s2) RTM_NO_EXCEPT
	{
		return scalard{ _mm_sub_sd(s2.value, _mm_mul_sd(s0.value, s1.value)) };
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Returns the negative multiplication/subtraction of the three inputs: -((s0 * s1) - s2)
	// This is mathematically equivalent to: s2 - (s0 * s1)
	//////////////////////////////////////////////////////////////////////////
	constexpr double scalar_neg_mul_sub(double s0, double s1, double s2) RTM_NO_EXCEPT
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
	RTM_DEPRECATED("Use a scalard 'alpha', to be removed in v2.0")
	inline scalard RTM_SIMD_CALL scalar_lerp(scalard start, scalard end, double alpha) RTM_NO_EXCEPT
	{
		// ((1.0 - alpha) * start) + (alpha * end) == (start - alpha * start) + (alpha * end)
		const scalard alpha_ = scalar_set(alpha);
		return scalar_mul_add(end, alpha_, scalar_neg_mul_sub(start, alpha_, start));
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the linear interpolation of the two inputs at the specified alpha.
	// The formula used is: ((1.0 - alpha) * start) + (alpha * end).
	// Interpolation is stable and will return 'start' when alpha is 0.0 and 'end' when it is 1.0.
	// This is the same instruction count when FMA is present but it might be slightly slower
	// due to the extra multiplication compared to: start + (alpha * (end - start)).
	//////////////////////////////////////////////////////////////////////////
	inline scalard RTM_SIMD_CALL scalar_lerp(scalard start, scalard end, scalard alpha) RTM_NO_EXCEPT
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
	constexpr double scalar_lerp(double start, double end, double alpha) RTM_NO_EXCEPT
	{
		// ((1.0 - alpha) * start) + (alpha * end) == (start - alpha * start) + (alpha * end)
		return (start - (alpha * start)) + (alpha * end);
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Returns the smallest of the two inputs.
	//////////////////////////////////////////////////////////////////////////
	inline scalard RTM_SIMD_CALL scalar_min(scalard lhs, scalard rhs) RTM_NO_EXCEPT
	{
		return scalard{ _mm_min_sd(lhs.value, rhs.value) };
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Returns the smallest of the two inputs.
	//////////////////////////////////////////////////////////////////////////
	inline double scalar_min(double left, double right) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_cvtsd_f64(_mm_min_sd(_mm_set1_pd(left), _mm_set1_pd(right)));
#else
		return std::min(left, right);
#endif
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Returns the largest of the two inputs.
	//////////////////////////////////////////////////////////////////////////
	inline scalard RTM_SIMD_CALL scalar_max(scalard lhs, scalard rhs) RTM_NO_EXCEPT
	{
		return scalard{ _mm_max_sd(lhs.value, rhs.value) };
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Returns the largest of the two inputs.
	//////////////////////////////////////////////////////////////////////////
	inline double scalar_max(double left, double right) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_cvtsd_f64(_mm_max_sd(_mm_set1_pd(left), _mm_set1_pd(right)));
#else
		return std::max(left, right);
#endif
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Returns true if both inputs are equal, false otherwise.
	//////////////////////////////////////////////////////////////////////////
	inline bool RTM_SIMD_CALL scalar_is_equal(scalard lhs, scalard rhs) RTM_NO_EXCEPT
	{
		return _mm_comieq_sd(lhs.value, rhs.value) != 0;
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Returns true if both inputs are equal, false otherwise.
	//////////////////////////////////////////////////////////////////////////
	constexpr bool scalar_is_equal(double lhs, double rhs) RTM_NO_EXCEPT
	{
		return lhs == rhs;
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Returns true if lhs < rhs, false otherwise.
	//////////////////////////////////////////////////////////////////////////
	inline bool RTM_SIMD_CALL scalar_is_lower(scalard lhs, scalard rhs) RTM_NO_EXCEPT
	{
		return _mm_comilt_sd(lhs.value, rhs.value) != 0;
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Returns true if lhs < rhs, false otherwise.
	//////////////////////////////////////////////////////////////////////////
	constexpr bool scalar_is_lower(double lhs, double rhs) RTM_NO_EXCEPT
	{
		return lhs < rhs;
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Returns true if lhs <= rhs, false otherwise.
	//////////////////////////////////////////////////////////////////////////
	inline bool RTM_SIMD_CALL scalar_is_lower_equal(scalard lhs, scalard rhs) RTM_NO_EXCEPT
	{
		return _mm_comile_sd(lhs.value, rhs.value) != 0;
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Returns true if lhs <= rhs, false otherwise.
	//////////////////////////////////////////////////////////////////////////
	constexpr bool scalar_is_lower_equal(double lhs, double rhs) RTM_NO_EXCEPT
	{
		return lhs <= rhs;
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Returns true if lhs > rhs, false otherwise.
	//////////////////////////////////////////////////////////////////////////
	inline bool RTM_SIMD_CALL scalar_is_greater(scalard lhs, scalard rhs) RTM_NO_EXCEPT
	{
		return _mm_comigt_sd(lhs.value, rhs.value) != 0;
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Returns true if lhs > rhs, false otherwise.
	//////////////////////////////////////////////////////////////////////////
	constexpr bool scalar_is_greater(double lhs, double rhs) RTM_NO_EXCEPT
	{
		return lhs > rhs;
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Returns true if lhs >= rhs, false otherwise.
	//////////////////////////////////////////////////////////////////////////
	inline bool RTM_SIMD_CALL scalar_is_greater_equal(scalard lhs, scalard rhs) RTM_NO_EXCEPT
	{
		return _mm_comige_sd(lhs.value, rhs.value) != 0;
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Returns true if lhs >= rhs, false otherwise.
	//////////////////////////////////////////////////////////////////////////
	constexpr bool scalar_is_greater_equal(double lhs, double rhs) RTM_NO_EXCEPT
	{
		return lhs >= rhs;
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Returns true if both inputs are nearly equal, otherwise false: abs(lhs - rhs) <= threshold
	//////////////////////////////////////////////////////////////////////////
	inline bool RTM_SIMD_CALL scalar_near_equal(scalard lhs, scalard rhs, scalard threshold) RTM_NO_EXCEPT
	{
		return scalar_is_lower_equal(scalar_abs(scalar_sub(lhs, rhs)), threshold);
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if both inputs are nearly equal, otherwise false: abs(lhs - rhs) <= 0.00001
	//////////////////////////////////////////////////////////////////////////
	RTM_DEPRECATED("Always specify a threshold explicitly, to be removed in v2.0")
	inline bool RTM_SIMD_CALL scalar_near_equal(scalard lhs, scalard rhs) RTM_NO_EXCEPT
	{
		return scalar_is_lower_equal(scalar_abs(scalar_sub(lhs, rhs)), scalar_set(0.00001));
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Returns true if both inputs are nearly equal, otherwise false: abs(lhs - rhs) <= threshold
	//////////////////////////////////////////////////////////////////////////
	inline bool scalar_near_equal(double lhs, double rhs, double threshold) RTM_NO_EXCEPT
	{
		return scalar_abs(lhs - rhs) <= threshold;
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if both inputs are nearly equal, otherwise false: abs(lhs - rhs) <= 0.00001
	//////////////////////////////////////////////////////////////////////////
	RTM_DEPRECATED("Always specify a threshold explicitly, to be removed in v2.0")
	inline bool scalar_near_equal(double lhs, double rhs) RTM_NO_EXCEPT
	{
		return scalar_abs(lhs - rhs) <= 0.00001;
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if the input is finite (not NaN or Inf), false otherwise.
	//////////////////////////////////////////////////////////////////////////
	inline bool scalar_is_finite(double input) RTM_NO_EXCEPT
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
	inline double scalar_symmetric_round(double input) RTM_NO_EXCEPT
	{
		return input >= 0.0 ? scalar_floor(input + 0.5) : scalar_ceil(input - 0.5);
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
	inline scalard RTM_SIMD_CALL scalar_round_bankers(scalard input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE4_INTRINSICS)
		return scalard{ _mm_cvtsd_f64(_mm_round_sd(input.value, input.value, _MM_FROUND_TO_NEAREST_INT | _MM_FROUND_NO_EXC)) };
#else
#if defined(_MSC_VER) && !defined(__clang__)
		constexpr __m128i abs_mask = { 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0x7FU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0x7FU };
#else
		constexpr __m128i abs_mask = { 0x7FFFFFFFFFFFFFFFULL, 0x7FFFFFFFFFFFFFFFULL };
#endif

		const __m128d sign_mask = _mm_set_pd(-0.0, -0.0);
		__m128d sign = _mm_and_pd(input.value, sign_mask);

		// We add the largest integer that a 64 bit floating point number can represent and subtract it afterwards.
		// This relies on the fact that if we had a fractional part, the new value cannot be represented accurately
		// and IEEE 754 will perform rounding for us. The default rounding mode is Banker's rounding.
		// This has the effect of removing the fractional part while simultaneously rounding.
		// Use the same sign as the input value to make sure we handle positive and negative values.
		const __m128d fractional_limit = _mm_set1_pd(4503599627370496.0); // 2^52
		__m128d truncating_offset = _mm_or_pd(sign, fractional_limit);
		__m128d integer_part = _mm_sub_sd(_mm_add_sd(input.value, truncating_offset), truncating_offset);

		__m128d abs_input = _mm_and_pd(input.value, _mm_castsi128_pd(abs_mask));
		__m128d is_input_large = _mm_cmpge_sd(abs_input, fractional_limit);
		__m128d result = _mm_or_pd(_mm_and_pd(is_input_large, input.value), _mm_andnot_pd(is_input_large, integer_part));

		return scalard{ result };
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
	inline double scalar_round_bankers(double input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return scalar_cast(scalar_round_bankers(scalar_set(input)));
#else
		int64_t whole = static_cast<int64_t>(input);
		double whole_f = static_cast<double>(whole);
		double remainder = scalar_abs(input - whole_f);
		if (remainder < 0.5)
			return whole_f;
		if (remainder > 0.5)
			return input >= 0.0 ? (whole_f + 1.0) : (whole_f - 1.0);

		if ((whole % 2) == 0)
			return whole_f;
		else
			return input >= 0.0 ? (whole_f + 1.0) : (whole_f - 1.0);
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the fractional part of the input.
	//////////////////////////////////////////////////////////////////////////
	inline double scalar_fraction(double value) RTM_NO_EXCEPT
	{
		return value - scalar_floor(value);
	}

	//////////////////////////////////////////////////////////////////////////
	// Safely casts an integral input into a float64 output.
	//////////////////////////////////////////////////////////////////////////
	template<typename SrcIntegralType>
	inline double scalar_safe_to_double(SrcIntegralType input) RTM_NO_EXCEPT
	{
		double input_f = double(input);
		RTM_ASSERT(SrcIntegralType(input_f) == input, "Conversion to double would result in truncation");
		return input_f;
	}

	//////////////////////////////////////////////////////////////////////////
	// Trigonometric functions
	//////////////////////////////////////////////////////////////////////////

	//////////////////////////////////////////////////////////////////////////
	// Returns the sine of the input angle.
	//////////////////////////////////////////////////////////////////////////
	inline double scalar_sin(double angle) RTM_NO_EXCEPT
	{
		return std::sin(angle);
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the cosine of the input angle.
	//////////////////////////////////////////////////////////////////////////
	inline double scalar_cos(double angle) RTM_NO_EXCEPT
	{
		return std::cos(angle);
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns both sine and cosine of the input angle.
	//////////////////////////////////////////////////////////////////////////
	inline void scalar_sincos(double angle, double& out_sin, double& out_cos) RTM_NO_EXCEPT
	{
		out_sin = scalar_sin(angle);
		out_cos = scalar_cos(angle);
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the arc-cosine of the input.
	//////////////////////////////////////////////////////////////////////////
	inline double scalar_acos(double value) RTM_NO_EXCEPT
	{
		return std::acos(value);
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the arc-tangent of [x/y] using the sign of the arguments to
	// determine the correct quadrant.
	//////////////////////////////////////////////////////////////////////////
	inline double scalar_atan2(double x, double y) RTM_NO_EXCEPT
	{
		return std::atan2(x, y);
	}
}

RTM_IMPL_FILE_PRAGMA_POP
