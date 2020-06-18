////////////////////////////////////////////////////////////////////////////////
// The MIT License (MIT)
//
// Copyright (c) 2020 Nicholas Frechette & Realtime Math contributors
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

#include <benchmark/benchmark.h>

#include <rtm/scalarf.h>

using namespace rtm;

RTM_FORCE_NOINLINE float RTM_SIMD_CALL scalar_floor_scalar(float input) RTM_NO_EXCEPT
{
	return std::floor(input);
}

#if defined(RTM_SSE2_INTRINSICS)
RTM_FORCE_NOINLINE float RTM_SIMD_CALL scalar_floor_sse2(float input) RTM_NO_EXCEPT
{
	__m128 input_s = _mm_set_ps1(input);

	// NaN, +- Infinity, and numbers larger or equal to 2^23 remain unchanged
	// since they have no fractional part.

	const __m128i abs_mask = _mm_set_epi32(0x7FFFFFFFULL, 0x7FFFFFFFULL, 0x7FFFFFFFULL, 0x7FFFFFFFULL);
	const __m128 fractional_limit = _mm_set_ps1(8388608.0F); // 2^23

	// Build our mask, larger values that have no fractional part, and infinities will be true
	// Smaller values and NaN will be false
	__m128 abs_input = _mm_and_ps(input_s, _mm_castsi128_ps(abs_mask));
	__m128 is_input_large = _mm_cmpge_ss(abs_input, fractional_limit);

	// Test if our input is NaN with (value != value), it is only true for NaN
	__m128 is_nan = _mm_cmpneq_ss(input_s, input_s);

	// Combine our masks to determine if we should return the original value
	__m128 use_original_input = _mm_or_ps(is_input_large, is_nan);

	// Convert to an integer and back. This does banker's rounding by default
	__m128 integer_part = _mm_cvtepi32_ps(_mm_cvtps_epi32(input_s));

	// Test if the returned value is greater than the original.
	// A negative input will round towards zero and be greater when we need it to be smaller.
	__m128 is_negative = _mm_cmpgt_ss(integer_part, input_s);

	// Convert our mask to a float, ~0 yields -1.0 since it is a valid signed integer
	// Positive values will yield a 0.0 bias
	__m128 bias = _mm_cvtepi32_ps(_mm_castps_si128(is_negative));

	// Add our bias to properly handle negative values
	integer_part = _mm_add_ss(integer_part, bias);

	__m128 result = _mm_or_ps(_mm_and_ps(use_original_input, input_s), _mm_andnot_ps(use_original_input, integer_part));
	return _mm_cvtss_f32(result);
}
#endif

#if defined(RTM_NEON_INTRINSICS)
RTM_FORCE_NOINLINE float RTM_SIMD_CALL scalar_floor_neon(float input) RTM_NO_EXCEPT
{
	// NaN, +- Infinity, and numbers larger or equal to 2^23 remain unchanged
	// since they have no fractional part.

	const float fractional_limit = 8388608.0F; // 2^23

	// Build our mask, larger values that have no fractional part, and infinities will be true
	// Smaller values and NaN will be false
	float abs_input = std::abs(input);
	bool is_input_large = abs_input >= fractional_limit;

	// Test if our input is NaN with (value != value), it is only true for NaN
	bool is_nan = input != input;

	// Combine our masks to determine if we should return the original value
	bool use_original_input = is_input_large | is_nan;

	// Convert to an integer and back. This does banker's rounding by default
	float integer_part = static_cast<float>(static_cast<int32_t>(input));

	// Test if the returned value is greater than the original.
	// A negative input will round towards zero and be greater when we need it to be smaller.
	bool is_negative = integer_part > input;

	// Subtract our bias to properly handle negative values
	integer_part = is_negative ? (integer_part - 1.0F) : integer_part;

	return use_original_input ? input : integer_part;
}
#endif

static void bm_scalar_floor_scalar(benchmark::State& state)
{
	float f0 = -123.134f;
	float f1 = 123.134f;
	float f2 = -123.134f;
	float f3 = 123.134f;
	float f4 = -123.134f;
	float f5 = 123.134f;
	float f6 = -123.134f;
	float f7 = 123.134f;

	for (auto _ : state)
	{
		f0 = scalar_floor_scalar(f0);
		f1 = scalar_floor_scalar(f1);
		f2 = scalar_floor_scalar(f2);
		f3 = scalar_floor_scalar(f3);
		f4 = scalar_floor_scalar(f4);
		f5 = scalar_floor_scalar(f5);
		f6 = scalar_floor_scalar(f6);
		f7 = scalar_floor_scalar(f7);
	}

	benchmark::DoNotOptimize(f0);
	benchmark::DoNotOptimize(f1);
	benchmark::DoNotOptimize(f2);
	benchmark::DoNotOptimize(f3);
	benchmark::DoNotOptimize(f4);
	benchmark::DoNotOptimize(f5);
	benchmark::DoNotOptimize(f6);
	benchmark::DoNotOptimize(f7);
}

BENCHMARK(bm_scalar_floor_scalar);

#if defined(RTM_SSE2_INTRINSICS)
static void bm_scalar_floor_sse2(benchmark::State& state)
{
	float f0 = -123.134f;
	float f1 = 123.134f;
	float f2 = -123.134f;
	float f3 = 123.134f;
	float f4 = -123.134f;
	float f5 = 123.134f;
	float f6 = -123.134f;
	float f7 = 123.134f;

	for (auto _ : state)
	{
		f0 = scalar_floor_sse2(f0);
		f1 = scalar_floor_sse2(f1);
		f2 = scalar_floor_sse2(f2);
		f3 = scalar_floor_sse2(f3);
		f4 = scalar_floor_sse2(f4);
		f5 = scalar_floor_sse2(f5);
		f6 = scalar_floor_sse2(f6);
		f7 = scalar_floor_sse2(f7);
	}

	benchmark::DoNotOptimize(f0);
	benchmark::DoNotOptimize(f1);
	benchmark::DoNotOptimize(f2);
	benchmark::DoNotOptimize(f3);
	benchmark::DoNotOptimize(f4);
	benchmark::DoNotOptimize(f5);
	benchmark::DoNotOptimize(f6);
	benchmark::DoNotOptimize(f7);
}

BENCHMARK(bm_scalar_floor_sse2);
#endif

#if defined(RTM_NEON_INTRINSICS)
static void bm_scalar_floor_neon(benchmark::State& state)
{
	float f0 = -123.134f;
	float f1 = 123.134f;
	float f2 = -123.134f;
	float f3 = 123.134f;
	float f4 = -123.134f;
	float f5 = 123.134f;
	float f6 = -123.134f;
	float f7 = 123.134f;

	for (auto _ : state)
	{
		f0 = scalar_floor_neon(f0);
		f1 = scalar_floor_neon(f1);
		f2 = scalar_floor_neon(f2);
		f3 = scalar_floor_neon(f3);
		f4 = scalar_floor_neon(f4);
		f5 = scalar_floor_neon(f5);
		f6 = scalar_floor_neon(f6);
		f7 = scalar_floor_neon(f7);
	}

	benchmark::DoNotOptimize(f0);
	benchmark::DoNotOptimize(f1);
	benchmark::DoNotOptimize(f2);
	benchmark::DoNotOptimize(f3);
	benchmark::DoNotOptimize(f4);
	benchmark::DoNotOptimize(f5);
	benchmark::DoNotOptimize(f6);
	benchmark::DoNotOptimize(f7);
}

BENCHMARK(bm_scalar_floor_neon);
#endif
