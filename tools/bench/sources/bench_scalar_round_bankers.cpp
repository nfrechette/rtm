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

RTM_FORCE_NOINLINE float RTM_SIMD_CALL scalar_round_bankers_scalar(float input) RTM_NO_EXCEPT
{
	if (!scalar_is_finite(input))
		return input;

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
}

#if defined(RTM_SSE2_INTRINSICS)
RTM_FORCE_NOINLINE float RTM_SIMD_CALL scalar_round_bankers_sse2(float input) RTM_NO_EXCEPT
{
	__m128 input_s = _mm_set_ps1(input);

	const __m128 sign_mask = _mm_set_ps(-0.0F, -0.0F, -0.0F, -0.0F);
	__m128 sign = _mm_and_ps(input_s, sign_mask);

	// We add the largest integer that a 32 bit floating point number can represent and subtract it afterwards.
	// This relies on the fact that if we had a fractional part, the new value cannot be represented accurately
	// and IEEE 754 will perform rounding for us. The default rounding mode is Banker's rounding.
	// This has the effect of removing the fractional part while simultaneously rounding.
	// Use the same sign as the input value to make sure we handle positive and negative values.
	const __m128 fractional_limit = _mm_set_ps1(8388608.0F); // 2^23
	__m128 truncating_offset = _mm_or_ps(sign, fractional_limit);
	__m128 integer_part = _mm_sub_ss(_mm_add_ss(input_s, truncating_offset), truncating_offset);

	// If our input was so large that it had no fractional part, return it unchanged
	// Otherwise return our integer part
	const __m128i abs_mask = _mm_set_epi32(0x7FFFFFFFULL, 0x7FFFFFFFULL, 0x7FFFFFFFULL, 0x7FFFFFFFULL);
	__m128 abs_input = _mm_and_ps(input_s, _mm_castsi128_ps(abs_mask));
	__m128 is_input_large = _mm_cmpge_ss(abs_input, fractional_limit);
	__m128 result = _mm_or_ps(_mm_and_ps(is_input_large, input_s), _mm_andnot_ps(is_input_large, integer_part));
	return _mm_cvtss_f32(result);
}
#endif

#if defined(RTM_NEON64_INTRINSICS) && defined(__ARM_FEATURE_DIRECTED_ROUNDING)
RTM_FORCE_NOINLINE float RTM_SIMD_CALL scalar_round_bankers_neon64(float input) RTM_NO_EXCEPT
{
	return vrndns_f32(input);
}
#endif

static void bm_scalar_round_bankers_scalar(benchmark::State& state)
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
		f0 = scalar_round_bankers_scalar(f0);
		f1 = scalar_round_bankers_scalar(f1);
		f2 = scalar_round_bankers_scalar(f2);
		f3 = scalar_round_bankers_scalar(f3);
		f4 = scalar_round_bankers_scalar(f4);
		f5 = scalar_round_bankers_scalar(f5);
		f6 = scalar_round_bankers_scalar(f6);
		f7 = scalar_round_bankers_scalar(f7);
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

BENCHMARK(bm_scalar_round_bankers_scalar);

#if defined(RTM_SSE2_INTRINSICS)
static void bm_scalar_round_bankers_sse2(benchmark::State& state)
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
		f0 = scalar_round_bankers_sse2(f0);
		f1 = scalar_round_bankers_sse2(f1);
		f2 = scalar_round_bankers_sse2(f2);
		f3 = scalar_round_bankers_sse2(f3);
		f4 = scalar_round_bankers_sse2(f4);
		f5 = scalar_round_bankers_sse2(f5);
		f6 = scalar_round_bankers_sse2(f6);
		f7 = scalar_round_bankers_sse2(f7);
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

BENCHMARK(bm_scalar_round_bankers_sse2);
#endif

#if defined(RTM_NEON64_INTRINSICS) && defined(__ARM_FEATURE_DIRECTED_ROUNDING)
static void bm_scalar_round_bankers_neon64(benchmark::State& state)
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
		f0 = scalar_round_bankers_neon64(f0);
		f1 = scalar_round_bankers_neon64(f1);
		f2 = scalar_round_bankers_neon64(f2);
		f3 = scalar_round_bankers_neon64(f3);
		f4 = scalar_round_bankers_neon64(f4);
		f5 = scalar_round_bankers_neon64(f5);
		f6 = scalar_round_bankers_neon64(f6);
		f7 = scalar_round_bankers_neon64(f7);
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

BENCHMARK(bm_scalar_round_bankers_neon64);
#endif
