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

#include <rtm/vector4f.h>

using namespace rtm;

RTM_FORCE_NOINLINE vector4f RTM_SIMD_CALL vector_round_symmetric_scalar(vector4f_arg0 input) RTM_NO_EXCEPT
{
	const vector4f half = vector_set(0.5F);
	const vector4f floored = vector_floor(vector_add(input, half));
	const vector4f ceiled = vector_ceil(vector_sub(input, half));
	const mask4f is_greater_equal = vector_greater_equal(input, vector_zero());
	return vector_select(is_greater_equal, floored, ceiled);
}

#if defined(RTM_SSE2_INTRINSICS)
RTM_FORCE_NOINLINE vector4f RTM_SIMD_CALL vector_round_symmetric_sse2(vector4f_arg0 input) RTM_NO_EXCEPT
{
	const __m128i abs_mask = _mm_set_epi32(0x7FFFFFFFULL, 0x7FFFFFFFULL, 0x7FFFFFFFULL, 0x7FFFFFFFULL);
	const __m128 fractional_limit = _mm_set_ps1(8388608.0F); // 2^23

	// Build our mask, larger values that have no fractional part, and infinities will be true
	// Smaller values and NaN will be false
	__m128 abs_input = _mm_and_ps(input, _mm_castsi128_ps(abs_mask));
	__m128 is_input_large = _mm_cmpge_ps(abs_input, fractional_limit);

	// Test if our input is NaN with (value != value), it is only true for NaN
	__m128 is_nan = _mm_cmpneq_ps(input, input);

	// Combine our masks to determine if we should return the original value
	__m128 use_original_input = _mm_or_ps(is_input_large, is_nan);

	const __m128 sign_mask = _mm_set_ps(-0.0F, -0.0F, -0.0F, -0.0F);
	__m128 sign = _mm_and_ps(input, sign_mask);

	// For positive values, we add a bias of 0.5.
	// For negative values, we add a bias of -0.5.
	__m128 bias = _mm_or_ps(sign, _mm_set_ps1(0.5F));
	__m128 biased_input = _mm_add_ps(input, bias);

	// Convert to an integer with truncation and back, this rounds towards zero.
	__m128 integer_part = _mm_cvtepi32_ps(_mm_cvttps_epi32(biased_input));

	return _mm_or_ps(_mm_and_ps(use_original_input, input), _mm_andnot_ps(use_original_input, integer_part));
}
#endif

#if defined(RTM_NEON_INTRINSICS)
RTM_FORCE_NOINLINE vector4f RTM_SIMD_CALL vector_round_symmetric_neon(vector4f_arg0 input) RTM_NO_EXCEPT
{
	// NaN, +- Infinity, and numbers larger or equal to 2^23 remain unchanged
	// since they have no fractional part.

	float32x4_t fractional_limit = vdupq_n_f32(8388608.0F); // 2^23

	// Build our mask, larger values that have no fractional part, and infinities will be true
	// Smaller values and NaN will be false
	uint32x4_t is_input_large = vcageq_f32(input, fractional_limit);

	// Test if our input is NaN with (value != value), it is only true for NaN
	uint32x4_t is_nan = vmvnq_u32(vceqq_f32(input, input));

	// Combine our masks to determine if we should return the original value
	uint32x4_t use_original_input = vorrq_u32(is_input_large, is_nan);

	uint32x4_t sign = vandq_u32(vreinterpretq_u32_f32(input), vdupq_n_f32(-0.0F));

	// For positive values, we add a bias of 0.5.
	// For negative values, we add a bias of -0.5.
	float32x4_t bias = vreinterpretq_f32_u32(vorrq_u32(sign, vreinterpretq_u32_f32(vdupq_n_f32(0.5F))));
	float32x4_t biased_input = vaddq_f32(input, bias);

	// Convert to an integer and back. This does banker's rounding by default
	float32x4_t integer_part = vcvtq_f32_s32(vcvtq_s32_f32(biased_input));

	return vbslq_f32(use_original_input, input, integer_part);
}
#endif

static void bm_vector_round_symmetric_scalar(benchmark::State& state)
{
	vector4f v0 = vector_set(-123.134f);
	vector4f v1 = vector_set(123.134f);
	vector4f v2 = vector_set(-123.134f);
	vector4f v3 = vector_set(123.134f);
	vector4f v4 = vector_set(-123.134f);
	vector4f v5 = vector_set(123.134f);
	vector4f v6 = vector_set(-123.134f);
	vector4f v7 = vector_set(123.134f);

	for (auto _ : state)
	{
		v0 = vector_round_symmetric_scalar(v0);
		v1 = vector_round_symmetric_scalar(v1);
		v2 = vector_round_symmetric_scalar(v2);
		v3 = vector_round_symmetric_scalar(v3);
		v4 = vector_round_symmetric_scalar(v4);
		v5 = vector_round_symmetric_scalar(v5);
		v6 = vector_round_symmetric_scalar(v6);
		v7 = vector_round_symmetric_scalar(v7);
	}

	benchmark::DoNotOptimize(v0);
	benchmark::DoNotOptimize(v1);
	benchmark::DoNotOptimize(v2);
	benchmark::DoNotOptimize(v3);
	benchmark::DoNotOptimize(v4);
	benchmark::DoNotOptimize(v5);
	benchmark::DoNotOptimize(v6);
	benchmark::DoNotOptimize(v7);
}

BENCHMARK(bm_vector_round_symmetric_scalar);

#if defined(RTM_SSE2_INTRINSICS)
static void bm_vector_round_symmetric_sse2(benchmark::State& state)
{
	vector4f v0 = vector_set(-123.134f);
	vector4f v1 = vector_set(123.134f);
	vector4f v2 = vector_set(-123.134f);
	vector4f v3 = vector_set(123.134f);
	vector4f v4 = vector_set(-123.134f);
	vector4f v5 = vector_set(123.134f);
	vector4f v6 = vector_set(-123.134f);
	vector4f v7 = vector_set(123.134f);

	for (auto _ : state)
	{
		v0 = vector_round_symmetric_sse2(v0);
		v1 = vector_round_symmetric_sse2(v1);
		v2 = vector_round_symmetric_sse2(v2);
		v3 = vector_round_symmetric_sse2(v3);
		v4 = vector_round_symmetric_sse2(v4);
		v5 = vector_round_symmetric_sse2(v5);
		v6 = vector_round_symmetric_sse2(v6);
		v7 = vector_round_symmetric_sse2(v7);
	}

	benchmark::DoNotOptimize(v0);
	benchmark::DoNotOptimize(v1);
	benchmark::DoNotOptimize(v2);
	benchmark::DoNotOptimize(v3);
	benchmark::DoNotOptimize(v4);
	benchmark::DoNotOptimize(v5);
	benchmark::DoNotOptimize(v6);
	benchmark::DoNotOptimize(v7);
}

BENCHMARK(bm_vector_round_symmetric_sse2);
#endif

#if defined(RTM_NEON_INTRINSICS)
static void bm_vector_round_symmetric_neon(benchmark::State& state)
{
	vector4f v0 = vector_set(-123.134f);
	vector4f v1 = vector_set(123.134f);
	vector4f v2 = vector_set(-123.134f);
	vector4f v3 = vector_set(123.134f);
	vector4f v4 = vector_set(-123.134f);
	vector4f v5 = vector_set(123.134f);
	vector4f v6 = vector_set(-123.134f);
	vector4f v7 = vector_set(123.134f);

	for (auto _ : state)
	{
		v0 = vector_round_symmetric_neon(v0);
		v1 = vector_round_symmetric_neon(v1);
		v2 = vector_round_symmetric_neon(v2);
		v3 = vector_round_symmetric_neon(v3);
		v4 = vector_round_symmetric_neon(v4);
		v5 = vector_round_symmetric_neon(v5);
		v6 = vector_round_symmetric_neon(v6);
		v7 = vector_round_symmetric_neon(v7);
	}

	benchmark::DoNotOptimize(v0);
	benchmark::DoNotOptimize(v1);
	benchmark::DoNotOptimize(v2);
	benchmark::DoNotOptimize(v3);
	benchmark::DoNotOptimize(v4);
	benchmark::DoNotOptimize(v5);
	benchmark::DoNotOptimize(v6);
	benchmark::DoNotOptimize(v7);
}

BENCHMARK(bm_vector_round_symmetric_neon);
#endif
