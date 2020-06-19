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

RTM_FORCE_NOINLINE vector4f RTM_SIMD_CALL vector_round_bankers_scalar(vector4f_arg0 input) RTM_NO_EXCEPT
{
	scalarf x = scalar_round_bankers(scalarf(vector_get_x(input)));
	scalarf y = scalar_round_bankers(scalarf(vector_get_y(input)));
	scalarf z = scalar_round_bankers(scalarf(vector_get_z(input)));
	scalarf w = scalar_round_bankers(scalarf(vector_get_w(input)));
	return vector_set(x, y, z, w);
}

#if defined(RTM_SSE2_INTRINSICS)
RTM_FORCE_NOINLINE vector4f RTM_SIMD_CALL vector_round_bankers_sse2(vector4f_arg0 input) RTM_NO_EXCEPT
{
	const __m128 sign_mask = _mm_set_ps(-0.0F, -0.0F, -0.0F, -0.0F);
	__m128 sign = _mm_and_ps(input, sign_mask);

	// We add the largest integer that a 32 bit floating point number can represent and subtract it afterwards.
	// This relies on the fact that if we had a fractional part, the new value cannot be represented accurately
	// and IEEE 754 will perform rounding for us. The default rounding mode is Banker's rounding.
	// This has the effect of removing the fractional part while simultaneously rounding.
	// Use the same sign as the input value to make sure we handle positive and negative values.
	const __m128 fractional_limit = _mm_set_ps1(8388608.0F); // 2^23
	__m128 truncating_offset = _mm_or_ps(sign, fractional_limit);
	__m128 integer_part = _mm_sub_ps(_mm_add_ps(input, truncating_offset), truncating_offset);

	// If our input was so large that it had no fractional part, return it unchanged
	// Otherwise return our integer part
	const __m128i abs_mask = _mm_set_epi32(0x7FFFFFFFULL, 0x7FFFFFFFULL, 0x7FFFFFFFULL, 0x7FFFFFFFULL);
	__m128 abs_input = _mm_and_ps(input, _mm_castsi128_ps(abs_mask));
	__m128 is_input_large = _mm_cmpge_ps(abs_input, fractional_limit);
	return _mm_or_ps(_mm_and_ps(is_input_large, input), _mm_andnot_ps(is_input_large, integer_part));
}
#endif

#if defined(RTM_NEON64_INTRINSICS)
RTM_FORCE_NOINLINE vector4f RTM_SIMD_CALL vector_round_bankers_neon64(vector4f_arg0 input) RTM_NO_EXCEPT
{
	return vrndnq_f32(input);
}
#endif

#if defined(RTM_NEON_INTRINSICS)
RTM_FORCE_NOINLINE vector4f RTM_SIMD_CALL vector_round_bankers_neon(vector4f_arg0 input) RTM_NO_EXCEPT
{
	uint32x4_t sign = vandq_u32(vreinterpretq_u32_f32(input), vdupq_n_f32(-0.0F));

	// We add the largest integer that a 32 bit floating point number can represent and subtract it afterwards.
	// This relies on the fact that if we had a fractional part, the new value cannot be represented accurately
	// and IEEE 754 will perform rounding for us. The default rounding mode is Banker's rounding.
	// This has the effect of removing the fractional part while simultaneously rounding.
	// Use the same sign as the input value to make sure we handle positive and negative values.
	float32x4_t fractional_limit = vdupq_n_f32(8388608.0F); // 2^23
	float32x4_t truncating_offset = vreinterpretq_f32_u32(vorrq_u32(sign, vreinterpretq_u32_f32(fractional_limit)));
	float32x4_t integer_part = vsubq_f32(vaddq_f32(input, truncating_offset), truncating_offset);

	// If our input was so large that it had no fractional part, return it unchanged
	// Otherwise return our integer part
	uint32x4_t is_input_large = vcageq_f32(input, fractional_limit);
	return vbslq_f32(is_input_large, input, integer_part);
}
#endif

static void bm_vector_round_bankers_scalar(benchmark::State& state)
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
		v0 = vector_round_bankers_scalar(v0);
		v1 = vector_round_bankers_scalar(v1);
		v2 = vector_round_bankers_scalar(v2);
		v3 = vector_round_bankers_scalar(v3);
		v4 = vector_round_bankers_scalar(v4);
		v5 = vector_round_bankers_scalar(v5);
		v6 = vector_round_bankers_scalar(v6);
		v7 = vector_round_bankers_scalar(v7);
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

BENCHMARK(bm_vector_round_bankers_scalar);

#if defined(RTM_SSE2_INTRINSICS)
static void bm_vector_round_bankers_sse2(benchmark::State& state)
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
		v0 = vector_round_bankers_sse2(v0);
		v1 = vector_round_bankers_sse2(v1);
		v2 = vector_round_bankers_sse2(v2);
		v3 = vector_round_bankers_sse2(v3);
		v4 = vector_round_bankers_sse2(v4);
		v5 = vector_round_bankers_sse2(v5);
		v6 = vector_round_bankers_sse2(v6);
		v7 = vector_round_bankers_sse2(v7);
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

BENCHMARK(bm_vector_round_bankers_sse2);
#endif

#if defined(RTM_NEON64_INTRINSICS)
static void bm_vector_round_bankers_neon64(benchmark::State& state)
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
		v0 = vector_round_bankers_neon64(v0);
		v1 = vector_round_bankers_neon64(v1);
		v2 = vector_round_bankers_neon64(v2);
		v3 = vector_round_bankers_neon64(v3);
		v4 = vector_round_bankers_neon64(v4);
		v5 = vector_round_bankers_neon64(v5);
		v6 = vector_round_bankers_neon64(v6);
		v7 = vector_round_bankers_neon64(v7);
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

BENCHMARK(bm_vector_round_bankers_neon64);
#endif

#if defined(RTM_NEON_INTRINSICS)
static void bm_vector_round_bankers_neon(benchmark::State& state)
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
		v0 = vector_round_bankers_neon(v0);
		v1 = vector_round_bankers_neon(v1);
		v2 = vector_round_bankers_neon(v2);
		v3 = vector_round_bankers_neon(v3);
		v4 = vector_round_bankers_neon(v4);
		v5 = vector_round_bankers_neon(v5);
		v6 = vector_round_bankers_neon(v6);
		v7 = vector_round_bankers_neon(v7);
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

BENCHMARK(bm_vector_round_bankers_neon);
#endif
