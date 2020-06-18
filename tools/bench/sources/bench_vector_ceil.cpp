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

RTM_FORCE_NOINLINE vector4f RTM_SIMD_CALL vector_ceil_scalar(vector4f_arg0 input) RTM_NO_EXCEPT
{
	scalarf x = vector_get_x(input);
	scalarf y = vector_get_y(input);
	scalarf z = vector_get_z(input);
	scalarf w = vector_get_w(input);
	return vector_set(scalar_ceil(x), scalar_ceil(y), scalar_ceil(z), scalar_ceil(w));
}

#if defined(RTM_SSE2_INTRINSICS)
RTM_FORCE_NOINLINE vector4f RTM_SIMD_CALL vector_ceil_sse2(vector4f_arg0 input) RTM_NO_EXCEPT
{
	// NaN, +- Infinity, and numbers larger or equal to 2^23 remain unchanged
	// since they have no fractional part.

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

	// Convert to an integer and back. This does banker's rounding by default
	__m128 integer_part = _mm_cvtepi32_ps(_mm_cvtps_epi32(input));

	// Test if the returned value is smaller than the original.
	// A positive input will round towards zero and be lower when we need it to be greater.
	__m128 is_positive = _mm_cmplt_ps(integer_part, input);

	// Convert our mask to a float, ~0 yields -1.0 since it is a valid signed integer
	// Negative values will yield a 0.0 bias
	__m128 bias = _mm_cvtepi32_ps(_mm_castps_si128(is_positive));

	// Subtract our bias to properly handle positive values
	integer_part = _mm_sub_ps(integer_part, bias);

	return _mm_or_ps(_mm_and_ps(use_original_input, input), _mm_andnot_ps(use_original_input, integer_part));
}
#endif

#if defined(RTM_NEON_INTRINSICS)
RTM_FORCE_NOINLINE vector4f RTM_SIMD_CALL vector_ceil_neon(vector4f_arg0 input) RTM_NO_EXCEPT
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

	// Convert to an integer and back. This does banker's rounding by default
	float32x4_t integer_part = vcvtq_f32_s32(vcvtq_s32_f32(input));

	// Test if the returned value is smaller than the original.
	// A positive input will round towards zero and be lower when we need it to be greater.
	uint32x4_t is_positive = vcltq_f32(integer_part, input);

	float32x4_t bias = vcvtq_f32_s32(is_positive);

	// Subtract our bias to properly handle positive values
	integer_part = vsubq_f32(integer_part, bias);

	return vbslq_f32(use_original_input, input, integer_part);
}
#endif

static void bm_vector_ceil_scalar(benchmark::State& state)
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
		v0 = vector_ceil_scalar(v0);
		v1 = vector_ceil_scalar(v1);
		v2 = vector_ceil_scalar(v2);
		v3 = vector_ceil_scalar(v3);
		v4 = vector_ceil_scalar(v4);
		v5 = vector_ceil_scalar(v5);
		v6 = vector_ceil_scalar(v6);
		v7 = vector_ceil_scalar(v7);
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

BENCHMARK(bm_vector_ceil_scalar);

#if defined(RTM_SSE2_INTRINSICS)
static void bm_vector_ceil_sse2(benchmark::State& state)
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
		v0 = vector_ceil_sse2(v0);
		v1 = vector_ceil_sse2(v1);
		v2 = vector_ceil_sse2(v2);
		v3 = vector_ceil_sse2(v3);
		v4 = vector_ceil_sse2(v4);
		v5 = vector_ceil_sse2(v5);
		v6 = vector_ceil_sse2(v6);
		v7 = vector_ceil_sse2(v7);
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

BENCHMARK(bm_vector_ceil_sse2);
#endif

#if defined(RTM_NEON_INTRINSICS)
static void bm_vector_ceil_neon(benchmark::State& state)
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
		v0 = vector_ceil_neon(v0);
		v1 = vector_ceil_neon(v1);
		v2 = vector_ceil_neon(v2);
		v3 = vector_ceil_neon(v3);
		v4 = vector_ceil_neon(v4);
		v5 = vector_ceil_neon(v5);
		v6 = vector_ceil_neon(v6);
		v7 = vector_ceil_neon(v7);
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

BENCHMARK(bm_vector_ceil_neon);
#endif
