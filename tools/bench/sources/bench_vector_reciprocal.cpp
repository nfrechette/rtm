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

RTM_FORCE_NOINLINE vector4f RTM_SIMD_CALL vector_reciprocal_scalar(vector4f_arg0 input) RTM_NO_EXCEPT
{
	return vector_div(vector_set(1.0F), input);
}

#if defined(RTM_SSE2_INTRINSICS)
RTM_FORCE_NOINLINE vector4f RTM_SIMD_CALL vector_reciprocal_sse2(vector4f_arg0 input) RTM_NO_EXCEPT
{
	// Perform two passes of Newton-Raphson iteration on the hardware estimate
	__m128 x0 = _mm_rcp_ps(input);

	// First iteration
	__m128 x1 = _mm_sub_ps(_mm_add_ps(x0, x0), _mm_mul_ps(input, _mm_mul_ps(x0, x0)));

	// Second iteration
	__m128 x2 = _mm_sub_ps(_mm_add_ps(x1, x1), _mm_mul_ps(input, _mm_mul_ps(x1, x1)));
	return x2;
}
#endif

#if defined(RTM_NEON_INTRINSICS)
RTM_FORCE_NOINLINE vector4f RTM_SIMD_CALL vector_reciprocal_neon(vector4f_arg0 input) RTM_NO_EXCEPT
{
	// Perform two passes of Newton-Raphson iteration on the hardware estimate
	float32x4_t x0 = vrecpeq_f32(input);

	// First iteration
	float32x4_t x1 = vmulq_f32(x0, vrecpsq_f32(x0, input));

	// Second iteration
	float32x4_t x2 = vmulq_f32(x1, vrecpsq_f32(x1, input));
	return x2;
}
#endif

static void bm_vector_reciprocal_scalar(benchmark::State& state)
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
		v0 = vector_reciprocal_scalar(v0);
		v1 = vector_reciprocal_scalar(v1);
		v2 = vector_reciprocal_scalar(v2);
		v3 = vector_reciprocal_scalar(v3);
		v4 = vector_reciprocal_scalar(v4);
		v5 = vector_reciprocal_scalar(v5);
		v6 = vector_reciprocal_scalar(v6);
		v7 = vector_reciprocal_scalar(v7);
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

BENCHMARK(bm_vector_reciprocal_scalar);

#if defined(RTM_SSE2_INTRINSICS)
static void bm_vector_reciprocal_sse2(benchmark::State& state)
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
		v0 = vector_reciprocal_sse2(v0);
		v1 = vector_reciprocal_sse2(v1);
		v2 = vector_reciprocal_sse2(v2);
		v3 = vector_reciprocal_sse2(v3);
		v4 = vector_reciprocal_sse2(v4);
		v5 = vector_reciprocal_sse2(v5);
		v6 = vector_reciprocal_sse2(v6);
		v7 = vector_reciprocal_sse2(v7);
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

BENCHMARK(bm_vector_reciprocal_sse2);
#endif

#if defined(RTM_NEON_INTRINSICS)
static void bm_vector_reciprocal_neon(benchmark::State& state)
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
		v0 = vector_reciprocal_neon(v0);
		v1 = vector_reciprocal_neon(v1);
		v2 = vector_reciprocal_neon(v2);
		v3 = vector_reciprocal_neon(v3);
		v4 = vector_reciprocal_neon(v4);
		v5 = vector_reciprocal_neon(v5);
		v6 = vector_reciprocal_neon(v6);
		v7 = vector_reciprocal_neon(v7);
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

BENCHMARK(bm_vector_reciprocal_neon);
#endif
