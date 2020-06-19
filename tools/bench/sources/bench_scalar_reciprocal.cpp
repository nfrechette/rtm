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

RTM_FORCE_NOINLINE float RTM_SIMD_CALL scalar_reciprocal_scalar(float input) RTM_NO_EXCEPT
{
	return 1.0f / input;
}

#if defined(RTM_SSE2_INTRINSICS)
RTM_FORCE_NOINLINE float RTM_SIMD_CALL scalar_reciprocal_sse2(float input) RTM_NO_EXCEPT
{
	__m128 input_s = _mm_set_ps1(input);

	// Perform two passes of Newton-Raphson iteration on the hardware estimate
	__m128 x0 = _mm_rcp_ss(input_s);

	// First iteration
	__m128 x1 = _mm_sub_ss(_mm_add_ss(x0, x0), _mm_mul_ss(input_s, _mm_mul_ss(x0, x0)));

	// Second iteration
	__m128 x2 = _mm_sub_ss(_mm_add_ss(x1, x1), _mm_mul_ss(input_s, _mm_mul_ss(x1, x1)));
	return _mm_cvtss_f32(x2);
}
#endif

#if defined(RTM_NEON64_INTRINSICS)
RTM_FORCE_NOINLINE float RTM_SIMD_CALL scalar_reciprocal_neon64(float input) RTM_NO_EXCEPT
{
	// Perform two passes of Newton-Raphson iteration on the hardware estimate
	float x0 = vrecpes_f32(input);

	// First iteration
	float x1 = x0 * vrecpss_f32(x0, input);

	// Second iteration
	float x2 = x1 * vrecpss_f32(x1, input);
	return x2;
}
#endif

#if defined(RTM_NEON_INTRINSICS)
RTM_FORCE_NOINLINE float RTM_SIMD_CALL scalar_reciprocal_neon(float input) RTM_NO_EXCEPT
{
	float32x2_t input_v = vdup_n_f32(input);

	// Perform two passes of Newton-Raphson iteration on the hardware estimate
	float32x2_t x0 = vrecpe_f32(input_v);

	// First iteration
	float32x2_t x1 = vmul_f32(x0, vrecps_f32(x0, input_v));

	// Second iteration
	float32x2_t x2 = vmul_f32(x1, vrecps_f32(x1, input_v));
	return vget_lane_f32(x2, 0);
}
#endif

static void bm_scalar_reciprocal_scalar(benchmark::State& state)
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
		f0 = scalar_reciprocal_scalar(f0);
		f1 = scalar_reciprocal_scalar(f1);
		f2 = scalar_reciprocal_scalar(f2);
		f3 = scalar_reciprocal_scalar(f3);
		f4 = scalar_reciprocal_scalar(f4);
		f5 = scalar_reciprocal_scalar(f5);
		f6 = scalar_reciprocal_scalar(f6);
		f7 = scalar_reciprocal_scalar(f7);
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

BENCHMARK(bm_scalar_reciprocal_scalar);

#if defined(RTM_SSE2_INTRINSICS)
static void bm_scalar_reciprocal_sse2(benchmark::State& state)
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
		f0 = scalar_reciprocal_sse2(f0);
		f1 = scalar_reciprocal_sse2(f1);
		f2 = scalar_reciprocal_sse2(f2);
		f3 = scalar_reciprocal_sse2(f3);
		f4 = scalar_reciprocal_sse2(f4);
		f5 = scalar_reciprocal_sse2(f5);
		f6 = scalar_reciprocal_sse2(f6);
		f7 = scalar_reciprocal_sse2(f7);
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

BENCHMARK(bm_scalar_reciprocal_sse2);
#endif

#if defined(RTM_NEON64_INTRINSICS)
static void bm_scalar_reciprocal_neon64(benchmark::State& state)
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
		f0 = scalar_reciprocal_neon64(f0);
		f1 = scalar_reciprocal_neon64(f1);
		f2 = scalar_reciprocal_neon64(f2);
		f3 = scalar_reciprocal_neon64(f3);
		f4 = scalar_reciprocal_neon64(f4);
		f5 = scalar_reciprocal_neon64(f5);
		f6 = scalar_reciprocal_neon64(f6);
		f7 = scalar_reciprocal_neon64(f7);
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

BENCHMARK(bm_scalar_reciprocal_neon64);
#endif

#if defined(RTM_NEON_INTRINSICS)
static void bm_scalar_reciprocal_neon(benchmark::State& state)
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
		f0 = scalar_reciprocal_neon(f0);
		f1 = scalar_reciprocal_neon(f1);
		f2 = scalar_reciprocal_neon(f2);
		f3 = scalar_reciprocal_neon(f3);
		f4 = scalar_reciprocal_neon(f4);
		f5 = scalar_reciprocal_neon(f5);
		f6 = scalar_reciprocal_neon(f6);
		f7 = scalar_reciprocal_neon(f7);
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

BENCHMARK(bm_scalar_reciprocal_neon);
#endif
