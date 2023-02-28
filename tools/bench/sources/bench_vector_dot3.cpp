////////////////////////////////////////////////////////////////////////////////
// The MIT License (MIT)
//
// Copyright (c) 2023 Nicholas Frechette & Realtime Math contributors
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

RTM_FORCE_NOINLINE float RTM_SIMD_CALL vector_dot3_scalar(vector4f_arg0 lhs, vector4f_arg1 rhs) RTM_NO_EXCEPT
{
	// Compiles down to this with ARM64:
	// fmul.4s v0, v0, v1
	// dup.4s v1, v0[1]
	// fadd.4s v1, v0, v1
	// dup.4s v0, v0[2]
	// fadd.4s v0, v0, v1
	float lhs_x = vector_get_x(lhs);
	float lhs_y = vector_get_y(lhs);
	float lhs_z = vector_get_z(lhs);

	float rhs_x = vector_get_x(rhs);
	float rhs_y = vector_get_y(rhs);
	float rhs_z = vector_get_z(rhs);

	return (lhs_x * rhs_x) + (lhs_y * rhs_y) + (lhs_z * rhs_z);
}

#if defined(RTM_NEON_INTRINSICS)
RTM_FORCE_NOINLINE float RTM_SIMD_CALL vector_dot3_neon(vector4f_arg0 lhs, vector4f_arg1 rhs) RTM_NO_EXCEPT
{
	// Compiles down to this with ARM64:
	// fmul.4s v0, v0, v1
	// faddp.2s v1, v0, v0
	// ext.16b v0, v0, v0, #0x8
	// fadd.2s v0, v0, v1
	float32x4_t x2_y2_z2_w2 = vmulq_f32(lhs, rhs);
	float32x2_t x2_y2 = vget_low_f32(x2_y2_z2_w2);
	float32x2_t z2_w2 = vget_high_f32(x2_y2_z2_w2);
	float32x2_t x2y2_x2y2 = vpadd_f32(x2_y2, x2_y2);
	float32x2_t z2_z2 = vdup_lane_f32(z2_w2, 0);
	float32x2_t x2y2z2_x2y2z2 = vadd_f32(x2y2_x2y2, z2_z2);
	return vget_lane_f32(x2y2z2_x2y2z2, 0);
}
#endif

#if defined(RTM_NEON64_INTRINSICS)
RTM_FORCE_NOINLINE float RTM_SIMD_CALL vector_dot3_neon64(vector4f_arg0 lhs, vector4f_arg1 rhs) RTM_NO_EXCEPT
{
#if defined(RTM_IMPL_VADDVQ_SUPPORTED)
	// Compiles down to:
	// fmul.4s v0, v0, v1
	// mov.s  v0[3], wzr
	// faddp.4s v0, v0, v0
	// faddp.2s s0, v0
	float32x4_t x2_y2_z2_w2 = vmulq_f32(lhs, rhs);
	float32x4_t x2_y2_z2 = vsetq_lane_f32(0.0F, x2_y2_z2_w2, 3);
	return vaddvq_f32(x2_y2_z2);
#else
	(void)lhs;
	(void)rhs;
	return 0.0F;
#endif
}
#endif

#if defined(RTM_SSE2_INTRINSICS)
RTM_FORCE_NOINLINE float RTM_SIMD_CALL vector_dot3_sse2(vector4f_arg0 lhs, vector4f_arg1 rhs) RTM_NO_EXCEPT
{
	__m128 x2_y2_z2_w2 = _mm_mul_ps(lhs, rhs);
	__m128 y2_0_0_0 = _mm_shuffle_ps(x2_y2_z2_w2, x2_y2_z2_w2, _MM_SHUFFLE(0, 0, 0, 1));
	__m128 x2y2_0_0_0 = _mm_add_ss(x2_y2_z2_w2, y2_0_0_0);
	__m128 z2_0_0_0 = _mm_shuffle_ps(x2_y2_z2_w2, x2_y2_z2_w2, _MM_SHUFFLE(0, 0, 0, 2));
	__m128 x2y2z2_0_0_0 = _mm_add_ss(x2y2_0_0_0, z2_0_0_0);
	return _mm_cvtss_f32(x2y2z2_0_0_0);
}
#endif

#if defined(RTM_SSE4_INTRINSICS)
RTM_FORCE_NOINLINE float RTM_SIMD_CALL vector_dot3_sse4(vector4f_arg0 lhs, vector4f_arg1 rhs) RTM_NO_EXCEPT
{
	return _mm_cvtss_f32(_mm_dp_ps(lhs, rhs, 0x7F));
}
#endif

static void bm_vector_dot3_scalar(benchmark::State& state)
{
	vector4f v0 = vector_set(-1.0F, 1.0F, -2.0F, -123.134F);
	vector4f v1 = vector_set(-1.021F, 1.120F, -2.0331F, -1231.134F);
	vector4f v2 = vector_set(-11.0F, 1.330F, -21.50F, -1253.134F);
	vector4f v3 = vector_set(0.15F, 115.60F, 21.0221F, 123.13134F);

	float sum0 = 0.0F;
	float sum1 = 0.0F;
	float sum2 = 0.0F;
	float sum3 = 0.0F;

	for (auto _ : state)
	{
		sum0 += vector_dot3_scalar(v0, v1);
		sum1 += vector_dot3_scalar(v1, v2);
		sum2 += vector_dot3_scalar(v2, v3);
		sum3 += vector_dot3_scalar(v3, v0);
	}

	benchmark::DoNotOptimize(sum0);
	benchmark::DoNotOptimize(sum1);
	benchmark::DoNotOptimize(sum2);
	benchmark::DoNotOptimize(sum3);
}

BENCHMARK(bm_vector_dot3_scalar);

#if defined(RTM_NEON_INTRINSICS)
static void bm_vector_dot3_neon(benchmark::State& state)
{
	vector4f v0 = vector_set(-1.0F, 1.0F, -2.0F, -123.134F);
	vector4f v1 = vector_set(-1.021F, 1.120F, -2.0331F, -1231.134F);
	vector4f v2 = vector_set(-11.0F, 1.330F, -21.50F, -1253.134F);
	vector4f v3 = vector_set(0.15F, 115.60F, 21.0221F, 123.13134F);

	float sum0 = 0.0F;
	float sum1 = 0.0F;
	float sum2 = 0.0F;
	float sum3 = 0.0F;

	for (auto _ : state)
	{
		sum0 += vector_dot3_neon(v0, v1);
		sum1 += vector_dot3_neon(v1, v2);
		sum2 += vector_dot3_neon(v2, v3);
		sum3 += vector_dot3_neon(v3, v0);
	}

	benchmark::DoNotOptimize(sum0);
	benchmark::DoNotOptimize(sum1);
	benchmark::DoNotOptimize(sum2);
	benchmark::DoNotOptimize(sum3);
}

BENCHMARK(bm_vector_dot3_neon);
#endif

#if defined(RTM_NEON64_INTRINSICS)
static void bm_vector_dot3_neon64(benchmark::State& state)
{
	vector4f v0 = vector_set(-1.0F, 1.0F, -2.0F, -123.134F);
	vector4f v1 = vector_set(-1.021F, 1.120F, -2.0331F, -1231.134F);
	vector4f v2 = vector_set(-11.0F, 1.330F, -21.50F, -1253.134F);
	vector4f v3 = vector_set(0.15F, 115.60F, 21.0221F, 123.13134F);

	float sum0 = 0.0F;
	float sum1 = 0.0F;
	float sum2 = 0.0F;
	float sum3 = 0.0F;

	for (auto _ : state)
	{
		sum0 += vector_dot3_neon64(v0, v1);
		sum1 += vector_dot3_neon64(v1, v2);
		sum2 += vector_dot3_neon64(v2, v3);
		sum3 += vector_dot3_neon64(v3, v0);
	}

	benchmark::DoNotOptimize(sum0);
	benchmark::DoNotOptimize(sum1);
	benchmark::DoNotOptimize(sum2);
	benchmark::DoNotOptimize(sum3);
}

BENCHMARK(bm_vector_dot3_neon64);
#endif

#if defined(RTM_SSE2_INTRINSICS)
static void bm_vector_dot3_sse2(benchmark::State& state)
{
	vector4f v0 = vector_set(-1.0F, 1.0F, -2.0F, -123.134F);
	vector4f v1 = vector_set(-1.021F, 1.120F, -2.0331F, -1231.134F);
	vector4f v2 = vector_set(-11.0F, 1.330F, -21.50F, -1253.134F);
	vector4f v3 = vector_set(0.15F, 115.60F, 21.0221F, 123.13134F);

	float sum0 = 0.0F;
	float sum1 = 0.0F;
	float sum2 = 0.0F;
	float sum3 = 0.0F;

	for (auto _ : state)
	{
		sum0 += vector_dot3_sse2(v0, v1);
		sum1 += vector_dot3_sse2(v1, v2);
		sum2 += vector_dot3_sse2(v2, v3);
		sum3 += vector_dot3_sse2(v3, v0);
	}

	benchmark::DoNotOptimize(sum0);
	benchmark::DoNotOptimize(sum1);
	benchmark::DoNotOptimize(sum2);
	benchmark::DoNotOptimize(sum3);
}

BENCHMARK(bm_vector_dot3_sse2);
#endif

#if defined(RTM_SSE4_INTRINSICS)
static void bm_vector_dot3_sse4(benchmark::State& state)
{
	vector4f v0 = vector_set(-1.0F, 1.0F, -2.0F, -123.134F);
	vector4f v1 = vector_set(-1.021F, 1.120F, -2.0331F, -1231.134F);
	vector4f v2 = vector_set(-11.0F, 1.330F, -21.50F, -1253.134F);
	vector4f v3 = vector_set(0.15F, 115.60F, 21.0221F, 123.13134F);

	float sum0 = 0.0F;
	float sum1 = 0.0F;
	float sum2 = 0.0F;
	float sum3 = 0.0F;

	for (auto _ : state)
	{
		sum0 += vector_dot3_sse4(v0, v1);
		sum1 += vector_dot3_sse4(v1, v2);
		sum2 += vector_dot3_sse4(v2, v3);
		sum3 += vector_dot3_sse4(v3, v0);
	}

	benchmark::DoNotOptimize(sum0);
	benchmark::DoNotOptimize(sum1);
	benchmark::DoNotOptimize(sum2);
	benchmark::DoNotOptimize(sum3);
}

BENCHMARK(bm_vector_dot3_sse4);
#endif
