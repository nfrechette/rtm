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

RTM_FORCE_NOINLINE vector4f RTM_SIMD_CALL vector_dot_scalar_v(vector4f_arg0 lhs, vector4f_arg1 rhs) RTM_NO_EXCEPT
{
	float lhs_x = vector_get_x(lhs);
	float lhs_y = vector_get_y(lhs);
	float lhs_z = vector_get_z(lhs);
	float lhs_w = vector_get_w(lhs);

	float rhs_x = vector_get_x(rhs);
	float rhs_y = vector_get_y(rhs);
	float rhs_z = vector_get_z(rhs);
	float rhs_w = vector_get_w(rhs);

	return vector_set((lhs_x * rhs_x) + (lhs_y * rhs_y) + (lhs_z * rhs_z) + (lhs_w * rhs_w));
}

#if defined(RTM_NEON_INTRINSICS)
RTM_FORCE_NOINLINE vector4f RTM_SIMD_CALL vector_dot_neon_v(vector4f_arg0 lhs, vector4f_arg1 rhs) RTM_NO_EXCEPT
{
	// Compiles down to this with ARM64:
	// fmul.4s v0, v0, v1
	// ext.16b v1, v0, v0, #0x8
	// fadd.2s v0, v0, v1
	// faddp.2s v0, v0, v0
	// mov.d  v0[1], v0[0]
	float32x4_t x2_y2_z2_w2 = vmulq_f32(lhs, rhs);
	float32x2_t x2_y2 = vget_low_f32(x2_y2_z2_w2);
	float32x2_t z2_w2 = vget_high_f32(x2_y2_z2_w2);
	float32x2_t x2z2_y2w2 = vadd_f32(x2_y2, z2_w2);
	float32x2_t x2y2z2w2 = vpadd_f32(x2z2_y2w2, x2z2_y2w2);
	return vcombine_f32(x2y2z2w2, x2y2z2w2);
}

RTM_FORCE_NOINLINE vector4f RTM_SIMD_CALL vector_dot_neon_fma_v(vector4f_arg0 lhs, vector4f_arg1 rhs) RTM_NO_EXCEPT
{
	// Compiles down to this with ARM64:
	// ext.16b v2, v0, v0, #0x8
	// ext.16b v3, v1, v1, #0x8
	// fmul.2s v0, v0, v1
	// fmla.2s v0, v3, v2
	// faddp.2s v0, v0, v0
	// mov.d  v0[1], v0[0]
	float32x2_t lhs_x_y = vget_low_f32(lhs);
	float32x2_t lhs_z_w = vget_high_f32(lhs);
	float32x2_t rhs_x_y = vget_low_f32(rhs);
	float32x2_t rhs_z_w = vget_high_f32(rhs);
	float32x2_t x2_y2 = vmul_f32(lhs_x_y, rhs_x_y);
	float32x2_t x2z2_y2w2 = RTM_VECTOR2F_MULV_ADD(x2_y2, lhs_z_w, rhs_z_w);
	float32x2_t x2y2z2w2 = vpadd_f32(x2z2_y2w2, x2z2_y2w2);
	return vcombine_f32(x2y2z2w2, x2y2z2w2);
}
#endif

#if defined(RTM_NEON64_INTRINSICS)
RTM_FORCE_NOINLINE vector4f RTM_SIMD_CALL vector_dot_neon64_v(vector4f_arg0 lhs, vector4f_arg1 rhs) RTM_NO_EXCEPT
{
#if defined(RTM_IMPL_VADDVQ_SUPPORTED)
	// Compiles down to:
	// fmul.4s v0, v0, v1
	// faddp.4s v0, v0, v0
	// faddp.2s s0, v0
	// dup.4s v0, v0[0]
	float32x4_t x2_y2_z2_w2 = vmulq_f32(lhs, rhs);
	return vdupq_n_f32(vaddvq_f32(x2_y2_z2_w2));
#else
	(void)rhs;
	return lhs;
#endif
}

RTM_FORCE_NOINLINE vector4f RTM_SIMD_CALL vector_dot_neon64_paddq_v(vector4f_arg0 lhs, vector4f_arg1 rhs) RTM_NO_EXCEPT
{
	// Compiles down to:
	// fmul.4s v0, v0, v1
	// faddp.4s v0, v0, v0
	// faddp.4s v0, v0, v0
	float32x4_t x2_y2_z2_w2 = vmulq_f32(lhs, rhs);
	float32x4_t x2y2_z2w2_x2y2_z2w2 = vpaddq_f32(x2_y2_z2_w2, x2_y2_z2_w2);
	float32x4_t x2y2z2w2_x2y2z2w2_x2y2z2w2_x2y2z2w2 = vpaddq_f32(x2y2_z2w2_x2y2_z2w2, x2y2_z2w2_x2y2_z2w2);
	return x2y2z2w2_x2y2z2w2_x2y2z2w2_x2y2z2w2;
}
#endif

#if defined(RTM_SSE2_INTRINSICS)
RTM_FORCE_NOINLINE vector4f RTM_SIMD_CALL vector_dot_sse2_v(vector4f_arg0 lhs, vector4f_arg1 rhs) RTM_NO_EXCEPT
{
	__m128 x2_y2_z2_w2 = _mm_mul_ps(lhs, rhs);
	__m128 z2_w2_0_0 = _mm_shuffle_ps(x2_y2_z2_w2, x2_y2_z2_w2, _MM_SHUFFLE(0, 0, 3, 2));
	__m128 x2z2_y2w2_0_0 = _mm_add_ps(x2_y2_z2_w2, z2_w2_0_0);
	__m128 y2w2_0_0_0 = _mm_shuffle_ps(x2z2_y2w2_0_0, x2z2_y2w2_0_0, _MM_SHUFFLE(0, 0, 0, 1));
	__m128 x2y2z2w2_0_0_0 = _mm_add_ps(x2z2_y2w2_0_0, y2w2_0_0_0);
	return _mm_shuffle_ps(x2y2z2w2_0_0_0, x2y2z2w2_0_0_0, _MM_SHUFFLE(0, 0, 0, 0));
}
#endif

#if defined(RTM_SSE4_INTRINSICS)
// SSE4 dot product instruction appears slower on Zen2
// On the hardware Github Actions run on, performance is identical with clang14
// Need to measure on Zen3 and Intel, if same speed as well, we should use dpps instruction
// since it leads to smaller assembly and improved inlining
RTM_FORCE_NOINLINE vector4f RTM_SIMD_CALL vector_dot_sse4_v(vector4f_arg0 lhs, vector4f_arg1 rhs) RTM_NO_EXCEPT
{
	return _mm_dp_ps(lhs, rhs, 0xFF);
}
#endif

static void bm_vector_dot_scalar_v(benchmark::State& state)
{
	vector4f v0 = vector_set(-1.0F, 1.0F, -2.0F, -123.134F);
	vector4f v1 = vector_set(-1.021F, 1.120F, -2.0331F, -1231.134F);
	vector4f v2 = vector_set(-11.0F, 1.330F, -21.50F, -1253.134F);
	vector4f v3 = vector_set(0.15F, 115.60F, 21.0221F, 123.13134F);

	vector4f sum0 = vector_zero();
	vector4f sum1 = vector_zero();
	vector4f sum2 = vector_zero();
	vector4f sum3 = vector_zero();

	for (auto _ : state)
	{
		sum0 = vector_add(sum0, vector_dot_scalar_v(v0, v1));
		sum1 = vector_add(sum1, vector_dot_scalar_v(v1, v2));
		sum2 = vector_add(sum2, vector_dot_scalar_v(v2, v3));
		sum3 = vector_add(sum3, vector_dot_scalar_v(v3, v0));
	}

	benchmark::DoNotOptimize(sum0);
	benchmark::DoNotOptimize(sum1);
	benchmark::DoNotOptimize(sum2);
	benchmark::DoNotOptimize(sum3);
}

BENCHMARK(bm_vector_dot_scalar_v);

#if defined(RTM_NEON_INTRINSICS)
static void bm_vector_dot_neon_v(benchmark::State& state)
{
	vector4f v0 = vector_set(-1.0F, 1.0F, -2.0F, -123.134F);
	vector4f v1 = vector_set(-1.021F, 1.120F, -2.0331F, -1231.134F);
	vector4f v2 = vector_set(-11.0F, 1.330F, -21.50F, -1253.134F);
	vector4f v3 = vector_set(0.15F, 115.60F, 21.0221F, 123.13134F);

	vector4f sum0 = vector_zero();
	vector4f sum1 = vector_zero();
	vector4f sum2 = vector_zero();
	vector4f sum3 = vector_zero();

	for (auto _ : state)
	{
		sum0 = vector_add(sum0, vector_dot_neon_v(v0, v1));
		sum1 = vector_add(sum1, vector_dot_neon_v(v1, v2));
		sum2 = vector_add(sum2, vector_dot_neon_v(v2, v3));
		sum3 = vector_add(sum3, vector_dot_neon_v(v3, v0));
	}

	benchmark::DoNotOptimize(sum0);
	benchmark::DoNotOptimize(sum1);
	benchmark::DoNotOptimize(sum2);
	benchmark::DoNotOptimize(sum3);
}

BENCHMARK(bm_vector_dot_neon_v);

static void bm_vector_dot_neon_fma_v(benchmark::State& state)
{
	vector4f v0 = vector_set(-1.0F, 1.0F, -2.0F, -123.134F);
	vector4f v1 = vector_set(-1.021F, 1.120F, -2.0331F, -1231.134F);
	vector4f v2 = vector_set(-11.0F, 1.330F, -21.50F, -1253.134F);
	vector4f v3 = vector_set(0.15F, 115.60F, 21.0221F, 123.13134F);

	vector4f sum0 = vector_zero();
	vector4f sum1 = vector_zero();
	vector4f sum2 = vector_zero();
	vector4f sum3 = vector_zero();

	for (auto _ : state)
	{
		sum0 = vector_add(sum0, vector_dot_neon_fma_v(v0, v1));
		sum1 = vector_add(sum1, vector_dot_neon_fma_v(v1, v2));
		sum2 = vector_add(sum2, vector_dot_neon_fma_v(v2, v3));
		sum3 = vector_add(sum3, vector_dot_neon_fma_v(v3, v0));
	}

	benchmark::DoNotOptimize(sum0);
	benchmark::DoNotOptimize(sum1);
	benchmark::DoNotOptimize(sum2);
	benchmark::DoNotOptimize(sum3);
}

BENCHMARK(bm_vector_dot_neon_fma_v);
#endif

#if defined(RTM_NEON64_INTRINSICS)
static void bm_vector_dot_neon64_v(benchmark::State& state)
{
	vector4f v0 = vector_set(-1.0F, 1.0F, -2.0F, -123.134F);
	vector4f v1 = vector_set(-1.021F, 1.120F, -2.0331F, -1231.134F);
	vector4f v2 = vector_set(-11.0F, 1.330F, -21.50F, -1253.134F);
	vector4f v3 = vector_set(0.15F, 115.60F, 21.0221F, 123.13134F);

	vector4f sum0 = vector_zero();
	vector4f sum1 = vector_zero();
	vector4f sum2 = vector_zero();
	vector4f sum3 = vector_zero();

	for (auto _ : state)
	{
		sum0 = vector_add(sum0, vector_dot_neon64_v(v0, v1));
		sum1 = vector_add(sum1, vector_dot_neon64_v(v1, v2));
		sum2 = vector_add(sum2, vector_dot_neon64_v(v2, v3));
		sum3 = vector_add(sum3, vector_dot_neon64_v(v3, v0));
	}

	benchmark::DoNotOptimize(sum0);
	benchmark::DoNotOptimize(sum1);
	benchmark::DoNotOptimize(sum2);
	benchmark::DoNotOptimize(sum3);
}

BENCHMARK(bm_vector_dot_neon64_v);

static void bm_vector_dot_neon64_paddq_v(benchmark::State& state)
{
	vector4f v0 = vector_set(-1.0F, 1.0F, -2.0F, -123.134F);
	vector4f v1 = vector_set(-1.021F, 1.120F, -2.0331F, -1231.134F);
	vector4f v2 = vector_set(-11.0F, 1.330F, -21.50F, -1253.134F);
	vector4f v3 = vector_set(0.15F, 115.60F, 21.0221F, 123.13134F);

	vector4f sum0 = vector_zero();
	vector4f sum1 = vector_zero();
	vector4f sum2 = vector_zero();
	vector4f sum3 = vector_zero();

	for (auto _ : state)
	{
		sum0 = vector_add(sum0, vector_dot_neon64_paddq_v(v0, v1));
		sum1 = vector_add(sum1, vector_dot_neon64_paddq_v(v1, v2));
		sum2 = vector_add(sum2, vector_dot_neon64_paddq_v(v2, v3));
		sum3 = vector_add(sum3, vector_dot_neon64_paddq_v(v3, v0));
	}

	benchmark::DoNotOptimize(sum0);
	benchmark::DoNotOptimize(sum1);
	benchmark::DoNotOptimize(sum2);
	benchmark::DoNotOptimize(sum3);
}

BENCHMARK(bm_vector_dot_neon64_paddq_v);
#endif

#if defined(RTM_SSE2_INTRINSICS)
static void bm_vector_dot_sse2_v(benchmark::State& state)
{
	vector4f v0 = vector_set(-1.0F, 1.0F, -2.0F, -123.134F);
	vector4f v1 = vector_set(-1.021F, 1.120F, -2.0331F, -1231.134F);
	vector4f v2 = vector_set(-11.0F, 1.330F, -21.50F, -1253.134F);
	vector4f v3 = vector_set(0.15F, 115.60F, 21.0221F, 123.13134F);

	vector4f sum0 = vector_zero();
	vector4f sum1 = vector_zero();
	vector4f sum2 = vector_zero();
	vector4f sum3 = vector_zero();

	for (auto _ : state)
	{
		sum0 = vector_add(sum0, vector_dot_sse2_v(v0, v1));
		sum1 = vector_add(sum1, vector_dot_sse2_v(v1, v2));
		sum2 = vector_add(sum2, vector_dot_sse2_v(v2, v3));
		sum3 = vector_add(sum3, vector_dot_sse2_v(v3, v0));
	}

	benchmark::DoNotOptimize(sum0);
	benchmark::DoNotOptimize(sum1);
	benchmark::DoNotOptimize(sum2);
	benchmark::DoNotOptimize(sum3);
}

BENCHMARK(bm_vector_dot_sse2_v);
#endif

#if defined(RTM_SSE4_INTRINSICS)
static void bm_vector_dot_sse4_v(benchmark::State& state)
{
	vector4f v0 = vector_set(-1.0F, 1.0F, -2.0F, -123.134F);
	vector4f v1 = vector_set(-1.021F, 1.120F, -2.0331F, -1231.134F);
	vector4f v2 = vector_set(-11.0F, 1.330F, -21.50F, -1253.134F);
	vector4f v3 = vector_set(0.15F, 115.60F, 21.0221F, 123.13134F);

	vector4f sum0 = vector_zero();
	vector4f sum1 = vector_zero();
	vector4f sum2 = vector_zero();
	vector4f sum3 = vector_zero();

	for (auto _ : state)
	{
		sum0 = vector_add(sum0, vector_dot_sse4_v(v0, v1));
		sum1 = vector_add(sum1, vector_dot_sse4_v(v1, v2));
		sum2 = vector_add(sum2, vector_dot_sse4_v(v2, v3));
		sum3 = vector_add(sum3, vector_dot_sse4_v(v3, v0));
	}

	benchmark::DoNotOptimize(sum0);
	benchmark::DoNotOptimize(sum1);
	benchmark::DoNotOptimize(sum2);
	benchmark::DoNotOptimize(sum3);
}

BENCHMARK(bm_vector_dot_sse4_v);
#endif
