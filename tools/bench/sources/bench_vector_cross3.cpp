////////////////////////////////////////////////////////////////////////////////
// The MIT License (MIT)
//
// Copyright (c) 2024 Nicholas Frechette & Realtime Math contributors
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

RTM_FORCE_NOINLINE vector4f RTM_SIMD_CALL vector_cross3_scalar(vector4f_arg0 lhs, vector4f_arg1 rhs) RTM_NO_EXCEPT
{
	// cross(a, b) =
	//    (a.y * b.z) - (a.z * b.y)
	//    (a.z * b.x) - (a.x * b.z)
	//    (a.x * b.y) - (a.y * b.x)
	//
	// cross(a, b) = (a.yzx * b.zxy) - (a.zxy * b.yzx)

	// Compiles down to this with ARM64:
	// mov    s3, v0[1]
	// mov    s4, v1[1]
	// ext.16b v2, v0, v0, #0x8
	// ext.8b v5, v0, v2, #0x4
	// mov.s  v2[1], v0[0]
	// fneg.2s v2, v2
	// ext.16b v6, v1, v1, #0x8
	// ext.8b v7, v1, v6, #0x4
	// fmul.2s v2, v7, v2
	// mov.s  v6[1], v1[0]
	// fmla.2s v2, v6, v5
	// fneg   s3, s3
	// fmul.s s1, s3, v1[0]
	// fmla.s s1, s4, v0[0]
	// movi   d0, #0000000000000000
	// mov.s  v0[0], v1[0]
	// mov.d  v2[1], v0[0]
	// mov.16b v0, v2

	const float lhs_x = vector_get_x(lhs);
	const float lhs_y = vector_get_y(lhs);
	const float lhs_z = vector_get_z(lhs);
	const float rhs_x = vector_get_x(rhs);
	const float rhs_y = vector_get_y(rhs);
	const float rhs_z = vector_get_z(rhs);
	return vector_set((lhs_y * rhs_z) - (lhs_z * rhs_y), (lhs_z * rhs_x) - (lhs_x * rhs_z), (lhs_x * rhs_y) - (lhs_y * rhs_x));
}

#if defined(RTM_NEON_INTRINSICS)
RTM_FORCE_NOINLINE vector4f RTM_SIMD_CALL vector_cross3_neon(vector4f_arg0 lhs, vector4f_arg1 rhs) RTM_NO_EXCEPT
{
	// cross(a, b) =
	//    (a.y * b.z) - (a.z * b.y)
	//    (a.z * b.x) - (a.x * b.z)
	//    (a.x * b.y) - (a.y * b.x)
	//  or ..
	//    (a.yzx * b.zxy) - (a.zxy * b.yzx)

	// Compiles down to this with ARM64:
	// ext.16b v2, v0, v0, #0x4
	// mov.s  v2[2], v0[0]
	// ext.16b v3, v0, v1, #0xc
	// mov.s  v3[0], v1[2]
	// fmul.4s v2, v2, v3
	// ext.16b v3, v0, v0, #0xc
	// ext.16b v4, v1, v1, #0x4
	// mov.s  v3[0], v0[2]
	// mov.s  v4[2], v1[0]
	// fmls.4s v2, v4, v3
	// mov.16b v0, v2

	float32x4_t lhs_yzwx = vextq_f32(lhs, lhs, 1);
	float32x4_t rhs_wxyz = vextq_f32(rhs, rhs, 3);

	float32x4_t lhs_yzx = vsetq_lane_f32(vgetq_lane_f32(lhs, 0), lhs_yzwx, 2);
	float32x4_t rhs_zxy = vsetq_lane_f32(vgetq_lane_f32(rhs, 2), rhs_wxyz, 0);

	// part_a = (a.yzx * b.zxy)
	float32x4_t part_a = vmulq_f32(lhs_yzx, rhs_zxy);

	float32x4_t lhs_wxyz = vextq_f32(lhs, lhs, 3);
	float32x4_t rhs_yzwx = vextq_f32(rhs, rhs, 1);
	float32x4_t lhs_zxy = vsetq_lane_f32(vgetq_lane_f32(lhs, 2), lhs_wxyz, 0);
	float32x4_t rhs_yzx = vsetq_lane_f32(vgetq_lane_f32(rhs, 0), rhs_yzwx, 2);

	return vmlsq_f32(part_a, lhs_zxy, rhs_yzx);
}
#endif

#if defined(RTM_SSE2_INTRINSICS)
RTM_FORCE_NOINLINE vector4f RTM_SIMD_CALL vector_cross3_sse2(vector4f_arg0 lhs, vector4f_arg1 rhs) RTM_NO_EXCEPT
{
	// cross(a, b).zxy = (a * b.yzx) - (a.yzx * b)
	__m128 lhs_yzx = _mm_shuffle_ps(lhs, lhs, _MM_SHUFFLE(3, 0, 2, 1));
	__m128 rhs_yzx = _mm_shuffle_ps(rhs, rhs, _MM_SHUFFLE(3, 0, 2, 1));
	__m128 tmp_zxy = _mm_sub_ps(_mm_mul_ps(lhs, rhs_yzx), _mm_mul_ps(lhs_yzx, rhs));

	// cross(a, b) = ((a * b.yzx) - (a.yzx * b)).yzx
	return _mm_shuffle_ps(tmp_zxy, tmp_zxy, _MM_SHUFFLE(3, 0, 2, 1));
}
#endif

static void bm_vector_cross3_scalar(benchmark::State& state)
{
	vector4f v0 = vector_set(-1.0F, 1.0F, -2.0F, -123.134F);
	vector4f v1 = vector_set(-1.021F, 1.120F, -2.0331F, -1231.134F);
	vector4f v2 = vector_set(-11.0F, 1.330F, -21.50F, -1253.134F);
	vector4f v3 = vector_set(0.15F, 115.60F, 21.0221F, 123.13134F);

	for (auto _ : state)
	{
		v0 = vector_cross3_scalar(v0, v1);
		v1 = vector_cross3_scalar(v1, v2);
		v2 = vector_cross3_scalar(v2, v3);
		v3 = vector_cross3_scalar(v3, v0);
	}

	benchmark::DoNotOptimize(v0);
	benchmark::DoNotOptimize(v1);
	benchmark::DoNotOptimize(v2);
	benchmark::DoNotOptimize(v3);
}

BENCHMARK(bm_vector_cross3_scalar);

#if defined(RTM_NEON_INTRINSICS)
static void bm_vector_cross3_neon(benchmark::State& state)
{
	vector4f v0 = vector_set(-1.0F, 1.0F, -2.0F, -123.134F);
	vector4f v1 = vector_set(-1.021F, 1.120F, -2.0331F, -1231.134F);
	vector4f v2 = vector_set(-11.0F, 1.330F, -21.50F, -1253.134F);
	vector4f v3 = vector_set(0.15F, 115.60F, 21.0221F, 123.13134F);

	for (auto _ : state)
	{
		v0 = vector_cross3_neon(v0, v1);
		v1 = vector_cross3_neon(v1, v2);
		v2 = vector_cross3_neon(v2, v3);
		v3 = vector_cross3_neon(v3, v0);
	}

	benchmark::DoNotOptimize(v0);
	benchmark::DoNotOptimize(v1);
	benchmark::DoNotOptimize(v2);
	benchmark::DoNotOptimize(v3);
}

BENCHMARK(bm_vector_cross3_neon);
#endif

#if defined(RTM_SSE2_INTRINSICS)
static void bm_vector_cross3_sse2(benchmark::State& state)
{
	vector4f v0 = vector_set(-1.0F, 1.0F, -2.0F, -123.134F);
	vector4f v1 = vector_set(-1.021F, 1.120F, -2.0331F, -1231.134F);
	vector4f v2 = vector_set(-11.0F, 1.330F, -21.50F, -1253.134F);
	vector4f v3 = vector_set(0.15F, 115.60F, 21.0221F, 123.13134F);

	for (auto _ : state)
	{
		v0 = vector_cross3_sse2(v0, v1);
		v1 = vector_cross3_sse2(v1, v2);
		v2 = vector_cross3_sse2(v2, v3);
		v3 = vector_cross3_sse2(v3, v0);
	}

	benchmark::DoNotOptimize(v0);
	benchmark::DoNotOptimize(v1);
	benchmark::DoNotOptimize(v2);
	benchmark::DoNotOptimize(v3);
}

BENCHMARK(bm_vector_cross3_sse2);
#endif
