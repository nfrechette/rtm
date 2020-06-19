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

RTM_FORCE_NOINLINE vector4f RTM_SIMD_CALL vector_atan2_scalar(vector4f_arg0 y, vector4f_arg1 x) RTM_NO_EXCEPT
{
	scalarf x_ = scalar_atan2(scalarf(vector_get_x(y)), scalarf(vector_get_x(x)));
	scalarf y_ = scalar_atan2(scalarf(vector_get_y(y)), scalarf(vector_get_y(x)));
	scalarf z = scalar_atan2(scalarf(vector_get_z(y)), scalarf(vector_get_z(x)));
	scalarf w = scalar_atan2(scalarf(vector_get_w(y)), scalarf(vector_get_w(x)));
	return vector_set(x_, y_, z, w);
}

#if defined(RTM_SSE2_INTRINSICS)
RTM_FORCE_NOINLINE vector4f RTM_SIMD_CALL vector_atan2_sse2(vector4f_arg0 y, vector4f_arg1 x) RTM_NO_EXCEPT
{
	// If X == 0.0 and Y != 0.0, we return PI/2 with the sign of Y
	// If X == 0.0 and Y == 0.0, we return 0.0
	// If X > 0.0, we return atan(y/x)
	// If X < 0.0, we return atan(y/x) + sign(Y) * PI
	// See: https://en.wikipedia.org/wiki/Atan2#Definition_and_computation

	const __m128 zero = _mm_setzero_ps();
	__m128 is_x_zero = _mm_cmpeq_ps(x, zero);
	__m128 is_y_zero = _mm_cmpeq_ps(y, zero);
	__m128 inputs_are_zero = _mm_and_ps(is_x_zero, is_y_zero);

	__m128 is_x_positive = _mm_cmpgt_ps(x, zero);

	const __m128 sign_mask = _mm_set_ps(-0.0F, -0.0F, -0.0F, -0.0F);
	__m128 y_sign = _mm_and_ps(y, sign_mask);

	// If X == 0.0, our offset is PI/2 otherwise it is PI both with the sign of Y
	__m128 half_pi = _mm_set_ps1(rtm::constants::half_pi());
	__m128 pi = _mm_set_ps1(rtm::constants::pi());
	__m128 offset = _mm_or_ps(_mm_and_ps(is_x_zero, half_pi), _mm_andnot_ps(is_x_zero, pi));
	offset = _mm_or_ps(offset, y_sign);

	// If X > 0.0, our offset is 0.0
	offset = _mm_andnot_ps(is_x_positive, offset);

	// If X == 0.0 and Y == 0.0, our offset is 0.0
	offset = _mm_andnot_ps(inputs_are_zero, offset);

	__m128 angle = _mm_div_ps(y, x);
	__m128 value = vector_atan(angle);

	// If X == 0.0, our value is 0.0 otherwise it is atan(y/x)
	value = _mm_andnot_ps(is_x_zero, value);

	// If X == 0.0 and Y == 0.0, our value is 0.0
	value = _mm_andnot_ps(inputs_are_zero, value);

	return _mm_add_ps(value, offset);
}
#endif

#if defined(RTM_NEON_INTRINSICS)
RTM_FORCE_NOINLINE vector4f RTM_SIMD_CALL vector_atan2_neon(vector4f_arg0 y, vector4f_arg1 x) RTM_NO_EXCEPT
{
	// If X == 0.0 and Y != 0.0, we return PI/2 with the sign of Y
	// If X == 0.0 and Y == 0.0, we return 0.0
	// If X > 0.0, we return atan(y/x)
	// If X < 0.0, we return atan(y/x) + sign(Y) * PI
	// See: https://en.wikipedia.org/wiki/Atan2#Definition_and_computation

#if defined(RTM_NEON64_INTRINSICS)
	uint32x4_t is_x_zero = vceqzq_f32(x);
	uint32x4_t is_y_zero = vceqzq_f32(y);
	uint32x4_t inputs_are_zero = vandq_u32(is_x_zero, is_y_zero);

	uint32x4_t is_x_positive = vcgtzq_f32(x);
#else
	float32x4_t zero = vdupq_n_f32(0.0F);
	uint32x4_t is_x_zero = vceqq_f32(x, zero);
	uint32x4_t is_y_zero = vceqq_f32(y, zero);
	uint32x4_t inputs_are_zero = vandq_u32(is_x_zero, is_y_zero);

	uint32x4_t is_x_positive = vcgtq_f32(x, zero);
#endif

	uint32x4_t y_sign = vandq_u32(vreinterpretq_u32_f32(y), vreinterpretq_u32_f32(vdupq_n_f32(-0.0F)));

	// If X == 0.0, our offset is PI/2 otherwise it is PI both with the sign of Y
	float32x4_t half_pi = vdupq_n_f32(rtm::constants::half_pi());
	float32x4_t pi = vdupq_n_f32(rtm::constants::pi());
	float32x4_t offset = vreinterpretq_f32_u32(vorrq_u32(vandq_u32(is_x_zero, vreinterpretq_u32_f32(half_pi)), vandq_u32(vmvnq_u32(is_x_zero), vreinterpretq_u32_f32(pi))));
	offset = vreinterpretq_f32_u32(vorrq_u32(vreinterpretq_u32_f32(offset), y_sign));

	// If X > 0.0, our offset is 0.0
	offset = vreinterpretq_f32_u32(vandq_u32(vmvnq_u32(is_x_positive), vreinterpretq_u32_f32(offset)));

	// If X == 0.0 and Y == 0.0, our offset is 0.0
	offset = vreinterpretq_f32_u32(vandq_u32(vmvnq_u32(inputs_are_zero), vreinterpretq_u32_f32(offset)));

	float32x4_t angle = vector_div(y, x);
	float32x4_t value = vector_atan(angle);

	// If X == 0.0, our value is 0.0 otherwise it is atan(y/x)
	value = vreinterpretq_f32_u32(vandq_u32(vmvnq_u32(is_x_zero), vreinterpretq_u32_f32(value)));

	// If X == 0.0 and Y == 0.0, our value is 0.0
	value = vreinterpretq_f32_u32(vandq_u32(vmvnq_u32(inputs_are_zero), vreinterpretq_u32_f32(value)));

	return vaddq_f32(value, offset);
}
#endif

static void bm_vector_atan2_scalar(benchmark::State& state)
{
	vector4f v0 = vector_set(-0.134f);
	vector4f v1 = vector_set(0.134f);
	vector4f v2 = vector_set(-0.134f);
	vector4f v3 = vector_set(0.134f);
	vector4f v4 = vector_set(-0.134f);
	vector4f v5 = vector_set(0.134f);
	vector4f v6 = vector_set(-0.134f);
	vector4f v7 = vector_set(0.134f);

	for (auto _ : state)
	{
		v0 = vector_atan2_scalar(v0, v1);
		v1 = vector_atan2_scalar(v1, v2);
		v2 = vector_atan2_scalar(v2, v3);
		v3 = vector_atan2_scalar(v3, v4);
		v4 = vector_atan2_scalar(v4, v5);
		v5 = vector_atan2_scalar(v5, v6);
		v6 = vector_atan2_scalar(v6, v7);
		v7 = vector_atan2_scalar(v7, v0);
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

BENCHMARK(bm_vector_atan2_scalar);

#if defined(RTM_SSE2_INTRINSICS)
static void bm_vector_atan2_sse2(benchmark::State& state)
{
	vector4f v0 = vector_set(-0.134f);
	vector4f v1 = vector_set(0.134f);
	vector4f v2 = vector_set(-0.134f);
	vector4f v3 = vector_set(0.134f);
	vector4f v4 = vector_set(-0.134f);
	vector4f v5 = vector_set(0.134f);
	vector4f v6 = vector_set(-0.134f);
	vector4f v7 = vector_set(0.134f);

	for (auto _ : state)
	{
		v0 = vector_atan2_sse2(v0, v1);
		v1 = vector_atan2_sse2(v1, v2);
		v2 = vector_atan2_sse2(v2, v3);
		v3 = vector_atan2_sse2(v3, v4);
		v4 = vector_atan2_sse2(v4, v5);
		v5 = vector_atan2_sse2(v5, v6);
		v6 = vector_atan2_sse2(v6, v7);
		v7 = vector_atan2_sse2(v7, v0);
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

BENCHMARK(bm_vector_atan2_sse2);
#endif

#if defined(RTM_NEON_INTRINSICS)
static void bm_vector_atan2_neon(benchmark::State& state)
{
	vector4f v0 = vector_set(-0.134f);
	vector4f v1 = vector_set(0.134f);
	vector4f v2 = vector_set(-0.134f);
	vector4f v3 = vector_set(0.134f);
	vector4f v4 = vector_set(-0.134f);
	vector4f v5 = vector_set(0.134f);
	vector4f v6 = vector_set(-0.134f);
	vector4f v7 = vector_set(0.134f);

	for (auto _ : state)
	{
		v0 = vector_atan2_neon(v0, v1);
		v1 = vector_atan2_neon(v1, v2);
		v2 = vector_atan2_neon(v2, v3);
		v3 = vector_atan2_neon(v3, v4);
		v4 = vector_atan2_neon(v4, v5);
		v5 = vector_atan2_neon(v5, v6);
		v6 = vector_atan2_neon(v6, v7);
		v7 = vector_atan2_neon(v7, v0);
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

BENCHMARK(bm_vector_atan2_neon);
#endif
