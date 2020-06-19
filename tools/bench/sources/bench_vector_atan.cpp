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

RTM_FORCE_NOINLINE vector4f RTM_SIMD_CALL vector_atan_scalar(vector4f_arg0 input) RTM_NO_EXCEPT
{
	scalarf x = scalar_atan(scalarf(vector_get_x(input)));
	scalarf y = scalar_atan(scalarf(vector_get_y(input)));
	scalarf z = scalar_atan(scalarf(vector_get_z(input)));
	scalarf w = scalar_atan(scalarf(vector_get_w(input)));
	return vector_set(x, y, z, w);
}

#if defined(RTM_SSE2_INTRINSICS)
RTM_FORCE_NOINLINE vector4f RTM_SIMD_CALL vector_atan_sse2(vector4f_arg0 input) RTM_NO_EXCEPT
{
	// Use a degree 13 minimax approximation polynomial
	// See: GPGPU Programming for Games and Science (David H. Eberly)

	// Discard our sign, we'll restore it later
	const __m128i abs_mask = _mm_set_epi32(0x7FFFFFFFULL, 0x7FFFFFFFULL, 0x7FFFFFFFULL, 0x7FFFFFFFULL);
	__m128 abs_value = _mm_and_ps(input, _mm_castsi128_ps(abs_mask));

	// Compute our value
	__m128 is_larger_than_one = _mm_cmpgt_ps(abs_value, _mm_set_ps1(1.0F));
	__m128 reciprocal = vector_reciprocal(abs_value);

	__m128 x = vector_select(is_larger_than_one, reciprocal, abs_value);

	__m128 x2 = _mm_mul_ps(x, x);

	__m128 result = _mm_add_ps(_mm_mul_ps(x2, _mm_set_ps1(7.2128853633444123e-3F)), _mm_set_ps1(-3.5059680836411644e-2F));
	result = _mm_add_ps(_mm_mul_ps(result, x2), _mm_set_ps1(8.1675882859940430e-2F));
	result = _mm_add_ps(_mm_mul_ps(result, x2), _mm_set_ps1(-1.3374657325451267e-1F));
	result = _mm_add_ps(_mm_mul_ps(result, x2), _mm_set_ps1(1.9856563505717162e-1F));
	result = _mm_add_ps(_mm_mul_ps(result, x2), _mm_set_ps1(-3.3324998579202170e-1F));
	result = _mm_add_ps(_mm_mul_ps(result, x2), _mm_set_ps1(1.0F));
	result = _mm_mul_ps(result, x);

	__m128 remapped = _mm_sub_ps(_mm_set_ps1(rtm::constants::half_pi()), result);

	// pi/2 - result
	result = vector_select(is_larger_than_one, remapped, result);

	// Keep the original sign
	return _mm_or_ps(result, _mm_and_ps(input, _mm_set_ps1(-0.0F)));
}
#endif

#if defined(RTM_NEON_INTRINSICS)
RTM_FORCE_NOINLINE vector4f RTM_SIMD_CALL vector_atan_neon(vector4f_arg0 input) RTM_NO_EXCEPT
{
	// Use a degree 13 minimax approximation polynomial
	// See: GPGPU Programming for Games and Science (David H. Eberly)

	// Discard our sign, we'll restore it later
	float32x4_t abs_value = vabsq_f32(input);

	// Compute our value
	uint32x4_t is_larger_than_one = vcagtq_f32(input, vdupq_n_f32(1.0F));
	float32x4_t reciprocal = vector_reciprocal(abs_value);

	float32x4_t x = vector_select(is_larger_than_one, reciprocal, abs_value);

	float32x4_t x2 = vmulq_f32(x, x);

#if defined(RTM_NEON64_INTRINSICS)
	float32x4_t result = vfmaq_n_f32(vdupq_n_f32(-3.5059680836411644e-2F), x2, 7.2128853633444123e-3F);
	result = vfmaq_f32(vdupq_n_f32(8.1675882859940430e-2F), result, x2);
	result = vfmaq_f32(vdupq_n_f32(-1.3374657325451267e-1F), result, x2);
	result = vfmaq_f32(vdupq_n_f32(1.9856563505717162e-1F), result, x2);
	result = vfmaq_f32(vdupq_n_f32(-3.3324998579202170e-1F), result, x2);
	result = vfmaq_f32(vdupq_n_f32(1.0F), result, x2);
#else
	float32x4_t result = vmlaq_n_f32(vdupq_n_f32(-3.5059680836411644e-2F), x2, 7.2128853633444123e-3F);
	result = vmlaq_f32(vdupq_n_f32(8.1675882859940430e-2F), result, x2);
	result = vmlaq_f32(vdupq_n_f32(-1.3374657325451267e-1F), result, x2);
	result = vmlaq_f32(vdupq_n_f32(1.9856563505717162e-1F), result, x2);
	result = vmlaq_f32(vdupq_n_f32(-3.3324998579202170e-1F), result, x2);
	result = vmlaq_f32(vdupq_n_f32(1.0F), result, x2);
#endif

	result = vmulq_f32(result, x);

	float32x4_t remapped = vsubq_f32(vdupq_n_f32(rtm::constants::half_pi()), result);

	// pi/2 - result
	result = vector_select(is_larger_than_one, remapped, result);

	// Keep the original sign
	return vreinterpretq_f32_u32(vorrq_u32(vreinterpretq_u32_f32(result), vandq_u32(vreinterpretq_u32_f32(input), vreinterpretq_u32_f32(vdupq_n_f32(-0.0F)))));
}
#endif

static void bm_vector_atan_scalar(benchmark::State& state)
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
		v0 = vector_atan_scalar(v0);
		v1 = vector_atan_scalar(v1);
		v2 = vector_atan_scalar(v2);
		v3 = vector_atan_scalar(v3);
		v4 = vector_atan_scalar(v4);
		v5 = vector_atan_scalar(v5);
		v6 = vector_atan_scalar(v6);
		v7 = vector_atan_scalar(v7);
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

BENCHMARK(bm_vector_atan_scalar);

#if defined(RTM_SSE2_INTRINSICS)
static void bm_vector_atan_sse2(benchmark::State& state)
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
		v0 = vector_atan_sse2(v0);
		v1 = vector_atan_sse2(v1);
		v2 = vector_atan_sse2(v2);
		v3 = vector_atan_sse2(v3);
		v4 = vector_atan_sse2(v4);
		v5 = vector_atan_sse2(v5);
		v6 = vector_atan_sse2(v6);
		v7 = vector_atan_sse2(v7);
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

BENCHMARK(bm_vector_atan_sse2);
#endif

#if defined(RTM_NEON_INTRINSICS)
static void bm_vector_atan_neon(benchmark::State& state)
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
		v0 = vector_atan_neon(v0);
		v1 = vector_atan_neon(v1);
		v2 = vector_atan_neon(v2);
		v3 = vector_atan_neon(v3);
		v4 = vector_atan_neon(v4);
		v5 = vector_atan_neon(v5);
		v6 = vector_atan_neon(v6);
		v7 = vector_atan_neon(v7);
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

BENCHMARK(bm_vector_atan_neon);
#endif
