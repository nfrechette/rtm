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

RTM_FORCE_NOINLINE vector4f RTM_SIMD_CALL vector_cos_scalar(vector4f_arg0 input) RTM_NO_EXCEPT
{
	scalarf x = scalar_cos(scalarf(vector_get_x(input)));
	scalarf y = scalar_cos(scalarf(vector_get_y(input)));
	scalarf z = scalar_cos(scalarf(vector_get_z(input)));
	scalarf w = scalar_cos(scalarf(vector_get_w(input)));
	return vector_set(x, y, z, w);
}

#if defined(RTM_SSE2_INTRINSICS)
RTM_FORCE_NOINLINE vector4f RTM_SIMD_CALL vector_cos_sse2(vector4f_arg0 input) RTM_NO_EXCEPT
{
	// Use a degree 10 minimax approximation polynomial
	// See: GPGPU Programming for Games and Science (David H. Eberly)

	// Remap our input in the [-pi, pi] range
	__m128 quotient = _mm_mul_ps(input, _mm_set_ps1(rtm::constants::one_div_two_pi()));
	quotient = vector_round_bankers(quotient);
	quotient = _mm_mul_ps(quotient, _mm_set_ps1(rtm::constants::two_pi()));
	__m128 x = _mm_sub_ps(input, quotient);

	// Remap our input in the [-pi/2, pi/2] range
	const __m128 sign_mask = _mm_set_ps(-0.0F, -0.0F, -0.0F, -0.0F);
	__m128 x_sign = _mm_and_ps(x, sign_mask);
	__m128 reference = _mm_or_ps(x_sign, _mm_set_ps1(rtm::constants::pi()));
	const __m128 reflection = _mm_sub_ps(reference, x);

	const __m128i abs_mask = _mm_set_epi32(0x7FFFFFFFULL, 0x7FFFFFFFULL, 0x7FFFFFFFULL, 0x7FFFFFFFULL);
	__m128 x_abs = _mm_and_ps(x, _mm_castsi128_ps(abs_mask));
	__m128 is_less_equal_than_half_pi = _mm_cmple_ps(x_abs, _mm_set_ps1(rtm::constants::half_pi()));

	x = RTM_VECTOR4F_SELECT(is_less_equal_than_half_pi, x, reflection);

	// Calculate our value
	const __m128 x2 = _mm_mul_ps(x, x);
	__m128 result = _mm_add_ps(_mm_mul_ps(x2, _mm_set_ps1(-2.6051615464872668e-7F)), _mm_set_ps1(2.4760495088926859e-5F));
	result = _mm_add_ps(_mm_mul_ps(result, x2), _mm_set_ps1(-1.3888377661039897e-3F));
	result = _mm_add_ps(_mm_mul_ps(result, x2), _mm_set_ps1(4.1666638865338612e-2F));
	result = _mm_add_ps(_mm_mul_ps(result, x2), _mm_set_ps1(-4.9999999508695869e-1F));
	result = _mm_add_ps(_mm_mul_ps(result, x2), _mm_set_ps1(1.0F));

	// Remap into [-pi, pi]
	return _mm_or_ps(result, _mm_andnot_ps(is_less_equal_than_half_pi, sign_mask));
}
#endif

#if defined(RTM_NEON_INTRINSICS)
RTM_FORCE_NOINLINE vector4f RTM_SIMD_CALL vector_cos_neon(vector4f_arg0 input) RTM_NO_EXCEPT
{
	// Use a degree 10 minimax approximation polynomial
	// See: GPGPU Programming for Games and Science (David H. Eberly)

	// Remap our input in the [-pi, pi] range
	float32x4_t quotient = vmulq_n_f32(input, rtm::constants::one_div_two_pi());
	quotient = vector_round_bankers(quotient);
	quotient = vmulq_n_f32(quotient, rtm::constants::two_pi());
	float32x4_t x = vsubq_f32(input, quotient);

	// Remap our input in the [-pi/2, pi/2] range
	uint32x4_t sign_mask = vreinterpretq_u32_f32(vdupq_n_f32(-0.0F));
	uint32x4_t sign = vandq_u32(vreinterpretq_u32_f32(x), sign_mask);
	float32x4_t reference = vreinterpretq_f32_u32(vorrq_u32(sign, vreinterpretq_u32_f32(vdupq_n_f32(rtm::constants::pi()))));

	float32x4_t reflection = vsubq_f32(reference, x);
	float32x4_t is_less_equal_than_half_pi = vcaleq_f32(x, vdupq_n_f32(rtm::constants::half_pi()));
	x = vbslq_f32(is_less_equal_than_half_pi, x, reflection);

	// Calculate our value
	float32x4_t x2 = vmulq_f32(x, x);

#if defined(RTM_NEON64_INTRINSICS)
	float32x4_t result = vfmaq_n_f32(vdupq_n_f32(2.4760495088926859e-5F), x2, -2.6051615464872668e-7F);
	result = vfmaq_f32(vdupq_n_f32(-1.3888377661039897e-3F), result, x2);
	result = vfmaq_f32(vdupq_n_f32(4.1666638865338612e-2F), result, x2);
	result = vfmaq_f32(vdupq_n_f32(-4.9999999508695869e-1F), result, x2);
	result = vfmaq_f32(vdupq_n_f32(1.0F), result, x2);
#else
	float32x4_t result = vmlaq_n_f32(vdupq_n_f32(2.4760495088926859e-5F), x2, -2.6051615464872668e-7F);
	result = vmlaq_f32(vdupq_n_f32(-1.3888377661039897e-3F), result, x2);
	result = vmlaq_f32(vdupq_n_f32(4.1666638865338612e-2F), result, x2);
	result = vmlaq_f32(vdupq_n_f32(-4.9999999508695869e-1F), result, x2);
	result = vmlaq_f32(vdupq_n_f32(1.0F), result, x2);
#endif

	// Remap into [-pi, pi]
	return vbslq_f32(is_less_equal_than_half_pi, result, vnegq_f32(result));
}
#endif

static void bm_vector_cos_scalar(benchmark::State& state)
{
	vector4f v0 = vector_set(-123.134f);
	vector4f v1 = vector_set(123.134f);
	vector4f v2 = vector_set(-123.134f);
	vector4f v3 = vector_set(123.134f);
	vector4f v4 = vector_set(-123.134f);
	vector4f v5 = vector_set(123.134f);
	vector4f v6 = vector_set(-123.134f);
	vector4f v7 = vector_set(123.134f);

	vector4f scale = vector_set(100.0F);

	for (auto _ : state)
	{
		v0 = vector_mul(vector_cos_scalar(v0), scale);
		v1 = vector_mul(vector_cos_scalar(v1), scale);
		v2 = vector_mul(vector_cos_scalar(v2), scale);
		v3 = vector_mul(vector_cos_scalar(v3), scale);
		v4 = vector_mul(vector_cos_scalar(v4), scale);
		v5 = vector_mul(vector_cos_scalar(v5), scale);
		v6 = vector_mul(vector_cos_scalar(v6), scale);
		v7 = vector_mul(vector_cos_scalar(v7), scale);
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

BENCHMARK(bm_vector_cos_scalar);

#if defined(RTM_SSE2_INTRINSICS)
static void bm_vector_cos_sse2(benchmark::State& state)
{
	vector4f v0 = vector_set(-123.134f);
	vector4f v1 = vector_set(123.134f);
	vector4f v2 = vector_set(-123.134f);
	vector4f v3 = vector_set(123.134f);
	vector4f v4 = vector_set(-123.134f);
	vector4f v5 = vector_set(123.134f);
	vector4f v6 = vector_set(-123.134f);
	vector4f v7 = vector_set(123.134f);

	vector4f scale = vector_set(100.0F);

	for (auto _ : state)
	{
		v0 = vector_mul(vector_cos_sse2(v0), scale);
		v1 = vector_mul(vector_cos_sse2(v1), scale);
		v2 = vector_mul(vector_cos_sse2(v2), scale);
		v3 = vector_mul(vector_cos_sse2(v3), scale);
		v4 = vector_mul(vector_cos_sse2(v4), scale);
		v5 = vector_mul(vector_cos_sse2(v5), scale);
		v6 = vector_mul(vector_cos_sse2(v6), scale);
		v7 = vector_mul(vector_cos_sse2(v7), scale);
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

BENCHMARK(bm_vector_cos_sse2);
#endif

#if defined(RTM_NEON_INTRINSICS)
static void bm_vector_cos_neon(benchmark::State& state)
{
	vector4f v0 = vector_set(-123.134f);
	vector4f v1 = vector_set(123.134f);
	vector4f v2 = vector_set(-123.134f);
	vector4f v3 = vector_set(123.134f);
	vector4f v4 = vector_set(-123.134f);
	vector4f v5 = vector_set(123.134f);
	vector4f v6 = vector_set(-123.134f);
	vector4f v7 = vector_set(123.134f);

	vector4f scale = vector_set(100.0F);

	for (auto _ : state)
	{
		v0 = vector_mul(vector_cos_neon(v0), scale);
		v1 = vector_mul(vector_cos_neon(v1), scale);
		v2 = vector_mul(vector_cos_neon(v2), scale);
		v3 = vector_mul(vector_cos_neon(v3), scale);
		v4 = vector_mul(vector_cos_neon(v4), scale);
		v5 = vector_mul(vector_cos_neon(v5), scale);
		v6 = vector_mul(vector_cos_neon(v6), scale);
		v7 = vector_mul(vector_cos_neon(v7), scale);
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

BENCHMARK(bm_vector_cos_neon);
#endif
