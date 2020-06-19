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

RTM_FORCE_NOINLINE vector4f RTM_SIMD_CALL vector_acos_scalar(vector4f_arg0 input) RTM_NO_EXCEPT
{
	scalarf x = scalar_acos(scalarf(vector_get_x(input)));
	scalarf y = scalar_acos(scalarf(vector_get_y(input)));
	scalarf z = scalar_acos(scalarf(vector_get_z(input)));
	scalarf w = scalar_acos(scalarf(vector_get_w(input)));
	return vector_set(x, y, z, w);
}

#if defined(RTM_SSE2_INTRINSICS)
RTM_FORCE_NOINLINE vector4f RTM_SIMD_CALL vector_acos_sse2(vector4f_arg0 input) RTM_NO_EXCEPT
{
	// Use the identity: acos(value) + asin(value) = PI/2
	// This ends up being: acos(value) = PI/2 - asin(value)
	// Since asin(value) = PI/2 - sqrt(1.0 - polynomial(value))
	// Our end result is acos(value) = sqrt(1.0 - polynomial(value))
	// This means we can re-use the same polynomial as asin()
	// See: GPGPU Programming for Games and Science (David H. Eberly)

	// We first calculate our scale: sqrt(1.0 - abs(value))
	// Use the sign bit to generate our absolute value since we'll re-use that constant
	const __m128 sign_bit = _mm_set_ps1(-0.0F);
	__m128 abs_value = _mm_andnot_ps(sign_bit, input);

	// Calculate our value
	__m128 result = _mm_add_ps(_mm_mul_ps(abs_value, _mm_set_ps1(-1.2690614339589956e-3F)), _mm_set_ps1(6.7072304676685235e-3F));
	result = _mm_add_ps(_mm_mul_ps(result, abs_value), _mm_set_ps1(-1.7162031184398074e-2F));
	result = _mm_add_ps(_mm_mul_ps(result, abs_value), _mm_set_ps1(3.0961594977611639e-2F));
	result = _mm_add_ps(_mm_mul_ps(result, abs_value), _mm_set_ps1(-5.0207843052845647e-2F));
	result = _mm_add_ps(_mm_mul_ps(result, abs_value), _mm_set_ps1(8.8986946573346160e-2F));
	result = _mm_add_ps(_mm_mul_ps(result, abs_value), _mm_set_ps1(-2.1459960076929829e-1F));
	result = _mm_add_ps(_mm_mul_ps(result, abs_value), _mm_set_ps1(1.5707963267948966F));

	// Scale our result
	__m128 scale = _mm_sqrt_ps(_mm_sub_ps(_mm_set_ps1(1.0F), abs_value));
	result = _mm_mul_ps(result, scale);

	// Normally the math is as follow:
	// If input is positive: result
	// If input is negative: PI - result = -result + PI

	// As such, the offset is 0.0 when the input is positive and PI when negative
	__m128 is_input_negative = _mm_cmplt_ps(input, _mm_setzero_ps());
	__m128 offset = _mm_and_ps(is_input_negative, _mm_set_ps1(rtm::constants::pi()));

	// And our result has the same sign of the input
	__m128 input_sign = _mm_and_ps(input, sign_bit);
	result = _mm_or_ps(result, input_sign);
	return _mm_add_ps(result, offset);
}
#endif

#if defined(RTM_NEON_INTRINSICS)
RTM_FORCE_NOINLINE vector4f RTM_SIMD_CALL vector_acos_neon(vector4f_arg0 input) RTM_NO_EXCEPT
{
	// Use the identity: acos(value) + asin(value) = PI/2
	// This ends up being: acos(value) = PI/2 - asin(value)
	// Since asin(value) = PI/2 - sqrt(1.0 - polynomial(value))
	// Our end result is acos(value) = sqrt(1.0 - polynomial(value))
	// This means we can re-use the same polynomial as asin()
	// See: GPGPU Programming for Games and Science (David H. Eberly)

	// We first calculate our scale: sqrt(1.0 - abs(value))
	// Use the sign bit to generate our absolute value since we'll re-use that constant
	float32x4_t abs_value = vabsq_f32(input);

	// Calculate our value
	float32x4_t result = vmlaq_n_f32(vdupq_n_f32(6.7072304676685235e-3F), abs_value, -1.2690614339589956e-3F);
	result = vmlaq_f32(vdupq_n_f32(-1.7162031184398074e-2F), result, abs_value);
	result = vmlaq_f32(vdupq_n_f32(3.0961594977611639e-2F), result, abs_value);
	result = vmlaq_f32(vdupq_n_f32(-5.0207843052845647e-2F), result, abs_value);
	result = vmlaq_f32(vdupq_n_f32(8.8986946573346160e-2F), result, abs_value);
	result = vmlaq_f32(vdupq_n_f32(-2.1459960076929829e-1F), result, abs_value);
	result = vmlaq_f32(vdupq_n_f32(1.5707963267948966F), result, abs_value);

	// Scale our result
	float32x4_t scale = vector_sqrt(vsubq_f32(vdupq_n_f32(1.0F), abs_value));
	result = vmulq_f32(result, scale);

	// Normally the math is as follow:
	// If input is positive: result
	// If input is negative: PI - result = -result + PI

	// As such, the offset is 0.0 when the input is positive and PI when negative
	uint32x4_t is_input_negative = vcltq_f32(input, vdupq_n_f32(0.0F));
	float32x4_t offset = vreinterpretq_f32_u32(vandq_u32(is_input_negative, vreinterpretq_u32_f32(vdupq_n_f32(rtm::constants::pi()))));

	// And our result has the same sign of the input
	uint32x4_t sign_mask = vreinterpretq_u32_f32(vdupq_n_f32(-0.0F));
	uint32x4_t input_sign = vandq_u32(vreinterpretq_u32_f32(input), sign_mask);

	result = vreinterpretq_f32_u32(vorrq_u32(vreinterpretq_u32_f32(result), input_sign));
	return vaddq_f32(result, offset);
}
#endif

static void bm_vector_acos_scalar(benchmark::State& state)
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
		v0 = vector_acos_scalar(v0);
		v1 = vector_acos_scalar(v1);
		v2 = vector_acos_scalar(v2);
		v3 = vector_acos_scalar(v3);
		v4 = vector_acos_scalar(v4);
		v5 = vector_acos_scalar(v5);
		v6 = vector_acos_scalar(v6);
		v7 = vector_acos_scalar(v7);
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

BENCHMARK(bm_vector_acos_scalar);

#if defined(RTM_SSE2_INTRINSICS)
static void bm_vector_acos_sse2(benchmark::State& state)
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
		v0 = vector_acos_sse2(v0);
		v1 = vector_acos_sse2(v1);
		v2 = vector_acos_sse2(v2);
		v3 = vector_acos_sse2(v3);
		v4 = vector_acos_sse2(v4);
		v5 = vector_acos_sse2(v5);
		v6 = vector_acos_sse2(v6);
		v7 = vector_acos_sse2(v7);
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

BENCHMARK(bm_vector_acos_sse2);
#endif

#if defined(RTM_NEON_INTRINSICS)
static void bm_vector_acos_neon(benchmark::State& state)
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
		v0 = vector_acos_neon(v0);
		v1 = vector_acos_neon(v1);
		v2 = vector_acos_neon(v2);
		v3 = vector_acos_neon(v3);
		v4 = vector_acos_neon(v4);
		v5 = vector_acos_neon(v5);
		v6 = vector_acos_neon(v6);
		v7 = vector_acos_neon(v7);
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

BENCHMARK(bm_vector_acos_neon);
#endif
