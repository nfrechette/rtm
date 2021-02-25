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

RTM_FORCE_NOINLINE float RTM_SIMD_CALL scalar_sin_scalar(float input) RTM_NO_EXCEPT
{
	// Use a degree 11 minimax approximation polynomial
	// See: GPGPU Programming for Games and Science (David H. Eberly)

	// Remap our input in the [-pi, pi] range
	float quotient = input * rtm::constants::one_div_two_pi();
	quotient = scalar_round_bankers(quotient);
	quotient = quotient * rtm::constants::two_pi();
	float x = input - quotient;

	// Remap our input in the [-pi/2, pi/2] range
	const float reference = std::copysign(rtm::constants::pi(), x);
	const float reflection = reference - x;
	const float x_abs = scalar_abs(x);
	x = x_abs <= rtm::constants::half_pi() ? x : reflection;

	// Calculate our value
	const float x2 = x * x;
	float result = (x2 * -2.3828544692960918e-8F) + 2.7521557770526783e-6F;
	result = (result * x2) - 1.9840782426250314e-4F;
	result = (result * x2) + 8.3333303183525942e-3F;
	result = (result * x2) - 1.6666666601721269e-1F;
	result = (result * x2) + 1.0F;
	result = result * x;
	return result;
}

RTM_FORCE_NOINLINE float RTM_SIMD_CALL scalar_sin_std(float input) RTM_NO_EXCEPT
{
	return std::sin(input);
}

#if defined(RTM_SSE2_INTRINSICS)
RTM_FORCE_NOINLINE float RTM_SIMD_CALL scalar_sin_sse2(float input) RTM_NO_EXCEPT
{
	__m128 input_s = _mm_set_ps1(input);

	// Use a degree 11 minimax approximation polynomial
	// See: GPGPU Programming for Games and Science (David H. Eberly)

	// Remap our input in the [-pi, pi] range
	__m128 quotient = _mm_mul_ss(input_s, _mm_set_ps1(rtm::constants::one_div_two_pi()));
	quotient = scalar_round_bankers(scalarf{ quotient }).value;
	quotient = _mm_mul_ss(quotient, _mm_set_ps1(rtm::constants::two_pi()));
	__m128 x = _mm_sub_ss(input_s, quotient);

	// Remap our input in the [-pi/2, pi/2] range
	const __m128 sign_mask = _mm_set_ps(-0.0F, -0.0F, -0.0F, -0.0F);
	__m128 sign = _mm_and_ps(x, sign_mask);
	__m128 reference = _mm_or_ps(sign, _mm_set_ps1(rtm::constants::pi()));

	const __m128 reflection = _mm_sub_ss(reference, x);
	const __m128i abs_mask = _mm_set_epi32(0x7FFFFFFFULL, 0x7FFFFFFFULL, 0x7FFFFFFFULL, 0x7FFFFFFFULL);
	const __m128 x_abs = _mm_and_ps(x, _mm_castsi128_ps(abs_mask));

	__m128 is_less_equal_than_half_pi = _mm_cmple_ss(x_abs, _mm_set_ps1(rtm::constants::half_pi()));

	x = RTM_VECTOR4F_SELECT(is_less_equal_than_half_pi, x, reflection);

	// Calculate our value
	const float x2 = _mm_cvtss_f32(_mm_mul_ss(x, x));
	float result = (x2 * -2.3828544692960918e-8F) + 2.7521557770526783e-6F;
	result = (result * x2) - 1.9840782426250314e-4F;
	result = (result * x2) + 8.3333303183525942e-3F;
	result = (result * x2) - 1.6666666601721269e-1F;
	result = (result * x2) + 1.0F;
	result = result * _mm_cvtss_f32(x);
	return result;
}
#endif

alignas(64) static constexpr float constants3_[12] =
{
	2.7521557770526783e-6F, rtm::constants::one_div_two_pi(),
	-2.3828544692960918e-8F, rtm::constants::two_pi(),
	-1.9840782426250314e-4F, rtm::constants::pi(),
	8.3333303183525942e-3F, -rtm::constants::pi(),

	-1.6666666601721269e-1F, rtm::constants::half_pi(),
	1.0F, 1.0F,
};

#if defined(RTM_NEON64_INTRINSICS)
RTM_FORCE_NOINLINE float RTM_SIMD_CALL scalar_sin_neon64(float input) RTM_NO_EXCEPT
{
	// Use a degree 11 minimax approximation polynomial
	// See: GPGPU Programming for Games and Science (David H. Eberly)

	const float32x2x4_t constants0 = vld4_f32(&constants3_[0]);

	float32x2_t input_v = vdup_n_f32(input);
	const float32x2x2_t constants1 = vld2_f32(&constants3_[8]);

	// Remap our input in the [-pi, pi] range
	float32x2_t quotient = vmul_n_f32(constants0.val[0], input);	// [y] SIMD lane
	bool is_x_positive = input >= 0.0F;
	quotient = vrndn_f32(quotient);
	float32x2_t reference_v = is_x_positive ? constants0.val[2] : constants0.val[3];	// [y] SIMD lane

#if defined(RTM_NEON64_INTRINSICS)
	float32x2_t x_v = vfms_lane_f32(input_v, quotient, constants0.val[1], 1);	// [y] SIMD lane
#else
	float32x2_t x_v = vmls_lane_f32(input_v, quotient, constants0.val[1], 1);	// [y] SIMD lane
#endif

	float32x2_t reflection_v = vsub_f32(reference_v, x_v);

	// Remap our input in the [-pi/2, pi/2] range
	uint32x2_t is_less_equal_than_half_pi = vcale_f32(x_v, constants1.val[0]);
	x_v = vbsl_f32(is_less_equal_than_half_pi, x_v, reflection_v);	// [y] SIMD lane

	// Calculate our value, we only care about the [x] SIMD lane
	float32x2_t x2 = vmul_lane_f32(x_v, x_v, 1);	// [y] SIMD lane

#if defined(RTM_NEON64_INTRINSICS)
	float32x2_t result = vfma_lane_f32(constants0.val[0], constants0.val[1], x2, 1);
	result = vfma_lane_f32(constants0.val[2], result, x2, 1);
	result = vfma_lane_f32(constants0.val[3], result, x2, 1);
	result = vfma_lane_f32(constants1.val[0], result, x2, 1);
	result = vfma_lane_f32(constants1.val[1], result, x2, 1);
#else
	float32x2_t result = vmla_lane_f32(constants0.val[0], constants0.val[1], x2, 1);
	result = vmla_lane_f32(constants0.val[2], result, x2, 1);
	result = vmla_lane_f32(constants0.val[3], result, x2, 1);
	result = vmla_lane_f32(constants1.val[0], result, x2, 1);
	result = vmla_lane_f32(constants1.val[1], result, x2, 1);
#endif

	result = vmul_lane_f32(result, x_v, 1);
	return vget_lane_f32(result, 0);
}
#endif

static void bm_scalar_sin_scalar(benchmark::State& state)
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
		f0 = scalar_sin_scalar(f0) * 100.0F;
		f1 = scalar_sin_scalar(f1) * 100.0F;
		f2 = scalar_sin_scalar(f2) * 100.0F;
		f3 = scalar_sin_scalar(f3) * 100.0F;
		f4 = scalar_sin_scalar(f4) * 100.0F;
		f5 = scalar_sin_scalar(f5) * 100.0F;
		f6 = scalar_sin_scalar(f6) * 100.0F;
		f7 = scalar_sin_scalar(f7) * 100.0F;
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

BENCHMARK(bm_scalar_sin_scalar);

static void bm_scalar_sin_std(benchmark::State& state)
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
		f0 = scalar_sin_std(f0) * 100.0F;
		f1 = scalar_sin_std(f1) * 100.0F;
		f2 = scalar_sin_std(f2) * 100.0F;
		f3 = scalar_sin_std(f3) * 100.0F;
		f4 = scalar_sin_std(f4) * 100.0F;
		f5 = scalar_sin_std(f5) * 100.0F;
		f6 = scalar_sin_std(f6) * 100.0F;
		f7 = scalar_sin_std(f7) * 100.0F;
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

BENCHMARK(bm_scalar_sin_std);

#if defined(RTM_SSE2_INTRINSICS)
static void bm_scalar_sin_sse2(benchmark::State& state)
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
		f0 = scalar_sin_sse2(f0) * 100.0F;
		f1 = scalar_sin_sse2(f1) * 100.0F;
		f2 = scalar_sin_sse2(f2) * 100.0F;
		f3 = scalar_sin_sse2(f3) * 100.0F;
		f4 = scalar_sin_sse2(f4) * 100.0F;
		f5 = scalar_sin_sse2(f5) * 100.0F;
		f6 = scalar_sin_sse2(f6) * 100.0F;
		f7 = scalar_sin_sse2(f7) * 100.0F;
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

BENCHMARK(bm_scalar_sin_sse2);
#endif

#if defined(RTM_NEON64_INTRINSICS)
static void bm_scalar_sin_neon64(benchmark::State& state)
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
		f0 = scalar_sin_neon64(f0) * 100.0F;
		f1 = scalar_sin_neon64(f1) * 100.0F;
		f2 = scalar_sin_neon64(f2) * 100.0F;
		f3 = scalar_sin_neon64(f3) * 100.0F;
		f4 = scalar_sin_neon64(f4) * 100.0F;
		f5 = scalar_sin_neon64(f5) * 100.0F;
		f6 = scalar_sin_neon64(f6) * 100.0F;
		f7 = scalar_sin_neon64(f7) * 100.0F;
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

BENCHMARK(bm_scalar_sin_neon64);
#endif
