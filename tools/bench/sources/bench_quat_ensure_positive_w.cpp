////////////////////////////////////////////////////////////////////////////////
// The MIT License (MIT)
//
// Copyright (c) 2022 Nicholas Frechette & Realtime Math contributors
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

#include <rtm/packing/quatf.h>
#include <rtm/impl/bit_cast.impl.h>

using namespace rtm;

RTM_FORCE_NOINLINE quatf RTM_SIMD_CALL quat_ensure_positive_w_scalar(quatf_arg0 input) RTM_NO_EXCEPT
{
	return quat_get_w(input) >= 0.f ? input : quat_neg(input);
}

#if defined(RTM_SSE2_INTRINSICS)
RTM_FORCE_NOINLINE quatf RTM_SIMD_CALL quat_ensure_positive_w_sse2(quatf_arg0 input) RTM_NO_EXCEPT
{
	constexpr __m128 sign_bit = { -0.0F, -0.0F, -0.0F, -0.0F };
	const __m128 input_sign = _mm_and_ps(input, sign_bit);
	const __m128 bias = _mm_shuffle_ps(input_sign, input_sign, _MM_SHUFFLE(3, 3, 3, 3));
	return _mm_xor_ps(input, bias);
}
#endif

#if defined(RTM_NEON_INTRINSICS)
RTM_FORCE_NOINLINE quatf RTM_SIMD_CALL quat_ensure_positive_w_neon(quatf_arg0 input) RTM_NO_EXCEPT
{
	alignas(16) constexpr uint32_t sign_bit_i[4] = { 0x80000000U, 0x80000000U, 0x80000000U, 0x80000000U };
	const uint32x4_t sign_bit = *rtm_impl::bit_cast<const uint32x4_t*>(&sign_bit_i[0]);
	const uint32x4_t input_u32 = vreinterpretq_u32_f32(input);
	const uint32x4_t input_sign = vandq_u32(input_u32, sign_bit);
	const uint32x4_t bias = vmovq_n_u32(vgetq_lane_u32(input_sign, 3));
	return vreinterpretq_f32_u32(veorq_u32(input_u32, bias));
}
#endif

static void bm_quat_ensure_positive_w_scalar(benchmark::State& state)
{
	quatf q0 = quat_identity();
	quatf q1 = quat_identity();
	quatf q2 = quat_identity();
	quatf q3 = quat_identity();

	for (auto _ : state)
	{
		q0 = quat_ensure_positive_w_scalar(q0);
		q1 = quat_ensure_positive_w_scalar(q1);
		q2 = quat_ensure_positive_w_scalar(q2);
		q3 = quat_ensure_positive_w_scalar(q3);
	}

	benchmark::DoNotOptimize(q0);
	benchmark::DoNotOptimize(q1);
	benchmark::DoNotOptimize(q2);
	benchmark::DoNotOptimize(q3);
}

BENCHMARK(bm_quat_ensure_positive_w_scalar);

#if defined(RTM_SSE2_INTRINSICS)
static void bm_quat_ensure_positive_w_sse2(benchmark::State& state)
{
	quatf q0 = quat_identity();
	quatf q1 = quat_identity();
	quatf q2 = quat_identity();
	quatf q3 = quat_identity();

	for (auto _ : state)
	{
		q0 = quat_ensure_positive_w_sse2(q0);
		q1 = quat_ensure_positive_w_sse2(q1);
		q2 = quat_ensure_positive_w_sse2(q2);
		q3 = quat_ensure_positive_w_sse2(q3);
	}

	benchmark::DoNotOptimize(q0);
	benchmark::DoNotOptimize(q1);
	benchmark::DoNotOptimize(q2);
	benchmark::DoNotOptimize(q3);
}

BENCHMARK(bm_quat_ensure_positive_w_sse2);
#endif

#if defined(RTM_NEON_INTRINSICS)
static void bm_quat_ensure_positive_w_neon(benchmark::State& state)
{
	quatf q0 = quat_identity();
	quatf q1 = quat_identity();
	quatf q2 = quat_identity();
	quatf q3 = quat_identity();

	for (auto _ : state)
	{
		q0 = quat_ensure_positive_w_neon(q0);
		q1 = quat_ensure_positive_w_neon(q1);
		q2 = quat_ensure_positive_w_neon(q2);
		q3 = quat_ensure_positive_w_neon(q3);
	}

	benchmark::DoNotOptimize(q0);
	benchmark::DoNotOptimize(q1);
	benchmark::DoNotOptimize(q2);
	benchmark::DoNotOptimize(q3);
}

BENCHMARK(bm_quat_ensure_positive_w_neon);
#endif
