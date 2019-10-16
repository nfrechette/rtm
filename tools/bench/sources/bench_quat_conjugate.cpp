////////////////////////////////////////////////////////////////////////////////
// The MIT License (MIT)
//
// Copyright (c) 2019 Nicholas Frechette & Realtime Math contributors
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

#include <rtm/quatf.h>

using namespace rtm;

inline quatf RTM_SIMD_CALL quat_conjugate_scalar(quatf_arg0 input) RTM_NO_EXCEPT
{
	return quat_set(-quat_get_x(input), -quat_get_y(input), -quat_get_z(input), quat_get_w(input));
}

inline quatf RTM_SIMD_CALL quat_conjugate_mul(quatf_arg0 input) RTM_NO_EXCEPT
{
#if defined(RTM_SSE2_INTRINSICS)
	constexpr __m128 signs = { -1.0f, -1.0f, -1.0f, 1.0f };
	return _mm_mul_ps(input, signs);
#else
	return quat_set(quat_get_x(input) * -1.0f, quat_get_y(input) * -1.0f, quat_get_z(input) * -1.0f, quat_get_w(input));
#endif
}

inline quatf RTM_SIMD_CALL quat_conjugate_best(quatf_arg0 input) RTM_NO_EXCEPT
{
#if defined(RTM_SSE2_INTRINSICS)
	constexpr __m128 signs = { -0.0f, -0.0f, -0.0f, 0.0f };
	return _mm_xor_ps(input, signs);
#elif defined(RTM_NEON64_INTRINSICS)
	const float32x4_t neg_input = vnegq_f32(input);
	return vsetq_lane_f32(vgetq_lane_f32(input, 3), neg_input, 3);
#else
	return quat_set(-quat_get_x(input), -quat_get_y(input), -quat_get_z(input), quat_get_w(input));
#endif
}

static void bm_quat_conjugate_scalar(benchmark::State& state)
{
	quatf q0 = quat_identity();

	for (auto _ : state)
		benchmark::DoNotOptimize(q0 = quat_conjugate_scalar(q0));
}

BENCHMARK(bm_quat_conjugate_scalar);

static void bm_quat_conjugate_mul(benchmark::State& state)
{
	quatf q0 = quat_identity();

	for (auto _ : state)
		benchmark::DoNotOptimize(q0 = quat_conjugate_mul(q0));
}

BENCHMARK(bm_quat_conjugate_mul);

static void bm_quat_conjugate_best(benchmark::State& state)
{
	quatf q0 = quat_identity();

	for (auto _ : state)
		benchmark::DoNotOptimize(q0 = quat_conjugate_best(q0));
}

BENCHMARK(bm_quat_conjugate_best);
