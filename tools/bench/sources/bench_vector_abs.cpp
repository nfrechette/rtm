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

#include <rtm/vector4f.h>

using namespace rtm;

RTM_FORCE_NOINLINE vector4f RTM_SIMD_CALL vector_abs_scalar(vector4f_arg0 input) RTM_NO_EXCEPT
{
	scalarf x = vector_get_x(input);
	scalarf y = vector_get_y(input);
	scalarf z = vector_get_z(input);
	scalarf w = vector_get_w(input);
	return vector_set(scalar_abs(x), scalar_abs(y), scalar_abs(z), scalar_abs(w));
}

#if defined(RTM_SSE2_INTRINSICS)
RTM_FORCE_NOINLINE vector4f RTM_SIMD_CALL vector_abs_sse2_maxsub(vector4f_arg0 input) RTM_NO_EXCEPT
{
	return vector_max(vector_sub(_mm_setzero_ps(), input), input);
}

// Wins on Haswell laptop x64 AVX
// Wins on Ryzen 2990X desktop VS2017 x64 AVX
// Wins on Ryzen 2990X desktop clang9 x64 AVX
RTM_FORCE_NOINLINE vector4f RTM_SIMD_CALL vector_abs_sse2_and(vector4f_arg0 input) RTM_NO_EXCEPT
{
	const __m128i abs_mask = _mm_set_epi32(0x7FFFFFFFULL, 0x7FFFFFFFULL, 0x7FFFFFFFULL, 0x7FFFFFFFULL);
	return _mm_and_ps(input, _mm_castsi128_ps(abs_mask));
}
#endif

static void bm_vector_abs_scalar(benchmark::State& state)
{
	vector4f v0 = vector_set(-1.0f, 1.0f, -2.0f, -123.134f);
	vector4f v1 = vector_set(-1.0f, 1.0f, -2.0f, -123.134f);
	vector4f v2 = vector_set(-1.0f, 1.0f, -2.0f, -123.134f);
	vector4f v3 = vector_set(-1.0f, 1.0f, -2.0f, -123.134f);
	vector4f v4 = vector_set(-1.0f, 1.0f, -2.0f, -123.134f);
	vector4f v5 = vector_set(-1.0f, 1.0f, -2.0f, -123.134f);
	vector4f v6 = vector_set(-1.0f, 1.0f, -2.0f, -123.134f);
	vector4f v7 = vector_set(-1.0f, 1.0f, -2.0f, -123.134f);

	for (auto _ : state)
	{
		v0 = vector_abs_scalar(v0);
		v1 = vector_abs_scalar(v1);
		v2 = vector_abs_scalar(v2);
		v3 = vector_abs_scalar(v3);
		v4 = vector_abs_scalar(v4);
		v5 = vector_abs_scalar(v5);
		v6 = vector_abs_scalar(v6);
		v7 = vector_abs_scalar(v7);
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

BENCHMARK(bm_vector_abs_scalar);

#if defined(RTM_SSE2_INTRINSICS)
// Wins on Ryzen 2990X desktop VS2017 x64 AVX
static void bm_vector_abs_sse2_maxsub(benchmark::State& state)
{
	vector4f v0 = vector_set(-1.0f, 1.0f, -2.0f, -123.134f);
	vector4f v1 = vector_set(-1.0f, 1.0f, -2.0f, -123.134f);
	vector4f v2 = vector_set(-1.0f, 1.0f, -2.0f, -123.134f);
	vector4f v3 = vector_set(-1.0f, 1.0f, -2.0f, -123.134f);
	vector4f v4 = vector_set(-1.0f, 1.0f, -2.0f, -123.134f);
	vector4f v5 = vector_set(-1.0f, 1.0f, -2.0f, -123.134f);
	vector4f v6 = vector_set(-1.0f, 1.0f, -2.0f, -123.134f);
	vector4f v7 = vector_set(-1.0f, 1.0f, -2.0f, -123.134f);

	for (auto _ : state)
	{
		v0 = vector_abs_sse2_maxsub(v0);
		v1 = vector_abs_sse2_maxsub(v1);
		v2 = vector_abs_sse2_maxsub(v2);
		v3 = vector_abs_sse2_maxsub(v3);
		v4 = vector_abs_sse2_maxsub(v4);
		v5 = vector_abs_sse2_maxsub(v5);
		v6 = vector_abs_sse2_maxsub(v6);
		v7 = vector_abs_sse2_maxsub(v7);
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

BENCHMARK(bm_vector_abs_sse2_maxsub);

static void bm_vector_abs_sse2_and(benchmark::State& state)
{
	vector4f v0 = vector_set(-1.0f, 1.0f, -2.0f, -123.134f);
	vector4f v1 = vector_set(-1.0f, 1.0f, -2.0f, -123.134f);
	vector4f v2 = vector_set(-1.0f, 1.0f, -2.0f, -123.134f);
	vector4f v3 = vector_set(-1.0f, 1.0f, -2.0f, -123.134f);
	vector4f v4 = vector_set(-1.0f, 1.0f, -2.0f, -123.134f);
	vector4f v5 = vector_set(-1.0f, 1.0f, -2.0f, -123.134f);
	vector4f v6 = vector_set(-1.0f, 1.0f, -2.0f, -123.134f);
	vector4f v7 = vector_set(-1.0f, 1.0f, -2.0f, -123.134f);

	for (auto _ : state)
	{
		v0 = vector_abs_sse2_and(v0);
		v1 = vector_abs_sse2_and(v1);
		v2 = vector_abs_sse2_and(v2);
		v3 = vector_abs_sse2_and(v3);
		v4 = vector_abs_sse2_and(v4);
		v5 = vector_abs_sse2_and(v5);
		v6 = vector_abs_sse2_and(v6);
		v7 = vector_abs_sse2_and(v7);
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

BENCHMARK(bm_vector_abs_sse2_and);
#endif
