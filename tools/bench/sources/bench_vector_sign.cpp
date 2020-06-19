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

// Wins on Ryzen 2990X desktop VS2017 x64 AVX
// Despite taking 5 instructions unlike sse2 which needs 2, this is consistently faster as well.
RTM_FORCE_NOINLINE vector4f RTM_SIMD_CALL vector_sign_ref(vector4f_arg0 input) RTM_NO_EXCEPT
{
	const mask4f mask = vector_greater_equal(input, vector_zero());
	return vector_select(mask, vector_set(1.0f), vector_set(-1.0f));
}

#if defined(RTM_SSE2_INTRINSICS)
// Wins on Haswell laptop x64 AVX
// Same performance as ref.
// Wins on Ryzen 2990X desktop clang9 x64 AVX
RTM_FORCE_NOINLINE vector4f RTM_SIMD_CALL vector_sign_sse2(vector4f_arg0 input) RTM_NO_EXCEPT
{
	constexpr __m128 signs = { -0.0f, -0.0f, -0.0f, -0.0f };
	constexpr __m128 one = { 1.0f, 1.0f, 1.0f, 1.0f };
	const __m128 sign_bits = _mm_and_ps(input, signs);	// Mask out the sign bit
	return _mm_or_ps(sign_bits, one);					// Copy the sign bit onto +-1.0f
}
#endif

static void bm_vector_sign_ref(benchmark::State& state)
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
		v0 = vector_sign_ref(v0);
		v1 = vector_sign_ref(v1);
		v2 = vector_sign_ref(v2);
		v3 = vector_sign_ref(v3);
		v4 = vector_sign_ref(v4);
		v5 = vector_sign_ref(v5);
		v6 = vector_sign_ref(v6);
		v7 = vector_sign_ref(v7);
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

BENCHMARK(bm_vector_sign_ref);

#if defined(RTM_SSE2_INTRINSICS)
static void bm_vector_sign_sse2(benchmark::State& state)
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
		v0 = vector_sign_sse2(v0);
		v1 = vector_sign_sse2(v1);
		v2 = vector_sign_sse2(v2);
		v3 = vector_sign_sse2(v3);
		v4 = vector_sign_sse2(v4);
		v5 = vector_sign_sse2(v5);
		v6 = vector_sign_sse2(v6);
		v7 = vector_sign_sse2(v7);
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

BENCHMARK(bm_vector_sign_sse2);
#endif
