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

inline vector4f RTM_SIMD_CALL vector_sign_ref(vector4f_arg0 input) RTM_NO_EXCEPT
{
	const mask4i mask = vector_greater_equal(input, vector_zero());
	return vector_select(mask, vector_set(1.0f), vector_set(-1.0f));
}

#if defined(RTM_SSE2_INTRINSICS)
inline vector4f RTM_SIMD_CALL vector_sign_sse2(vector4f_arg0 input) RTM_NO_EXCEPT
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

	for (auto _ : state)
		benchmark::DoNotOptimize(v0 = vector_sign_ref(v0));
}

BENCHMARK(bm_vector_sign_ref);

#if defined(RTM_SSE2_INTRINSICS)
static void bm_vector_sign_sse2(benchmark::State& state)
{
	vector4f v0 = vector_set(-1.0f, 1.0f, -2.0f, -123.134f);

	for (auto _ : state)
		benchmark::DoNotOptimize(v0 = vector_sign_sse2(v0));
}

BENCHMARK(bm_vector_sign_sse2);
#endif
