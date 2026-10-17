////////////////////////////////////////////////////////////////////////////////
// The MIT License (MIT)
//
// Copyright (c) 2026 Nicholas Frechette & Realtime Math contributors
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

#include <rtm/vqvd.h>

using namespace rtm;

// The current library implementation
RTM_FORCE_NOINLINE vqvd RTM_SIMD_CALL vqv_mul_current(vqvd_arg0 lhs, vqvd_arg1 rhs) RTM_NO_EXCEPT
{
	return vqv_mul(lhs, rhs);
}

static void bm_vqvd_mul_current(benchmark::State& state)
{
	// The scale is positive, thus vqv_mul does not use the matrix code path
	const quatd rotation = quat_from_euler(scalar_deg_to_rad(10.0), scalar_deg_to_rad(20.0), scalar_deg_to_rad(30.0));
	vqvd t0 = vqv_identity();
	vqvd t1 = vqv_set(rotation, vector_set(1.0, 2.0, 3.0), vector_set(1.0));
	vqvd t2 = vqv_identity();
	vqvd t3 = vqv_set(rotation, vector_set(1.0, 2.0, 3.0), vector_set(1.0));

	for (auto _ : state)
	{
		t0 = vqv_mul_current(t0, t1);
		t2 = vqv_mul_current(t2, t3);
	}

	benchmark::DoNotOptimize(t0);
	benchmark::DoNotOptimize(t1);
	benchmark::DoNotOptimize(t2);
	benchmark::DoNotOptimize(t3);
}

BENCHMARK(bm_vqvd_mul_current);
