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

#include <rtm/vqd.h>

using namespace rtm;

// The current library implementation
RTM_FORCE_NOINLINE vqd RTM_SIMD_CALL vq_mul_current(vqd_arg0 lhs, vqd_arg1 rhs) RTM_NO_EXCEPT
{
	return vq_mul(lhs, rhs);
}

static void bm_vqd_mul_current(benchmark::State& state)
{
	const quatd rotation = quat_from_euler(scalar_deg_to_rad(10.0), scalar_deg_to_rad(20.0), scalar_deg_to_rad(30.0));
	vqd t0 = vq_identity();
	vqd t1 = vq_set(rotation, vector_set(1.0, 2.0, 3.0));
	vqd t2 = vq_identity();
	vqd t3 = vq_set(rotation, vector_set(1.0, 2.0, 3.0));

	for (auto _ : state)
	{
		t0 = vq_mul_current(t0, t1);
		t2 = vq_mul_current(t2, t3);
	}

	benchmark::DoNotOptimize(t0);
	benchmark::DoNotOptimize(t1);
	benchmark::DoNotOptimize(t2);
	benchmark::DoNotOptimize(t3);
}

BENCHMARK(bm_vqd_mul_current);

static void bm_vqd_mul_current_rhs_chain(benchmark::State& state)
{
	const quatd rotation = quat_from_euler(scalar_deg_to_rad(10.0), scalar_deg_to_rad(20.0), scalar_deg_to_rad(30.0));
	vqd t0 = vq_identity();
	vqd t1 = vq_set(rotation, vector_set(1.0, 2.0, 3.0));
	vqd t2 = vq_identity();
	vqd t3 = vq_set(rotation, vector_set(1.0, 2.0, 3.0));

	for (auto _ : state)
	{
		// The result goes into the second argument, as with a parent transform in a hierarchy
		// This measures how the second argument is passed: by register or through memory
		t0 = vq_mul_current(t1, t0);
		t2 = vq_mul_current(t3, t2);
	}

	benchmark::DoNotOptimize(t0);
	benchmark::DoNotOptimize(t1);
	benchmark::DoNotOptimize(t2);
	benchmark::DoNotOptimize(t3);
}

BENCHMARK(bm_vqd_mul_current_rhs_chain);
