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

#include <rtm/vector4d.h>

using namespace rtm;

RTM_FORCE_NOINLINE double RTM_SIMD_CALL vector_dot_scalar(vector4d_arg0 lhs, vector4d_arg1 rhs) RTM_NO_EXCEPT
{
	const double lhs_x = vector_get_x(lhs);
	const double lhs_y = vector_get_y(lhs);
	const double lhs_z = vector_get_z(lhs);
	const double lhs_w = vector_get_w(lhs);

	const double rhs_x = vector_get_x(rhs);
	const double rhs_y = vector_get_y(rhs);
	const double rhs_z = vector_get_z(rhs);
	const double rhs_w = vector_get_w(rhs);

	return (lhs_x * rhs_x) + (lhs_y * rhs_y) + (lhs_z * rhs_z) + (lhs_w * rhs_w);
}

// The current library implementation
RTM_FORCE_NOINLINE double RTM_SIMD_CALL vector_dot_current(vector4d_arg0 lhs, vector4d_arg1 rhs) RTM_NO_EXCEPT
{
	return vector_dot(lhs, rhs);
}

static void bm_vector4d_dot_scalar(benchmark::State& state)
{
	vector4d v0 = vector_set(-1.0, 1.0, -2.0, -123.134);
	vector4d v1 = vector_set(-1.021, 1.120, -2.0331, -1231.134);
	vector4d v2 = vector_set(-11.0, 1.330, -21.50, -1253.134);
	vector4d v3 = vector_set(0.15, 115.60, 21.0221, 123.13134);

	double sum0 = 0.0;
	double sum1 = 0.0;
	double sum2 = 0.0;
	double sum3 = 0.0;

	for (auto _ : state)
	{
		// The inputs do not change: hide them from the compiler, otherwise it can move the calls out of the loop
		benchmark::DoNotOptimize(v0);
		benchmark::DoNotOptimize(v1);
		benchmark::DoNotOptimize(v2);
		benchmark::DoNotOptimize(v3);

		sum0 += vector_dot_scalar(v0, v1);
		sum1 += vector_dot_scalar(v1, v2);
		sum2 += vector_dot_scalar(v2, v3);
		sum3 += vector_dot_scalar(v3, v0);
	}

	benchmark::DoNotOptimize(sum0);
	benchmark::DoNotOptimize(sum1);
	benchmark::DoNotOptimize(sum2);
	benchmark::DoNotOptimize(sum3);
}

BENCHMARK(bm_vector4d_dot_scalar);

static void bm_vector4d_dot_current(benchmark::State& state)
{
	vector4d v0 = vector_set(-1.0, 1.0, -2.0, -123.134);
	vector4d v1 = vector_set(-1.021, 1.120, -2.0331, -1231.134);
	vector4d v2 = vector_set(-11.0, 1.330, -21.50, -1253.134);
	vector4d v3 = vector_set(0.15, 115.60, 21.0221, 123.13134);

	double sum0 = 0.0;
	double sum1 = 0.0;
	double sum2 = 0.0;
	double sum3 = 0.0;

	for (auto _ : state)
	{
		// The inputs do not change: hide them from the compiler, otherwise it can move the calls out of the loop
		benchmark::DoNotOptimize(v0);
		benchmark::DoNotOptimize(v1);
		benchmark::DoNotOptimize(v2);
		benchmark::DoNotOptimize(v3);

		sum0 += vector_dot_current(v0, v1);
		sum1 += vector_dot_current(v1, v2);
		sum2 += vector_dot_current(v2, v3);
		sum3 += vector_dot_current(v3, v0);
	}

	benchmark::DoNotOptimize(sum0);
	benchmark::DoNotOptimize(sum1);
	benchmark::DoNotOptimize(sum2);
	benchmark::DoNotOptimize(sum3);
}

BENCHMARK(bm_vector4d_dot_current);
