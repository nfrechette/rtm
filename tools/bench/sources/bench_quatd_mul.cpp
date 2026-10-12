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

#include <rtm/quatd.h>

using namespace rtm;

RTM_FORCE_NOINLINE quatd RTM_SIMD_CALL quat_mul_scalar(quatd_arg0 lhs, quatd_arg1 rhs) RTM_NO_EXCEPT
{
	const double lhs_x = quat_get_x(lhs);
	const double lhs_y = quat_get_y(lhs);
	const double lhs_z = quat_get_z(lhs);
	const double lhs_w = quat_get_w(lhs);

	const double rhs_x = quat_get_x(rhs);
	const double rhs_y = quat_get_y(rhs);
	const double rhs_z = quat_get_z(rhs);
	const double rhs_w = quat_get_w(rhs);

	const double x = (rhs_w * lhs_x) + (rhs_x * lhs_w) + (rhs_y * lhs_z) - (rhs_z * lhs_y);
	const double y = (rhs_w * lhs_y) - (rhs_x * lhs_z) + (rhs_y * lhs_w) + (rhs_z * lhs_x);
	const double z = (rhs_w * lhs_z) + (rhs_x * lhs_y) - (rhs_y * lhs_x) + (rhs_z * lhs_w);
	const double w = (rhs_w * lhs_w) - (rhs_x * lhs_x) - (rhs_y * lhs_y) - (rhs_z * lhs_z);

	return quat_set(x, y, z, w);
}

// The current library implementation
RTM_FORCE_NOINLINE quatd RTM_SIMD_CALL quat_mul_current(quatd_arg0 lhs, quatd_arg1 rhs) RTM_NO_EXCEPT
{
	return quat_mul(lhs, rhs);
}

static void bm_quatd_mul_scalar(benchmark::State& state)
{
	quatd q0 = quat_identity();
	quatd q1 = quat_identity();
	quatd q2 = quat_identity();
	quatd q3 = quat_identity();

	for (auto _ : state)
	{
		q0 = quat_mul_scalar(q0, q0);
		q1 = quat_mul_scalar(q1, q1);
		q2 = quat_mul_scalar(q2, q2);
		q3 = quat_mul_scalar(q3, q3);
	}

	benchmark::DoNotOptimize(q0);
	benchmark::DoNotOptimize(q1);
	benchmark::DoNotOptimize(q2);
	benchmark::DoNotOptimize(q3);
}

BENCHMARK(bm_quatd_mul_scalar);

static void bm_quatd_mul_current(benchmark::State& state)
{
	quatd q0 = quat_identity();
	quatd q1 = quat_identity();
	quatd q2 = quat_identity();
	quatd q3 = quat_identity();

	for (auto _ : state)
	{
		q0 = quat_mul_current(q0, q0);
		q1 = quat_mul_current(q1, q1);
		q2 = quat_mul_current(q2, q2);
		q3 = quat_mul_current(q3, q3);
	}

	benchmark::DoNotOptimize(q0);
	benchmark::DoNotOptimize(q1);
	benchmark::DoNotOptimize(q2);
	benchmark::DoNotOptimize(q3);
}

BENCHMARK(bm_quatd_mul_current);
