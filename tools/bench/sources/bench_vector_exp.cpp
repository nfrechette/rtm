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

#include <rtm/vector4f.h>
#include <rtm/vector4d.h>

#include <cmath>

using namespace rtm;

RTM_FORCE_NOINLINE vector4f RTM_SIMD_CALL vector_exp_std_f(vector4f_arg0 input) RTM_NO_EXCEPT
{
	const float x = std::exp(float(vector_get_x(input)));
	const float y = std::exp(float(vector_get_y(input)));
	const float z = std::exp(float(vector_get_z(input)));
	const float w = std::exp(float(vector_get_w(input)));
	return vector_set(x, y, z, w);
}

RTM_FORCE_NOINLINE vector4f RTM_SIMD_CALL vector_exp_rtm_f(vector4f_arg0 input) RTM_NO_EXCEPT
{
	return vector_exp(input);
}

RTM_FORCE_NOINLINE vector4d RTM_SIMD_CALL vector_exp_std_d(vector4d_arg0 input) RTM_NO_EXCEPT
{
	const double x = std::exp(double(vector_get_x(input)));
	const double y = std::exp(double(vector_get_y(input)));
	const double z = std::exp(double(vector_get_z(input)));
	const double w = std::exp(double(vector_get_w(input)));
	return vector_set(x, y, z, w);
}

RTM_FORCE_NOINLINE vector4d RTM_SIMD_CALL vector_exp_rtm_d(vector4d_arg0 input) RTM_NO_EXCEPT
{
	return vector_exp(input);
}

static void bm_vector_exp_std_f(benchmark::State& state)
{
	vector4f v0 = vector_set(0.25F);
	vector4f v1 = vector_set(-0.75F);
	vector4f v2 = vector_set(0.25F);
	vector4f v3 = vector_set(-0.75F);
	vector4f v4 = vector_set(0.25F);
	vector4f v5 = vector_set(-0.75F);
	vector4f v6 = vector_set(0.25F);
	vector4f v7 = vector_set(-0.75F);

	const vector4f offset = vector_set(1.5F);

	for (auto _ : state)
	{
		// The values stay near the fixed point of exp(v) - 1.5
		v0 = vector_sub(vector_exp_std_f(v0), offset);
		v1 = vector_sub(vector_exp_std_f(v1), offset);
		v2 = vector_sub(vector_exp_std_f(v2), offset);
		v3 = vector_sub(vector_exp_std_f(v3), offset);
		v4 = vector_sub(vector_exp_std_f(v4), offset);
		v5 = vector_sub(vector_exp_std_f(v5), offset);
		v6 = vector_sub(vector_exp_std_f(v6), offset);
		v7 = vector_sub(vector_exp_std_f(v7), offset);
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

BENCHMARK(bm_vector_exp_std_f);

static void bm_vector_exp_rtm_f(benchmark::State& state)
{
	vector4f v0 = vector_set(0.25F);
	vector4f v1 = vector_set(-0.75F);
	vector4f v2 = vector_set(0.25F);
	vector4f v3 = vector_set(-0.75F);
	vector4f v4 = vector_set(0.25F);
	vector4f v5 = vector_set(-0.75F);
	vector4f v6 = vector_set(0.25F);
	vector4f v7 = vector_set(-0.75F);

	const vector4f offset = vector_set(1.5F);

	for (auto _ : state)
	{
		// The values stay near the fixed point of exp(v) - 1.5
		v0 = vector_sub(vector_exp_rtm_f(v0), offset);
		v1 = vector_sub(vector_exp_rtm_f(v1), offset);
		v2 = vector_sub(vector_exp_rtm_f(v2), offset);
		v3 = vector_sub(vector_exp_rtm_f(v3), offset);
		v4 = vector_sub(vector_exp_rtm_f(v4), offset);
		v5 = vector_sub(vector_exp_rtm_f(v5), offset);
		v6 = vector_sub(vector_exp_rtm_f(v6), offset);
		v7 = vector_sub(vector_exp_rtm_f(v7), offset);
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

BENCHMARK(bm_vector_exp_rtm_f);

static void bm_vector_exp_std_d(benchmark::State& state)
{
	vector4d v0 = vector_set(0.25);
	vector4d v1 = vector_set(-0.75);
	vector4d v2 = vector_set(0.25);
	vector4d v3 = vector_set(-0.75);
	vector4d v4 = vector_set(0.25);
	vector4d v5 = vector_set(-0.75);
	vector4d v6 = vector_set(0.25);
	vector4d v7 = vector_set(-0.75);

	const vector4d offset = vector_set(1.5);

	for (auto _ : state)
	{
		// The values stay near the fixed point of exp(v) - 1.5
		v0 = vector_sub(vector_exp_std_d(v0), offset);
		v1 = vector_sub(vector_exp_std_d(v1), offset);
		v2 = vector_sub(vector_exp_std_d(v2), offset);
		v3 = vector_sub(vector_exp_std_d(v3), offset);
		v4 = vector_sub(vector_exp_std_d(v4), offset);
		v5 = vector_sub(vector_exp_std_d(v5), offset);
		v6 = vector_sub(vector_exp_std_d(v6), offset);
		v7 = vector_sub(vector_exp_std_d(v7), offset);
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

BENCHMARK(bm_vector_exp_std_d);

static void bm_vector_exp_rtm_d(benchmark::State& state)
{
	vector4d v0 = vector_set(0.25);
	vector4d v1 = vector_set(-0.75);
	vector4d v2 = vector_set(0.25);
	vector4d v3 = vector_set(-0.75);
	vector4d v4 = vector_set(0.25);
	vector4d v5 = vector_set(-0.75);
	vector4d v6 = vector_set(0.25);
	vector4d v7 = vector_set(-0.75);

	const vector4d offset = vector_set(1.5);

	for (auto _ : state)
	{
		// The values stay near the fixed point of exp(v) - 1.5
		v0 = vector_sub(vector_exp_rtm_d(v0), offset);
		v1 = vector_sub(vector_exp_rtm_d(v1), offset);
		v2 = vector_sub(vector_exp_rtm_d(v2), offset);
		v3 = vector_sub(vector_exp_rtm_d(v3), offset);
		v4 = vector_sub(vector_exp_rtm_d(v4), offset);
		v5 = vector_sub(vector_exp_rtm_d(v5), offset);
		v6 = vector_sub(vector_exp_rtm_d(v6), offset);
		v7 = vector_sub(vector_exp_rtm_d(v7), offset);
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

BENCHMARK(bm_vector_exp_rtm_d);
