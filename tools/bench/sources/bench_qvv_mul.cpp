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

#include <rtm/qvvf.h>

using namespace rtm;

RTM_FORCE_NOINLINE qvvf RTM_SIMD_CALL qvv_mul_ref(qvvf_arg0 lhs, qvvf_arg1 rhs) RTM_NO_EXCEPT
{
	const vector4f min_scale = vector_min(lhs.scale, rhs.scale);
	const vector4f scale = vector_mul(lhs.scale, rhs.scale);

	if (vector_any_less_than3(min_scale, vector_zero()))
	{
		// If we have negative scale, we go through a matrix
		const matrix3x4f lhs_mtx = matrix_from_qvv(lhs);
		const matrix3x4f rhs_mtx = matrix_from_qvv(rhs);
		matrix3x4f result_mtx = matrix_mul(lhs_mtx, rhs_mtx);
		result_mtx = matrix_remove_scale(result_mtx);

		const vector4f sign = vector_sign(scale);
		result_mtx.x_axis = vector_mul(result_mtx.x_axis, vector_dup_x(sign));
		result_mtx.y_axis = vector_mul(result_mtx.y_axis, vector_dup_y(sign));
		result_mtx.z_axis = vector_mul(result_mtx.z_axis, vector_dup_z(sign));

		const quatf rotation = quat_from_matrix(result_mtx);
		const vector4f translation = result_mtx.w_axis;
		return qvv_set(rotation, translation, scale);
	}
	else
	{
		const quatf rotation = quat_mul(lhs.rotation, rhs.rotation);
		const vector4f translation = vector_add(quat_mul_vector3(vector_mul(lhs.translation, rhs.scale), rhs.rotation), rhs.translation);
		return qvv_set(rotation, translation, scale);
	}
}

#if defined(RTM_SSE2_INTRINSICS)
// Wins on Haswell laptop x64 AVX
// Wins on Ryzen 2990X desktop VS2017 x64 AVX
// Wins on Ryzen 2990X desktop clang9 x64 AVX
RTM_FORCE_NOINLINE qvvf RTM_SIMD_CALL qvv_mul_sse2(qvvf_arg0 lhs, qvvf_arg1 rhs) RTM_NO_EXCEPT
{
	const vector4f min_scale = vector_min(lhs.scale, rhs.scale);
	const vector4f scale = vector_mul(lhs.scale, rhs.scale);

	if (vector_any_less_than3(min_scale, vector_zero()))
	{
		// If we have negative scale, we go through a matrix
		const matrix3x4f lhs_mtx = matrix_from_qvv(lhs);
		const matrix3x4f rhs_mtx = matrix_from_qvv(rhs);
		matrix3x4f result_mtx = matrix_mul(lhs_mtx, rhs_mtx);
		result_mtx = matrix_remove_scale(result_mtx);

		constexpr __m128 signs = { -0.0f, -0.0f, -0.0f, -0.0f };
		const __m128 sign_bits = _mm_and_ps(scale, signs);	// Mask out the sign bit

		result_mtx.x_axis = _mm_xor_ps(result_mtx.x_axis, _mm_shuffle_ps(sign_bits, sign_bits, _MM_SHUFFLE(0, 0, 0, 0)));
		result_mtx.y_axis = _mm_xor_ps(result_mtx.y_axis, _mm_shuffle_ps(sign_bits, sign_bits, _MM_SHUFFLE(1, 1, 1, 1)));
		result_mtx.z_axis = _mm_xor_ps(result_mtx.z_axis, _mm_shuffle_ps(sign_bits, sign_bits, _MM_SHUFFLE(2, 2, 2, 2)));

		const quatf rotation = quat_from_matrix(result_mtx);
		const vector4f translation = result_mtx.w_axis;
		return qvv_set(rotation, translation, scale);
	}
	else
	{
		const quatf rotation = quat_mul(lhs.rotation, rhs.rotation);
		const vector4f translation = vector_add(quat_mul_vector3(vector_mul(lhs.translation, rhs.scale), rhs.rotation), rhs.translation);
		return qvv_set(rotation, translation, scale);
	}
}
#endif

static void bm_qvv_mul_ref(benchmark::State& state)
{
	qvvf t0 = qvv_identity();
	qvvf t1 = qvv_set(quat_identity(), vector_zero(), vector_set(-1.0f));
	qvvf t2 = qvv_identity();
	qvvf t3 = qvv_set(quat_identity(), vector_zero(), vector_set(-1.0f));

	for (auto _ : state)
	{
		t0 = qvv_mul_ref(t0, t1);
		t2 = qvv_mul_ref(t2, t3);
	}

	benchmark::DoNotOptimize(t0);
	benchmark::DoNotOptimize(t1);
	benchmark::DoNotOptimize(t2);
	benchmark::DoNotOptimize(t3);
}

BENCHMARK(bm_qvv_mul_ref);

#if defined(RTM_SSE2_INTRINSICS)
static void bm_qvv_mul_sse2(benchmark::State& state)
{
	qvvf t0 = qvv_identity();
	qvvf t1 = qvv_set(quat_identity(), vector_zero(), vector_set(-1.0f));
	qvvf t2 = qvv_identity();
	qvvf t3 = qvv_set(quat_identity(), vector_zero(), vector_set(-1.0f));

	for (auto _ : state)
	{
		t0 = qvv_mul_sse2(t0, t1);
		t2 = qvv_mul_sse2(t2, t3);
	}

	benchmark::DoNotOptimize(t0);
	benchmark::DoNotOptimize(t1);
	benchmark::DoNotOptimize(t2);
	benchmark::DoNotOptimize(t3);
}

BENCHMARK(bm_qvv_mul_sse2);
#endif
