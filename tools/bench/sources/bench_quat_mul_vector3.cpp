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

inline vector4f RTM_SIMD_CALL quat_mul_vector3_ref(vector4f_arg0 vector, quatf_arg1 rotation) RTM_NO_EXCEPT
{
	quatf vector_quat = quat_set_w(vector_to_quat(vector), 0.0f);
	quatf inv_rotation = quat_conjugate(rotation);
	return quat_to_vector(quat_mul(quat_mul(inv_rotation, vector_quat), rotation));
}

#if defined(RTM_FMA_INTRINSICS)
inline vector4f RTM_SIMD_CALL quat_mul_vector3_fma(vector4f_arg0 vector, quatf_arg1 rotation) RTM_NO_EXCEPT
{
	const __m128 inv_rotation = quat_conjugate(rotation);

	// Normally when we multiply our inverse rotation quaternion with the input vector as a quaternion with W = 0.0.
	// As a result, we can strip the whole part that uses W saving a few instructions.
	// Because we have the rotation and its inverse, we also can use them to avoid flipping the signs
	// when lining up our SIMD additions. For the first quaternion multiplication, we can avoid 3 XORs by
	// doing 2 shuffles instead. The same trick can also be used with the second quaternion multiplication.
	// We also don't care about the result W lane but it comes for free.

	// temp = quat_mul(inv_rotation, vector_quat)
	__m128 temp;
	{
		const __m128 rotation_tmp0 = _mm_shuffle_ps(rotation, inv_rotation, _MM_SHUFFLE(3, 0, 2, 1));		// r.y, r.z, -r.x, -r.w
		const __m128 rotation_tmp1 = _mm_shuffle_ps(rotation, inv_rotation, _MM_SHUFFLE(3, 1, 2, 0));		// r.x, r.z, -r.y, -r.w

		const __m128 v_xxxx = _mm_shuffle_ps(vector, vector, _MM_SHUFFLE(0, 0, 0, 0));
		const __m128 v_yyyy = _mm_shuffle_ps(vector, vector, _MM_SHUFFLE(1, 1, 1, 1));
		const __m128 v_zzzz = _mm_shuffle_ps(vector, vector, _MM_SHUFFLE(2, 2, 2, 2));

		const __m128 rotation_tmp2 = _mm_shuffle_ps(rotation_tmp0, rotation_tmp1, _MM_SHUFFLE(0, 2, 1, 3));	// -r.w, r.z, -r.y, r.x
		const __m128 lwrx_lzrx_lyrx_lxrx = _mm_mul_ps(v_xxxx, rotation_tmp2);

		const __m128 rotation_tmp3 = _mm_shuffle_ps(inv_rotation, rotation, _MM_SHUFFLE(1, 0, 3, 2));		// -r.z, -r.w, r.x, r.y
		const __m128 rotation_tmp4 = _mm_shuffle_ps(rotation_tmp0, rotation_tmp1, _MM_SHUFFLE(1, 3, 2, 0));	// r.y, -r.x, -r.w, r.z

		temp = _mm_fmadd_ps(v_zzzz, rotation_tmp4, _mm_fmadd_ps(v_yyyy, rotation_tmp3, lwrx_lzrx_lyrx_lxrx));
	}

	// result = quat_mul(temp, rotation)
	{
		const __m128 rotation_tmp0 = _mm_shuffle_ps(rotation, inv_rotation, _MM_SHUFFLE(2, 0, 2, 0));		// r.x, r.z, -r.x, -r.z

		__m128 r_xxxx = _mm_shuffle_ps(rotation_tmp0, rotation_tmp0, _MM_SHUFFLE(2, 0, 2, 0));				// r.x, -r.x, r.x, -r.x
		__m128 r_yyyy = _mm_shuffle_ps(rotation, inv_rotation, _MM_SHUFFLE(1, 1, 1, 1));					// r.y, r.y, -r.y, -r.y
		__m128 r_zzzz = _mm_shuffle_ps(rotation_tmp0, rotation_tmp0, _MM_SHUFFLE(3, 1, 1, 3));				// -r.z, r.z, r.z, -r.z
		__m128 r_wwww = _mm_shuffle_ps(rotation, rotation, _MM_SHUFFLE(3, 3, 3, 3));						// r.w, r.w, r.w, r.w

		__m128 lxrw_lyrw_lzrw_lwrw = _mm_mul_ps(r_wwww, temp);

		__m128 t_wzyx = _mm_shuffle_ps(temp, temp, _MM_SHUFFLE(0, 1, 2, 3));
		__m128 t_zwxy = _mm_shuffle_ps(t_wzyx, t_wzyx, _MM_SHUFFLE(2, 3, 0, 1));
		__m128 t_yxwz = _mm_shuffle_ps(t_zwxy, t_zwxy, _MM_SHUFFLE(0, 1, 2, 3));

		__m128 result0 = _mm_fmadd_ps(r_xxxx, t_wzyx, lxrw_lyrw_lzrw_lwrw);
		__m128 result1 = _mm_fmadd_ps(r_yyyy, t_zwxy, result0);
		return _mm_fmadd_ps(r_zzzz, t_yxwz, result1);
	}
}
#endif

#if defined(RTM_SSE2_INTRINSICS)
inline vector4f RTM_SIMD_CALL quat_mul_vector3_sse2(vector4f_arg0 vector, quatf_arg1 rotation) RTM_NO_EXCEPT
{
	const __m128 inv_rotation = quat_conjugate(rotation);

	// Normally when we multiply our inverse rotation quaternion with the input vector as a quaternion with W = 0.0.
	// As a result, we can strip the whole part that uses W saving a few instructions.
	// Because we have the rotation and its inverse, we also can use them to avoid flipping the signs
	// when lining up our SIMD additions. For the first quaternion multiplication, we can avoid 3 XORs by
	// doing 2 shuffles instead. The same trick can also be used with the second quaternion multiplication.
	// We also don't care about the result W lane but it comes for free.

	// temp = quat_mul(inv_rotation, vector_quat)
	__m128 temp;
	{
		const __m128 rotation_tmp0 = _mm_shuffle_ps(rotation, inv_rotation, _MM_SHUFFLE(3, 0, 2, 1));		// r.y, r.z, -r.x, -r.w
		const __m128 rotation_tmp1 = _mm_shuffle_ps(rotation, inv_rotation, _MM_SHUFFLE(3, 1, 2, 0));		// r.x, r.z, -r.y, -r.w

		const __m128 v_xxxx = _mm_shuffle_ps(vector, vector, _MM_SHUFFLE(0, 0, 0, 0));
		const __m128 v_yyyy = _mm_shuffle_ps(vector, vector, _MM_SHUFFLE(1, 1, 1, 1));
		const __m128 v_zzzz = _mm_shuffle_ps(vector, vector, _MM_SHUFFLE(2, 2, 2, 2));

		const __m128 rotation_tmp2 = _mm_shuffle_ps(rotation_tmp0, rotation_tmp1, _MM_SHUFFLE(0, 2, 1, 3));	// -r.w, r.z, -r.y, r.x
		const __m128 lwrx_lzrx_lyrx_lxrx = _mm_mul_ps(v_xxxx, rotation_tmp2);

		const __m128 rotation_tmp3 = _mm_shuffle_ps(inv_rotation, rotation, _MM_SHUFFLE(1, 0, 3, 2));		// -r.z, -r.w, r.x, r.y
		const __m128 lzry_lwry_lxry_lyry = _mm_mul_ps(v_yyyy, rotation_tmp3);

		const __m128 rotation_tmp4 = _mm_shuffle_ps(rotation_tmp0, rotation_tmp1, _MM_SHUFFLE(1, 3, 2, 0));	// r.y, -r.x, -r.w, r.z
		const __m128 lyrz_lxrz_lwrz_lzrz = _mm_mul_ps(v_zzzz, rotation_tmp4);

		temp = _mm_add_ps(_mm_add_ps(lwrx_lzrx_lyrx_lxrx, lzry_lwry_lxry_lyry), lyrz_lxrz_lwrz_lzrz);
	}

	// result = quat_mul(temp, rotation)
	{
		const __m128 rotation_tmp0 = _mm_shuffle_ps(rotation, inv_rotation, _MM_SHUFFLE(2, 0, 2, 0));		// r.x, r.z, -r.x, -r.z

		__m128 r_xxxx = _mm_shuffle_ps(rotation_tmp0, rotation_tmp0, _MM_SHUFFLE(2, 0, 2, 0));				// r.x, -r.x, r.x, -r.x
		__m128 r_yyyy = _mm_shuffle_ps(rotation, inv_rotation, _MM_SHUFFLE(1, 1, 1, 1));					// r.y, r.y, -r.y, -r.y
		__m128 r_zzzz = _mm_shuffle_ps(rotation_tmp0, rotation_tmp0, _MM_SHUFFLE(3, 1, 1, 3));				// -r.z, r.z, r.z, -r.z
		__m128 r_wwww = _mm_shuffle_ps(rotation, rotation, _MM_SHUFFLE(3, 3, 3, 3));						// r.w, r.w, r.w, r.w

		__m128 lxrw_lyrw_lzrw_lwrw = _mm_mul_ps(r_wwww, temp);

		__m128 t_wzyx = _mm_shuffle_ps(temp, temp, _MM_SHUFFLE(0, 1, 2, 3));
		__m128 lwrx_lzrx_lyrx_lxrx = _mm_mul_ps(r_xxxx, t_wzyx);

		__m128 t_zwxy = _mm_shuffle_ps(t_wzyx, t_wzyx, _MM_SHUFFLE(2, 3, 0, 1));
		__m128 lzry_lwry_lxry_lyry = _mm_mul_ps(r_yyyy, t_zwxy);

		__m128 t_yxwz = _mm_shuffle_ps(t_zwxy, t_zwxy, _MM_SHUFFLE(0, 1, 2, 3));
		__m128 lyrz_lxrz_lwrz_lzrz = _mm_mul_ps(r_zzzz, t_yxwz);

		__m128 result0 = _mm_add_ps(lxrw_lyrw_lzrw_lwrw, lwrx_lzrx_lyrx_lxrx);
		__m128 result1 = _mm_add_ps(lzry_lwry_lxry_lyry, lyrz_lxrz_lwrz_lzrz);
		return _mm_add_ps(result0, result1);
	}
}
#endif

#if defined(RTM_NEON_INTRINSICS)
inline vector4f RTM_SIMD_CALL quat_mul_vector3_neon_scalar(vector4f_arg0 vector, quatf_arg1 rotation) RTM_NO_EXCEPT
{
	const float32x4_t n_rotation = vnegq_f32(rotation);

	// temp = quat_mul(inv_rotation, vector_quat)
	float temp_x;
	float temp_y;
	float temp_z;
	float temp_w;
	{
		const float lhs_x = quat_get_x(n_rotation);
		const float lhs_y = quat_get_y(n_rotation);
		const float lhs_z = quat_get_z(n_rotation);
		const float lhs_w = quat_get_w(rotation);

		const float rhs_x = quat_get_x(vector);
		const float rhs_y = quat_get_y(vector);
		const float rhs_z = quat_get_z(vector);

		temp_x = (rhs_x * lhs_w) + (rhs_y * lhs_z) - (rhs_z * lhs_y);
		temp_y =  -(rhs_x * lhs_z) + (rhs_y * lhs_w) + (rhs_z * lhs_x);
		temp_z = (rhs_x * lhs_y) - (rhs_y * lhs_x) + (rhs_z * lhs_w);
		temp_w =  -(rhs_x * lhs_x) - (rhs_y * lhs_y) - (rhs_z * lhs_z);
	}

	// result = quat_mul(temp, rotation)
	{
		const float lhs_x = temp_x;
		const float lhs_y = temp_y;
		const float lhs_z = temp_z;
		const float lhs_w = temp_w;

		const float rhs_x = quat_get_x(rotation);
		const float rhs_y = quat_get_y(rotation);
		const float rhs_z = quat_get_z(rotation);
		const float rhs_w = quat_get_w(rotation);

		const float x = (rhs_w * lhs_x) + (rhs_x * lhs_w) + (rhs_y * lhs_z) - (rhs_z * lhs_y);
		const float y = (rhs_w * lhs_y) - (rhs_x * lhs_z) + (rhs_y * lhs_w) + (rhs_z * lhs_x);
		const float z = (rhs_w * lhs_z) + (rhs_x * lhs_y) - (rhs_y * lhs_x) + (rhs_z * lhs_w);

		return vector_set(x, y, z, z);
	}
}

inline vector4f RTM_SIMD_CALL quat_mul_vector3_neon64(vector4f_arg0 vector, quatf_arg1 rotation) RTM_NO_EXCEPT
{
	const float32x4_t n_rotation = vnegq_f32(rotation);

	// Normally when we multiply our inverse rotation quaternion with the input vector as a quaternion with W = 0.0.
	// As a result, we can strip the whole part that uses W saving a few instructions.
	// We calculate the negation of the input rotation in order to avoid multiplying with a control value to flip signs.
	// This takes a few more instructions but they are cheaper than floating point operations on ARM64.
	// We also don't care about the result W lane but it comes for free.

	// temp = quat_mul(inv_rotation, vector_quat)
	float32x4_t temp;
	{
		const float32x4_t r_zwxy = vrev64q_f32(rotation);													// r.z, r.w, r.x, r.y
		const float32x4_t nr_zwxy = vrev64q_f32(n_rotation);												// -r.z, -r.w, -r.x, -r.y
		const float32x2x2_t tmp1 = vzip_f32(vget_low_f32(r_zwxy), vget_high_f32(r_zwxy));					// r.z, r.x, r.w, r.y
		const float32x2x2_t tmp2 = vzip_f32(vget_low_f32(nr_zwxy), vget_high_f32(nr_zwxy));					// -r.z, -r.x, -r.w, -r.y
		const float32x2x2_t tmp3 = vzip_f32(tmp2.val[1], tmp1.val[0]);										// -r.w, r.z, -r.y, r.x
		const float32x2x2_t tmp4 = vzip_f32(tmp1.val[1], tmp2.val[0]);										// r.w, -r.z, r.y, -r.x

		const float32x4_t l_wzyx = vcombine_f32(tmp3.val[0], tmp3.val[1]);									// -r.w, r.z, -r.y, r.x
		const float32x4_t l_zwxy = vcombine_f32(vget_high_f32(n_rotation), vget_low_f32(rotation));			// -r.z, -r.w, r.x, r.y
		const float32x4_t l_yxwz = vcombine_f32(tmp4.val[1], tmp3.val[0]);									// r.y, -r.x, -r.w, r.z

		const float32x2_t r_xy = vget_low_f32(vector);
		const float32x2_t r_zw = vget_high_f32(vector);

		const float32x4_t lwrx_lzrx_lyrx_lxrx = vmulq_lane_f32(l_wzyx, r_xy, 0);

#if defined(RTM_NEON64_INTRINSICS)
		const float32x4_t result1 = vfmaq_lane_f32(lwrx_lzrx_lyrx_lxrx, l_zwxy, r_xy, 1);
		temp = vfmaq_lane_f32(result1, l_yxwz, r_zw, 0);
#else
		const float32x4_t result1 = vmlaq_lane_f32(lwrx_lzrx_lyrx_lxrx, l_zwxy, r_xy, 1);
		temp = vmlaq_lane_f32(result1, l_yxwz, r_zw, 0);
#endif
	}

	// result = quat_mul(temp, rotation)
	{
		const float32x4_t neg_temp = vnegq_f32(temp);														// -t.x, -t.y, -t.z, -t.w
		const float32x4_t t_zwxy = vrev64q_f32(temp);														// t.z, t.w, t.x, t.y
		const float32x4_t nt_zwxy = vrev64q_f32(neg_temp);													// -t.z, -t.w, -t.x, -t.y

		const float32x2x2_t tmp1 = vzip_f32(vget_low_f32(t_zwxy), vget_high_f32(t_zwxy));					// t.z, t.x, t.w, t.y
		const float32x2x2_t tmp2 = vzip_f32(vget_low_f32(nt_zwxy), vget_high_f32(nt_zwxy));					// -t.z, -t.x, -t.w, -t.y
		const float32x2x2_t tmp3 = vzip_f32(tmp2.val[1], tmp1.val[0]);										// -t.w, t.z, -t.y, t.x
		const float32x2x2_t tmp4 = vzip_f32(tmp1.val[1], tmp2.val[0]);										// t.w, -t.z, t.y, -t.x

		const float32x4_t l_wzyx = vcombine_f32(tmp3.val[0], tmp3.val[1]);									// -t.w, t.z, -t.y, t.x
		const float32x4_t l_zwxy = vcombine_f32(vget_high_f32(neg_temp), vget_low_f32(temp));				// -t.z, -t.w, t.x, t.y
		const float32x4_t l_yxwz = vcombine_f32(tmp4.val[1], tmp3.val[0]);									// t.y, -t.x, -t.w, t.z

		const float32x2_t r_xy = vget_low_f32(rotation);
		const float32x2_t r_zw = vget_high_f32(rotation);

		const float32x4_t lxrw_lyrw_lzrw_lwrw = vmulq_lane_f32(temp, r_zw, 1);

#if defined(RTM_NEON64_INTRINSICS)
		const float32x4_t result0 = vfmaq_lane_f32(lxrw_lyrw_lzrw_lwrw, l_wzyx, r_xy, 0);
		const float32x4_t result1 = vfmaq_lane_f32(result0, l_zwxy, r_xy, 1);
		return vfmaq_lane_f32(result1, l_yxwz, r_zw, 0);
#else
		const float32x4_t result0 = vmlaq_lane_f32(lxrw_lyrw_lzrw_lwrw, l_wzyx, r_xy, 0);
		const float32x4_t result1 = vmlaq_lane_f32(result0, l_zwxy, r_xy, 1);
		return vmlaq_lane_f32(result1, l_yxwz, r_zw, 0);
#endif
	}
}

inline vector4f RTM_SIMD_CALL quat_mul_vector3_neon(vector4f_arg0 vector, quatf_arg1 rotation) RTM_NO_EXCEPT
{
	alignas(16) constexpr float control_wzyx_f[4] = { 1.0f, -1.0f, 1.0f, -1.0f };
	alignas(16) constexpr float control_zwxy_f[4] = { 1.0f, 1.0f, -1.0f, -1.0f };
	alignas(16) constexpr float control_yxwz_f[4] = { -1.0f, 1.0f, 1.0f, -1.0f };

	const float32x4_t control_wzyx = *reinterpret_cast<const float32x4_t*>(&control_wzyx_f[0]);
	const float32x4_t control_zwxy = *reinterpret_cast<const float32x4_t*>(&control_zwxy_f[0]);
	const float32x4_t control_yxwz = *reinterpret_cast<const float32x4_t*>(&control_yxwz_f[0]);

	const float32x4_t inv_rotation = quat_conjugate(rotation);

	// Normally when we multiply our inverse rotation quaternion with the input vector as a quaternion with W = 0.0.
	// As a result, we can strip the whole part that uses W saving a few instructions.
	// We also don't care about the result W lane but it comes for free.
	// With ARMv7, it seems that floating point instructions aren't the bottleneck and using shuffles to move things
	// around to take advantage of sign flipping like the other implementations does not benefit.

	// temp = quat_mul(inv_rotation, vector_quat)
	float32x4_t temp;
	{
		const float32x2_t r_xy = vget_low_f32(vector);
		const float32x2_t r_zw = vget_high_f32(vector);

		const float32x4_t l_yxwz = vrev64q_f32(inv_rotation);
		const float32x4_t l_wzyx = vcombine_f32(vget_high_f32(l_yxwz), vget_low_f32(l_yxwz));
		const float32x4_t l_zwxy = vrev64q_f32(l_wzyx);

		const float32x4_t lwrx_lzrx_lyrx_lxrx = vmulq_lane_f32(l_wzyx, r_xy, 0);
		const float32x4_t lzry_lwry_lxry_lyry = vmulq_lane_f32(l_zwxy, r_xy, 1);
		const float32x4_t lyrz_lxrz_lwrz_lzrz = vmulq_lane_f32(l_yxwz, r_zw, 0);

		const float32x4_t result0 = vmulq_f32(lwrx_lzrx_lyrx_lxrx, control_wzyx);

#if defined(RTM_NEON64_INTRINSICS)
		const float32x4_t result1 = vfmaq_f32(result0, lzry_lwry_lxry_lyry, control_zwxy);
		temp = vfmaq_f32(result1, lyrz_lxrz_lwrz_lzrz, control_yxwz);
#else
		const float32x4_t result1 = vmlaq_f32(result0, lzry_lwry_lxry_lyry, control_zwxy);
		temp = vmlaq_f32(result1, lyrz_lxrz_lwrz_lzrz, control_yxwz);
#endif
	}

	// result = quat_mul(temp, rotation)
	{
		const float32x2_t r_xy = vget_low_f32(rotation);
		const float32x2_t r_zw = vget_high_f32(rotation);

		const float32x4_t lxrw_lyrw_lzrw_lwrw = vmulq_lane_f32(temp, r_zw, 1);

		const float32x4_t l_yxwz = vrev64q_f32(temp);
		const float32x4_t l_wzyx = vcombine_f32(vget_high_f32(l_yxwz), vget_low_f32(l_yxwz));
		const float32x4_t l_zwxy = vrev64q_f32(l_wzyx);

		const float32x4_t lwrx_lzrx_lyrx_lxrx = vmulq_lane_f32(l_wzyx, r_xy, 0);
		const float32x4_t lzry_lwry_lxry_lyry = vmulq_lane_f32(l_zwxy, r_xy, 1);
		const float32x4_t lyrz_lxrz_lwrz_lzrz = vmulq_lane_f32(l_yxwz, r_zw, 0);

#if defined(RTM_NEON64_INTRINSICS)
		const float32x4_t result0 = vfmaq_f32(lxrw_lyrw_lzrw_lwrw, lwrx_lzrx_lyrx_lxrx, control_wzyx);
		const float32x4_t result1 = vfmaq_f32(result0, lzry_lwry_lxry_lyry, control_zwxy);
		return vfmaq_f32(result1, lyrz_lxrz_lwrz_lzrz, control_yxwz);
#else
		const float32x4_t result0 = vmlaq_f32(lxrw_lyrw_lzrw_lwrw, lwrx_lzrx_lyrx_lxrx, control_wzyx);
		const float32x4_t result1 = vmlaq_f32(result0, lzry_lwry_lxry_lyry, control_zwxy);
		return vmlaq_f32(result1, lyrz_lxrz_lwrz_lzrz, control_yxwz);
#endif
	}
}
#endif

static void bm_quat_mul_vector3_ref(benchmark::State& state)
{
	vector4f v0 = vector_set(12.0f, 32.0f, -2.0f);
	quatf q0 = quat_identity();

	for (auto _ : state)
		benchmark::DoNotOptimize(v0 = quat_mul_vector3_ref(q0, v0));
}

BENCHMARK(bm_quat_mul_vector3_ref);

#if defined(RTM_FMA_INTRINSICS)
static void bm_quat_mul_vector3_fma(benchmark::State& state)
{
	vector4f v0 = vector_set(12.0f, 32.0f, -2.0f);
	quatf q0 = quat_identity();

	for (auto _ : state)
		benchmark::DoNotOptimize(v0 = quat_mul_vector3_fma(q0, v0));
}

BENCHMARK(bm_quat_mul_vector3_fma);
#endif

#if defined(RTM_SSE2_INTRINSICS)
static void bm_quat_mul_vector3_sse2(benchmark::State& state)
{
	vector4f v0 = vector_set(12.0f, 32.0f, -2.0f);
	quatf q0 = quat_identity();

	for (auto _ : state)
		benchmark::DoNotOptimize(v0 = quat_mul_vector3_sse2(q0, v0));
}

BENCHMARK(bm_quat_mul_vector3_sse2);
#endif

#if defined(RTM_NEON_INTRINSICS)
static void bm_quat_mul_vector3_neon_scalar(benchmark::State& state)
{
	vector4f v0 = vector_set(12.0f, 32.0f, -2.0f);
	quatf q0 = quat_identity();

	for (auto _ : state)
		benchmark::DoNotOptimize(v0 = quat_mul_vector3_neon_scalar(q0, v0));
}

BENCHMARK(bm_quat_mul_vector3_neon_scalar);

static void bm_quat_mul_vector3_neon64(benchmark::State& state)
{
	vector4f v0 = vector_set(12.0f, 32.0f, -2.0f);
	quatf q0 = quat_identity();

	for (auto _ : state)
		benchmark::DoNotOptimize(v0 = quat_mul_vector3_neon64(q0, v0));
}

BENCHMARK(bm_quat_mul_vector3_neon64);

static void bm_quat_mul_vector3_neon(benchmark::State& state)
{
	vector4f v0 = vector_set(12.0f, 32.0f, -2.0f);
	quatf q0 = quat_identity();

	for (auto _ : state)
		benchmark::DoNotOptimize(v0 = quat_mul_vector3_neon(q0, v0));
}

BENCHMARK(bm_quat_mul_vector3_neon);
#endif
