#pragma once

////////////////////////////////////////////////////////////////////////////////
// The MIT License (MIT)
//
// Copyright (c) 2017 Nicholas Frechette & Animation Compression Library contributors
// Copyright (c) 2018 Nicholas Frechette & Realtime Math contributors
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

#include "rtm/math.h"
#include "rtm/anglef.h"
#include "rtm/scalarf.h"
#include "rtm/vector4f.h"
#include "rtm/impl/compiler_utils.h"
#include "rtm/impl/memory_utils.h"
#include "rtm/impl/quat_common.h"

RTM_IMPL_FILE_PRAGMA_PUSH

namespace rtm
{
	//////////////////////////////////////////////////////////////////////////
	// Setters, getters, and casts
	//////////////////////////////////////////////////////////////////////////



	//////////////////////////////////////////////////////////////////////////
	// Loads an unaligned quaternion from memory.
	//////////////////////////////////////////////////////////////////////////
	inline quatf RTM_SIMD_CALL quat_load(const float* input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_loadu_ps(input);
#elif defined(RTM_NEON_INTRINSICS)
		return vld1q_f32(input);
#else
		return quat_set(input[0], input[1], input[2], input[3]);
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Loads an unaligned quaternion from memory.
	//////////////////////////////////////////////////////////////////////////
	RTM_DEPRECATED("Use quat_load instead, to be removed in v2.0")
	inline quatf RTM_SIMD_CALL quat_unaligned_load(const float* input) RTM_NO_EXCEPT
	{
		return quat_set(input[0], input[1], input[2], input[3]);
	}

	//////////////////////////////////////////////////////////////////////////
	// Casts a vector4 to a quaternion.
	//////////////////////////////////////////////////////////////////////////
	inline quatf RTM_SIMD_CALL vector_to_quat(vector4f_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS) || defined(RTM_NEON_INTRINSICS)
		return input;
#else
		return quatf{ input.x, input.y, input.z, input.w };
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Casts a quaternion float64 variant to a float32 variant.
	//////////////////////////////////////////////////////////////////////////
	inline quatf RTM_SIMD_CALL quat_cast(const quatd& input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_shuffle_ps(_mm_cvtpd_ps(input.xy), _mm_cvtpd_ps(input.zw), _MM_SHUFFLE(1, 0, 1, 0));
#else
		return quat_set(float(input.x), float(input.y), float(input.z), float(input.w));
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the quaternion [x] component (real part).
	//////////////////////////////////////////////////////////////////////////
	inline float RTM_SIMD_CALL quat_get_x(quatf_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_cvtss_f32(input);
#elif defined(RTM_NEON_INTRINSICS)
		return vgetq_lane_f32(input, 0);
#else
		return input.x;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the quaternion [y] component (real part).
	//////////////////////////////////////////////////////////////////////////
	inline float RTM_SIMD_CALL quat_get_y(quatf_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_cvtss_f32(_mm_shuffle_ps(input, input, _MM_SHUFFLE(1, 1, 1, 1)));
#elif defined(RTM_NEON_INTRINSICS)
		return vgetq_lane_f32(input, 1);
#else
		return input.y;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the quaternion [z] component (real part).
	//////////////////////////////////////////////////////////////////////////
	inline float RTM_SIMD_CALL quat_get_z(quatf_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_cvtss_f32(_mm_shuffle_ps(input, input, _MM_SHUFFLE(2, 2, 2, 2)));
#elif defined(RTM_NEON_INTRINSICS)
		return vgetq_lane_f32(input, 2);
#else
		return input.z;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the quaternion [w] component (imaginary part).
	//////////////////////////////////////////////////////////////////////////
	inline float RTM_SIMD_CALL quat_get_w(quatf_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_cvtss_f32(_mm_shuffle_ps(input, input, _MM_SHUFFLE(3, 3, 3, 3)));
#elif defined(RTM_NEON_INTRINSICS)
		return vgetq_lane_f32(input, 3);
#else
		return input.w;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Sets the quaternion [x] component (real part) and returns the new value.
	//////////////////////////////////////////////////////////////////////////
	inline quatf RTM_SIMD_CALL quat_set_x(quatf_arg0 input, float lane_value) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_move_ss(input, _mm_set_ss(lane_value));
#elif defined(RTM_NEON_INTRINSICS)
		return vsetq_lane_f32(lane_value, input, 0);
#else
		return quatf{ lane_value, input.y, input.z, input.w };
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Sets the quaternion [y] component (real part) and returns the new value.
	//////////////////////////////////////////////////////////////////////////
	inline quatf RTM_SIMD_CALL quat_set_y(quatf_arg0 input, float lane_value) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE4_INTRINSICS)
		return _mm_insert_ps(input, _mm_set_ss(lane_value), 0x10);
#elif defined(RTM_SSE2_INTRINSICS)
		const __m128 yxzw = _mm_shuffle_ps(input, input, _MM_SHUFFLE(3, 2, 0, 1));
		const __m128 vxzw = _mm_move_ss(yxzw, _mm_set_ss(lane_value));
		return _mm_shuffle_ps(vxzw, vxzw, _MM_SHUFFLE(3, 2, 0, 1));
#elif defined(RTM_NEON_INTRINSICS)
		return vsetq_lane_f32(lane_value, input, 1);
#else
		return quatf{ input.x, lane_value, input.z, input.w };
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Sets the quaternion [z] component (real part) and returns the new value.
	//////////////////////////////////////////////////////////////////////////
	inline quatf RTM_SIMD_CALL quat_set_z(quatf_arg0 input, float lane_value) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE4_INTRINSICS)
		return _mm_insert_ps(input, _mm_set_ss(lane_value), 0x20);
#elif defined(RTM_SSE2_INTRINSICS)
		const __m128 zyxw = _mm_shuffle_ps(input, input, _MM_SHUFFLE(3, 0, 1, 2));
		const __m128 vyxw = _mm_move_ss(zyxw, _mm_set_ss(lane_value));
		return _mm_shuffle_ps(vyxw, vyxw, _MM_SHUFFLE(3, 0, 1, 2));
#elif defined(RTM_NEON_INTRINSICS)
		return vsetq_lane_f32(lane_value, input, 2);
#else
		return quatf{ input.x, input.y, lane_value, input.w };
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Sets the quaternion [w] component (imaginary part) and returns the new value.
	//////////////////////////////////////////////////////////////////////////
	inline quatf RTM_SIMD_CALL quat_set_w(quatf_arg0 input, float lane_value) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE4_INTRINSICS)
		return _mm_insert_ps(input, _mm_set_ss(lane_value), 0x30);
#elif defined(RTM_SSE2_INTRINSICS)
		const __m128 wyzx = _mm_shuffle_ps(input, input, _MM_SHUFFLE(0, 2, 1, 3));
		const __m128 vyzx = _mm_move_ss(wyzx, _mm_set_ss(lane_value));
		return _mm_shuffle_ps(vyzx, vyzx, _MM_SHUFFLE(0, 2, 1, 3));
#elif defined(RTM_NEON_INTRINSICS)
		return vsetq_lane_f32(lane_value, input, 3);
#else
		return quatf{ input.x, input.y, input.z, lane_value };
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Writes a quaternion to unaligned memory.
	//////////////////////////////////////////////////////////////////////////
	inline void RTM_SIMD_CALL quat_store(quatf_arg0 input, float* output) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		_mm_storeu_ps(output, input);
#else
		output[0] = quat_get_x(input);
		output[1] = quat_get_y(input);
		output[2] = quat_get_z(input);
		output[3] = quat_get_w(input);
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Writes a quaternion to unaligned memory.
	//////////////////////////////////////////////////////////////////////////
	RTM_DEPRECATED("Use quat_store instead, to be removed in v2.0")
	inline void RTM_SIMD_CALL quat_unaligned_write(quatf_arg0 input, float* output) RTM_NO_EXCEPT
	{
		quat_store(input, output);
	}



	//////////////////////////////////////////////////////////////////////////
	// Arithmetic
	//////////////////////////////////////////////////////////////////////////



	//////////////////////////////////////////////////////////////////////////
	// Returns the quaternion conjugate.
	//////////////////////////////////////////////////////////////////////////
	inline quatf RTM_SIMD_CALL quat_conjugate(quatf_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		constexpr __m128 signs = { -0.0F, -0.0F, -0.0F, 0.0F };
		return _mm_xor_ps(input, signs);
#elif defined(RTM_NEON_INTRINSICS)
		alignas(16) constexpr uint32_t signs_u[4] = { 0x80000000U, 0x80000000U, 0x80000000U, 0 };
		const uint32x4_t signs = *reinterpret_cast<const uint32x4_t*>(&signs_u[0]);
		return vreinterpretq_f32_u32(veorq_u32(vreinterpretq_u32_f32(input), signs));
#else
		return quat_set(-quat_get_x(input), -quat_get_y(input), -quat_get_z(input), quat_get_w(input));
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Multiplies two quaternions.
	// Note that due to floating point rounding, the result might not be perfectly normalized.
	// Multiplication order is as follow: local_to_world = quat_mul(local_to_object, object_to_world)
	//////////////////////////////////////////////////////////////////////////
	inline quatf RTM_SIMD_CALL quat_mul(quatf_arg0 lhs, quatf_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE4_INTRINSICS) && 0
		// TODO: Profile this, the accuracy is the same as with SSE2, should be binary exact
		constexpr __m128 signs_x = { 1.0F,  1.0F,  1.0F, -1.0F };
		constexpr __m128 signs_y = { 1.0F, -1.0F,  1.0F,  1.0F };
		constexpr __m128 signs_z = { 1.0F,  1.0F, -1.0F,  1.0F };
		constexpr __m128 signs_w = { 1.0F, -1.0F, -1.0F, -1.0F };
		// x = dot(rhs.wxyz, lhs.xwzy * signs_x)
		// y = dot(rhs.wxyz, lhs.yzwx * signs_y)
		// z = dot(rhs.wxyz, lhs.zyxw * signs_z)
		// w = dot(rhs.wxyz, lhs.wxyz * signs_w)
		__m128 rhs_wxyz = _mm_shuffle_ps(rhs, rhs, _MM_SHUFFLE(2, 1, 0, 3));
		__m128 lhs_xwzy = _mm_shuffle_ps(lhs, lhs, _MM_SHUFFLE(1, 2, 3, 0));
		__m128 lhs_yzwx = _mm_shuffle_ps(lhs, lhs, _MM_SHUFFLE(0, 3, 2, 1));
		__m128 lhs_zyxw = _mm_shuffle_ps(lhs, lhs, _MM_SHUFFLE(3, 0, 1, 2));
		__m128 lhs_wxyz = _mm_shuffle_ps(lhs, lhs, _MM_SHUFFLE(2, 1, 0, 3));
		__m128 x = _mm_dp_ps(rhs_wxyz, _mm_mul_ps(lhs_xwzy, signs_x), 0xFF);
		__m128 y = _mm_dp_ps(rhs_wxyz, _mm_mul_ps(lhs_yzwx, signs_y), 0xFF);
		__m128 z = _mm_dp_ps(rhs_wxyz, _mm_mul_ps(lhs_zyxw, signs_z), 0xFF);
		__m128 w = _mm_dp_ps(rhs_wxyz, _mm_mul_ps(lhs_wxyz, signs_w), 0xFF);
		__m128 xxyy = _mm_shuffle_ps(x, y, _MM_SHUFFLE(0, 0, 0, 0));
		__m128 zzww = _mm_shuffle_ps(z, w, _MM_SHUFFLE(0, 0, 0, 0));
		return _mm_shuffle_ps(xxyy, zzww, _MM_SHUFFLE(2, 0, 2, 0));
#elif defined(RTM_SSE2_INTRINSICS)
		constexpr __m128 control_wzyx = {  0.0F, -0.0F,  0.0F, -0.0F };
		constexpr __m128 control_zwxy = {  0.0F,  0.0F, -0.0F, -0.0F };
		constexpr __m128 control_yxwz = { -0.0F,  0.0F,  0.0F, -0.0F };

		const __m128 r_xxxx = _mm_shuffle_ps(rhs, rhs, _MM_SHUFFLE(0, 0, 0, 0));
		const __m128 r_yyyy = _mm_shuffle_ps(rhs, rhs, _MM_SHUFFLE(1, 1, 1, 1));
		const __m128 r_zzzz = _mm_shuffle_ps(rhs, rhs, _MM_SHUFFLE(2, 2, 2, 2));
		const __m128 r_wwww = _mm_shuffle_ps(rhs, rhs, _MM_SHUFFLE(3, 3, 3, 3));

		const __m128 lxrw_lyrw_lzrw_lwrw = _mm_mul_ps(r_wwww, lhs);
		const __m128 l_wzyx = _mm_shuffle_ps(lhs, lhs,_MM_SHUFFLE(0, 1, 2, 3));

		const __m128 lwrx_lzrx_lyrx_lxrx = _mm_mul_ps(r_xxxx, l_wzyx);
		const __m128 l_zwxy = _mm_shuffle_ps(l_wzyx, l_wzyx,_MM_SHUFFLE(2, 3, 0, 1));

		const __m128 lwrx_nlzrx_lyrx_nlxrx = _mm_xor_ps(lwrx_lzrx_lyrx_lxrx, control_wzyx);

		const __m128 lzry_lwry_lxry_lyry = _mm_mul_ps(r_yyyy, l_zwxy);
		const __m128 l_yxwz = _mm_shuffle_ps(l_zwxy, l_zwxy,_MM_SHUFFLE(0, 1, 2, 3));

		const __m128 lzry_lwry_nlxry_nlyry = _mm_xor_ps(lzry_lwry_lxry_lyry, control_zwxy);

		const __m128 lyrz_lxrz_lwrz_lzrz = _mm_mul_ps(r_zzzz, l_yxwz);
		const __m128 result0 = _mm_add_ps(lxrw_lyrw_lzrw_lwrw, lwrx_nlzrx_lyrx_nlxrx);

		const __m128 nlyrz_lxrz_lwrz_wlzrz = _mm_xor_ps(lyrz_lxrz_lwrz_lzrz, control_yxwz);
		const __m128 result1 = _mm_add_ps(lzry_lwry_nlxry_nlyry, nlyrz_lxrz_lwrz_wlzrz);
		return _mm_add_ps(result0, result1);
#else
		// On ARMv7 and ARM64, the scalar version is faster than SIMD variants.

		const float lhs_x = quat_get_x(lhs);
		const float lhs_y = quat_get_y(lhs);
		const float lhs_z = quat_get_z(lhs);
		const float lhs_w = quat_get_w(lhs);

		const float rhs_x = quat_get_x(rhs);
		const float rhs_y = quat_get_y(rhs);
		const float rhs_z = quat_get_z(rhs);
		const float rhs_w = quat_get_w(rhs);

		const float x = (rhs_w * lhs_x) + (rhs_x * lhs_w) + (rhs_y * lhs_z) - (rhs_z * lhs_y);
		const float y = (rhs_w * lhs_y) - (rhs_x * lhs_z) + (rhs_y * lhs_w) + (rhs_z * lhs_x);
		const float z = (rhs_w * lhs_z) + (rhs_x * lhs_y) - (rhs_y * lhs_x) + (rhs_z * lhs_w);
		const float w = (rhs_w * lhs_w) - (rhs_x * lhs_x) - (rhs_y * lhs_y) - (rhs_z * lhs_z);

		return quat_set(x, y, z, w);
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Multiplies a quaternion and a 3D vector, rotating it.
	// Multiplication order is as follow: world_position = quat_mul_vector3(local_position, local_to_world)
	//////////////////////////////////////////////////////////////////////////
	inline vector4f RTM_SIMD_CALL quat_mul_vector3(vector4f_arg0 vector, quatf_arg1 rotation) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
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
#elif defined(RTM_NEON_INTRINSICS)

		// Normally when we multiply our inverse rotation quaternion with the input vector as a quaternion with W = 0.0.
		// As a result, we can strip the whole part that uses W saving a few instructions.
		// We also don't care about the result W lane.
		// On ARMv7 and ARM64, the scalar version is faster than SIMD variants.

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
			temp_y = -(rhs_x * lhs_z) + (rhs_y * lhs_w) + (rhs_z * lhs_x);
			temp_z = (rhs_x * lhs_y) - (rhs_y * lhs_x) + (rhs_z * lhs_w);
			temp_w = -(rhs_x * lhs_x) - (rhs_y * lhs_y) - (rhs_z * lhs_z);
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
#else
		quatf vector_quat = quat_set_w(vector_to_quat(vector), 0.0f);
		quatf inv_rotation = quat_conjugate(rotation);
		return quat_to_vector(quat_mul(quat_mul(inv_rotation, vector_quat), rotation));
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the squared length/norm of the quaternion.
	//////////////////////////////////////////////////////////////////////////
	inline float RTM_SIMD_CALL quat_length_squared(quatf_arg0 input) RTM_NO_EXCEPT
	{
		return vector_length_squared(quat_to_vector(input));
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the length/norm of the quaternion.
	//////////////////////////////////////////////////////////////////////////
	inline float RTM_SIMD_CALL quat_length(quatf_arg0 input) RTM_NO_EXCEPT
	{
		return vector_length(quat_to_vector(input));
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the reciprocal length/norm of the quaternion.
	//////////////////////////////////////////////////////////////////////////
	inline float RTM_SIMD_CALL quat_length_reciprocal(quatf_arg0 input) RTM_NO_EXCEPT
	{
		return vector_length_reciprocal(quat_to_vector(input));
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns a normalized quaternion.
	// Note that if the input quaternion is invalid (pure zero or with NaN/Inf),
	// the result is undefined.
	//////////////////////////////////////////////////////////////////////////
	inline quatf RTM_SIMD_CALL quat_normalize(quatf_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		// We first calculate the dot product to get the length squared: dot(input, input)
		__m128 x2_y2_z2_w2 = _mm_mul_ps(input, input);
		__m128 z2_w2_0_0 = _mm_shuffle_ps(x2_y2_z2_w2, x2_y2_z2_w2, _MM_SHUFFLE(0, 0, 3, 2));
		__m128 x2z2_y2w2_0_0 = _mm_add_ps(x2_y2_z2_w2, z2_w2_0_0);
		__m128 y2w2_0_0_0 = _mm_shuffle_ps(x2z2_y2w2_0_0, x2z2_y2w2_0_0, _MM_SHUFFLE(0, 0, 0, 1));
		__m128 x2y2z2w2_0_0_0 = _mm_add_ps(x2z2_y2w2_0_0, y2w2_0_0_0);

		// Keep the dot product result as a scalar within the first lane, it is faster to
		// calculate the reciprocal square root of a single lane VS all 4 lanes
		__m128 dot = x2y2z2w2_0_0_0;

		// Calculate the reciprocal square root to get the inverse length of our vector
		// Perform two passes of Newton-Raphson iteration on the hardware estimate
		__m128 half = _mm_set_ss(0.5F);
		__m128 input_half_v = _mm_mul_ss(dot, half);
		__m128 x0 = _mm_rsqrt_ss(dot);

		// First iteration
		__m128 x1 = _mm_mul_ss(x0, x0);
		x1 = _mm_sub_ss(half, _mm_mul_ss(input_half_v, x1));
		x1 = _mm_add_ss(_mm_mul_ss(x0, x1), x0);

		// Second iteration
		__m128 x2 = _mm_mul_ss(x1, x1);
		x2 = _mm_sub_ss(half, _mm_mul_ss(input_half_v, x2));
		x2 = _mm_add_ss(_mm_mul_ss(x1, x2), x1);

		// Broadcast the vector length reciprocal to all 4 lanes in order to multiply it with the vector
		__m128 inv_len = _mm_shuffle_ps(x2, x2, _MM_SHUFFLE(0, 0, 0, 0));

		// Multiply the rotation by it's inverse length in order to normalize it
		return _mm_mul_ps(input, inv_len);
#elif defined (RTM_NEON_INTRINSICS)
		// Use sqrt/div/mul to normalize because the sqrt/div are faster than rsqrt
		float inv_len = 1.0F / scalar_sqrt(vector_length_squared(input));
		return vector_mul(input, inv_len);
#else
		// Reciprocal is more accurate to normalize with
		float inv_len = quat_length_reciprocal(input);
		return vector_to_quat(vector_mul(quat_to_vector(input), inv_len));
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the linear interpolation between start and end for a given alpha value.
	//////////////////////////////////////////////////////////////////////////
	inline quatf RTM_SIMD_CALL quat_lerp(quatf_arg0 start, quatf_arg1 end, float alpha) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		// Calculate the vector4 dot product: dot(start, end)
		__m128 dot;
#if defined(RTM_SSE4_INTRINSICS)
		// The dpps instruction isn't as accurate but we don't care here, we only need the sign of the
		// dot product. If both rotations are on opposite ends of the hypersphere, the result will be
		// very negative. If we are on the edge, the rotations are nearly opposite but not quite which
		// means that the linear interpolation here will have terrible accuracy to begin with. It is designed
		// for interpolating rotations that are reasonably close together. The bias check is mainly necessary
		// because the W component is often kept positive which flips the sign.
		// Using the dpps instruction reduces the number of registers that we need and helps the function get
		// inlined.
		dot = _mm_dp_ps(start, end, 0xFF);
#else
		{
			__m128 x2_y2_z2_w2 = _mm_mul_ps(start, end);
			__m128 z2_w2_0_0 = _mm_shuffle_ps(x2_y2_z2_w2, x2_y2_z2_w2, _MM_SHUFFLE(0, 0, 3, 2));
			__m128 x2z2_y2w2_0_0 = _mm_add_ps(x2_y2_z2_w2, z2_w2_0_0);
			__m128 y2w2_0_0_0 = _mm_shuffle_ps(x2z2_y2w2_0_0, x2z2_y2w2_0_0, _MM_SHUFFLE(0, 0, 0, 1));
			__m128 x2y2z2w2_0_0_0 = _mm_add_ps(x2z2_y2w2_0_0, y2w2_0_0_0);
			// Shuffle the dot product to all SIMD lanes, there is no _mm_and_ss and loading
			// the constant from memory with the 'and' instruction is faster, it uses fewer registers
			// and fewer instructions
			dot = _mm_shuffle_ps(x2y2z2w2_0_0_0, x2y2z2w2_0_0_0, _MM_SHUFFLE(0, 0, 0, 0));
		}
#endif

		// Calculate the bias, if the dot product is positive or zero, there is no bias
		// but if it is negative, we want to flip the 'end' rotation XYZW components
		__m128 bias = _mm_and_ps(dot, _mm_set_ps1(-0.0F));

		// Lerp the rotation after applying the bias
		__m128 interpolated_rotation = _mm_add_ps(_mm_mul_ps(_mm_sub_ps(_mm_xor_ps(end, bias), start), _mm_set_ps1(alpha)), start);

		// Now we need to normalize the resulting rotation. We first calculate the
		// dot product to get the length squared: dot(interpolated_rotation, interpolated_rotation)
		__m128 x2_y2_z2_w2 = _mm_mul_ps(interpolated_rotation, interpolated_rotation);
		__m128 z2_w2_0_0 = _mm_shuffle_ps(x2_y2_z2_w2, x2_y2_z2_w2, _MM_SHUFFLE(0, 0, 3, 2));
		__m128 x2z2_y2w2_0_0 = _mm_add_ps(x2_y2_z2_w2, z2_w2_0_0);
		__m128 y2w2_0_0_0 = _mm_shuffle_ps(x2z2_y2w2_0_0, x2z2_y2w2_0_0, _MM_SHUFFLE(0, 0, 0, 1));
		__m128 x2y2z2w2_0_0_0 = _mm_add_ps(x2z2_y2w2_0_0, y2w2_0_0_0);

		// Keep the dot product result as a scalar within the first lane, it is faster to
		// calculate the reciprocal square root of a single lane VS all 4 lanes
		dot = x2y2z2w2_0_0_0;

		// Calculate the reciprocal square root to get the inverse length of our vector
		// Perform two passes of Newton-Raphson iteration on the hardware estimate
		__m128 half = _mm_set_ss(0.5F);
		__m128 input_half_v = _mm_mul_ss(dot, half);
		__m128 x0 = _mm_rsqrt_ss(dot);

		// First iteration
		__m128 x1 = _mm_mul_ss(x0, x0);
		x1 = _mm_sub_ss(half, _mm_mul_ss(input_half_v, x1));
		x1 = _mm_add_ss(_mm_mul_ss(x0, x1), x0);

		// Second iteration
		__m128 x2 = _mm_mul_ss(x1, x1);
		x2 = _mm_sub_ss(half, _mm_mul_ss(input_half_v, x2));
		x2 = _mm_add_ss(_mm_mul_ss(x1, x2), x1);

		// Broadcast the vector length reciprocal to all 4 lanes in order to multiply it with the vector
		__m128 inv_len = _mm_shuffle_ps(x2, x2, _MM_SHUFFLE(0, 0, 0, 0));

		// Multiply the rotation by it's inverse length in order to normalize it
		return _mm_mul_ps(interpolated_rotation, inv_len);
#elif defined (RTM_NEON64_INTRINSICS)
		// On ARM64 with NEON, we load 1.0 once and use it twice which is faster than
		// using a AND/XOR with the bias (same number of instructions)
		float dot = vector_dot(start, end);
		float bias = dot >= 0.0F ? 1.0F : -1.0F;
		vector4f interpolated_rotation = vector_neg_mul_sub(vector_neg_mul_sub(end, bias, start), alpha, start);
		// Use sqrt/div/mul to normalize because the sqrt/div are faster than rsqrt
		float inv_len = 1.0F / scalar_sqrt(vector_length_squared(interpolated_rotation));
		return vector_mul(interpolated_rotation, inv_len);
#elif defined(RTM_NEON_INTRINSICS)
		// Calculate the vector4 dot product: dot(start, end)
		float32x4_t x2_y2_z2_w2 = vmulq_f32(start, end);
		float32x2_t x2_y2 = vget_low_f32(x2_y2_z2_w2);
		float32x2_t z2_w2 = vget_high_f32(x2_y2_z2_w2);
		float32x2_t x2z2_y2w2 = vadd_f32(x2_y2, z2_w2);
		float32x2_t x2y2z2w2 = vpadd_f32(x2z2_y2w2, x2z2_y2w2);

		// Calculate the bias, if the dot product is positive or zero, there is no bias
		// but if it is negative, we want to flip the 'end' rotation XYZW components
		// On ARM-v7-A, the AND/XOR trick is faster than the cmp/fsel
		uint32x2_t bias = vand_u32(vreinterpret_u32_f32(x2y2z2w2), vdup_n_u32(0x80000000));

		// Lerp the rotation after applying the bias
		float32x4_t end_biased = vreinterpretq_f32_u32(veorq_u32(vreinterpretq_u32_f32(end), vcombine_u32(bias, bias)));
		float32x4_t interpolated_rotation = vmlaq_n_f32(start, vsubq_f32(end_biased, start), alpha);

		// Now we need to normalize the resulting rotation. We first calculate the
		// dot product to get the length squared: dot(interpolated_rotation, interpolated_rotation)
		x2_y2_z2_w2 = vmulq_f32(interpolated_rotation, interpolated_rotation);
		x2_y2 = vget_low_f32(x2_y2_z2_w2);
		z2_w2 = vget_high_f32(x2_y2_z2_w2);
		x2z2_y2w2 = vadd_f32(x2_y2, z2_w2);
		x2y2z2w2 = vpadd_f32(x2z2_y2w2, x2z2_y2w2);

		float dot = vget_lane_f32(x2y2z2w2, 0);

		// Use sqrt/div/mul to normalize because the sqrt/div are faster than rsqrt
		float inv_len = 1.0F / scalar_sqrt(dot);
		return vector_mul(interpolated_rotation, inv_len);
#else
		// To ensure we take the shortest path, we apply a bias if the dot product is negative
		vector4f start_vector = quat_to_vector(start);
		vector4f end_vector = quat_to_vector(end);
		float dot = vector_dot(start_vector, end_vector);
		float bias = dot >= 0.0F ? 1.0F : -1.0F;
		vector4f interpolated_rotation = vector_neg_mul_sub(vector_neg_mul_sub(end_vector, bias, start_vector), alpha, start_vector);
		// TODO: Test with this instead: Rotation = (B * Alpha) + (A * (Bias * (1.f - Alpha)));
		//vector4f value = vector_add(vector_mul(end_vector, alpha), vector_mul(start_vector, bias * (1.0f - alpha)));
		return quat_normalize(vector_to_quat(interpolated_rotation));
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns a component wise negated quaternion.
	//////////////////////////////////////////////////////////////////////////
	inline quatf RTM_SIMD_CALL quat_neg(quatf_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		constexpr __m128 signs = { -0.0F, -0.0F, -0.0F, -0.0F };
		return _mm_xor_ps(input, signs);
#elif defined(RTM_NEON_INTRINSICS)
		return vnegq_f32(input);
#else
		return vector_to_quat(vector_mul(quat_to_vector(input), -1.0F));
#endif
	}



	//////////////////////////////////////////////////////////////////////////
	// Conversion to/from axis/angle/euler
	//////////////////////////////////////////////////////////////////////////



	//////////////////////////////////////////////////////////////////////////
	// Returns the rotation axis and rotation angle that make up the input quaternion.
	//////////////////////////////////////////////////////////////////////////
	inline void RTM_SIMD_CALL quat_to_axis_angle(quatf_arg0 input, vector4f& out_axis, anglef& out_angle) RTM_NO_EXCEPT
	{
		constexpr float epsilon = 1.0E-8F;
		constexpr float epsilon_squared = epsilon * epsilon;

		out_angle = radians(scalar_acos(quat_get_w(input)) * 2.0F);

		const float scale_sq = scalar_max(1.0F - quat_get_w(input) * quat_get_w(input), 0.0F);
		out_axis = scale_sq >= epsilon_squared ? vector_mul(quat_to_vector(input), vector_set(scalar_sqrt_reciprocal(scale_sq))) : vector_set(1.0F, 0.0F, 0.0F);
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the rotation axis part of the input quaternion.
	//////////////////////////////////////////////////////////////////////////
	inline vector4f RTM_SIMD_CALL quat_get_axis(quatf_arg0 input) RTM_NO_EXCEPT
	{
		constexpr float epsilon = 1.0E-8F;
		constexpr float epsilon_squared = epsilon * epsilon;

		const float scale_sq = scalar_max(1.0F - quat_get_w(input) * quat_get_w(input), 0.0F);
		return scale_sq >= epsilon_squared ? vector_mul(quat_to_vector(input), vector_set(scalar_sqrt_reciprocal(scale_sq))) : vector_set(1.0F, 0.0F, 0.0F);
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the rotation angle part of the input quaternion.
	//////////////////////////////////////////////////////////////////////////
	inline anglef RTM_SIMD_CALL quat_get_angle(quatf_arg0 input) RTM_NO_EXCEPT
	{
		return radians(scalar_acos(quat_get_w(input)) * 2.0F);
	}

	//////////////////////////////////////////////////////////////////////////
	// Creates a quaternion from a rotation axis and a rotation angle.
	//////////////////////////////////////////////////////////////////////////
	inline quatf RTM_SIMD_CALL quat_from_axis_angle(vector4f_arg0 axis, anglef angle) RTM_NO_EXCEPT
	{
		float s;
		float c;
		scalar_sincos(0.5F * angle.as_radians(), s, c);

		return vector_to_quat(vector_set_w(vector_mul(vector_set(s), axis), c));
	}

	//////////////////////////////////////////////////////////////////////////
	// Creates a quaternion from Euler Pitch/Yaw/Roll angles.
	// Pitch is around the Y axis (right)
	// Yaw is around the Z axis (up)
	// Roll is around the X axis (forward)
	//////////////////////////////////////////////////////////////////////////
	inline quatf RTM_SIMD_CALL quat_from_euler(anglef pitch, anglef yaw, anglef roll) RTM_NO_EXCEPT
	{
		float sp;
		float sy;
		float sr;
		float cp;
		float cy;
		float cr;

		scalar_sincos(pitch.as_radians() * 0.5F, sp, cp);
		scalar_sincos(yaw.as_radians() * 0.5F, sy, cy);
		scalar_sincos(roll.as_radians() * 0.5F, sr, cr);

		return quat_set(cr * sp * sy - sr * cp * cy,
			-cr * sp * cy - sr * cp * sy,
			cr * cp * sy - sr * sp * cy,
			cr * cp * cy + sr * sp * sy);
	}



	//////////////////////////////////////////////////////////////////////////
	// Comparisons and masking
	//////////////////////////////////////////////////////////////////////////



	//////////////////////////////////////////////////////////////////////////
	// Returns true if the input quaternion does not contain any NaN or Inf, otherwise false.
	//////////////////////////////////////////////////////////////////////////
	inline bool RTM_SIMD_CALL quat_is_finite(quatf_arg0 input) RTM_NO_EXCEPT
	{
		return scalar_is_finite(quat_get_x(input)) && scalar_is_finite(quat_get_y(input)) && scalar_is_finite(quat_get_z(input)) && scalar_is_finite(quat_get_w(input));
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if the input quaternion is normalized, otherwise false.
	//////////////////////////////////////////////////////////////////////////
	inline bool RTM_SIMD_CALL quat_is_normalized(quatf_arg0 input, float threshold = 0.00001F) RTM_NO_EXCEPT
	{
		float length_squared = quat_length_squared(input);
		return scalar_abs(length_squared - 1.0F) < threshold;
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if the two quaternions are nearly equal component wise, otherwise false.
	//////////////////////////////////////////////////////////////////////////
	inline bool RTM_SIMD_CALL quat_near_equal(quatf_arg0 lhs, quatf_arg1 rhs, float threshold = 0.00001F) RTM_NO_EXCEPT
	{
		return vector_all_near_equal(quat_to_vector(lhs), quat_to_vector(rhs), threshold);
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if the input quaternion is nearly equal to the identity quaternion
	// by comparing its rotation angle.
	//////////////////////////////////////////////////////////////////////////
	inline bool RTM_SIMD_CALL quat_near_identity(quatf_arg0 input, anglef threshold_angle = radians(0.00284714461F)) RTM_NO_EXCEPT
	{
		// Because of floating point precision, we cannot represent very small rotations.
		// The closest float to 1.0 that is not 1.0 itself yields:
		// acos(0.99999994f) * 2.0f  = 0.000690533954 rad
		//
		// An error threshold of 1.e-6f is used by default.
		// acos(1.f - 1.e-6f) * 2.0f = 0.00284714461 rad
		// acos(1.f - 1.e-7f) * 2.0f = 0.00097656250 rad
		//
		// We don't really care about the angle value itself, only if it's close to 0.
		// This will happen whenever quat.w is close to 1.0.
		// If the quat.w is close to -1.0, the angle will be near 2*PI which is close to
		// a negative 0 rotation. By forcing quat.w to be positive, we'll end up with
		// the shortest path.
		const float positive_w_angle = scalar_acos(scalar_abs(quat_get_w(input))) * 2.0F;
		return positive_w_angle < threshold_angle.as_radians();
	}
}

RTM_IMPL_FILE_PRAGMA_POP
