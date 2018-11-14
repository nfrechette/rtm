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
#include "rtm/quatf.h"
#include "rtm/vector4f.h"
#include "rtm/affine_matrix_32.h"

namespace rtm
{
	constexpr qvvf RTM_SIMD_CALL qvv_set(quatf_arg0 rotation, vector4f_arg1 translation, vector4f_arg2 scale)
	{
		return qvvf{ rotation, translation, scale };
	}

	inline qvvf RTM_SIMD_CALL qvv_identity_32()
	{
		return qvv_set(quat_identity_32(), vector_zero_32(), vector_set(1.0f));
	}

	inline qvvf RTM_SIMD_CALL qvv_cast(const qvvd& input)
	{
		return qvvf{ quat_cast(input.rotation), vector_cast(input.translation), vector_cast(input.scale) };
	}

	// Multiplication order is as follow: local_to_world = qvv_mul(local_to_object, object_to_world)
	// NOTE: When scale is present, multiplication will not properly handle skew/shear, use affine matrices instead
	inline qvvf RTM_SIMD_CALL qvv_mul(qvvf_arg0 lhs, qvvf_arg1 rhs)
	{
		const vector4f min_scale = vector_min(lhs.scale, rhs.scale);
		const vector4f scale = vector_mul(lhs.scale, rhs.scale);

		if (vector_any_less_than3(min_scale, vector_zero_32()))
		{
			// If we have negative scale, we go through a matrix
			const matrix3x4f lhs_mtx = matrix_from_qvv(lhs);
			const matrix3x4f rhs_mtx = matrix_from_qvv(rhs);
			matrix3x4f result_mtx = matrix_mul(lhs_mtx, rhs_mtx);
			result_mtx = matrix_remove_scale(result_mtx);

			const vector4f sign = vector_sign(scale);
			result_mtx.x_axis = vector_mul(result_mtx.x_axis, vector_mix_xxxx(sign));
			result_mtx.y_axis = vector_mul(result_mtx.y_axis, vector_mix_yyyy(sign));
			result_mtx.z_axis = vector_mul(result_mtx.z_axis, vector_mix_zzzz(sign));

			const quatf rotation = quat_from_matrix(result_mtx);
			const vector4f translation = result_mtx.w_axis;
			return qvv_set(rotation, translation, scale);
		}
		else
		{
			const quatf rotation = quat_mul(lhs.rotation, rhs.rotation);
			const vector4f translation = vector_add(quat_rotate(rhs.rotation, vector_mul(lhs.translation, rhs.scale)), rhs.translation);
			return qvv_set(rotation, translation, scale);
		}
	}

	// Multiplication order is as follow: local_to_world = qvv_mul(local_to_object, object_to_world)
	inline qvvf RTM_SIMD_CALL qvv_mul_no_scale(qvvf_arg0 lhs, qvvf_arg1 rhs)
	{
		const quatf rotation = quat_mul(lhs.rotation, rhs.rotation);
		const vector4f translation = vector_add(quat_rotate(rhs.rotation, lhs.translation), rhs.translation);
		return qvv_set(rotation, translation, vector_set(1.0f));
	}

	inline vector4f RTM_SIMD_CALL qvv_mul_position(qvvf_arg0 lhs, vector4f_arg1 rhs)
	{
		return vector_add(quat_rotate(lhs.rotation, vector_mul(lhs.scale, rhs)), lhs.translation);
	}

	inline vector4f RTM_SIMD_CALL qvv_mul_position_no_scale(qvvf_arg0 lhs, vector4f_arg1 rhs)
	{
		return vector_add(quat_rotate(lhs.rotation, rhs), lhs.translation);
	}

	inline qvvf RTM_SIMD_CALL qvv_inverse(qvvf_arg0 input)
	{
		const quatf inv_rotation = quat_conjugate(input.rotation);
		const vector4f inv_scale = vector_reciprocal(input.scale);
		const vector4f inv_translation = vector_neg(quat_rotate(inv_rotation, vector_mul(input.translation, inv_scale)));
		return qvv_set(inv_rotation, inv_translation, inv_scale);
	}

	inline qvvf RTM_SIMD_CALL qvv_inverse_no_scale(qvvf_arg0 input)
	{
		const quatf inv_rotation = quat_conjugate(input.rotation);
		const vector4f inv_translation = vector_neg(quat_rotate(inv_rotation, input.translation));
		return qvv_set(inv_rotation, inv_translation, vector_set(1.0f));
	}

	inline qvvf RTM_SIMD_CALL qvv_normalize(qvvf_arg0 input)
	{
		const quatf rotation = quat_normalize(input.rotation);
		return qvv_set(rotation, input.translation, input.scale);
	}
}
