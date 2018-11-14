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

#include "rtm/error.h"
#include "rtm/math.h"
#include "rtm/vector4f.h"
#include "rtm/quatf.h"

namespace rtm
{
	//////////////////////////////////////////////////////////////////////////
	// An 4x4 affine matrix represents a 3D rotation, 3D translation, and 3D scale.
	// It properly deals with skew/shear when present but once scale with mirroring is combined,
	// it cannot be safely extracted back.
	//
	// Affine matrices have their last column always equal to [0, 0, 0, 1]
	//
	// X axis == forward
	// Y axis == right
	// Z axis == up
	//////////////////////////////////////////////////////////////////////////

	inline matrix3x4f RTM_SIMD_CALL matrix_set(vector4f_arg0 x_axis, vector4f_arg1 y_axis, vector4f_arg2 z_axis, vector4f_arg3 w_axis)
	{
		RTM_ASSERT(vector_get_w(x_axis) == 0.0f, "X axis does not have a W component == 0.0");
		RTM_ASSERT(vector_get_w(y_axis) == 0.0f, "Y axis does not have a W component == 0.0");
		RTM_ASSERT(vector_get_w(z_axis) == 0.0f, "Z axis does not have a W component == 0.0");
		RTM_ASSERT(vector_get_w(w_axis) == 1.0f, "W axis does not have a W component == 1.0");
		return matrix3x4f{x_axis, y_axis, z_axis, w_axis};
	}

	inline matrix3x4f RTM_SIMD_CALL matrix_set(quatf_arg0 quat, vector4f_arg1 translation, vector4f_arg2 scale)
	{
		RTM_ASSERT(quat_is_normalized(quat), "Quaternion is not normalized");

		const float x2 = quat_get_x(quat) + quat_get_x(quat);
		const float y2 = quat_get_y(quat) + quat_get_y(quat);
		const float z2 = quat_get_z(quat) + quat_get_z(quat);
		const float xx = quat_get_x(quat) * x2;
		const float xy = quat_get_x(quat) * y2;
		const float xz = quat_get_x(quat) * z2;
		const float yy = quat_get_y(quat) * y2;
		const float yz = quat_get_y(quat) * z2;
		const float zz = quat_get_z(quat) * z2;
		const float wx = quat_get_w(quat) * x2;
		const float wy = quat_get_w(quat) * y2;
		const float wz = quat_get_w(quat) * z2;

		vector4f x_axis = vector_mul(vector_set(1.0f - (yy + zz), xy + wz, xz - wy, 0.0f), vector_get_x(scale));
		vector4f y_axis = vector_mul(vector_set(xy - wz, 1.0f - (xx + zz), yz + wx, 0.0f), vector_get_y(scale));
		vector4f z_axis = vector_mul(vector_set(xz + wy, yz - wx, 1.0f - (xx + yy), 0.0f), vector_get_z(scale));
		vector4f w_axis = vector_set(vector_get_x(translation), vector_get_y(translation), vector_get_z(translation), 1.0f);
		return matrix_set(x_axis, y_axis, z_axis, w_axis);
	}

	inline matrix3x4f RTM_SIMD_CALL matrix_identity_32()
	{
		return matrix_set(vector_set(1.0f, 0.0f, 0.0f, 0.0f), vector_set(0.0f, 1.0f, 0.0f, 0.0f), vector_set(0.0f, 0.0f, 1.0f, 0.0f), vector_set(0.0f, 0.0f, 0.0f, 1.0f));
	}

	inline matrix3x4f RTM_SIMD_CALL matrix_cast(const matrix3x4d& input)
	{
		return matrix_set(vector_cast(input.x_axis), vector_cast(input.y_axis), vector_cast(input.z_axis), vector_cast(input.w_axis));
	}

	inline matrix3x4f RTM_SIMD_CALL matrix_from_quat(quatf_arg0 quat)
	{
		RTM_ASSERT(quat_is_normalized(quat), "Quaternion is not normalized");

		const float x2 = quat_get_x(quat) + quat_get_x(quat);
		const float y2 = quat_get_y(quat) + quat_get_y(quat);
		const float z2 = quat_get_z(quat) + quat_get_z(quat);
		const float xx = quat_get_x(quat) * x2;
		const float xy = quat_get_x(quat) * y2;
		const float xz = quat_get_x(quat) * z2;
		const float yy = quat_get_y(quat) * y2;
		const float yz = quat_get_y(quat) * z2;
		const float zz = quat_get_z(quat) * z2;
		const float wx = quat_get_w(quat) * x2;
		const float wy = quat_get_w(quat) * y2;
		const float wz = quat_get_w(quat) * z2;

		vector4f x_axis = vector_set(1.0f - (yy + zz), xy + wz, xz - wy, 0.0f);
		vector4f y_axis = vector_set(xy - wz, 1.0f - (xx + zz), yz + wx, 0.0f);
		vector4f z_axis = vector_set(xz + wy, yz - wx, 1.0f - (xx + yy), 0.0f);
		vector4f w_axis = vector_set(0.0f, 0.0f, 0.0f, 1.0f);
		return matrix_set(x_axis, y_axis, z_axis, w_axis);
	}

	inline matrix3x4f RTM_SIMD_CALL matrix_from_translation(vector4f_arg0 translation)
	{
		return matrix_set(vector_set(1.0f, 0.0f, 0.0f, 0.0f), vector_set(0.0f, 1.0f, 0.0f, 0.0f), vector_set(0.0f, 0.0f, 1.0f, 0.0f), vector_set(vector_get_x(translation), vector_get_y(translation), vector_get_z(translation), 1.0f));
	}

	inline matrix3x4f RTM_SIMD_CALL matrix_from_scale(vector4f_arg0 scale)
	{
		RTM_ASSERT(!vector_any_near_equal3(scale, vector_zero_32()), "Scale cannot be zero");
		return matrix_set(vector_set(vector_get_x(scale), 0.0f, 0.0f, 0.0f), vector_set(0.0f, vector_get_y(scale), 0.0f, 0.0f), vector_set(0.0f, 0.0f, vector_get_z(scale), 0.0f), vector_set(0.0f, 0.0f, 0.0f, 1.0f));
	}

	inline matrix3x4f RTM_SIMD_CALL matrix_from_qvv(qvvf_arg0 transform)
	{
		return matrix_set(transform.rotation, transform.translation, transform.scale);
	}

	inline const vector4f& matrix_get_axis(const matrix3x4f& input, MatrixAxis axis)
	{
		switch (axis)
		{
		case MatrixAxis::X: return input.x_axis;
		case MatrixAxis::Y: return input.y_axis;
		case MatrixAxis::Z: return input.z_axis;
		case MatrixAxis::W: return input.w_axis;
		default:
			RTM_ASSERT(false, "Invalid matrix axis");
			return input.x_axis;
		}
	}

	inline vector4f& matrix_get_axis(matrix3x4f& input, MatrixAxis axis)
	{
		switch (axis)
		{
		case MatrixAxis::X: return input.x_axis;
		case MatrixAxis::Y: return input.y_axis;
		case MatrixAxis::Z: return input.z_axis;
		case MatrixAxis::W: return input.w_axis;
		default:
			RTM_ASSERT(false, "Invalid matrix axis");
			return input.x_axis;
		}
	}

	inline quatf RTM_SIMD_CALL quat_from_matrix(matrix3x4f_arg0 input)
	{
		if (vector_all_near_equal3(input.x_axis, vector_zero_32()) || vector_all_near_equal3(input.y_axis, vector_zero_32()) || vector_all_near_equal3(input.z_axis, vector_zero_32()))
		{
			// Zero scale not supported, return the identity
			return quat_identity_32();
		}

		const float mtx_trace = vector_get_x(input.x_axis) + vector_get_y(input.y_axis) + vector_get_z(input.z_axis);
		if (mtx_trace > 0.0f)
		{
			const float inv_trace = sqrt_reciprocal(mtx_trace + 1.0f);
			const float half_inv_trace = inv_trace * 0.5f;

			const float x = (vector_get_z(input.y_axis) - vector_get_y(input.z_axis)) * half_inv_trace;
			const float y = (vector_get_x(input.z_axis) - vector_get_z(input.x_axis)) * half_inv_trace;
			const float z = (vector_get_y(input.x_axis) - vector_get_x(input.y_axis)) * half_inv_trace;
			const float w = reciprocal(inv_trace) * 0.5f;

			return quat_normalize(quat_set(x, y, z, w));
		}
		else
		{
			int8_t best_axis = 0;
			if (vector_get_y(input.y_axis) > vector_get_x(input.x_axis))
				best_axis = 1;
			if (vector_get_z(input.z_axis) > vector_get_component(matrix_get_axis(input, MatrixAxis(best_axis)), VectorMix(best_axis)))
				best_axis = 2;

			const int8_t next_best_axis = (best_axis + 1) % 3;
			const int8_t next_next_best_axis = (next_best_axis + 1) % 3;

			const float mtx_pseudo_trace = 1.0f +
				vector_get_component(matrix_get_axis(input, MatrixAxis(best_axis)), VectorMix(best_axis)) -
				vector_get_component(matrix_get_axis(input, MatrixAxis(next_best_axis)), VectorMix(next_best_axis)) -
				vector_get_component(matrix_get_axis(input, MatrixAxis(next_next_best_axis)), VectorMix(next_next_best_axis));

			const float inv_pseudo_trace = sqrt_reciprocal(mtx_pseudo_trace);
			const float half_inv_pseudo_trace = inv_pseudo_trace * 0.5f;

			float quat_values[4];
			quat_values[best_axis] = reciprocal(inv_pseudo_trace) * 0.5f;
			quat_values[next_best_axis] = half_inv_pseudo_trace *
				(vector_get_component(matrix_get_axis(input, MatrixAxis(best_axis)), VectorMix(next_best_axis)) +
					vector_get_component(matrix_get_axis(input, MatrixAxis(next_best_axis)), VectorMix(best_axis)));
			quat_values[next_next_best_axis] = half_inv_pseudo_trace *
				(vector_get_component(matrix_get_axis(input, MatrixAxis(best_axis)), VectorMix(next_next_best_axis)) +
					vector_get_component(matrix_get_axis(input, MatrixAxis(next_next_best_axis)), VectorMix(best_axis)));
			quat_values[3] = half_inv_pseudo_trace *
				(vector_get_component(matrix_get_axis(input, MatrixAxis(next_best_axis)), VectorMix(next_next_best_axis)) -
					vector_get_component(matrix_get_axis(input, MatrixAxis(next_next_best_axis)), VectorMix(next_best_axis)));

			return quat_normalize(quat_unaligned_load(&quat_values[0]));
		}
	}

	// Multiplication order is as follow: local_to_world = matrix_mul(local_to_object, object_to_world)
	inline matrix3x4f RTM_SIMD_CALL matrix_mul(matrix3x4f_arg0 lhs, matrix3x4f_argn rhs)
	{
		vector4f tmp = vector_mul(vector_mix_xxxx(lhs.x_axis), rhs.x_axis);
		tmp = vector_mul_add(vector_mix_yyyy(lhs.x_axis), rhs.y_axis, tmp);
		tmp = vector_mul_add(vector_mix_zzzz(lhs.x_axis), rhs.z_axis, tmp);
		vector4f x_axis = tmp;

		tmp = vector_mul(vector_mix_xxxx(lhs.y_axis), rhs.x_axis);
		tmp = vector_mul_add(vector_mix_yyyy(lhs.y_axis), rhs.y_axis, tmp);
		tmp = vector_mul_add(vector_mix_zzzz(lhs.y_axis), rhs.z_axis, tmp);
		vector4f y_axis = tmp;

		tmp = vector_mul(vector_mix_xxxx(lhs.z_axis), rhs.x_axis);
		tmp = vector_mul_add(vector_mix_yyyy(lhs.z_axis), rhs.y_axis, tmp);
		tmp = vector_mul_add(vector_mix_zzzz(lhs.z_axis), rhs.z_axis, tmp);
		vector4f z_axis = tmp;

		tmp = vector_mul(vector_mix_xxxx(lhs.w_axis), rhs.x_axis);
		tmp = vector_mul_add(vector_mix_yyyy(lhs.w_axis), rhs.y_axis, tmp);
		tmp = vector_mul_add(vector_mix_zzzz(lhs.w_axis), rhs.z_axis, tmp);
		vector4f w_axis = vector_add(rhs.w_axis, tmp);
		return matrix_set(x_axis, y_axis, z_axis, w_axis);
	}

	inline vector4f RTM_SIMD_CALL matrix_mul_position(matrix3x4f_arg0 lhs, vector4f_arg4 rhs)
	{
		vector4f tmp0;
		vector4f tmp1;

		tmp0 = vector_mul(vector_mix_xxxx(rhs), lhs.x_axis);
		tmp0 = vector_mul_add(vector_mix_yyyy(rhs), lhs.y_axis, tmp0);
		tmp1 = vector_mul_add(vector_mix_zzzz(rhs), lhs.z_axis, lhs.w_axis);

		return vector_add(tmp0, tmp1);
	}

	namespace rtm_impl
	{
		// Note: This is a generic matrix 4x4 transpose, the resulting matrix is no longer
		// affine because the last column is no longer [0,0,0,1]
		inline matrix3x4f RTM_SIMD_CALL matrix_transpose(matrix3x4f_arg0 input)
		{
			vector4f tmp0 = vector_mix_xyab(input.x_axis, input.y_axis);
			vector4f tmp1 = vector_mix_zwcd(input.x_axis, input.y_axis);
			vector4f tmp2 = vector_mix_xyab(input.z_axis, input.w_axis);
			vector4f tmp3 = vector_mix_zwcd(input.z_axis, input.w_axis);

			vector4f x_axis = vector_mix_xzac(tmp0, tmp2);
			vector4f y_axis = vector_mix_ywbd(tmp0, tmp2);
			vector4f z_axis = vector_mix_xzac(tmp1, tmp3);
			vector4f w_axis = vector_mix_ywbd(tmp1, tmp3);
			return matrix3x4f{ x_axis, y_axis, z_axis, w_axis };
		}
	}

	inline matrix3x4f RTM_SIMD_CALL matrix_inverse(matrix3x4f_arg0 input)
	{
		// TODO: This is a generic matrix inverse function, implement the affine version?
		matrix3x4f input_transposed = rtm_impl::matrix_transpose(input);

		vector4f v00 = vector_mix_xxyy(input_transposed.z_axis);
		vector4f v01 = vector_mix_xxyy(input_transposed.x_axis);
		vector4f v02 = vector_mix_xzac(input_transposed.z_axis, input_transposed.x_axis);
		vector4f v10 = vector_mix_zwzw(input_transposed.w_axis);
		vector4f v11 = vector_mix_zwzw(input_transposed.y_axis);
		vector4f v12 = vector_mix_ywbd(input_transposed.w_axis, input_transposed.y_axis);

		vector4f d0 = vector_mul(v00, v10);
		vector4f d1 = vector_mul(v01, v11);
		vector4f d2 = vector_mul(v02, v12);

		v00 = vector_mix_zwzw(input_transposed.z_axis);
		v01 = vector_mix_zwzw(input_transposed.x_axis);
		v02 = vector_mix_ywbd(input_transposed.z_axis, input_transposed.x_axis);
		v10 = vector_mix_xxyy(input_transposed.w_axis);
		v11 = vector_mix_xxyy(input_transposed.y_axis);
		v12 = vector_mix_xzac(input_transposed.w_axis, input_transposed.y_axis);

		d0 = vector_neg_mul_sub(v00, v10, d0);
		d1 = vector_neg_mul_sub(v01, v11, d1);
		d2 = vector_neg_mul_sub(v02, v12, d2);

		v00 = vector_mix_yzxy(input_transposed.y_axis);
		v01 = vector_mix_zxyx(input_transposed.x_axis);
		v02 = vector_mix_yzxy(input_transposed.w_axis);
		vector4f v03 = vector_mix_zxyx(input_transposed.z_axis);
		v10 = vector_mix_bywx(d0, d2);
		v11 = vector_mix_wbyz(d0, d2);
		v12 = vector_mix_dywx(d1, d2);
		vector4f v13 = vector_mix_wdyz(d1, d2);

		vector4f c0 = vector_mul(v00, v10);
		vector4f c2 = vector_mul(v01, v11);
		vector4f c4 = vector_mul(v02, v12);
		vector4f c6 = vector_mul(v03, v13);

		v00 = vector_mix_zwyz(input_transposed.y_axis);
		v01 = vector_mix_wzwy(input_transposed.x_axis);
		v02 = vector_mix_zwyz(input_transposed.w_axis);
		v03 = vector_mix_wzwy(input_transposed.z_axis);
		v10 = vector_mix_wxya(d0, d2);
		v11 = vector_mix_zyax(d0, d2);
		v12 = vector_mix_wxyc(d1, d2);
		v13 = vector_mix_zycx(d1, d2);

		c0 = vector_neg_mul_sub(v00, v10, c0);
		c2 = vector_neg_mul_sub(v01, v11, c2);
		c4 = vector_neg_mul_sub(v02, v12, c4);
		c6 = vector_neg_mul_sub(v03, v13, c6);

		v00 = vector_mix_wxwx(input_transposed.y_axis);
		v01 = vector_mix_ywxz(input_transposed.x_axis);
		v02 = vector_mix_wxwx(input_transposed.w_axis);
		v03 = vector_mix_ywxz(input_transposed.z_axis);
		v10 = vector_mix_zbaz(d0, d2);
		v11 = vector_mix_bxwa(d0, d2);
		v12 = vector_mix_zdcz(d1, d2);
		v13 = vector_mix_dxwc(d1, d2);

		vector4f c1 = vector_neg_mul_sub(v00, v10, c0);
		c0 = vector_mul_add(v00, v10, c0);
		vector4f c3 = vector_mul_add(v01, v11, c2);
		c2 = vector_neg_mul_sub(v01, v11, c2);
		vector4f c5 = vector_neg_mul_sub(v02, v12, c4);
		c4 = vector_mul_add(v02, v12, c4);
		vector4f c7 = vector_mul_add(v03, v13, c6);
		c6 = vector_neg_mul_sub(v03, v13, c6);

		vector4f x_axis = vector_mix_xbzd(c0, c1);
		vector4f y_axis = vector_mix_xbzd(c2, c3);
		vector4f z_axis = vector_mix_xbzd(c4, c5);
		vector4f w_axis = vector_mix_xbzd(c6, c7);

		float det = vector_dot(x_axis, input_transposed.x_axis);
		vector4f inv_det = vector_set(reciprocal(det));

		x_axis = vector_mul(x_axis, inv_det);
		y_axis = vector_mul(y_axis, inv_det);
		z_axis = vector_mul(z_axis, inv_det);
		w_axis = vector_mul(w_axis, inv_det);

#if defined(RTM_NO_INTRINSICS)
		w_axis = vector_set(vector_get_x(w_axis), vector_get_y(w_axis), vector_get_z(w_axis), 1.0f);
#endif

		return matrix_set(x_axis, y_axis, z_axis, w_axis);
	}

	inline matrix3x4f RTM_SIMD_CALL matrix_remove_scale(matrix3x4f_arg0 input)
	{
		matrix3x4f result;
		result.x_axis = vector_normalize3(input.x_axis);
		result.y_axis = vector_normalize3(input.y_axis);
		result.z_axis = vector_normalize3(input.z_axis);
		result.w_axis = input.w_axis;
		return result;
	}
}
