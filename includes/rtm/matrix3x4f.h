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
#include "rtm/vector4f.h"
#include "rtm/quatf.h"
#include "rtm/impl/compiler_utils.h"
#include "rtm/impl/matrix_common.h"
#include "rtm/impl/matrix_affine_common.h"

RTM_IMPL_FILE_PRAGMA_PUSH

namespace rtm
{
	//////////////////////////////////////////////////////////////////////////
	// Converts a rotation 3x3 matrix into a 3x4 affine matrix.
	//////////////////////////////////////////////////////////////////////////
	inline matrix3x4f RTM_SIMD_CALL matrix_from_rotation(matrix3x3f_arg0 rotation) RTM_NO_EXCEPT
	{
		return matrix3x4f{ rotation.x_axis, rotation.y_axis, rotation.z_axis, vector_zero() };
	}

	//////////////////////////////////////////////////////////////////////////
	// Converts a translation vector into a 3x4 affine matrix.
	//////////////////////////////////////////////////////////////////////////
	inline matrix3x4f RTM_SIMD_CALL matrix_from_translation(vector4f_arg0 translation) RTM_NO_EXCEPT
	{
		return matrix3x4f{ vector_set(1.0F, 0.0F, 0.0F, 0.0F), vector_set(0.0F, 1.0F, 0.0F, 0.0F), vector_set(0.0F, 0.0F, 1.0F, 0.0F), translation };
	}

	//////////////////////////////////////////////////////////////////////////
	// Sets a 3x4 affine matrix from a rotation quaternion, translation, and 3D scale.
	//////////////////////////////////////////////////////////////////////////
	inline matrix3x4f RTM_SIMD_CALL matrix_from_qvv(quatf_arg0 quat, vector4f_arg1 translation, vector4f_arg2 scale) RTM_NO_EXCEPT
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

		const float scale_x = vector_get_x(scale);
		const float scale_y = vector_get_y(scale);
		const float scale_z = vector_get_z(scale);

		const vector4f x_axis = vector_mul(vector_set(1.0F - (yy + zz), xy + wz, xz - wy, 0.0F), scale_x);
		const vector4f y_axis = vector_mul(vector_set(xy - wz, 1.0F - (xx + zz), yz + wx, 0.0F), scale_y);
		const vector4f z_axis = vector_mul(vector_set(xz + wy, yz - wx, 1.0F - (xx + yy), 0.0F), scale_z);
		return matrix3x4f{ x_axis, y_axis, z_axis, translation };
	}

	//////////////////////////////////////////////////////////////////////////
	// Converts a QVV transform into a 3x4 affine matrix.
	//////////////////////////////////////////////////////////////////////////
	inline matrix3x4f RTM_SIMD_CALL matrix_from_qvv(qvvf_arg0 transform) RTM_NO_EXCEPT
	{
		return matrix_from_qvv(transform.rotation, transform.translation, transform.scale);
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the desired 3x4 affine matrix axis.
	//////////////////////////////////////////////////////////////////////////
	constexpr const vector4f& matrix_get_axis(const matrix3x4f& input, axis4 axis) RTM_NO_EXCEPT
	{
		return axis == axis4::x ? input.x_axis : (axis == axis4::y ? input.y_axis : (axis == axis4::z ? input.z_axis : input.w_axis));
	}

	//////////////////////////////////////////////////////////////////////////
	// Converts a 3x4 affine matrix into a rotation quaternion.
	//////////////////////////////////////////////////////////////////////////
	inline quatf RTM_SIMD_CALL quat_from_matrix(matrix3x4f_arg0 input) RTM_NO_EXCEPT
	{
		return rtm_impl::quat_from_matrix(input.x_axis, input.y_axis, input.z_axis);
	}

	//////////////////////////////////////////////////////////////////////////
	// Multiplies two 3x4 affine matrices.
	// Multiplication order is as follow: local_to_world = matrix_mul(local_to_object, object_to_world)
	//////////////////////////////////////////////////////////////////////////
	inline matrix3x4f RTM_SIMD_CALL matrix_mul(matrix3x4f_arg0 lhs, matrix3x4f_arg1 rhs) RTM_NO_EXCEPT
	{
		vector4f tmp = vector_mul(vector_dup_x(lhs.x_axis), rhs.x_axis);
		tmp = vector_mul_add(vector_dup_y(lhs.x_axis), rhs.y_axis, tmp);
		tmp = vector_mul_add(vector_dup_z(lhs.x_axis), rhs.z_axis, tmp);
		vector4f x_axis = tmp;

		tmp = vector_mul(vector_dup_x(lhs.y_axis), rhs.x_axis);
		tmp = vector_mul_add(vector_dup_y(lhs.y_axis), rhs.y_axis, tmp);
		tmp = vector_mul_add(vector_dup_z(lhs.y_axis), rhs.z_axis, tmp);
		vector4f y_axis = tmp;

		tmp = vector_mul(vector_dup_x(lhs.z_axis), rhs.x_axis);
		tmp = vector_mul_add(vector_dup_y(lhs.z_axis), rhs.y_axis, tmp);
		tmp = vector_mul_add(vector_dup_z(lhs.z_axis), rhs.z_axis, tmp);
		vector4f z_axis = tmp;

		tmp = vector_mul(vector_dup_x(lhs.w_axis), rhs.x_axis);
		tmp = vector_mul_add(vector_dup_y(lhs.w_axis), rhs.y_axis, tmp);
		tmp = vector_mul_add(vector_dup_z(lhs.w_axis), rhs.z_axis, tmp);
		vector4f w_axis = vector_add(rhs.w_axis, tmp);
		return matrix3x4f{ x_axis, y_axis, z_axis, w_axis };
	}

	//////////////////////////////////////////////////////////////////////////
	// Multiplies a 3x4 affine matrix and a 3D point.
	// Multiplication order is as follow: world_position = matrix_mul(local_position, local_to_world)
	//////////////////////////////////////////////////////////////////////////
	inline vector4f RTM_SIMD_CALL matrix_mul_point3(vector4f_arg0 point, matrix3x4f_arg0 mtx) RTM_NO_EXCEPT
	{
		vector4f tmp0;
		vector4f tmp1;

		tmp0 = vector_mul(vector_dup_x(point), mtx.x_axis);
		tmp0 = vector_mul_add(vector_dup_y(point), mtx.y_axis, tmp0);
		tmp1 = vector_mul_add(vector_dup_z(point), mtx.z_axis, mtx.w_axis);

		return vector_add(tmp0, tmp1);
	}

	//////////////////////////////////////////////////////////////////////////
	// Transposes a 3x4 affine matrix.
	// Note: This transposes the upper 3x3 rotation/scale part of the matrix
	// and it discards the translation. This is because a transposed 4x4 affine
	// matrix is no longer affine due to its last row no longer being [0, 0, 0, 1].
	// The most common usage of an affine transpose operation is to construct the
	// inverse transpose used to transform normal bi-vectors.
	//////////////////////////////////////////////////////////////////////////
	inline matrix3x3f RTM_SIMD_CALL matrix_transpose(matrix3x4f_arg0 input) RTM_NO_EXCEPT
	{
		const vector4f v00_v01_v10_v11 = vector_mix<mix4::x, mix4::y, mix4::a, mix4::b>(input.x_axis, input.y_axis);
		const vector4f v02_v03_v12_v13 = vector_mix<mix4::z, mix4::w, mix4::c, mix4::d>(input.x_axis, input.y_axis);

		const vector4f x_axis = vector_mix<mix4::x, mix4::z, mix4::a, mix4::c>(v00_v01_v10_v11, input.z_axis);
		const vector4f y_axis = vector_mix<mix4::y, mix4::w, mix4::b, mix4::d>(v00_v01_v10_v11, input.z_axis);
		const vector4f z_axis = vector_mix<mix4::x, mix4::z, mix4::c, mix4::c>(v02_v03_v12_v13, input.z_axis);
		return matrix3x3f{ x_axis, y_axis, z_axis };
	}

	//////////////////////////////////////////////////////////////////////////
	// Inverses a 3x4 affine matrix.
	// If the input matrix is not invertible, the result is undefined.
	// For a safe alternative, supply a fallback value and a threshold.
	//////////////////////////////////////////////////////////////////////////
	inline matrix3x4f RTM_SIMD_CALL matrix_inverse(matrix3x4f_arg0 input) RTM_NO_EXCEPT
	{
		// Invert the 3x3 portion of the matrix that contains the rotation and 3D scale
		const vector4f v00_v01_v10_v11 = vector_mix<mix4::x, mix4::y, mix4::a, mix4::b>(input.x_axis, input.y_axis);
		const vector4f v02_v03_v12_v13 = vector_mix<mix4::z, mix4::w, mix4::c, mix4::d>(input.x_axis, input.y_axis);

		const vector4f v00_v10_v20_v22 = vector_mix<mix4::x, mix4::z, mix4::a, mix4::c>(v00_v01_v10_v11, input.z_axis);
		const vector4f v01_v11_v21_v23 = vector_mix<mix4::y, mix4::w, mix4::b, mix4::d>(v00_v01_v10_v11, input.z_axis);
		const vector4f v02_v12_v22_v22 = vector_mix<mix4::x, mix4::z, mix4::c, mix4::c>(v02_v03_v12_v13, input.z_axis);

		const vector4f v11_v21_v01 = vector_mix<mix4::y, mix4::z, mix4::b, mix4::c>(v01_v11_v21_v23, input.x_axis);
		const vector4f v22_v02_v12_v10 = vector_mix<mix4::z, mix4::x, mix4::c, mix4::a>(v02_v12_v22_v22, input.y_axis);
		const vector4f v11v22_v21v02_v01v12 = vector_mul(v11_v21_v01, v22_v02_v12_v10);

		const vector4f v01_v02_v11_v12 = vector_mix<mix4::y, mix4::z, mix4::b, mix4::c>(input.x_axis, input.y_axis);

		const vector4f v12_v01_v11 = vector_mix<mix4::w, mix4::x, mix4::b, mix4::c>(v01_v02_v11_v12, v01_v11_v21_v23);
		const vector4f v21_v22_v02 = vector_mix<mix4::y, mix4::z, mix4::b, mix4::c>(input.z_axis, v22_v02_v12_v10);

		vector4f x_axis = vector_neg_mul_sub(v12_v01_v11, v21_v22_v02, v11v22_v21v02_v01v12);

		const vector4f v20_v00_v10_v22 = vector_mix<mix4::z, mix4::x, mix4::d, mix4::a>(v00_v10_v20_v22, v22_v02_v12_v10);
		const vector4f v12_v22_v02 = vector_mix<mix4::y, mix4::z, mix4::c, mix4::c>(v02_v12_v22_v22, v21_v22_v02);
		const vector4f v20v12_v00v22_v10v02 = vector_mul(v20_v00_v10_v22, v12_v22_v02);

		const vector4f v10_v02_v00 = vector_mix<mix4::w, mix4::y, mix4::b, mix4::c>(v22_v02_v12_v10, v20_v00_v10_v22);
		const vector4f v22_v20_v12 = vector_mix<mix4::w, mix4::x, mix4::a, mix4::c>(v20_v00_v10_v22, v12_v22_v02);

		vector4f y_axis = vector_neg_mul_sub(v10_v02_v00, v22_v20_v12, v20v12_v00v22_v10v02);

		const vector4f v10_v20_v00 = vector_mix<mix4::z, mix4::x, mix4::c, mix4::c>(v20_v00_v10_v22, v10_v02_v00);
		const vector4f v21_v01_v11 = vector_mix<mix4::y, mix4::z, mix4::c, mix4::c>(v11_v21_v01, v12_v01_v11);
		const vector4f v10v21_v20v01_v00v11 = vector_mul(v10_v20_v00, v21_v01_v11);

		const vector4f v20_v00_v01 = vector_mix<mix4::y, mix4::z, mix4::c, mix4::c>(v10_v20_v00, v11_v21_v01);
		const vector4f v11_v21_v10 = vector_mix<mix4::x, mix4::y, mix4::a, mix4::c>(v11_v21_v01, v10_v20_v00);

		vector4f z_axis = vector_neg_mul_sub(v20_v00_v01, v11_v21_v10, v10v21_v20v01_v00v11);

		const vector4f o00_o00_o10_o10 = vector_mix<mix4::x, mix4::x, mix4::a, mix4::a>(x_axis, y_axis);
		const vector4f o00_o10_o20 = vector_mix<mix4::x, mix4::z, mix4::a, mix4::a>(o00_o00_o10_o10, z_axis);

		const scalarf det = vector_dot3(o00_o10_o20, input.x_axis);
		const vector4f inv_det = vector_set(scalar_reciprocal(det));

		x_axis = vector_mul(x_axis, inv_det);
		y_axis = vector_mul(y_axis, inv_det);
		z_axis = vector_mul(z_axis, inv_det);

		// Invert the translation
		const vector4f tmp0 = vector_mul(vector_dup_z(input.w_axis), z_axis);
		const vector4f tmp1 = vector_mul_add(vector_dup_y(input.w_axis), y_axis, tmp0);
		vector4f w_axis = vector_neg(vector_mul_add(vector_dup_x(input.w_axis), x_axis, tmp1));

		return matrix3x4f{ x_axis, y_axis, z_axis, w_axis };
	}

	//////////////////////////////////////////////////////////////////////////
	// Inverses a 3x4 affine matrix.
	// If the input matrix has a determinant whose absolute value is below the supplied threshold, the
	// fall back value is returned instead.
	//////////////////////////////////////////////////////////////////////////
	inline matrix3x4f RTM_SIMD_CALL matrix_inverse(matrix3x4f_arg0 input, matrix3x4f_arg1 fallback, float threshold = 1.0E-8F) RTM_NO_EXCEPT
	{
		// Invert the 3x3 portion of the matrix that contains the rotation and 3D scale
		const vector4f v00_v01_v10_v11 = vector_mix<mix4::x, mix4::y, mix4::a, mix4::b>(input.x_axis, input.y_axis);
		const vector4f v02_v03_v12_v13 = vector_mix<mix4::z, mix4::w, mix4::c, mix4::d>(input.x_axis, input.y_axis);

		const vector4f v00_v10_v20_v22 = vector_mix<mix4::x, mix4::z, mix4::a, mix4::c>(v00_v01_v10_v11, input.z_axis);
		const vector4f v01_v11_v21_v23 = vector_mix<mix4::y, mix4::w, mix4::b, mix4::d>(v00_v01_v10_v11, input.z_axis);
		const vector4f v02_v12_v22_v22 = vector_mix<mix4::x, mix4::z, mix4::c, mix4::c>(v02_v03_v12_v13, input.z_axis);

		const vector4f v11_v21_v01 = vector_mix<mix4::y, mix4::z, mix4::b, mix4::c>(v01_v11_v21_v23, input.x_axis);
		const vector4f v22_v02_v12_v10 = vector_mix<mix4::z, mix4::x, mix4::c, mix4::a>(v02_v12_v22_v22, input.y_axis);
		const vector4f v11v22_v21v02_v01v12 = vector_mul(v11_v21_v01, v22_v02_v12_v10);

		const vector4f v01_v02_v11_v12 = vector_mix<mix4::y, mix4::z, mix4::b, mix4::c>(input.x_axis, input.y_axis);

		const vector4f v12_v01_v11 = vector_mix<mix4::w, mix4::x, mix4::b, mix4::c>(v01_v02_v11_v12, v01_v11_v21_v23);
		const vector4f v21_v22_v02 = vector_mix<mix4::y, mix4::z, mix4::b, mix4::c>(input.z_axis, v22_v02_v12_v10);

		vector4f x_axis = vector_neg_mul_sub(v12_v01_v11, v21_v22_v02, v11v22_v21v02_v01v12);

		const vector4f v20_v00_v10_v22 = vector_mix<mix4::z, mix4::x, mix4::d, mix4::a>(v00_v10_v20_v22, v22_v02_v12_v10);
		const vector4f v12_v22_v02 = vector_mix<mix4::y, mix4::z, mix4::c, mix4::c>(v02_v12_v22_v22, v21_v22_v02);
		const vector4f v20v12_v00v22_v10v02 = vector_mul(v20_v00_v10_v22, v12_v22_v02);

		const vector4f v10_v02_v00 = vector_mix<mix4::w, mix4::y, mix4::b, mix4::c>(v22_v02_v12_v10, v20_v00_v10_v22);
		const vector4f v22_v20_v12 = vector_mix<mix4::w, mix4::x, mix4::a, mix4::c>(v20_v00_v10_v22, v12_v22_v02);

		vector4f y_axis = vector_neg_mul_sub(v10_v02_v00, v22_v20_v12, v20v12_v00v22_v10v02);

		const vector4f v10_v20_v00 = vector_mix<mix4::z, mix4::x, mix4::c, mix4::c>(v20_v00_v10_v22, v10_v02_v00);
		const vector4f v21_v01_v11 = vector_mix<mix4::y, mix4::z, mix4::c, mix4::c>(v11_v21_v01, v12_v01_v11);
		const vector4f v10v21_v20v01_v00v11 = vector_mul(v10_v20_v00, v21_v01_v11);

		const vector4f v20_v00_v01 = vector_mix<mix4::y, mix4::z, mix4::c, mix4::c>(v10_v20_v00, v11_v21_v01);
		const vector4f v11_v21_v10 = vector_mix<mix4::x, mix4::y, mix4::a, mix4::c>(v11_v21_v01, v10_v20_v00);

		vector4f z_axis = vector_neg_mul_sub(v20_v00_v01, v11_v21_v10, v10v21_v20v01_v00v11);

		const vector4f o00_o00_o10_o10 = vector_mix<mix4::x, mix4::x, mix4::a, mix4::a>(x_axis, y_axis);
		const vector4f o00_o10_o20 = vector_mix<mix4::x, mix4::z, mix4::a, mix4::a>(o00_o00_o10_o10, z_axis);

		const scalarf det = vector_dot3(o00_o10_o20, input.x_axis);
		if (scalar_cast(scalar_abs(det)) < threshold)
			return fallback;

		const vector4f inv_det = vector_set(scalar_reciprocal(det));

		x_axis = vector_mul(x_axis, inv_det);
		y_axis = vector_mul(y_axis, inv_det);
		z_axis = vector_mul(z_axis, inv_det);

		// Invert the translation
		const vector4f tmp0 = vector_mul(vector_dup_z(input.w_axis), z_axis);
		const vector4f tmp1 = vector_mul_add(vector_dup_y(input.w_axis), y_axis, tmp0);
		vector4f w_axis = vector_neg(vector_mul_add(vector_dup_x(input.w_axis), x_axis, tmp1));

		return matrix3x4f{ x_axis, y_axis, z_axis, w_axis };
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the determinant of the 3x3 rotation/scale part of the input 3x4 matrix.
	//////////////////////////////////////////////////////////////////////////
	inline scalarf RTM_SIMD_CALL matrix_determinant(matrix3x4f_arg0 input) RTM_NO_EXCEPT
	{
		const vector4f v00_v01_v10_v11 = vector_mix<mix4::x, mix4::y, mix4::a, mix4::b>(input.x_axis, input.y_axis);
		const vector4f v02_v03_v12_v13 = vector_mix<mix4::z, mix4::w, mix4::c, mix4::d>(input.x_axis, input.y_axis);

		const vector4f v00_v10_v20_v22 = vector_mix<mix4::x, mix4::z, mix4::a, mix4::c>(v00_v01_v10_v11, input.z_axis);
		const vector4f v01_v11_v21_v23 = vector_mix<mix4::y, mix4::w, mix4::b, mix4::d>(v00_v01_v10_v11, input.z_axis);
		const vector4f v02_v12_v22_v22 = vector_mix<mix4::x, mix4::z, mix4::c, mix4::c>(v02_v03_v12_v13, input.z_axis);

		const vector4f v11_v21_v01 = vector_mix<mix4::y, mix4::z, mix4::b, mix4::c>(v01_v11_v21_v23, input.x_axis);
		const vector4f v22_v02_v12_v10 = vector_mix<mix4::z, mix4::x, mix4::c, mix4::a>(v02_v12_v22_v22, input.y_axis);
		const vector4f v11v22_v21v02_v01v12 = vector_mul(v11_v21_v01, v22_v02_v12_v10);

		const vector4f v01_v02_v11_v12 = vector_mix<mix4::y, mix4::z, mix4::b, mix4::c>(input.x_axis, input.y_axis);

		const vector4f v12_v01_v11 = vector_mix<mix4::w, mix4::x, mix4::b, mix4::c>(v01_v02_v11_v12, v01_v11_v21_v23);
		const vector4f v21_v22_v02 = vector_mix<mix4::y, mix4::z, mix4::b, mix4::c>(input.z_axis, v22_v02_v12_v10);

		vector4f x_axis = vector_neg_mul_sub(v12_v01_v11, v21_v22_v02, v11v22_v21v02_v01v12);

		const vector4f v20_v00_v10_v22 = vector_mix<mix4::z, mix4::x, mix4::d, mix4::a>(v00_v10_v20_v22, v22_v02_v12_v10);
		const vector4f v12_v22_v02 = vector_mix<mix4::y, mix4::z, mix4::c, mix4::c>(v02_v12_v22_v22, v21_v22_v02);
		const vector4f v20v12_v00v22_v10v02 = vector_mul(v20_v00_v10_v22, v12_v22_v02);

		const vector4f v10_v02_v00 = vector_mix<mix4::w, mix4::y, mix4::b, mix4::c>(v22_v02_v12_v10, v20_v00_v10_v22);
		const vector4f v22_v20_v12 = vector_mix<mix4::w, mix4::x, mix4::a, mix4::c>(v20_v00_v10_v22, v12_v22_v02);

		vector4f y_axis = vector_neg_mul_sub(v10_v02_v00, v22_v20_v12, v20v12_v00v22_v10v02);

		const vector4f v10_v20_v00 = vector_mix<mix4::z, mix4::x, mix4::c, mix4::c>(v20_v00_v10_v22, v10_v02_v00);
		const vector4f v21_v01_v11 = vector_mix<mix4::y, mix4::z, mix4::c, mix4::c>(v11_v21_v01, v12_v01_v11);
		const vector4f v10v21_v20v01_v00v11 = vector_mul(v10_v20_v00, v21_v01_v11);

		const vector4f v20_v00_v01 = vector_mix<mix4::y, mix4::z, mix4::c, mix4::c>(v10_v20_v00, v11_v21_v01);
		const vector4f v11_v21_v10 = vector_mix<mix4::x, mix4::y, mix4::a, mix4::c>(v11_v21_v01, v10_v20_v00);

		vector4f z_axis = vector_neg_mul_sub(v20_v00_v01, v11_v21_v10, v10v21_v20v01_v00v11);

		const vector4f o00_o00_o10_o10 = vector_mix<mix4::x, mix4::x, mix4::a, mix4::a>(x_axis, y_axis);
		const vector4f o00_o10_o20 = vector_mix<mix4::x, mix4::z, mix4::a, mix4::a>(o00_o00_o10_o10, z_axis);

		return vector_dot3(o00_o10_o20, input.x_axis);
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the minor of the 3x3 rotation/scale part of the input 3x4 matrix.
	// See: https://en.wikipedia.org/wiki/Minor_(linear_algebra)
	// The minor is the determinant of the sub-matrix input when the specified
	// row and column are removed.
	//////////////////////////////////////////////////////////////////////////
	inline scalarf RTM_SIMD_CALL matrix_minor(matrix3x4f_arg0 input, axis3 row, axis3 column) RTM_NO_EXCEPT
	{
		// The minor boils down to calculating the determinant of a 2x2 matrix.
		// det([a, b], [c, d]) = (a * d) - (b * c)

		// Find which two rows we need.
		vector4f row0;
		vector4f row1;
		if (row == axis3::x)
		{
			row0 = input.y_axis;
			row1 = input.z_axis;
		}
		else if (row == axis3::y)
		{
			row0 = input.x_axis;
			row1 = input.z_axis;
		}
		else
		{
			row0 = input.x_axis;
			row1 = input.y_axis;
		}

		// Because our input is a 3x3 matrix, there are only a few possibilities for the 2x2 part:
		// row0 = [x0, y0, z0]
		// row1 = [x1, y1, z1]
		// det([x0, y0], [x1, y1]) = (x0 * y1) - (y0 * x1) (z removed)
		// det([x0, z0], [x1, z1]) = (x0 * z1) - (z0 * x1) (y removed)
		// det([y0, z0], [y1, z1]) = (y0 * z1) - (z0 * y1) (x removed)
		// det([column0, column1], [column2, column3]) = (column0 * column3) - (column1 * column2)

		// For performance reasons, we can compute all three determinants at the same time.
		const vector4f column0 = vector_mix<mix4::x, mix4::x, mix4::y, mix4::y>(row0, row0);
		const vector4f column1 = vector_mix<mix4::y, mix4::z, mix4::z, mix4::z>(row0, row0);
		const vector4f column2 = vector_mix<mix4::x, mix4::x, mix4::y, mix4::y>(row1, row1);
		const vector4f column3 = vector_mix<mix4::y, mix4::z, mix4::z, mix4::z>(row1, row1);

		const vector4f determinants = vector_neg_mul_sub(column1, column2, vector_mul(column0, column3));

		// Extract the one we need
		if (column == axis3::x)
			return vector_get_z(determinants);
		else if (column == axis3::y)
			return vector_get_y(determinants);
		else
			return vector_get_x(determinants);
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the cofactor matrix of the 3x3 rotation/scale part of the input 3x4 matrix.
	// See: https://en.wikipedia.org/wiki/Minor_(linear_algebra)#Cofactor_expansion_of_the_determinant
	// The proper way to transform a normal by an affine matrix with non-uniform scale
	// is to multiply the normal with the cofactor matrix.
	// See: https://github.com/graphitemaster/normals_revisited
	//////////////////////////////////////////////////////////////////////////
	inline matrix3x3f RTM_SIMD_CALL matrix_cofactor(matrix3x4f_arg0 input) RTM_NO_EXCEPT
	{
		const vector4f x_axis = vector_cross3(input.y_axis, input.z_axis);
		const vector4f y_axis = vector_cross3(input.z_axis, input.x_axis);
		const vector4f z_axis = vector_cross3(input.x_axis, input.y_axis);
		return matrix3x3f{ x_axis, y_axis, z_axis };
	}

	//////////////////////////////////////////////////////////////////////////
	// Removes the 3D scale from a 3x4 affine matrix.
	// Note that if the scaling is 0.0 for a particular axis, the original rotation axis cannot
	// be recovered trivially and no attempt is done to do so. In theory, we could use the other axes
	// to try and recover it.
	// TODO: Implement rotation recovering, perhaps in a separate function and rename this
	// one to matrix_remove_non_zero_scale(..)
	//////////////////////////////////////////////////////////////////////////
	inline matrix3x4f RTM_SIMD_CALL matrix_remove_scale(matrix3x4f_arg0 input) RTM_NO_EXCEPT
	{
		matrix3x4f result;
		result.x_axis = vector_normalize3(input.x_axis, input.x_axis);
		result.y_axis = vector_normalize3(input.y_axis, input.y_axis);
		result.z_axis = vector_normalize3(input.z_axis, input.z_axis);
		result.w_axis = input.w_axis;
		return result;
	}
}

RTM_IMPL_FILE_PRAGMA_POP
