#pragma once

////////////////////////////////////////////////////////////////////////////////
// The MIT License (MIT)
//
// Copyright (c) 2024 Nicholas Frechette & Realtime Math contributors
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
#include "rtm/mask4f.h"
#include "rtm/matrix3x3f.h"
#include "rtm/matrix3x4f.h"
#include "rtm/quatf.h"
#include "rtm/vector4f.h"
#include "rtm/version.h"
#include "rtm/impl/compiler_utils.h"
#include "rtm/impl/matrix_affine_common.h"
#include "rtm/experimental/impl/vqm_common.h"

RTM_IMPL_FILE_PRAGMA_PUSH

namespace rtm
{
	RTM_IMPL_VERSION_NAMESPACE_BEGIN

	//////////////////////////////////////////////////////////////////////////
	// Reference: "The VQM-Group and its Applications" by Michael Aristidou and Xin Li.
	//
	// Conventions:
	// - RTM uses row vectors. A VQM transform T = [v, q, M] transforms a point p as follows:
	//   p' = p * M * R + v
	//   M is the 3x3 scale/shear matrix, R is the rotation matrix of the quaternion q, and v is the translation.
	// - R^T is the transpose of R. It is the rotation matrix of the conjugate of q.
	// - quat_mul_vector3(a, q) is equal to a * R. Thus a rotation of each row of a matrix A with q gives A * R.
	//
	// Why VQM:
	// - Rotation, translation, and scale/shear do not interpolate in the same way. Thus VQM keeps them separate.
	// - In an affine matrix, the scale/shear and the rotation mix in the upper 3x3 part.
	//   More than one decomposition gives the same 3x3 part. Thus it is difficult to find them again.
	// - A quaternion is compact and numerically stable, and it interpolates well. Thus VQM keeps the rotation as a quaternion.
	//
	// Multiplication:
	// - Let T1 = [v1, q1, M1] and T2 = [v2, q2, M2]. T1 transforms the point first, then T2 transforms the result:
	//   p'' = (p * M1 * R1 + v1) * M2 * R2 + v2
	//   p'' = p * M1 * R1 * M2 * R2 + v1 * M2 * R2 + v2
	// - The result is T3 = [v3, q3, M3] with R3 = R1 * R2. Then:
	//   M3 * R1 * R2 = M1 * R1 * M2 * R2
	//   M3 = (M1 * R1) * (M2 * R1^T)
	//   v3 = v1 * M2 * R2 + v2
	// - M1 * R1 is M1 with each row rotated by q1. M2 * R1^T is M2 with each row rotated by the conjugate of q1.
	//   Thus the multiplication does not need a rotation matrix.
	// - The rotation of T1 moves into M3. Thus M3 can contain shear when M1 and M2 contain only scale.
	//   This is why VQV, which stores only a scale, does not form a group.
	//
	// Inverse:
	// - From p' = p * M * R + v, we get p = (p' - v) * R^T * M^-1.
	// - The inverse is T' = [v', q', M'] with R' = R^T. Then:
	//   M' * R^T = R^T * M^-1
	//   M' = R^T * (M^-1 * R)
	//   v' = -v * R^T * M^-1
	// - M^-1 * R is M^-1 with each row rotated by q. The matrix R^T is on the left side, thus it is not a row rotation.
	//   The function uses the rotation matrix of the conjugate of q for it.
	//////////////////////////////////////////////////////////////////////////

	//////////////////////////////////////////////////////////////////////////
	// A VQM transform contains a 3D rotation (quaternion), a 3D translation (vector3),
	// and a 3D scale/shear part (matrix 3x3). The scale/shear part can contain a
	// non-uniform scale, shear, and a reflection.
	// VQM transforms form a group: the multiplication and the inverse are well defined.
	// The multiplication is associative but not commutative, as with quaternions and matrices.
	// The rotation must be a normalized quaternion.
	//////////////////////////////////////////////////////////////////////////
#if 0	// defined in types.h
	struct vqmf
	{
		// The internal format is meant to be opaque, use vqm_* accessors
		quatf		rotation;
		vector4f	translation;
		vector4f	x_axis;
		vector4f	y_axis;
		vector4f	z_axis;
	};
#endif

	template<> struct related_types<vqmf> : related_types<float> {};

	//////////////////////////////////////////////////////////////////////////
	// Casts a VQM transform float64 variant to a float32 variant.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vqmf RTM_SIMD_CALL vqm_cast(const vqmd& input) RTM_NO_EXCEPT
	{
		vqmf result;
		result.rotation = quat_cast(input.rotation);
		result.translation = vector_cast(input.translation);
		result.x_axis = vector_cast(input.x_axis);
		result.y_axis = vector_cast(input.y_axis);
		result.z_axis = vector_cast(input.z_axis);

		return result;
	}

	//////////////////////////////////////////////////////////////////////////
	// Converts a 3x3 matrix into a VQM transform.
	// The matrix can contain rotation, scale, shear, and reflection. The translation is zero.
	// The rotation part is a pure rotation. If the matrix contains a reflection, the
	// scale/shear part contains it.
	// More than one VQM transform gives the same matrix. The function returns one of them.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vqmf RTM_SIMD_CALL vqm_from_matrix(matrix3x3f_arg0 input) RTM_NO_EXCEPT
	{
		// Let A be the input matrix. We find a rotation matrix R and a scale/shear matrix M with:
		// A = M * R
		// For each rotation matrix R, the matrix M = A * R^T satisfies this equation because R * R^T = I.
		// Thus the choice of R selects the result, and the product M * R is equal to A for each R.
		//
		// We find R with the Gram-Schmidt process on the rows of A:
		//     - The first axis of R is the largest row of A, normalized.
		//     - The second axis of R is the part of a second row that is orthogonal to the first axis, normalized.
		//       Of the two remaining rows, we use the row that has the largest orthogonal part, for precision.
		//     - The third axis of R is the cross product of the first two axes.
		// With u a unit vector, cross(cross(u, a), u) = a - dot(a, u) * u is the part of a that is orthogonal to u.
		// The length of cross(u, a) is the length of that orthogonal part.
		// The third axis has the sign that gives R a determinant of +1. Thus R is a rotation. If A contains a
		// reflection, M contains it and the determinant of M is negative.
		// When A has no shear and a positive scale (A = S * R0 with S diagonal), R is equal to R0 and M is equal to S.
		//
		// If the largest row is zero, all the rows are zero. R is the identity and M is equal to A.
		// If the orthogonal parts of the two remaining rows are zero, all the rows are parallel to the first axis.
		// Then we use an axis that is orthogonal to the first axis, as vqv_from_matrix does.

		// We use a threshold to test for zero because division with near zero values is imprecise
		// and might not yield a normalized result
		const float zero_scale_threshold = 1.0E-8F;

		const vector4f* input_axes = &input.x_axis;

		const float axis_lengths[3] = { vector_length3(input.x_axis), vector_length3(input.y_axis), vector_length3(input.z_axis) };

		int32_t first_index = 0;
		if (axis_lengths[1] > axis_lengths[first_index])
			first_index = 1;
		if (axis_lengths[2] > axis_lengths[first_index])
			first_index = 2;

		const float first_length = axis_lengths[first_index];
		if (first_length < zero_scale_threshold)
		{
			// The largest row is zero which means all three are zero
			// We use the identity rotation since its value is not recoverable
			vqmf result;
			result.rotation = quat_identity();
			result.translation = vector_zero();
			result.x_axis = input.x_axis;
			result.y_axis = input.y_axis;
			result.z_axis = input.z_axis;

			return result;
		}

		const vector4f first_axis = vector_div(input_axes[first_index], vector_set(first_length));

		// The two other rows follow the first row in the cyclic order X -> Y -> Z -> X
		const int32_t next_index = (first_index + 1) % 3;
		const int32_t last_index = (first_index + 2) % 3;

		const vector4f next_normal = vector_cross3(first_axis, input_axes[next_index]);
		const vector4f last_normal = vector_cross3(first_axis, input_axes[last_index]);
		const float next_orthogonal_length = vector_length3(next_normal);
		const float last_orthogonal_length = vector_length3(last_normal);

		int32_t second_index;
		int32_t third_index;
		vector4f normal;
		if (next_orthogonal_length >= last_orthogonal_length)
		{
			second_index = next_index;
			third_index = last_index;
			normal = next_normal;
		}
		else
		{
			second_index = last_index;
			third_index = next_index;
			normal = last_normal;
		}

		if (scalar_max(next_orthogonal_length, last_orthogonal_length) < zero_scale_threshold)
		{
			// All the rows are parallel to the first axis
			// We build an orthogonal basis around the first axis
			const matrix3x3f identity3x3 = matrix_identity();
			const vector4f first_y_dot = vector_dot3_as_vector(first_axis, identity3x3.y_axis);
			const mask4f is_first_parallel_to_y = vector_greater_equal(vector_abs(first_y_dot), vector_set(1.0F - zero_scale_threshold));
			const vector4f orthogonal_axis = vector_select(is_first_parallel_to_y, identity3x3.z_axis, identity3x3.y_axis);

			second_index = next_index;
			third_index = last_index;
			normal = vector_cross3(first_axis, orthogonal_axis);
		}

		// The second axis is orthogonal to the first axis, even when the normal is not exactly orthogonal to it
		// The third axis is orthogonal to the two others and (first, second, third) is a right-handed basis
		const vector4f second_axis = vector_normalize3(vector_cross3(normal, first_axis));
		const vector4f third_axis = vector_cross3(first_axis, second_axis);

		matrix3x3f rotation;
		vector4f* rotation_axes = &rotation.x_axis;
		rotation_axes[first_index] = first_axis;
		rotation_axes[second_index] = second_axis;

		// When the rows (first, second, third) are in the cyclic order X -> Y -> Z, the basis has the same
		// winding as the identity. Otherwise, we negate the third axis to get a determinant of +1.
		rotation_axes[third_index] = second_index == next_index ? third_axis : vector_neg(third_axis);

		// M = A * R^T
		const matrix3x3f scale_shear = matrix_mul(input, matrix_transpose(rotation));

		vqmf result;
		result.rotation = rtm_impl::quat_from_matrix(rotation.x_axis, rotation.y_axis, rotation.z_axis);
		result.translation = vector_zero();
		result.x_axis = scale_shear.x_axis;
		result.y_axis = scale_shear.y_axis;
		result.z_axis = scale_shear.z_axis;

		return result;
	}

	//////////////////////////////////////////////////////////////////////////
	// Converts a 3x4 affine matrix into a VQM transform.
	// See vqm_from_matrix(matrix3x3f) for the rotation and the scale/shear parts.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vqmf RTM_SIMD_CALL vqm_from_matrix(matrix3x4f_arg0 input) RTM_NO_EXCEPT
	{
		const matrix3x3f rotation_scale_shear = matrix_set(input.x_axis, input.y_axis, input.z_axis);

		vqmf result = vqm_from_matrix(rotation_scale_shear);
		result.translation = input.w_axis;

		return result;
	}

	//////////////////////////////////////////////////////////////////////////
	// Creates a VQM transform from a rotation quaternion, a translation, and a 3D scale.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vqmf RTM_SIMD_CALL vqm_set(quatf_arg0 rotation, vector4f_arg1 translation, vector4f_arg2 scale) RTM_NO_EXCEPT
	{
		vector4f zero = vector_zero();

		vqmf result;
		result.rotation = rotation;
		result.translation = translation;
		result.x_axis = vector_set_x(zero, vector_get_x_as_scalar(scale));
		result.y_axis = vector_set_y(zero, vector_get_y_as_scalar(scale));
		result.z_axis = vector_set_z(zero, vector_get_z_as_scalar(scale));

		return result;
	}

	//////////////////////////////////////////////////////////////////////////
	// Creates a VQM transform from a rotation quaternion, a translation, and a 3x3 scale/shear matrix.
	// The scale/shear matrix applies before the rotation: p' = p * M * R + t.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vqmf RTM_SIMD_CALL vqm_set(quatf_arg0 rotation, vector4f_arg1 translation, matrix3x3f_arg0 scale_shear) RTM_NO_EXCEPT
	{
		vqmf result;
		result.rotation = rotation;
		result.translation = translation;
		result.x_axis = scale_shear.x_axis;
		result.y_axis = scale_shear.y_axis;
		result.z_axis = scale_shear.z_axis;

		return result;
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the rotation part of a VQM transform.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE quatf RTM_SIMD_CALL vqm_get_rotation(const vqmf& input) RTM_NO_EXCEPT
	{
		return input.rotation;
	}

	//////////////////////////////////////////////////////////////////////////
	// Sets the rotation part of a VQM and returns the new value.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vqmf RTM_SIMD_CALL vqm_set_rotation(const vqmf& input, quatf_arg0 rotation) RTM_NO_EXCEPT
	{
		vqmf result = input;
		result.rotation = rotation;
		return result;
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the translation part of a VQM transform.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4f RTM_SIMD_CALL vqm_get_translation(const vqmf& input) RTM_NO_EXCEPT
	{
		return input.translation;
	}

	//////////////////////////////////////////////////////////////////////////
	// Sets the translation part of a VQM and returns the new value.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vqmf RTM_SIMD_CALL vqm_set_translation(const vqmf& input, vector4f_arg0 translation) RTM_NO_EXCEPT
	{
		vqmf result = input;
		result.translation = translation;
		return result;
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the scale part of a VQM transform: the diagonal of the scale/shear matrix.
	// When the scale/shear matrix contains shear, the scale is not the length of its rows.
	// Use vqm_get_scale_shear to get the full matrix.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4f RTM_SIMD_CALL vqm_get_scale(const vqmf& input) RTM_NO_EXCEPT
	{
		vector4f xyxy = vector_mix<mix4::x, mix4::b, mix4::x, mix4::b>(input.x_axis, input.y_axis);
		return vector_mix<mix4::x, mix4::y, mix4::c, mix4::d>(xyxy, input.z_axis);
	}

	//////////////////////////////////////////////////////////////////////////
	// Sets the scale part of a VQM and returns the new value.
	// The function sets the diagonal of the scale/shear matrix and keeps the other entries (the shear).
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vqmf RTM_SIMD_CALL vqm_set_scale(const vqmf& input, vector4f_arg0 scale) RTM_NO_EXCEPT
	{
		vqmf result = input;
		result.x_axis = vector_set_x(input.x_axis, vector_get_x_as_scalar(scale));
		result.y_axis = vector_set_y(input.y_axis, vector_get_y_as_scalar(scale));
		result.z_axis = vector_set_z(input.z_axis, vector_get_z_as_scalar(scale));
		return result;
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the scale/shear part of a VQM transform as a 3x3 matrix.
	// The matrix applies before the rotation: p' = p * M * R + t.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE matrix3x3f RTM_SIMD_CALL vqm_get_scale_shear(const vqmf& input) RTM_NO_EXCEPT
	{
		return matrix_set(input.x_axis, input.y_axis, input.z_axis);
	}

	//////////////////////////////////////////////////////////////////////////
	// Sets the scale/shear part of a VQM and returns the new value.
	// The matrix applies before the rotation: p' = p * M * R + t.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vqmf RTM_SIMD_CALL vqm_set_scale_shear(const vqmf& input, matrix3x3f_arg0 scale_shear) RTM_NO_EXCEPT
	{
		vqmf result = input;
		result.x_axis = scale_shear.x_axis;
		result.y_axis = scale_shear.y_axis;
		result.z_axis = scale_shear.z_axis;
		return result;
	}

	//////////////////////////////////////////////////////////////////////////
	// Adds two VQM transforms.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vqmf RTM_SIMD_CALL vqm_add(const vqmf& lhs, const vqmf& rhs) RTM_NO_EXCEPT
	{
		// T2 + T1 = [v2, q2, M2] + [v1, q1, M1] = [v2 + v1, q2 + q1, M2 + M1]
		vqmf result;
		result.rotation = quat_add(lhs.rotation, rhs.rotation);
		result.translation = vector_add(lhs.translation, rhs.translation);
		result.x_axis = vector_add(lhs.x_axis, rhs.x_axis);
		result.y_axis = vector_add(lhs.y_axis, rhs.y_axis);
		result.z_axis = vector_add(lhs.z_axis, rhs.z_axis);

		return result;
	}

	//////////////////////////////////////////////////////////////////////////
	// Multiplies two VQM transforms.
	// Multiplication order is as follow: local_to_world = vqm_mul(local_to_object, object_to_world)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vqmf RTM_SIMD_CALL vqm_mul(const vqmf& lhs, const vqmf& rhs) RTM_NO_EXCEPT
	{
		// lhs = [v1, q1, M1] and rhs = [v2, q2, M2]
		// result = [v1 * M2 * R2 + v2, quat_mul(q1, q2), (M1 * R1) * (M2 * R1^T)]
		// See the multiplication section at the top of this file

		const quatf inv_lhs_rotation = quat_conjugate(lhs.rotation);

		matrix3x3f lhs_scale_shear = matrix_set(lhs.x_axis, lhs.y_axis, lhs.z_axis);
		matrix3x3f rhs_scale_shear = matrix_set(rhs.x_axis, rhs.y_axis, rhs.z_axis);

		vqmf result;
		result.rotation = quat_mul(lhs.rotation, rhs.rotation);
		result.translation = vector_add(quat_mul_vector3(matrix_mul_vector3(lhs.translation, rhs_scale_shear), rhs.rotation), rhs.translation);

		rhs_scale_shear.x_axis = quat_mul_vector3(rhs_scale_shear.x_axis, inv_lhs_rotation);
		rhs_scale_shear.y_axis = quat_mul_vector3(rhs_scale_shear.y_axis, inv_lhs_rotation);
		rhs_scale_shear.z_axis = quat_mul_vector3(rhs_scale_shear.z_axis, inv_lhs_rotation);

		lhs_scale_shear.x_axis = quat_mul_vector3(lhs_scale_shear.x_axis, lhs.rotation);
		lhs_scale_shear.y_axis = quat_mul_vector3(lhs_scale_shear.y_axis, lhs.rotation);
		lhs_scale_shear.z_axis = quat_mul_vector3(lhs_scale_shear.z_axis, lhs.rotation);

		matrix3x3f scale_shear = matrix_mul(lhs_scale_shear, rhs_scale_shear);
		result.x_axis = scale_shear.x_axis;
		result.y_axis = scale_shear.y_axis;
		result.z_axis = scale_shear.z_axis;

		return result;
	}

	//////////////////////////////////////////////////////////////////////////
	// Multiplies a VQM transform with a scalar.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vqmf RTM_SIMD_CALL vqm_mul(const vqmf& vqm, float scalar) RTM_NO_EXCEPT
	{
		// T * s = [v, q, M] * s = [v * s, q * s, M * s]
		vector4f scalar_v = vector_set(scalar);

		vqmf result;
		result.rotation = quat_mul(vqm.rotation, scalar);
		result.translation = vector_mul(vqm.translation, scalar_v);
		result.x_axis = vector_mul(vqm.x_axis, scalar_v);
		result.y_axis = vector_mul(vqm.y_axis, scalar_v);
		result.z_axis = vector_mul(vqm.z_axis, scalar_v);

		return result;
	}

	//////////////////////////////////////////////////////////////////////////
	// Multiplies a VQM transform and a 3D point.
	// Multiplication order is as follow: world_position = vqm_mul_point3(local_position, local_to_world)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vector4f RTM_SIMD_CALL vqm_mul_point3(vector4f_arg0 point, const vqmf& vqm) RTM_NO_EXCEPT
	{
		// p' = p * M * R + v

		const matrix3x3f scale_shear = matrix_set(vqm.x_axis, vqm.y_axis, vqm.z_axis);
		return vector_add(quat_mul_vector3(matrix_mul_vector3(point, scale_shear), vqm.rotation), vqm.translation);
	}

	//////////////////////////////////////////////////////////////////////////
	// Multiplies a VQM transform and a 3D vector. The translation does not apply.
	// Multiplication order is as follow: world_direction = vqm_mul_vector3(local_direction, local_to_world)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vector4f RTM_SIMD_CALL vqm_mul_vector3(vector4f_arg0 vec3, const vqmf& vqm) RTM_NO_EXCEPT
	{
		// vec3' = vec3 * M * R

		const matrix3x3f scale_shear = matrix_set(vqm.x_axis, vqm.y_axis, vqm.z_axis);
		return quat_mul_vector3(matrix_mul_vector3(vec3, scale_shear), vqm.rotation);
	}

	namespace rtm_impl
	{
		//////////////////////////////////////////////////////////////////////////
		// Returns the inverse of the input VQM transform from the inverse of its scale/shear part.
		//////////////////////////////////////////////////////////////////////////
		RTM_DISABLE_SECURITY_COOKIE_CHECK inline vqmf RTM_SIMD_CALL vqm_inverse(const vqmf& input, matrix3x3f_arg0 inv_scale_shear) RTM_NO_EXCEPT
		{
			// T = [v, q, M] and T^-1 = [-v * R^T * M^-1, conjugate(q), R^T * (M^-1 * R)]
			// See the inverse section at the top of this file
			// The caller gives M^-1. Thus the function computes only one matrix inverse.

			const quatf inv_rotation = quat_conjugate(input.rotation);

			// Rotate the inverse scale/shear matrix
			matrix3x3f inv_rotated_scale_shear;
			inv_rotated_scale_shear.x_axis = quat_mul_vector3(inv_scale_shear.x_axis, input.rotation);
			inv_rotated_scale_shear.y_axis = quat_mul_vector3(inv_scale_shear.y_axis, input.rotation);
			inv_rotated_scale_shear.z_axis = quat_mul_vector3(inv_scale_shear.z_axis, input.rotation);

			// Build our inverse rotation matrix
			// TODO: We could build the matrix directly from the quaternion which is cheaper than rotating 3 axes, need to profile
			matrix3x3f inv_rotation_mtx = matrix_identity();
			inv_rotation_mtx.x_axis = quat_mul_vector3(inv_rotation_mtx.x_axis, inv_rotation);
			inv_rotation_mtx.y_axis = quat_mul_vector3(inv_rotation_mtx.y_axis, inv_rotation);
			inv_rotation_mtx.z_axis = quat_mul_vector3(inv_rotation_mtx.z_axis, inv_rotation);

			// Multiply our two matrices
			inv_rotated_scale_shear = matrix_mul(inv_rotation_mtx, inv_rotated_scale_shear);

			vqmf result;
			result.rotation = inv_rotation;
			result.x_axis = inv_rotated_scale_shear.x_axis;
			result.y_axis = inv_rotated_scale_shear.y_axis;
			result.z_axis = inv_rotated_scale_shear.z_axis;
			result.translation = matrix_mul_vector3(quat_mul_vector3(vector_neg(input.translation), inv_rotation), inv_scale_shear);

			return result;
		}
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the inverse of the input VQM transform.
	// If the scale/shear part has a zero determinant (e.g. zero scale), the result is undefined.
	// For a safe alternative, supply a fallback scale value and a threshold.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vqmf RTM_SIMD_CALL vqm_inverse(const vqmf& input) RTM_NO_EXCEPT
	{
		const matrix3x3f scale_shear = matrix_set(input.x_axis, input.y_axis, input.z_axis);
		return rtm_impl::vqm_inverse(input, matrix_inverse(scale_shear));
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the inverse of the input VQM transform.
	// If the scale/shear part has a determinant whose absolute value is below the supplied
	// threshold, the function uses the fallback scale in place of the scale/shear part.
	// The fallback scale part has no shear. It must not contain a zero scale.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vqmf RTM_SIMD_CALL vqm_inverse(const vqmf& input, vector4f_arg0 fallback_scale, float threshold = 1.0E-8F) RTM_NO_EXCEPT
	{
		const matrix3x3f scale_shear = matrix_set(input.x_axis, input.y_axis, input.z_axis);

		// matrix_inverse returns its fallback value when the determinant is below the threshold
		// Thus we give it the inverse of the fallback scale part
		const vector4f inv_fallback_scale = vector_reciprocal(fallback_scale);
		const vector4f zero = vector_zero();
		const matrix3x3f inv_fallback_scale_shear = matrix_set(
			vector_set_x(zero, vector_get_x_as_scalar(inv_fallback_scale)),
			vector_set_y(zero, vector_get_y_as_scalar(inv_fallback_scale)),
			vector_set_z(zero, vector_get_z_as_scalar(inv_fallback_scale)));

		return rtm_impl::vqm_inverse(input, matrix_inverse(scale_shear, inv_fallback_scale_shear, threshold));
	}

	//////////////////////////////////////////////////////////////////////////
	// Converts a VQM transform into a 3x4 affine matrix.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline matrix3x4f RTM_SIMD_CALL matrix_from_vqm(const vqmf& input) RTM_NO_EXCEPT
	{
		matrix3x4f result = matrix_set(input.x_axis, input.y_axis, input.z_axis, input.translation);

		result.x_axis = quat_mul_vector3(result.x_axis, input.rotation);
		result.y_axis = quat_mul_vector3(result.y_axis, input.rotation);
		result.z_axis = quat_mul_vector3(result.z_axis, input.rotation);

		return result;
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns a VQM transform with the rotation part normalized.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vqmf RTM_SIMD_CALL vqm_normalize(const vqmf& input) RTM_NO_EXCEPT
	{
		vqmf result;
		result.rotation = quat_normalize(input.rotation);
		result.x_axis = input.x_axis;
		result.y_axis = input.y_axis;
		result.z_axis = input.z_axis;
		result.translation = input.translation;

		return result;
	}

	//////////////////////////////////////////////////////////////////////////
	// Per component linear interpolation of the two inputs at the specified alpha.
	// The function interpolates the rotation with quat_lerp(..). It interpolates the
	// translation and each axis of the scale/shear part with vector_lerp(..).
	// The formula used is: ((1.0 - alpha) * start) + (alpha * end).
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vqmf RTM_SIMD_CALL vqm_lerp(const vqmf& start, const vqmf& end, float alpha) RTM_NO_EXCEPT
	{
		vqmf result;
		result.rotation = quat_lerp(start.rotation, end.rotation, alpha);
		result.translation = vector_lerp(start.translation, end.translation, alpha);
		result.x_axis = vector_lerp(start.x_axis, end.x_axis, alpha);
		result.y_axis = vector_lerp(start.y_axis, end.y_axis, alpha);
		result.z_axis = vector_lerp(start.z_axis, end.z_axis, alpha);

		return result;
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Per component linear interpolation of the two inputs at the specified alpha.
	// The function interpolates the rotation with quat_lerp(..). It interpolates the
	// translation and each axis of the scale/shear part with vector_lerp(..).
	// The formula used is: ((1.0 - alpha) * start) + (alpha * end).
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vqmf RTM_SIMD_CALL vqm_lerp(const vqmf& start, const vqmf& end, scalarf_arg0 alpha) RTM_NO_EXCEPT
	{
		vqmf result;
		result.rotation = quat_lerp(start.rotation, end.rotation, alpha);
		result.translation = vector_lerp(start.translation, end.translation, alpha);
		result.x_axis = vector_lerp(start.x_axis, end.x_axis, alpha);
		result.y_axis = vector_lerp(start.y_axis, end.y_axis, alpha);
		result.z_axis = vector_lerp(start.z_axis, end.z_axis, alpha);

		return result;
	}
#endif

	namespace rtm_impl
	{
		//////////////////////////////////////////////////////////////////////////
		// Sets the scale/shear part of the vqm_slerp(..) result.
		// The function does not change the rotation and the translation of the result.
		//////////////////////////////////////////////////////////////////////////
		RTM_DISABLE_SECURITY_COOKIE_CHECK inline void RTM_SIMD_CALL vqm_slerp_scale_shear(const vqmf& start, const vqmf& end, vector4f_arg0 alpha, vqmf& result) RTM_NO_EXCEPT
		{
			// The scale is the diagonal of the scale/shear part: M = N * S with S = diag(M) and N the shear
			// We divide each column by its scale to find N. Then we interpolate S with eerp and N with lerp,
			// and we multiply each column of the result by the new scale.
			// Eerp is not defined when the two scales of a column have a different sign or when one scale is zero.
			// For these columns, we use a scale of 1.0 at the two ends: then the column is a lerp of M.
			const vector4f zero = vector_zero();
			const vector4f one = vector_set(1.0F);
			const vector4f start_scale = vqm_get_scale(start);
			const vector4f end_scale = vqm_get_scale(end);
			const mask4f is_eerp_defined = mask_or(vector_greater_than(vector_min(start_scale, end_scale), zero), vector_less_than(vector_max(start_scale, end_scale), zero));
			const vector4f safe_start_scale = vector_select(is_eerp_defined, start_scale, one);
			const vector4f safe_end_scale = vector_select(is_eerp_defined, end_scale, one);

			// eerp(1.0, 1.0) is 1.0
			const vector4f scale = vector_eerp(safe_start_scale, safe_end_scale, alpha);

			// Each axis is a row of M: a division by the scale vector divides each column by its scale
			result.x_axis = vector_mul(vector_lerp(vector_div(start.x_axis, safe_start_scale), vector_div(end.x_axis, safe_end_scale), alpha), scale);
			result.y_axis = vector_mul(vector_lerp(vector_div(start.y_axis, safe_start_scale), vector_div(end.y_axis, safe_end_scale), alpha), scale);
			result.z_axis = vector_mul(vector_lerp(vector_div(start.z_axis, safe_start_scale), vector_div(end.z_axis, safe_end_scale), alpha), scale);
		}
	}

	//////////////////////////////////////////////////////////////////////////
	// Spherical interpolation of the two inputs at the specified alpha.
	// The function interpolates the rotation with quat_slerp(..) and the translation with vector_lerp(..).
	// The scale is the diagonal of the scale/shear part. The function interpolates the scale with
	// vector_eerp(..) and the remaining shear with vector_lerp(..).
	// If the two scales of a column have a different sign or if one scale is zero, the function
	// interpolates that column of the scale/shear part with vector_lerp(..).
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vqmf RTM_SIMD_CALL vqm_slerp(const vqmf& start, const vqmf& end, float alpha) RTM_NO_EXCEPT
	{
		vqmf result;
		result.rotation = quat_slerp(start.rotation, end.rotation, alpha);
		result.translation = vector_lerp(start.translation, end.translation, alpha);
		rtm_impl::vqm_slerp_scale_shear(start, end, vector_set(alpha), result);

		return result;
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Spherical interpolation of the two inputs at the specified alpha.
	// The function interpolates the rotation with quat_slerp(..) and the translation with vector_lerp(..).
	// The scale is the diagonal of the scale/shear part. The function interpolates the scale with
	// vector_eerp(..) and the remaining shear with vector_lerp(..).
	// If the two scales of a column have a different sign or if one scale is zero, the function
	// interpolates that column of the scale/shear part with vector_lerp(..).
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vqmf RTM_SIMD_CALL vqm_slerp(const vqmf& start, const vqmf& end, scalarf_arg0 alpha) RTM_NO_EXCEPT
	{
		vqmf result;
		result.rotation = quat_slerp(start.rotation, end.rotation, alpha);
		result.translation = vector_lerp(start.translation, end.translation, alpha);
		rtm_impl::vqm_slerp_scale_shear(start, end, vector_set(alpha), result);

		return result;
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Returns true if the input VQM does not contain any NaN or Inf, otherwise false.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL vqm_is_finite(const vqmf& input) RTM_NO_EXCEPT
	{
		return quat_is_finite(input.rotation)
			&& vector_is_finite3(input.translation)
			&& vector_is_finite3(input.x_axis)
			&& vector_is_finite3(input.y_axis)
			&& vector_is_finite3(input.z_axis);
	}

	RTM_IMPL_VERSION_NAMESPACE_END
}

RTM_IMPL_FILE_PRAGMA_POP
