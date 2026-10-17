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
#include "rtm/impl/vqm_common.h"

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
	// - The function builds R1 once from q1. Then it computes M1 * R1, M2 * R1^T, and their product with matrix multiplications.
	//   A rotation of each row with the quaternion (quat_mul_vector3) gives the same result, but it is slower.
	// - The rotation of T1 moves into M3. Thus M3 can contain shear when M1 and M2 contain only scale.
	//   This is why VQV, which stores only a scale, does not form a group.
	//
	// Inverse:
	// - From p' = p * M * R + v, we get p = (p' - v) * R^T * M^-1.
	// - The inverse is T' = [v', q', M'] with R' = R^T. Then:
	//   M' * R^T = R^T * M^-1
	//   M' = R^T * (M^-1 * R)
	//   v' = -v * R^T * M^-1
	// - The function builds R once from q. Then it computes R^T * (M^-1 * R) with matrix multiplications.
	//////////////////////////////////////////////////////////////////////////

	namespace rtm_impl
	{
		//////////////////////////////////////////////////////////////////////////
		// Returns the translation register of the input with m22 in the [w] lane.
		// When the m22 slot holds the flag, the [w] lane is 1.0. See vqm_common.h for the flag.
		//////////////////////////////////////////////////////////////////////////
		RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4f RTM_SIMD_CALL vqm_get_translation_m22(const vqmf& input) RTM_NO_EXCEPT
		{
			return vqm_has_identity_scale_shear(input) ? vector_set_w(input.translation_m22, 1.0F) : input.translation_m22;
		}

		//////////////////////////////////////////////////////////////////////////
		// Creates a VQM transform from a rotation quaternion, a translation, and a 3x3 scale/shear matrix.
		// The result does not hold the flag, even when the scale/shear matrix is the identity.
		//////////////////////////////////////////////////////////////////////////
		RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vqmf RTM_SIMD_CALL vqm_set_no_flag(quatf_arg0 rotation, vector4f_arg1 translation, matrix3x3f_arg0 scale_shear) RTM_NO_EXCEPT
		{
			// The [w] lanes hold the third row of the scale/shear matrix
			vqmf result;
			result.rotation = rotation;
			result.translation_m22 = vector_mix<mix4::x, mix4::y, mix4::z, mix4::c>(translation, scale_shear.z_axis);
			result.x_axis_m20 = vector_mix<mix4::x, mix4::y, mix4::z, mix4::a>(scale_shear.x_axis, scale_shear.z_axis);
			result.y_axis_m21 = vector_mix<mix4::x, mix4::y, mix4::z, mix4::b>(scale_shear.y_axis, scale_shear.z_axis);

			return result;
		}

		//////////////////////////////////////////////////////////////////////////
		// Creates a VQM transform from a rotation quaternion and a translation.
		// The scale/shear part is the identity and the m22 slot holds the flag.
		//////////////////////////////////////////////////////////////////////////
		RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vqmf RTM_SIMD_CALL vqm_set_identity_scale_shear(quatf_arg0 rotation, vector4f_arg1 translation) RTM_NO_EXCEPT
		{
			vqmf result;
			result.rotation = rotation;
			result.translation_m22 = vector_set_w(translation, vqm_identity_scale_shear_flag<float>());
			result.x_axis_m20 = vector_set(1.0F, 0.0F, 0.0F, 0.0F);
			result.y_axis_m21 = vector_set(0.0F, 1.0F, 0.0F, 0.0F);

			return result;
		}

		//////////////////////////////////////////////////////////////////////////
		// Returns the scale/shear part of a VQM transform as a 3x3 matrix.
		// The m22 slot of the input must not hold the flag.
		//////////////////////////////////////////////////////////////////////////
		RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE matrix3x3f RTM_SIMD_CALL vqm_get_scale_shear_no_flag(const vqmf& input) RTM_NO_EXCEPT
		{
			// The [w] lanes hold the third row of the scale/shear matrix
			const vector4f m20_m20_m21_m21 = vector_mix<mix4::w, mix4::w, mix4::d, mix4::d>(input.x_axis_m20, input.y_axis_m21);
			const vector4f z_axis = vector_mix<mix4::x, mix4::z, mix4::d, mix4::d>(m20_m20_m21_m21, input.translation_m22);
			return matrix_set(input.x_axis_m20, input.y_axis_m21, z_axis);
		}

		//////////////////////////////////////////////////////////////////////////
		// Returns the input with the flag in the m22 slot if the scale/shear part is exactly the identity.
		// Otherwise, the function returns the input. The m22 slot of the input must not hold the flag.
		//////////////////////////////////////////////////////////////////////////
		RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vqmf RTM_SIMD_CALL vqm_set_flag_if_identity(const vqmf& input) RTM_NO_EXCEPT
		{
			// The rows hold m20 and m21 in their [w] lane: the identity rows have 0.0 there
			const bool is_identity = vector_all_equal(input.x_axis_m20, vector_set(1.0F, 0.0F, 0.0F, 0.0F))
				&& vector_all_equal(input.y_axis_m21, vector_set(0.0F, 1.0F, 0.0F, 0.0F))
				&& float(vector_get_w(input.translation_m22)) == 1.0F;

			if (!is_identity)
				return input;

			vqmf result = input;
			result.translation_m22 = vector_set_w(input.translation_m22, vqm_identity_scale_shear_flag<float>());
			return result;
		}
	}

	//////////////////////////////////////////////////////////////////////////
	// Casts a VQM transform float64 variant to a float32 variant.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vqmf RTM_SIMD_CALL vqm_cast(const vqmd& input) RTM_NO_EXCEPT
	{
		vqmf result;
		result.rotation = quat_cast(input.rotation);
		result.translation_m22 = vector_cast(input.translation_m22);
		result.x_axis_m20 = vector_cast(input.x_axis_m20);
		result.y_axis_m21 = vector_cast(input.y_axis_m21);

		// A conversion does not always keep the payload of a NaN, thus we set the flag again
		if (rtm_impl::vqm_has_identity_scale_shear(input))
			result.translation_m22 = vector_set_w(result.translation_m22, rtm_impl::vqm_identity_scale_shear_flag<float>());

		return result;
	}

	//////////////////////////////////////////////////////////////////////////
	// Creates a VQM transform from a rotation quaternion, a translation, and a 3x3 scale/shear matrix.
	// The scale/shear matrix applies before the rotation: p' = p * M * R + t.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vqmf RTM_SIMD_CALL vqm_set(quatf_arg0 rotation, vector4f_arg1 translation, matrix3x3f_arg0 scale_shear) RTM_NO_EXCEPT
	{
		return rtm_impl::vqm_set_flag_if_identity(rtm_impl::vqm_set_no_flag(rotation, translation, scale_shear));
	}

	//////////////////////////////////////////////////////////////////////////
	// Creates a VQM transform from a rotation quaternion, a translation, and a 3D scale.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vqmf RTM_SIMD_CALL vqm_set(quatf_arg0 rotation, vector4f_arg1 translation, vector4f_arg2 scale) RTM_NO_EXCEPT
	{
		if (vector_all_equal3(scale, vector_set(1.0F)))
			return rtm_impl::vqm_set_identity_scale_shear(rotation, translation);

		const vector4f zero = vector_zero();

		vqmf result;
		result.rotation = rotation;
		result.translation_m22 = vector_mix<mix4::x, mix4::y, mix4::z, mix4::c>(translation, scale);
		result.x_axis_m20 = vector_mix<mix4::a, mix4::y, mix4::z, mix4::w>(zero, scale);
		result.y_axis_m21 = vector_mix<mix4::x, mix4::b, mix4::z, mix4::w>(zero, scale);

		return result;
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the scale/shear part of a VQM transform as a 3x3 matrix.
	// The matrix applies before the rotation: p' = p * M * R + t.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE matrix3x3f RTM_SIMD_CALL vqm_get_scale_shear(vqmf_arg0 input) RTM_NO_EXCEPT
	{
		if (rtm_impl::vqm_has_identity_scale_shear(input))
			return matrix_identity();

		return rtm_impl::vqm_get_scale_shear_no_flag(input);
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
			return vqm_set(quat_identity(), vector_zero(), input);
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

		return vqm_set(rtm_impl::quat_from_matrix(rotation.x_axis, rotation.y_axis, rotation.z_axis), vector_zero(), scale_shear);
	}

	//////////////////////////////////////////////////////////////////////////
	// Converts a 3x4 affine matrix into a VQM transform.
	// See vqm_from_matrix(matrix3x3f) for the rotation and the scale/shear parts.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vqmf RTM_SIMD_CALL vqm_from_matrix(matrix3x4f_arg0 input) RTM_NO_EXCEPT
	{
		const matrix3x3f rotation_scale_shear = matrix_set(input.x_axis, input.y_axis, input.z_axis);

		vqmf result = vqm_from_matrix(rotation_scale_shear);

		// The [w] lane holds m22
		result.translation_m22 = vector_mix<mix4::x, mix4::y, mix4::z, mix4::d>(input.w_axis, result.translation_m22);

		return result;
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the rotation part of a VQM transform.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE quatf RTM_SIMD_CALL vqm_get_rotation(vqmf_arg0 input) RTM_NO_EXCEPT
	{
		return input.rotation;
	}

	//////////////////////////////////////////////////////////////////////////
	// Sets the rotation part of a VQM and returns the new value.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vqmf RTM_SIMD_CALL vqm_set_rotation(vqmf_arg0 input, quatf_arg1 rotation) RTM_NO_EXCEPT
	{
		vqmf result = input;
		result.rotation = rotation;
		return result;
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the translation part of a VQM transform. The [w] component is undefined.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4f RTM_SIMD_CALL vqm_get_translation(vqmf_arg0 input) RTM_NO_EXCEPT
	{
		return input.translation_m22;
	}

	//////////////////////////////////////////////////////////////////////////
	// Sets the translation part of a VQM and returns the new value.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vqmf RTM_SIMD_CALL vqm_set_translation(vqmf_arg0 input, vector4f_arg1 translation) RTM_NO_EXCEPT
	{
		// The [w] lane holds m22
		vqmf result = input;
		result.translation_m22 = vector_mix<mix4::x, mix4::y, mix4::z, mix4::d>(translation, input.translation_m22);
		return result;
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the scale part of a VQM transform: the diagonal of the scale/shear matrix.
	// When the scale/shear matrix contains shear, the scale is not the length of its rows.
	// Use vqm_get_scale_shear to get the full matrix.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4f RTM_SIMD_CALL vqm_get_scale(vqmf_arg0 input) RTM_NO_EXCEPT
	{
		// The scale is [m00, m11, m22]
		const vector4f m00_m11 = vector_mix<mix4::x, mix4::b, mix4::z, mix4::w>(input.x_axis_m20, input.y_axis_m21);
		return vector_mix<mix4::x, mix4::y, mix4::d, mix4::d>(m00_m11, rtm_impl::vqm_get_translation_m22(input));
	}

	//////////////////////////////////////////////////////////////////////////
	// Sets the scale part of a VQM and returns the new value.
	// The function sets the diagonal of the scale/shear matrix and keeps the other entries (the shear).
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vqmf RTM_SIMD_CALL vqm_set_scale(vqmf_arg0 input, vector4f_arg1 scale) RTM_NO_EXCEPT
	{
		// The scale is [m00, m11, m22]
		// When the m22 slot holds the flag, the other entries hold their identity values and the new m22 replaces the flag
		vqmf result;
		result.rotation = input.rotation;
		result.translation_m22 = vector_mix<mix4::x, mix4::y, mix4::z, mix4::c>(input.translation_m22, scale);
		result.x_axis_m20 = vector_mix<mix4::a, mix4::y, mix4::z, mix4::w>(input.x_axis_m20, scale);
		result.y_axis_m21 = vector_mix<mix4::x, mix4::b, mix4::z, mix4::w>(input.y_axis_m21, scale);
		return rtm_impl::vqm_set_flag_if_identity(result);
	}

	//////////////////////////////////////////////////////////////////////////
	// Sets the scale/shear part of a VQM and returns the new value.
	// The matrix applies before the rotation: p' = p * M * R + t.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vqmf RTM_SIMD_CALL vqm_set_scale_shear(vqmf_arg0 input, matrix3x3f_arg1 scale_shear) RTM_NO_EXCEPT
	{
		return vqm_set(input.rotation, input.translation_m22, scale_shear);
	}

	//////////////////////////////////////////////////////////////////////////
	// Adds two VQM transforms.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vqmf RTM_SIMD_CALL vqm_add(vqmf_arg0 lhs, vqmf_arg1 rhs) RTM_NO_EXCEPT
	{
		// T2 + T1 = [v2, q2, M2] + [v1, q1, M1] = [v2 + v1, q2 + q1, M2 + M1]
		// Each register holds components of only one part. Thus a sum of the registers gives the sum of the parts.
		// The result does not hold the flag: the sum of two identity matrices is not the identity
		vqmf result;
		result.rotation = quat_add(lhs.rotation, rhs.rotation);
		result.translation_m22 = vector_add(rtm_impl::vqm_get_translation_m22(lhs), rtm_impl::vqm_get_translation_m22(rhs));
		result.x_axis_m20 = vector_add(lhs.x_axis_m20, rhs.x_axis_m20);
		result.y_axis_m21 = vector_add(lhs.y_axis_m21, rhs.y_axis_m21);

		return result;
	}

	//////////////////////////////////////////////////////////////////////////
	// Multiplies two VQM transforms.
	// Multiplication order is as follow: local_to_world = vqm_mul(local_to_object, object_to_world)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vqmf RTM_SIMD_CALL vqm_mul(vqmf_arg0 lhs, vqmf_arg1 rhs) RTM_NO_EXCEPT
	{
		// lhs = [v1, q1, M1] and rhs = [v2, q2, M2]
		// result = [v1 * M2 * R2 + v2, quat_mul(q1, q2), (M1 * R1) * (M2 * R1^T)]
		// See the multiplication section at the top of this file

		const quatf rotation = quat_mul(lhs.rotation, rhs.rotation);

		if (rtm_impl::vqm_has_identity_scale_shear(rhs))
		{
			// M2 = I, thus v3 = v1 * R2 + v2 and M3 = M1 * R1 * R1^T = M1
			// The result has the scale/shear part of lhs: the flag too, if lhs holds it
			const vector4f translation = vector_add(quat_mul_vector3(lhs.translation_m22, rhs.rotation), rhs.translation_m22);

			vqmf result;
			result.rotation = rotation;
			result.translation_m22 = vector_mix<mix4::x, mix4::y, mix4::z, mix4::d>(translation, lhs.translation_m22);
			result.x_axis_m20 = lhs.x_axis_m20;
			result.y_axis_m21 = lhs.y_axis_m21;
			return result;
		}

		const matrix3x3f lhs_scale_shear = vqm_get_scale_shear(lhs);
		const matrix3x3f rhs_scale_shear = rtm_impl::vqm_get_scale_shear_no_flag(rhs);

		const vector4f translation = vector_add(quat_mul_vector3(matrix_mul_vector3(lhs.translation_m22, rhs_scale_shear), rhs.rotation), rhs.translation_m22);

		// M3 = (M1 * R1) * (M2 * R1^T)
		// M1 * R1 does not depend on rhs, thus a chain of products (a hierarchy) has a shorter dependency path
		const matrix3x3f lhs_rotation = matrix_from_quat(lhs.rotation);
		const matrix3x3f lhs_rotated_scale_shear = matrix_mul(lhs_scale_shear, lhs_rotation);
		const matrix3x3f rhs_rotated_scale_shear = matrix_mul(rhs_scale_shear, matrix_transpose(lhs_rotation));

		const matrix3x3f scale_shear = matrix_mul(lhs_rotated_scale_shear, rhs_rotated_scale_shear);
		return rtm_impl::vqm_set_no_flag(rotation, translation, scale_shear);
	}

	//////////////////////////////////////////////////////////////////////////
	// Multiplies a VQM transform with a scalar.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vqmf RTM_SIMD_CALL vqm_mul(vqmf_arg0 vqm, float scalar) RTM_NO_EXCEPT
	{
		// T * s = [v, q, M] * s = [v * s, q * s, M * s]
		// The result does not hold the flag
		vector4f scalar_v = vector_set(scalar);

		vqmf result;
		result.rotation = quat_mul(vqm.rotation, scalar);
		result.translation_m22 = vector_mul(rtm_impl::vqm_get_translation_m22(vqm), scalar_v);
		result.x_axis_m20 = vector_mul(vqm.x_axis_m20, scalar_v);
		result.y_axis_m21 = vector_mul(vqm.y_axis_m21, scalar_v);

		return result;
	}

	//////////////////////////////////////////////////////////////////////////
	// Multiplies a VQM transform and a 3D point.
	// Multiplication order is as follow: world_position = vqm_mul_point3(local_position, local_to_world)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vector4f RTM_SIMD_CALL vqm_mul_point3(vector4f_arg0 point, vqmf_arg1 vqm) RTM_NO_EXCEPT
	{
		// p' = p * M * R + v

		if (rtm_impl::vqm_has_identity_scale_shear(vqm))
			return vector_add(quat_mul_vector3(point, vqm.rotation), vqm.translation_m22);

		const matrix3x3f scale_shear = rtm_impl::vqm_get_scale_shear_no_flag(vqm);
		return vector_add(quat_mul_vector3(matrix_mul_vector3(point, scale_shear), vqm.rotation), vqm.translation_m22);
	}

	//////////////////////////////////////////////////////////////////////////
	// Multiplies a VQM transform and a 3D vector. The translation does not apply.
	// Multiplication order is as follow: world_direction = vqm_mul_vector3(local_direction, local_to_world)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vector4f RTM_SIMD_CALL vqm_mul_vector3(vector4f_arg0 vec3, vqmf_arg1 vqm) RTM_NO_EXCEPT
	{
		// vec3' = vec3 * M * R

		if (rtm_impl::vqm_has_identity_scale_shear(vqm))
			return quat_mul_vector3(vec3, vqm.rotation);

		const matrix3x3f scale_shear = rtm_impl::vqm_get_scale_shear_no_flag(vqm);
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
			const vector4f inv_translation = matrix_mul_vector3(quat_mul_vector3(vector_neg(input.translation_m22), inv_rotation), inv_scale_shear);

			// M' = R^T * (M^-1 * R)
			const matrix3x3f rotation = matrix_from_quat(input.rotation);
			const matrix3x3f inv_rotated_scale_shear = matrix_mul(matrix_transpose(rotation), matrix_mul(inv_scale_shear, rotation));

			return vqm_set_no_flag(inv_rotation, inv_translation, inv_rotated_scale_shear);
		}

		//////////////////////////////////////////////////////////////////////////
		// Returns the inverse of the input VQM transform. The m22 slot of the input must hold the flag.
		//////////////////////////////////////////////////////////////////////////
		RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vqmf RTM_SIMD_CALL vqm_inverse_identity_scale_shear(const vqmf& input) RTM_NO_EXCEPT
		{
			// M = I, thus T^-1 = [-v * R^T, conjugate(q), I]
			const quatf inv_rotation = quat_conjugate(input.rotation);
			const vector4f inv_translation = quat_mul_vector3(vector_neg(input.translation_m22), inv_rotation);
			return vqm_set_identity_scale_shear(inv_rotation, inv_translation);
		}
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the inverse of the input VQM transform.
	// If the scale/shear part has a zero determinant (e.g. zero scale), the result is undefined.
	// For a safe alternative, supply a fallback scale value and a threshold.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vqmf RTM_SIMD_CALL vqm_inverse(vqmf_arg0 input) RTM_NO_EXCEPT
	{
		if (rtm_impl::vqm_has_identity_scale_shear(input))
			return rtm_impl::vqm_inverse_identity_scale_shear(input);

		const matrix3x3f scale_shear = rtm_impl::vqm_get_scale_shear_no_flag(input);
		return rtm_impl::vqm_inverse(input, matrix_inverse(scale_shear));
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the inverse of the input VQM transform.
	// If the scale/shear part has a determinant whose absolute value is below the supplied
	// threshold, the function uses the fallback scale in place of the scale/shear part.
	// The fallback scale part has no shear. It must not contain a zero scale.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vqmf RTM_SIMD_CALL vqm_inverse(vqmf_arg0 input, vector4f_arg1 fallback_scale, float threshold = 1.0E-8F) RTM_NO_EXCEPT
	{
		if (rtm_impl::vqm_has_identity_scale_shear(input))
			return rtm_impl::vqm_inverse_identity_scale_shear(input);

		const matrix3x3f scale_shear = rtm_impl::vqm_get_scale_shear_no_flag(input);

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
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline matrix3x4f RTM_SIMD_CALL matrix_from_vqm(vqmf_arg0 input) RTM_NO_EXCEPT
	{
		const matrix3x3f scale_shear = vqm_get_scale_shear(input);

		const vector4f x_axis = quat_mul_vector3(scale_shear.x_axis, input.rotation);
		const vector4f y_axis = quat_mul_vector3(scale_shear.y_axis, input.rotation);
		const vector4f z_axis = quat_mul_vector3(scale_shear.z_axis, input.rotation);

		// The [w] lane of the translation is 1.0 or m22, not the flag
		return matrix_set(x_axis, y_axis, z_axis, rtm_impl::vqm_get_translation_m22(input));
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns a VQM transform with the rotation part normalized.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vqmf RTM_SIMD_CALL vqm_normalize(vqmf_arg0 input) RTM_NO_EXCEPT
	{
		vqmf result = input;
		result.rotation = quat_normalize(input.rotation);
		return result;
	}

	//////////////////////////////////////////////////////////////////////////
	// Per component linear interpolation of the two inputs at the specified alpha.
	// The function interpolates the rotation with quat_lerp(..). It interpolates the
	// translation and each axis of the scale/shear part with vector_lerp(..).
	// The formula used is: ((1.0 - alpha) * start) + (alpha * end).
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vqmf RTM_SIMD_CALL vqm_lerp(vqmf_arg0 start, vqmf_arg1 end, float alpha) RTM_NO_EXCEPT
	{
		vqmf result;
		result.rotation = quat_lerp(start.rotation, end.rotation, alpha);

		if (rtm_impl::vqm_has_identity_scale_shear(start) && rtm_impl::vqm_has_identity_scale_shear(end))
		{
			// The two scale/shear parts are the identity: the result keeps the identity and the flag
			// A lerp of the identity rows does not always give exactly 1.0 on the diagonal
			result.translation_m22 = vector_mix<mix4::x, mix4::y, mix4::z, mix4::d>(vector_lerp(start.translation_m22, end.translation_m22, alpha), start.translation_m22);
			result.x_axis_m20 = start.x_axis_m20;
			result.y_axis_m21 = start.y_axis_m21;
			return result;
		}

		result.translation_m22 = vector_lerp(rtm_impl::vqm_get_translation_m22(start), rtm_impl::vqm_get_translation_m22(end), alpha);
		result.x_axis_m20 = vector_lerp(start.x_axis_m20, end.x_axis_m20, alpha);
		result.y_axis_m21 = vector_lerp(start.y_axis_m21, end.y_axis_m21, alpha);

		return result;
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Per component linear interpolation of the two inputs at the specified alpha.
	// The function interpolates the rotation with quat_lerp(..). It interpolates the
	// translation and each axis of the scale/shear part with vector_lerp(..).
	// The formula used is: ((1.0 - alpha) * start) + (alpha * end).
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vqmf RTM_SIMD_CALL vqm_lerp(vqmf_arg0 start, vqmf_arg1 end, scalarf_arg2 alpha) RTM_NO_EXCEPT
	{
		vqmf result;
		result.rotation = quat_lerp(start.rotation, end.rotation, alpha);

		if (rtm_impl::vqm_has_identity_scale_shear(start) && rtm_impl::vqm_has_identity_scale_shear(end))
		{
			// The two scale/shear parts are the identity: the result keeps the identity and the flag
			// A lerp of the identity rows does not always give exactly 1.0 on the diagonal
			result.translation_m22 = vector_mix<mix4::x, mix4::y, mix4::z, mix4::d>(vector_lerp(start.translation_m22, end.translation_m22, alpha), start.translation_m22);
			result.x_axis_m20 = start.x_axis_m20;
			result.y_axis_m21 = start.y_axis_m21;
			return result;
		}

		result.translation_m22 = vector_lerp(rtm_impl::vqm_get_translation_m22(start), rtm_impl::vqm_get_translation_m22(end), alpha);
		result.x_axis_m20 = vector_lerp(start.x_axis_m20, end.x_axis_m20, alpha);
		result.y_axis_m21 = vector_lerp(start.y_axis_m21, end.y_axis_m21, alpha);

		return result;
	}
#endif

	namespace rtm_impl
	{
		//////////////////////////////////////////////////////////////////////////
		// Returns the scale/shear part of the vqm_slerp(..) result.
		//////////////////////////////////////////////////////////////////////////
		RTM_DISABLE_SECURITY_COOKIE_CHECK inline matrix3x3f RTM_SIMD_CALL vqm_slerp_scale_shear(const vqmf& start, const vqmf& end, vector4f_arg0 alpha) RTM_NO_EXCEPT
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

			const matrix3x3f start_scale_shear = vqm_get_scale_shear(start);
			const matrix3x3f end_scale_shear = vqm_get_scale_shear(end);

			// Each axis is a row of M: a division by the scale vector divides each column by its scale
			const vector4f x_axis = vector_mul(vector_lerp(vector_div(start_scale_shear.x_axis, safe_start_scale), vector_div(end_scale_shear.x_axis, safe_end_scale), alpha), scale);
			const vector4f y_axis = vector_mul(vector_lerp(vector_div(start_scale_shear.y_axis, safe_start_scale), vector_div(end_scale_shear.y_axis, safe_end_scale), alpha), scale);
			const vector4f z_axis = vector_mul(vector_lerp(vector_div(start_scale_shear.z_axis, safe_start_scale), vector_div(end_scale_shear.z_axis, safe_end_scale), alpha), scale);
			return matrix_set(x_axis, y_axis, z_axis);
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
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vqmf RTM_SIMD_CALL vqm_slerp(vqmf_arg0 start, vqmf_arg1 end, float alpha) RTM_NO_EXCEPT
	{
		const quatf rotation = quat_slerp(start.rotation, end.rotation, alpha);
		const vector4f translation = vector_lerp(start.translation_m22, end.translation_m22, alpha);

		// The two scale/shear parts are the identity: the result keeps the identity and the flag
		if (rtm_impl::vqm_has_identity_scale_shear(start) && rtm_impl::vqm_has_identity_scale_shear(end))
			return rtm_impl::vqm_set_identity_scale_shear(rotation, translation);

		const matrix3x3f scale_shear = rtm_impl::vqm_slerp_scale_shear(start, end, vector_set(alpha));
		return rtm_impl::vqm_set_no_flag(rotation, translation, scale_shear);
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
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vqmf RTM_SIMD_CALL vqm_slerp(vqmf_arg0 start, vqmf_arg1 end, scalarf_arg2 alpha) RTM_NO_EXCEPT
	{
		const quatf rotation = quat_slerp(start.rotation, end.rotation, alpha);
		const vector4f translation = vector_lerp(start.translation_m22, end.translation_m22, alpha);

		// The two scale/shear parts are the identity: the result keeps the identity and the flag
		if (rtm_impl::vqm_has_identity_scale_shear(start) && rtm_impl::vqm_has_identity_scale_shear(end))
			return rtm_impl::vqm_set_identity_scale_shear(rotation, translation);

		const matrix3x3f scale_shear = rtm_impl::vqm_slerp_scale_shear(start, end, vector_set(alpha));
		return rtm_impl::vqm_set_no_flag(rotation, translation, scale_shear);
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Returns true if the input VQM does not contain any NaN or Inf, otherwise false.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL vqm_is_finite(vqmf_arg0 input) RTM_NO_EXCEPT
	{
		// All the lanes are used. The flag is a NaN, thus the function tests m22 without it.
		return quat_is_finite(input.rotation)
			&& vector_is_finite(rtm_impl::vqm_get_translation_m22(input))
			&& vector_is_finite(input.x_axis_m20)
			&& vector_is_finite(input.y_axis_m21);
	}

	RTM_IMPL_VERSION_NAMESPACE_END
}

RTM_IMPL_FILE_PRAGMA_POP
