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
#include "rtm/mask4d.h"
#include "rtm/matrix3x3d.h"
#include "rtm/matrix3x4d.h"
#include "rtm/quatd.h"
#include "rtm/vector4d.h"
#include "rtm/version.h"
#include "rtm/impl/compiler_utils.h"
#include "rtm/impl/matrix_affine_common.h"
#include "rtm/experimental/impl/vqm_common.h"

RTM_IMPL_FILE_PRAGMA_PUSH

namespace rtm
{
	RTM_IMPL_VERSION_NAMESPACE_BEGIN

	//////////////////////////////////////////////////////////////////////////
	// See here for details: The VQM-Group and its Applications
	//                       By Michael Aristidou and Xin Li
	//
	// Key insights:
	// - We wish to maintain rotation, translation, and scale/shear separately as they do not
	//   interpolate the same way. In particular, scale/shear and rotation mix together in the
	//   upper 3x3 part of affine matrices and it is difficult to manage them correctly as there
	//   is no unique way to decompose them.
	// - Let us define [T1, R1, S1] and [T2, R2, S2] as two transforms with 3 affine matrices each
	//   We will ignore translation since it mostly lives in its own dimension (4th) and does not
	//   interfere with scale/shear/rotation.
	// - We define multiplication as follow:
	//   (R2 * S2) * (R1 * S1) = (R3 * S3)
	//   By construction, we wish R3 = R2 * R1, we substitute
	//   (R2 * S2) * (R1 * S1) = (R2 * R1 * S3)
	//   We solve for S3, which is the scale/shear matrix we are looking for
	//   (R2^-1 * R2) * S2 * (R1 * S1) = R1 * S3
	//   R1^-1 * (R2^-1 * R2) * S2 * (R1 * S1) = S3
	//   R2^-1 * R2 cancel out and we get
	//   R1^-1 * S2 * R1 * S1 = S3
	//   In plain english, to compute our scale/shear matrix, we rotate S1 into the space of S2
	//   by multiplying with R1, then we scale/shear the result, and return into the space of S1
	//   by applying the inverse R1 rotation. A sensible result.
	// - This is all well and good with matrices, but we wish to retain rotation as a quaternion
	//   for its numerical stability, compact nature, and superior interpolation. How do we multiply
	//   a matrix with a quaternion?
	// - A key insight is that if we apply a rotation matrix onto any other affine matrix (e.g. a
	//   scale/shear or other pure rotation matrix), what occurs under the matrix multiplication is
	//   that each column of the affine matrix is rotated by our rotation matrix. This is something
	//   we can easily achieve as well with a quaternion: by using the sandwich product. From this
	//   key insight, the various identities in the paper follow as matrix multiplication with a pure
	//   rotation matrix is equivalent to the sandwich product of each column of the other matrix.
	//
	// Some VQM identities:
	// - If we treat M as a homogenous quaternion matrix, q a quaternion, and r a pure quaternion,
	//   then: q * (M * r) * q^-1 = (q * M * q^-1) * r
	//   In plain english, applying scale/shear to a point and rotating that point is equivalent to
	//   rotating the scale/shear matrix and applying the result to that point.
	//
	// - If M and N are homogenous quaternion matrices, and q1 and q2 quaternions then:
	//   (q2 * N * q2^-1) * (q1 * M * q1^-1) = q2 * (N * (q1 * M * q1^-1)) * q2^-1
	//   In plain english, the product of two rotated scale/shear matrices is equivalent to the
	//   rotated product of one scale/shear matrix with another rotated scale/shear matrix meaning
	//   rotation can occur before the multiplication or after due to assossiativity. This is
	//   straightforward to see if the rotations are expressed in matrix form.
	//////////////////////////////////////////////////////////////////////////

	//////////////////////////////////////////////////////////////////////////
	// A VQM transform represents a 3D rotation (quaternion), 3D translation (vector3),
	// and 3D non-uniform scale and shear (matrix 3x3).
	// VQM forms a group with a well defined multiplication and inverse. Its
	// multiplication is assossiative but not commutative (like quaternions/matrices).
	// Rotations are assumed to represent a single turn (normalized quaternion).
	//////////////////////////////////////////////////////////////////////////
#if 0	// defined in types.h
	struct vqmd
	{
		// The internal format is meant to be opaque, use vqm_* accessors
		quatd		rotation;
		vector4d	translation;
		vector4d	x_axis;
		vector4d	y_axis;
		vector4d	z_axis;
	};
#endif

	template<> struct related_types<vqmd> : related_types<double> {};

	//////////////////////////////////////////////////////////////////////////
	// Casts a VQM transform float32 variant to a float64 variant.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vqmd RTM_SIMD_CALL vqm_cast(const vqmf& input) RTM_NO_EXCEPT
	{
		vqmd result;
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
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vqmd RTM_SIMD_CALL vqm_from_matrix(matrix3x3d_arg0 input) RTM_NO_EXCEPT
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
		const double zero_scale_threshold = 1.0E-8;

		const vector4d* input_axes = &input.x_axis;

		const double axis_lengths[3] = { vector_length3(input.x_axis), vector_length3(input.y_axis), vector_length3(input.z_axis) };

		int32_t first_index = 0;
		if (axis_lengths[1] > axis_lengths[first_index])
			first_index = 1;
		if (axis_lengths[2] > axis_lengths[first_index])
			first_index = 2;

		const double first_length = axis_lengths[first_index];
		if (first_length < zero_scale_threshold)
		{
			// The largest row is zero which means all three are zero
			// We use the identity rotation since its value is not recoverable
			vqmd result;
			result.rotation = quat_identity();
			result.translation = vector_zero();
			result.x_axis = input.x_axis;
			result.y_axis = input.y_axis;
			result.z_axis = input.z_axis;

			return result;
		}

		const vector4d first_axis = vector_div(input_axes[first_index], vector_set(first_length));

		// The two other rows follow the first row in the cyclic order X -> Y -> Z -> X
		const int32_t next_index = (first_index + 1) % 3;
		const int32_t last_index = (first_index + 2) % 3;

		const vector4d next_normal = vector_cross3(first_axis, input_axes[next_index]);
		const vector4d last_normal = vector_cross3(first_axis, input_axes[last_index]);
		const double next_orthogonal_length = vector_length3(next_normal);
		const double last_orthogonal_length = vector_length3(last_normal);

		int32_t second_index;
		int32_t third_index;
		vector4d normal;
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
			const matrix3x3d identity3x3 = matrix_identity();
			const vector4d first_y_dot = vector_dot3_as_vector(first_axis, identity3x3.y_axis);
			const mask4d is_first_parallel_to_y = vector_greater_equal(vector_abs(first_y_dot), vector_set(1.0 - zero_scale_threshold));
			const vector4d orthogonal_axis = vector_select(is_first_parallel_to_y, identity3x3.z_axis, identity3x3.y_axis);

			second_index = next_index;
			third_index = last_index;
			normal = vector_cross3(first_axis, orthogonal_axis);
		}

		// The second axis is orthogonal to the first axis, even when the normal is not exactly orthogonal to it
		// The third axis is orthogonal to the two others and (first, second, third) is a right-handed basis
		const vector4d second_axis = vector_normalize3(vector_cross3(normal, first_axis));
		const vector4d third_axis = vector_cross3(first_axis, second_axis);

		matrix3x3d rotation;
		vector4d* rotation_axes = &rotation.x_axis;
		rotation_axes[first_index] = first_axis;
		rotation_axes[second_index] = second_axis;

		// When the rows (first, second, third) are in the cyclic order X -> Y -> Z, the basis has the same
		// winding as the identity. Otherwise, we negate the third axis to get a determinant of +1.
		rotation_axes[third_index] = second_index == next_index ? third_axis : vector_neg(third_axis);

		// M = A * R^T
		const matrix3x3d scale_shear = matrix_mul(input, matrix_transpose(rotation));

		vqmd result;
		result.rotation = rtm_impl::quat_from_matrix(rotation.x_axis, rotation.y_axis, rotation.z_axis);
		result.translation = vector_zero();
		result.x_axis = scale_shear.x_axis;
		result.y_axis = scale_shear.y_axis;
		result.z_axis = scale_shear.z_axis;

		return result;
	}

	//////////////////////////////////////////////////////////////////////////
	// Converts a 3x4 affine matrix into a VQM transform.
	// See vqm_from_matrix(matrix3x3d) for the rotation and the scale/shear parts.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vqmd RTM_SIMD_CALL vqm_from_matrix(matrix3x4d_arg0 input) RTM_NO_EXCEPT
	{
		const matrix3x3d rotation_scale_shear = matrix_set(input.x_axis, input.y_axis, input.z_axis);

		vqmd result = vqm_from_matrix(rotation_scale_shear);
		result.translation = input.w_axis;

		return result;
	}

	//////////////////////////////////////////////////////////////////////////
	// Creates a VQM transform from a rotation quaternion, a translation, and a 3D scale.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vqmd RTM_SIMD_CALL vqm_set(quatd_arg0 rotation, vector4d_arg1 translation, vector4d_arg2 scale) RTM_NO_EXCEPT
	{
		vector4d zero = vector_zero();

		vqmd result;
		result.rotation = rotation;
		result.translation = translation;
		result.x_axis = vector_set_x(zero, vector_get_x_as_scalar(scale));
		result.y_axis = vector_set_y(zero, vector_get_y_as_scalar(scale));
		result.z_axis = vector_set_z(zero, vector_get_z_as_scalar(scale));

		return result;
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the rotation part of a VQM transform.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE quatd RTM_SIMD_CALL vqm_get_rotation(const vqmd& input) RTM_NO_EXCEPT
	{
		return input.rotation;
	}

	//////////////////////////////////////////////////////////////////////////
	// Sets the rotation part of a VQM and returns the new value.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vqmd RTM_SIMD_CALL vqm_set_rotation(const vqmd& input, quatd_arg0 rotation) RTM_NO_EXCEPT
	{
		vqmd result = input;
		result.rotation = rotation;
		return result;
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the translation part of a VQM transform.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vqm_get_translation(const vqmd& input) RTM_NO_EXCEPT
	{
		return input.translation;
	}

	//////////////////////////////////////////////////////////////////////////
	// Sets the translation part of a VQM and returns the new value.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vqmd RTM_SIMD_CALL vqm_set_translation(const vqmd& input, vector4d_arg0 translation) RTM_NO_EXCEPT
	{
		vqmd result = input;
		result.translation = translation;
		return result;
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the scale part of a VQM transform.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vqm_get_scale(const vqmd& input) RTM_NO_EXCEPT
	{
		vector4d xyxy = vector_mix<mix4::x, mix4::b, mix4::x, mix4::b>(input.x_axis, input.y_axis);
		return vector_mix<mix4::x, mix4::y, mix4::c, mix4::d>(xyxy, input.z_axis);
	}

	//////////////////////////////////////////////////////////////////////////
	// Sets the scale part of a VQM and returns the new value.
	// This preserves existing shear.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vqmd RTM_SIMD_CALL vqm_set_scale(const vqmd& input, vector4d_arg0 scale) RTM_NO_EXCEPT
	{
		vqmd result = input;
		result.x_axis = vector_set_x(input.x_axis, vector_get_x_as_scalar(scale));
		result.y_axis = vector_set_y(input.y_axis, vector_get_y_as_scalar(scale));
		result.z_axis = vector_set_z(input.z_axis, vector_get_z_as_scalar(scale));
		return result;
	}

	//////////////////////////////////////////////////////////////////////////
	// Adds two VQM transforms.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vqmd RTM_SIMD_CALL vqm_add(const vqmd& lhs, const vqmd& rhs) RTM_NO_EXCEPT
	{
		// T2 + T1 = [v2, q2, M2] + [v1, q1, M1] = [v2 + v1, q2 + q1, M2 + M1]
		vqmd result;
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
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vqmd RTM_SIMD_CALL vqm_mul(const vqmd& lhs, const vqmd& rhs) RTM_NO_EXCEPT
	{
		// T2 * T1 = [v2, q2, M2] * [v1, q1, M1] = [q2 * (M2 * v1) * q2^-1 + v2, q2 * q1, (q1^-1 * M2 * q1)(q1 * M1 * q1^-1)]

		const quatd inv_lhs_rotation = quat_conjugate(lhs.rotation);

		matrix3x3d lhs_scale_shear = matrix_set(lhs.x_axis, lhs.y_axis, lhs.z_axis);
		matrix3x3d rhs_scale_shear = matrix_set(rhs.x_axis, rhs.y_axis, rhs.z_axis);

		vqmd result;
		result.rotation = quat_mul(lhs.rotation, rhs.rotation);
		result.translation = vector_add(quat_mul_vector3(matrix_mul_vector3(lhs.translation, rhs_scale_shear), rhs.rotation), rhs.translation);

		rhs_scale_shear.x_axis = quat_mul_vector3(rhs_scale_shear.x_axis, inv_lhs_rotation);
		rhs_scale_shear.y_axis = quat_mul_vector3(rhs_scale_shear.y_axis, inv_lhs_rotation);
		rhs_scale_shear.z_axis = quat_mul_vector3(rhs_scale_shear.z_axis, inv_lhs_rotation);

		lhs_scale_shear.x_axis = quat_mul_vector3(lhs_scale_shear.x_axis, lhs.rotation);
		lhs_scale_shear.y_axis = quat_mul_vector3(lhs_scale_shear.y_axis, lhs.rotation);
		lhs_scale_shear.z_axis = quat_mul_vector3(lhs_scale_shear.z_axis, lhs.rotation);

		matrix3x3d scale_shear = matrix_mul(lhs_scale_shear, rhs_scale_shear);
		result.x_axis = scale_shear.x_axis;
		result.y_axis = scale_shear.y_axis;
		result.z_axis = scale_shear.z_axis;

		return result;
	}

	//////////////////////////////////////////////////////////////////////////
	// Multiplies a VQM transform with a scalar.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vqmd RTM_SIMD_CALL vqm_mul(const vqmd& vqm, double scalar) RTM_NO_EXCEPT
	{
		// s * T = s * [v, q, M] = [s * v, s * q, s * M]
		vector4d scalar_v = vector_set(scalar);

		vqmd result;
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
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vector4d RTM_SIMD_CALL vqm_mul_point3(vector4d_arg0 point, const vqmd& vqm) RTM_NO_EXCEPT
	{
		// T * p = [v, q, M] * p = (q * (M * p) * q^-1) + v

		const matrix3x3d scale_shear = matrix_set(vqm.x_axis, vqm.y_axis, vqm.z_axis);
		return vector_add(quat_mul_vector3(matrix_mul_vector3(point, scale_shear), vqm.rotation), vqm.translation);
	}

	//////////////////////////////////////////////////////////////////////////
	// Multiplies a VQM transform and a 3D vector.
	// Multiplication order is as follow: world_position = vqm_mul_point3(local_position, local_to_world)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vector4d RTM_SIMD_CALL vqm_mul_vector3(vector4d_arg0 vec3, const vqmd& vqm) RTM_NO_EXCEPT
	{
		// T * vec3 = [v, q, M] * p = (q * (M * vec3) * q^-1)

		const matrix3x3d scale_shear = matrix_set(vqm.x_axis, vqm.y_axis, vqm.z_axis);
		return quat_mul_vector3(matrix_mul_vector3(vec3, scale_shear), vqm.rotation);
	}

	namespace rtm_impl
	{
		//////////////////////////////////////////////////////////////////////////
		// Returns the inverse of the input VQM transform from the inverse of its scale/shear part.
		//////////////////////////////////////////////////////////////////////////
		RTM_DISABLE_SECURITY_COOKIE_CHECK inline vqmd RTM_SIMD_CALL vqm_inverse(const vqmd& input, matrix3x3d_arg0 inv_scale_shear) RTM_NO_EXCEPT
		{
			// T^-1 = [v, q, M]^-1 = [M^-1 * (q^-1 * -v * q), q^-1, q * (q * M * q^-1)^-1 * q^-1]
			// Note that (q * M * q^-1)^-1 != (q^-1 * M^-1 * q)
			// However, let us convert that last part into matrix representation
			// q * (q * M * q^-1)^-1 * q^-1 = Mq * (Mq * M)^-1
			// Mq * (Mq * M)^-1 = (Mq * M^-1) * Mq^-1
			// (Mq * M^-1) * Mq^-1 = (q * M^-1 * q^-1) * Mq^-1
			// Unfortunately, we cannot convert the remaining Mq^-1 matrix back into a quaternion product
			// because it does not rotate anything (multiplication is on left side instead of right)
			// However, we can solve this by introducing the identity matrix
			// (q * M^-1 * q^-1) * (Mq^-1 * I) = (q * M^-1 * q^-1) * (q^-1 * I * q)
			// This is better because it allows us to compute a single matrix inverse as opposed to two

			const quatd inv_rotation = quat_conjugate(input.rotation);

			// Rotate the inverse scale/shear matrix
			matrix3x3d inv_rotated_scale_shear;
			inv_rotated_scale_shear.x_axis = quat_mul_vector3(inv_scale_shear.x_axis, input.rotation);
			inv_rotated_scale_shear.y_axis = quat_mul_vector3(inv_scale_shear.y_axis, input.rotation);
			inv_rotated_scale_shear.z_axis = quat_mul_vector3(inv_scale_shear.z_axis, input.rotation);

			// Build our inverse rotation matrix
			// TODO: We could build the matrix directly from the quaternion which is cheaper than rotating 3 axes, need to profile
			matrix3x3d inv_rotation_mtx = matrix_identity();
			inv_rotation_mtx.x_axis = quat_mul_vector3(inv_rotation_mtx.x_axis, inv_rotation);
			inv_rotation_mtx.y_axis = quat_mul_vector3(inv_rotation_mtx.y_axis, inv_rotation);
			inv_rotation_mtx.z_axis = quat_mul_vector3(inv_rotation_mtx.z_axis, inv_rotation);

			// Multiply our two matrices
			inv_rotated_scale_shear = matrix_mul(inv_rotation_mtx, inv_rotated_scale_shear);

			vqmd result;
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
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vqmd RTM_SIMD_CALL vqm_inverse(const vqmd& input) RTM_NO_EXCEPT
	{
		const matrix3x3d scale_shear = matrix_set(input.x_axis, input.y_axis, input.z_axis);
		return rtm_impl::vqm_inverse(input, matrix_inverse(scale_shear));
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the inverse of the input VQM transform.
	// If the scale/shear part has a determinant whose absolute value is below the supplied
	// threshold, the function uses the fallback scale in place of the scale/shear part.
	// The fallback scale part has no shear. It must not contain a zero scale.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vqmd RTM_SIMD_CALL vqm_inverse(const vqmd& input, vector4d_arg0 fallback_scale, double threshold = 1.0E-8) RTM_NO_EXCEPT
	{
		const matrix3x3d scale_shear = matrix_set(input.x_axis, input.y_axis, input.z_axis);

		// matrix_inverse returns its fallback value when the determinant is below the threshold
		// Thus we give it the inverse of the fallback scale part
		const vector4d inv_fallback_scale = vector_reciprocal(fallback_scale);
		const vector4d zero = vector_zero();
		const matrix3x3d inv_fallback_scale_shear = matrix_set(
			vector_set_x(zero, vector_get_x_as_scalar(inv_fallback_scale)),
			vector_set_y(zero, vector_get_y_as_scalar(inv_fallback_scale)),
			vector_set_z(zero, vector_get_z_as_scalar(inv_fallback_scale)));

		return rtm_impl::vqm_inverse(input, matrix_inverse(scale_shear, inv_fallback_scale_shear, threshold));
	}

	//////////////////////////////////////////////////////////////////////////
	// Converts a VQM transform into a 3x4 affine matrix.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline matrix3x4d RTM_SIMD_CALL matrix_from_vqm(const vqmd& input) RTM_NO_EXCEPT
	{
		matrix3x4d result = matrix_set(input.x_axis, input.y_axis, input.z_axis, input.translation);

		result.x_axis = quat_mul_vector3(result.x_axis, input.rotation);
		result.y_axis = quat_mul_vector3(result.y_axis, input.rotation);
		result.z_axis = quat_mul_vector3(result.z_axis, input.rotation);

		return result;
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns a VQM transform with the rotation part normalized.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vqmd RTM_SIMD_CALL vqm_normalize(const vqmd& input) RTM_NO_EXCEPT
	{
		vqmd result;
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
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vqmd RTM_SIMD_CALL vqm_lerp(const vqmd& start, const vqmd& end, double alpha) RTM_NO_EXCEPT
	{
		vqmd result;
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
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vqmd RTM_SIMD_CALL vqm_lerp(const vqmd& start, const vqmd& end, scalard_arg0 alpha) RTM_NO_EXCEPT
	{
		vqmd result;
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
		RTM_DISABLE_SECURITY_COOKIE_CHECK inline void RTM_SIMD_CALL vqm_slerp_scale_shear(const vqmd& start, const vqmd& end, vector4d_arg0 alpha, vqmd& result) RTM_NO_EXCEPT
		{
			// The scale is the diagonal of the scale/shear part: M = N * S with S = diag(M) and N the shear
			// We divide each column by its scale to find N. Then we interpolate S with eerp and N with lerp,
			// and we multiply each column of the result by the new scale.
			// Eerp is not defined when the two scales of a column have a different sign or when one scale is zero.
			// For these columns, we use a scale of 1.0 at the two ends: then the column is a lerp of M.
			const vector4d zero = vector_zero();
			const vector4d one = vector_set(1.0);
			const vector4d start_scale = vqm_get_scale(start);
			const vector4d end_scale = vqm_get_scale(end);
			const mask4d is_eerp_defined = mask_or(vector_greater_than(vector_min(start_scale, end_scale), zero), vector_less_than(vector_max(start_scale, end_scale), zero));
			const vector4d safe_start_scale = vector_select(is_eerp_defined, start_scale, one);
			const vector4d safe_end_scale = vector_select(is_eerp_defined, end_scale, one);

			// eerp(1.0, 1.0) is 1.0
			const vector4d scale = vector_eerp(safe_start_scale, safe_end_scale, alpha);

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
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vqmd RTM_SIMD_CALL vqm_slerp(const vqmd& start, const vqmd& end, double alpha) RTM_NO_EXCEPT
	{
		vqmd result;
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
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vqmd RTM_SIMD_CALL vqm_slerp(const vqmd& start, const vqmd& end, scalard_arg0 alpha) RTM_NO_EXCEPT
	{
		vqmd result;
		result.rotation = quat_slerp(start.rotation, end.rotation, alpha);
		result.translation = vector_lerp(start.translation, end.translation, alpha);
		rtm_impl::vqm_slerp_scale_shear(start, end, vector_set(alpha), result);

		return result;
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Returns true if the input VQM does not contain any NaN or Inf, otherwise false.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL vqm_is_finite(const vqmd& input) RTM_NO_EXCEPT
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
