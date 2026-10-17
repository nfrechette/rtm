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
#include "rtm/matrix3x3f.h"
#include "rtm/quatf.h"
#include "rtm/vector4f.h"
#include "rtm/version.h"
#include "rtm/impl/compiler_utils.h"
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
	// Returns the scale part of a VQM transform.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4f RTM_SIMD_CALL vqm_get_scale(const vqmf& input) RTM_NO_EXCEPT
	{
		vector4f xyxy = vector_mix<mix4::x, mix4::b, mix4::x, mix4::b>(input.x_axis, input.y_axis);
		return vector_mix<mix4::x, mix4::y, mix4::c, mix4::d>(xyxy, input.z_axis);
	}

	//////////////////////////////////////////////////////////////////////////
	// Sets the scale part of a VQM and returns the new value.
	// This preserves existing shear.
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
		// T2 * T1 = [v2, q2, M2] * [v1, q1, M1] = [q2 * (M2 * v1) * q2^-1 + v2, q2 * q1, (q1^-1 * M2 * q1)(q1 * M1 * q1^-1)]

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
		// s * T = s * [v, q, M] = [s * v, s * q, s * M]
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
		// T * p = [v, q, M] * p = (q * (M * p) * q^-1) + v

		const matrix3x3f scale_shear = matrix_set(vqm.x_axis, vqm.y_axis, vqm.z_axis);
		return vector_add(quat_mul_vector3(matrix_mul_vector3(point, scale_shear), vqm.rotation), vqm.translation);
	}

	//////////////////////////////////////////////////////////////////////////
	// Multiplies a VQM transform and a 3D vector.
	// Multiplication order is as follow: world_position = vqm_mul_point3(local_position, local_to_world)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vector4f RTM_SIMD_CALL vqm_mul_vector3(vector4f_arg0 vec3, const vqmf& vqm) RTM_NO_EXCEPT
	{
		// T * vec3 = [v, q, M] * p = (q * (M * vec3) * q^-1)

		const matrix3x3f scale_shear = matrix_set(vqm.x_axis, vqm.y_axis, vqm.z_axis);
		return quat_mul_vector3(matrix_mul_vector3(vec3, scale_shear), vqm.rotation);
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the inverse of the input VQM transform.
	// If zero scale is contained, the result is undefined.
	// For a safe alternative, supply a fallback scale value and a threshold.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vqmf RTM_SIMD_CALL vqm_inverse(const vqmf& input) RTM_NO_EXCEPT
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

		const matrix3x3f scale_shear = matrix_set(input.x_axis, input.y_axis, input.z_axis);

		const matrix3x3f inv_scale_shear = matrix_inverse(scale_shear);
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

	//////////////////////////////////////////////////////////////////////////
	// Converts a VQM transform into a 3x4 affine matrix.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline matrix3x4f RTM_SIMD_CALL vqm_to_matrix(const vqmf& input) RTM_NO_EXCEPT
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
