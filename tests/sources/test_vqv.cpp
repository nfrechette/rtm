////////////////////////////////////////////////////////////////////////////////
// The MIT License (MIT)
//
// Copyright (c) 2018 Nicholas Frechette & Animation Compression Library contributors
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

#include "catch2.impl.h"

#include <rtm/vqvf.h>
#include <rtm/vqvd.h>

#include <limits>

using namespace rtm;

// Compares the 3x4 affine matrices of two VQV transforms.
// With a negative scale, two different VQV transforms can have the same matrix.
template<typename TransformType, typename FloatType>
static void check_vqv_near_equal(const TransformType& expected, const TransformType& actual, const FloatType threshold)
{
	const auto expected_mtx = matrix_from_vqv(expected);
	const auto actual_mtx = matrix_from_vqv(actual);
	CHECK(vector_all_near_equal3(expected_mtx.x_axis, actual_mtx.x_axis, threshold));
	CHECK(vector_all_near_equal3(expected_mtx.y_axis, actual_mtx.y_axis, threshold));
	CHECK(vector_all_near_equal3(expected_mtx.z_axis, actual_mtx.z_axis, threshold));
	CHECK(vector_all_near_equal3(expected_mtx.w_axis, actual_mtx.w_axis, threshold));
}

template<typename TransformType, typename FloatType>
static void test_vqv_interpolation(const FloatType threshold)
{
	using QuatType = decltype(TransformType::rotation);
	using Vector4Type = decltype(TransformType::translation);
	using ScalarType = typename related_types<FloatType>::scalar;

	FloatType alpha = FloatType(0.33);
	ScalarType alpha_s = scalar_set(alpha);
	QuatType quat0 = quat_normalize(quat_from_euler(scalar_deg_to_rad(FloatType(30.0)), scalar_deg_to_rad(FloatType(-45.0)), scalar_deg_to_rad(FloatType(90.0))));
	QuatType quat1 = quat_normalize(quat_from_euler(scalar_deg_to_rad(FloatType(45.0)), scalar_deg_to_rad(FloatType(60.0)), scalar_deg_to_rad(FloatType(120.0))));

	QuatType quat_ref_lerp = quat_lerp(quat0, quat1, alpha);
	QuatType quat_ref_lerp_s = quat_lerp(quat0, quat1, alpha_s);
	QuatType quat_ref_slerp = quat_slerp(quat0, quat1, alpha);
	QuatType quat_ref_slerp_s = quat_slerp(quat0, quat1, alpha_s);

	Vector4Type trans0 = vector_set(FloatType(-0.001138), FloatType(0.91623), FloatType(-1.624598));
	Vector4Type trans1 = vector_set(FloatType(2.5), FloatType(-1.75), FloatType(0.5));

	Vector4Type trans_ref = vector_lerp(trans0, trans1, alpha);
	Vector4Type trans_ref_s = vector_lerp(trans0, trans1, alpha_s);

	Vector4Type scale0 = vector_set(FloatType(-1.915), FloatType(0.23656), FloatType(-3.7811));
	Vector4Type scale1 = vector_set(FloatType(-0.2113), FloatType(12.22335), FloatType(-1.7261));

	Vector4Type scale_ref = vector_lerp(scale0, scale1, alpha);
	Vector4Type scale_ref_s = vector_lerp(scale0, scale1, alpha_s);

	// vqv_slerp interpolates the scale with eerp: the two scales of each component have the same sign
	Vector4Type scale_ref_eerp = vector_eerp(scale0, scale1, alpha);
	Vector4Type scale_ref_eerp_s = vector_eerp(scale0, scale1, alpha_s);

	TransformType transform0 = vqv_set(quat0, trans0, scale0);
	TransformType transform1 = vqv_set(quat1, trans1, scale1);

	TransformType transform_ref_lerp = vqv_set(quat_ref_lerp, trans_ref, scale_ref);
	TransformType transform_ref_slerp = vqv_set(quat_ref_slerp, trans_ref, scale_ref_eerp);

	TransformType transform_ref_lerp_s = vqv_set(quat_ref_lerp_s, trans_ref_s, scale_ref_s);
	TransformType transform_ref_slerp_s = vqv_set(quat_ref_slerp_s, trans_ref_s, scale_ref_eerp_s);

	TransformType transform_lerp = vqv_lerp(transform0, transform1, alpha);
	TransformType transform_lerp_s = vqv_lerp(transform0, transform1, alpha_s);

	TransformType transform_slerp = vqv_slerp(transform0, transform1, alpha);
	TransformType transform_slerp_s = vqv_slerp(transform0, transform1, alpha_s);

	CHECK(quat_near_equal(transform_lerp.rotation, transform_ref_lerp.rotation, threshold));
	CHECK(quat_near_equal(transform_lerp_s.rotation, transform_ref_lerp_s.rotation, threshold));
	CHECK(vector_all_near_equal3(transform_lerp.translation, transform_ref_lerp.translation, threshold));
	CHECK(vector_all_near_equal3(transform_lerp_s.translation, transform_ref_lerp_s.translation, threshold));
	CHECK(vector_all_near_equal3(transform_lerp.scale, transform_ref_lerp.scale, threshold));
	CHECK(vector_all_near_equal3(transform_lerp_s.scale, transform_ref_lerp_s.scale, threshold));

	CHECK(quat_near_equal(transform_slerp.rotation, transform_ref_slerp.rotation, threshold));
	CHECK(quat_near_equal(transform_slerp_s.rotation, transform_ref_slerp_s.rotation, threshold));
	CHECK(vector_all_near_equal3(transform_slerp.translation, transform_ref_slerp.translation, threshold));
	CHECK(vector_all_near_equal3(transform_slerp_s.translation, transform_ref_slerp_s.translation, threshold));
	CHECK(vector_all_near_equal3(transform_slerp.scale, transform_ref_slerp.scale, threshold));
	CHECK(vector_all_near_equal3(transform_slerp_s.scale, transform_ref_slerp_s.scale, threshold));

	// When the two scales of a component have a different sign or when one scale is zero, vqv_slerp uses lerp for that component
	{
		const Vector4Type mixed_scale0 = vector_set(FloatType(2.0), FloatType(-1.0), FloatType(0.0));
		const Vector4Type mixed_scale1 = vector_set(FloatType(8.0), FloatType(4.0), FloatType(5.0));
		const TransformType mixed_slerp = vqv_slerp(vqv_set(quat0, trans0, mixed_scale0), vqv_set(quat1, trans1, mixed_scale1), alpha);
		const TransformType mixed_slerp_s = vqv_slerp(vqv_set(quat0, trans0, mixed_scale0), vqv_set(quat1, trans1, mixed_scale1), alpha_s);
		const Vector4Type mixed_ref = vector_set(scalar_eerp(FloatType(2.0), FloatType(8.0), alpha), scalar_lerp(FloatType(-1.0), FloatType(4.0), alpha), scalar_lerp(FloatType(0.0), FloatType(5.0), alpha));
		CHECK(vqv_is_finite(mixed_slerp));
		CHECK(vector_all_near_equal3(mixed_slerp.scale, mixed_ref, threshold));
		CHECK(vector_all_near_equal3(mixed_slerp_s.scale, mixed_ref, threshold));
	}
}

template<typename TransformType, typename FloatType>
static void test_vqv_impl(const TransformType& identity, const FloatType threshold)
{
	using QuatType = decltype(TransformType::rotation);
	using Vector4Type = decltype(TransformType::translation);
	using Matrix3x3Type = typename related_types<FloatType>::matrix3x3;
	using Matrix3x4Type = typename related_types<FloatType>::matrix3x4;

	{
		Vector4Type zero = vector_set(FloatType(0.0));
		Vector4Type one = vector_set(FloatType(1.0));
		QuatType q_identity = quat_set(FloatType(0.0), FloatType(0.0), FloatType(0.0), FloatType(1.0));
		TransformType tmp = vqv_set(q_identity, zero, one);
		CHECK(quat_near_equal(identity.rotation, tmp.rotation, threshold));
		CHECK(vector_all_near_equal3(identity.translation, tmp.translation, threshold));
		CHECK(vector_all_near_equal3(identity.scale, tmp.scale, threshold));
		CHECK(quat_near_equal(q_identity, tmp.rotation, threshold));
		CHECK(vector_all_near_equal3(zero, tmp.translation, threshold));
		CHECK(vector_all_near_equal3(one, tmp.scale, threshold));
	}

	{
		QuatType rotation_around_z = quat_from_euler(scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(90.0)), scalar_deg_to_rad(FloatType(0.0)));
		Matrix3x3Type mtx = matrix_from_quat(rotation_around_z);
		TransformType transform = vqv_from_matrix(mtx);
		CHECK(quat_near_equal(rotation_around_z, transform.rotation, threshold));
		CHECK(vector_all_near_equal3(identity.translation, transform.translation, threshold));
		CHECK(vector_all_near_equal3(identity.scale, transform.scale, threshold));
	}

	{
		QuatType rotation_around_z = quat_from_euler(scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(90.0)), scalar_deg_to_rad(FloatType(0.0)));
		Vector4Type translation = vector_set(FloatType(1.0), FloatType(2.0), FloatType(3.0));

		// Default scale
		Matrix3x4Type mtx = matrix_from_vq(rotation_around_z, translation);
		TransformType transform = vqv_from_matrix(mtx);
		CHECK(quat_near_equal(rotation_around_z, transform.rotation, threshold));
		CHECK(vector_all_near_equal3(translation, transform.translation, threshold));
		CHECK(vector_all_near_equal3(identity.scale, transform.scale, threshold));

		// All positive scale
		Vector4Type scale = vector_set(FloatType(1.2), FloatType(0.2), FloatType(1.6));
		mtx = matrix_from_vqv(rotation_around_z, translation, scale);
		transform = vqv_from_matrix(mtx);
		CHECK(quat_near_equal(rotation_around_z, transform.rotation, threshold));
		CHECK(vector_all_near_equal3(translation, transform.translation, threshold));
		CHECK(vector_all_near_equal3(scale, transform.scale, threshold));

		// X has zero scale
		scale = vector_set(FloatType(0.0), FloatType(0.2), FloatType(1.6));
		mtx = matrix_from_vqv(rotation_around_z, translation, scale);
		transform = vqv_from_matrix(mtx);
		Matrix3x4Type mtx2 = matrix_from_vqv(transform);
		CHECK(vector_all_near_equal3(mtx.x_axis, mtx2.x_axis, threshold));
		CHECK(vector_all_near_equal3(mtx.y_axis, mtx2.y_axis, threshold));
		CHECK(vector_all_near_equal3(mtx.z_axis, mtx2.z_axis, threshold));
		CHECK(vector_all_near_equal3(mtx.w_axis, mtx2.w_axis, threshold));

		// Y has zero scale
		scale = vector_set(FloatType(1.2), FloatType(0.0), FloatType(1.6));
		mtx = matrix_from_vqv(rotation_around_z, translation, scale);
		transform = vqv_from_matrix(mtx);
		mtx2 = matrix_from_vqv(transform);
		CHECK(vector_all_near_equal3(mtx.x_axis, mtx2.x_axis, threshold));
		CHECK(vector_all_near_equal3(mtx.y_axis, mtx2.y_axis, threshold));
		CHECK(vector_all_near_equal3(mtx.z_axis, mtx2.z_axis, threshold));
		CHECK(vector_all_near_equal3(mtx.w_axis, mtx2.w_axis, threshold));

		// Z has zero scale
		scale = vector_set(FloatType(1.2), FloatType(0.2), FloatType(0.0));
		mtx = matrix_from_vqv(rotation_around_z, translation, scale);
		transform = vqv_from_matrix(mtx);
		mtx2 = matrix_from_vqv(transform);
		CHECK(vector_all_near_equal3(mtx.x_axis, mtx2.x_axis, threshold));
		CHECK(vector_all_near_equal3(mtx.y_axis, mtx2.y_axis, threshold));
		CHECK(vector_all_near_equal3(mtx.z_axis, mtx2.z_axis, threshold));
		CHECK(vector_all_near_equal3(mtx.w_axis, mtx2.w_axis, threshold));

		// XY have zero scale
		scale = vector_set(FloatType(0.0), FloatType(0.0), FloatType(1.6));
		mtx = matrix_from_vqv(rotation_around_z, translation, scale);
		transform = vqv_from_matrix(mtx);
		mtx2 = matrix_from_vqv(transform);
		CHECK(vector_all_near_equal3(mtx.x_axis, mtx2.x_axis, threshold));
		CHECK(vector_all_near_equal3(mtx.y_axis, mtx2.y_axis, threshold));
		CHECK(vector_all_near_equal3(mtx.z_axis, mtx2.z_axis, threshold));
		CHECK(vector_all_near_equal3(mtx.w_axis, mtx2.w_axis, threshold));

		// XZ have zero scale
		scale = vector_set(FloatType(0.0), FloatType(0.2), FloatType(0.0));
		mtx = matrix_from_vqv(rotation_around_z, translation, scale);
		transform = vqv_from_matrix(mtx);
		mtx2 = matrix_from_vqv(transform);
		CHECK(vector_all_near_equal3(mtx.x_axis, mtx2.x_axis, threshold));
		CHECK(vector_all_near_equal3(mtx.y_axis, mtx2.y_axis, threshold));
		CHECK(vector_all_near_equal3(mtx.z_axis, mtx2.z_axis, threshold));
		CHECK(vector_all_near_equal3(mtx.w_axis, mtx2.w_axis, threshold));

		// YZ have zero scale
		scale = vector_set(FloatType(1.2), FloatType(0.0), FloatType(0.0));
		mtx = matrix_from_vqv(rotation_around_z, translation, scale);
		transform = vqv_from_matrix(mtx);
		mtx2 = matrix_from_vqv(transform);
		CHECK(vector_all_near_equal3(mtx.x_axis, mtx2.x_axis, threshold));
		CHECK(vector_all_near_equal3(mtx.y_axis, mtx2.y_axis, threshold));
		CHECK(vector_all_near_equal3(mtx.z_axis, mtx2.z_axis, threshold));
		CHECK(vector_all_near_equal3(mtx.w_axis, mtx2.w_axis, threshold));

		// XYZ have zero scale
		scale = vector_set(FloatType(0.0), FloatType(0.0), FloatType(0.0));
		mtx = matrix_from_vqv(rotation_around_z, translation, scale);
		transform = vqv_from_matrix(mtx);
		mtx2 = matrix_from_vqv(transform);
		CHECK(quat_near_equal(identity.rotation, transform.rotation, threshold));
		CHECK(vector_all_near_equal3(mtx.x_axis, mtx2.x_axis, threshold));
		CHECK(vector_all_near_equal3(mtx.y_axis, mtx2.y_axis, threshold));
		CHECK(vector_all_near_equal3(mtx.z_axis, mtx2.z_axis, threshold));
		CHECK(vector_all_near_equal3(mtx.w_axis, mtx2.w_axis, threshold));

		// X has negative scale
		scale = vector_set(FloatType(-1.2), FloatType(0.2), FloatType(1.6));
		mtx = matrix_from_vqv(rotation_around_z, translation, scale);
		transform = vqv_from_matrix(mtx);
		mtx2 = matrix_from_vqv(transform);
		CHECK(vector_all_near_equal3(mtx.x_axis, mtx2.x_axis, threshold));
		CHECK(vector_all_near_equal3(mtx.y_axis, mtx2.y_axis, threshold));
		CHECK(vector_all_near_equal3(mtx.z_axis, mtx2.z_axis, threshold));
		CHECK(vector_all_near_equal3(mtx.w_axis, mtx2.w_axis, threshold));

		// Y has negative scale
		scale = vector_set(FloatType(1.2), FloatType(-0.2), FloatType(1.6));
		mtx = matrix_from_vqv(rotation_around_z, translation, scale);
		transform = vqv_from_matrix(mtx);
		mtx2 = matrix_from_vqv(transform);
		CHECK(vector_all_near_equal3(mtx.x_axis, mtx2.x_axis, threshold));
		CHECK(vector_all_near_equal3(mtx.y_axis, mtx2.y_axis, threshold));
		CHECK(vector_all_near_equal3(mtx.z_axis, mtx2.z_axis, threshold));
		CHECK(vector_all_near_equal3(mtx.w_axis, mtx2.w_axis, threshold));

		// Z has negative scale
		scale = vector_set(FloatType(1.2), FloatType(0.2), FloatType(-1.6));
		mtx = matrix_from_vqv(rotation_around_z, translation, scale);
		transform = vqv_from_matrix(mtx);
		mtx2 = matrix_from_vqv(transform);
		CHECK(vector_all_near_equal3(mtx.x_axis, mtx2.x_axis, threshold));
		CHECK(vector_all_near_equal3(mtx.y_axis, mtx2.y_axis, threshold));
		CHECK(vector_all_near_equal3(mtx.z_axis, mtx2.z_axis, threshold));
		CHECK(vector_all_near_equal3(mtx.w_axis, mtx2.w_axis, threshold));

		// XY have negative scale
		scale = vector_set(FloatType(-1.2), FloatType(-0.2), FloatType(1.6));
		mtx = matrix_from_vqv(rotation_around_z, translation, scale);
		transform = vqv_from_matrix(mtx);
		mtx2 = matrix_from_vqv(transform);
		CHECK(vector_all_near_equal3(mtx.x_axis, mtx2.x_axis, threshold));
		CHECK(vector_all_near_equal3(mtx.y_axis, mtx2.y_axis, threshold));
		CHECK(vector_all_near_equal3(mtx.z_axis, mtx2.z_axis, threshold));
		CHECK(vector_all_near_equal3(mtx.w_axis, mtx2.w_axis, threshold));

		// XZ have negative scale
		scale = vector_set(FloatType(-1.2), FloatType(0.2), FloatType(-1.6));
		mtx = matrix_from_vqv(rotation_around_z, translation, scale);
		transform = vqv_from_matrix(mtx);
		mtx2 = matrix_from_vqv(transform);
		CHECK(vector_all_near_equal3(mtx.x_axis, mtx2.x_axis, threshold));
		CHECK(vector_all_near_equal3(mtx.y_axis, mtx2.y_axis, threshold));
		CHECK(vector_all_near_equal3(mtx.z_axis, mtx2.z_axis, threshold));
		CHECK(vector_all_near_equal3(mtx.w_axis, mtx2.w_axis, threshold));

		// YZ have negative scale
		scale = vector_set(FloatType(1.2), FloatType(-0.2), FloatType(-1.6));
		mtx = matrix_from_vqv(rotation_around_z, translation, scale);
		transform = vqv_from_matrix(mtx);
		mtx2 = matrix_from_vqv(transform);
		CHECK(vector_all_near_equal3(mtx.x_axis, mtx2.x_axis, threshold));
		CHECK(vector_all_near_equal3(mtx.y_axis, mtx2.y_axis, threshold));
		CHECK(vector_all_near_equal3(mtx.z_axis, mtx2.z_axis, threshold));
		CHECK(vector_all_near_equal3(mtx.w_axis, mtx2.w_axis, threshold));

		// XYZ have negative scale
		scale = vector_set(FloatType(-1.2), FloatType(-0.2), FloatType(-1.6));
		mtx = matrix_from_vqv(rotation_around_z, translation, scale);
		transform = vqv_from_matrix(mtx);
		mtx2 = matrix_from_vqv(transform);
		CHECK(vector_all_near_equal3(mtx.x_axis, mtx2.x_axis, threshold));
		CHECK(vector_all_near_equal3(mtx.y_axis, mtx2.y_axis, threshold));
		CHECK(vector_all_near_equal3(mtx.z_axis, mtx2.z_axis, threshold));
		CHECK(vector_all_near_equal3(mtx.w_axis, mtx2.w_axis, threshold));
	}

	{
		Vector4Type x_axis = vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0));
		Vector4Type y_axis = vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0));
		Vector4Type test_scale = vector_set(FloatType(1.2));

		QuatType rotation_around_z = quat_from_euler(scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(90.0)), scalar_deg_to_rad(FloatType(0.0)));
		TransformType transform_a = vqv_set(rotation_around_z, x_axis, test_scale);
		Vector4Type result = vqv_mul_point3(x_axis, transform_a);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(1.0), FloatType(1.2), FloatType(0.0)), threshold));
		result = vqv_mul_point3(y_axis, transform_a);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(-0.2), FloatType(0.0), FloatType(0.0)), threshold));

		QuatType rotation_around_x = quat_from_euler(scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(90.0)));
		TransformType transform_b = vqv_set(rotation_around_x, y_axis, test_scale);
		result = vqv_mul_point3(x_axis, transform_b);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(1.2), FloatType(1.0), FloatType(0.0)), threshold));
		result = vqv_mul_point3(y_axis, transform_b);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(0.0), FloatType(1.0), FloatType(-1.2)), threshold));

		TransformType transform_ab = vqv_mul(transform_a, transform_b);
		TransformType transform_ba = vqv_mul(transform_b, transform_a);
		result = vqv_mul_point3(x_axis, transform_ab);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(1.2), FloatType(1.0), FloatType(-1.44)), threshold));
		CHECK(vector_all_near_equal3(result, vqv_mul_point3(vqv_mul_point3(x_axis, transform_a), transform_b), threshold));
		result = vqv_mul_point3(y_axis, transform_ab);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(-0.24), FloatType(1.0), FloatType(0.0)), threshold));
		CHECK(vector_all_near_equal3(result, vqv_mul_point3(vqv_mul_point3(y_axis, transform_a), transform_b), threshold));
		result = vqv_mul_point3(x_axis, transform_ba);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(-0.2), FloatType(1.44), FloatType(0.0)), threshold));
		CHECK(vector_all_near_equal3(result, vqv_mul_point3(vqv_mul_point3(x_axis, transform_b), transform_a), threshold));
		result = vqv_mul_point3(y_axis, transform_ba);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(-0.2), FloatType(0.0), FloatType(-1.44)), threshold));
		CHECK(vector_all_near_equal3(result, vqv_mul_point3(vqv_mul_point3(y_axis, transform_b), transform_a), threshold));
	}

	{
		// A non-uniform scale, thus the checks find scale lanes in the wrong order
		const Vector4Type point = vector_set(FloatType(0.5), FloatType(-1.25), FloatType(2.0));
		const Vector4Type scale_a = vector_set(FloatType(1.5), FloatType(0.25), FloatType(2.0));
		const Vector4Type scale_b = vector_set(FloatType(0.5), FloatType(3.0), FloatType(1.25));
		const Vector4Type translation_a = vector_set(FloatType(1.0), FloatType(2.0), FloatType(3.0));
		const Vector4Type translation_b = vector_set(FloatType(-4.0), FloatType(0.5), FloatType(2.5));
		const QuatType rotation_a = quat_from_euler(scalar_deg_to_rad(FloatType(30.0)), scalar_deg_to_rad(FloatType(-45.0)), scalar_deg_to_rad(FloatType(90.0)));
		const QuatType rotation_b = quat_from_euler(scalar_deg_to_rad(FloatType(45.0)), scalar_deg_to_rad(FloatType(60.0)), scalar_deg_to_rad(FloatType(120.0)));

		// A single transform: compare with the matrix that has the same transform
		const TransformType transform_a = vqv_set(rotation_a, translation_a, scale_a);
		const Matrix3x4Type matrix_a = matrix_from_vqv(rotation_a, translation_a, scale_a);
		CHECK(vector_all_near_equal3(matrix_mul_point3(point, matrix_a), vqv_mul_point3(point, transform_a), threshold));

		// A VQV transform can represent the product only when the right input has a uniform scale,
		// or when the left input has no rotation
		const TransformType transform_b_uniform = vqv_set(rotation_b, translation_b, vector_set(FloatType(1.5)));
		const TransformType transform_ab = vqv_mul(transform_a, transform_b_uniform);
		CHECK(vector_all_near_equal3(vector_mul(scale_a, vector_set(FloatType(1.5))), transform_ab.scale, threshold));
		CHECK(vector_all_near_equal3(vqv_mul_point3(vqv_mul_point3(point, transform_a), transform_b_uniform), vqv_mul_point3(point, transform_ab), threshold));

		const TransformType transform_a_no_rotation = vqv_set(identity.rotation, translation_a, scale_a);
		const TransformType transform_b = vqv_set(rotation_b, translation_b, scale_b);
		const TransformType transform_ab2 = vqv_mul(transform_a_no_rotation, transform_b);
		CHECK(vector_all_near_equal3(vector_mul(scale_a, scale_b), transform_ab2.scale, threshold));
		CHECK(vector_all_near_equal3(vqv_mul_point3(vqv_mul_point3(point, transform_a_no_rotation), transform_b), vqv_mul_point3(point, transform_ab2), threshold));
	}

	{
		// A non-uniform scale and a rotation: a VQV transform cannot hold the inverse
		const Vector4Type point = vector_set(FloatType(0.5), FloatType(-1.25), FloatType(2.0));
		const Vector4Type scale_a = vector_set(FloatType(1.5), FloatType(0.25), FloatType(2.0));
		const Vector4Type scale_b = vector_set(FloatType(0.5), FloatType(3.0), FloatType(1.25));
		const Vector4Type translation_a = vector_set(FloatType(1.0), FloatType(2.0), FloatType(3.0));
		const Vector4Type translation_b = vector_set(FloatType(-4.0), FloatType(0.5), FloatType(2.5));
		const QuatType rotation_a = quat_from_euler(scalar_deg_to_rad(FloatType(30.0)), scalar_deg_to_rad(FloatType(-45.0)), scalar_deg_to_rad(FloatType(90.0)));
		const QuatType rotation_b = quat_from_euler(scalar_deg_to_rad(FloatType(45.0)), scalar_deg_to_rad(FloatType(60.0)), scalar_deg_to_rad(FloatType(120.0)));

		const TransformType transform_a = vqv_set(rotation_a, translation_a, scale_a);
		const TransformType transform_b = vqv_set(rotation_b, translation_b, scale_b);
		const Matrix3x4Type inv_matrix_a = matrix_inverse(matrix_from_vqv(transform_a));
		const Matrix3x4Type inv_matrix_b = matrix_inverse(matrix_from_vqv(transform_b));

		// The inverse of a point is exact: compare with the inverse of the matrix
		CHECK(vector_all_near_equal3(matrix_mul_point3(point, inv_matrix_a), vqv_inverse_mul_point3(point, transform_a), threshold));
		CHECK(vector_all_near_equal3(point, vqv_inverse_mul_point3(vqv_mul_point3(point, transform_a), transform_a), threshold));

		// lhs * inverse(rhs): the translation is the exact position of the lhs origin
		const TransformType a_mul_inv_b = vqv_mul_inverse(transform_a, transform_b);
		CHECK(quat_near_equal(quat_mul(rotation_a, quat_conjugate(rotation_b)), a_mul_inv_b.rotation, threshold));
		CHECK(vector_all_near_equal3(matrix_mul_point3(translation_a, inv_matrix_b), a_mul_inv_b.translation, threshold));
		CHECK(vector_all_near_equal3(vector_div(scale_a, scale_b), a_mul_inv_b.scale, threshold));

		// The multiplication with rhs gives lhs back
		check_vqv_near_equal(transform_a, vqv_mul(a_mul_inv_b, transform_b), threshold);
		check_vqv_near_equal(identity, vqv_mul_inverse(transform_a, transform_a), threshold);

		// inverse(lhs) * rhs: the multiplication of lhs with the result gives rhs
		const TransformType inv_a_mul_b = vqv_inverse_mul(transform_a, transform_b);
		CHECK(quat_near_equal(quat_mul(quat_conjugate(rotation_a), rotation_b), inv_a_mul_b.rotation, threshold));
		CHECK(vector_all_near_equal3(vector_div(scale_b, scale_a), inv_a_mul_b.scale, threshold));
		check_vqv_near_equal(transform_b, vqv_mul(transform_a, inv_a_mul_b), threshold);
		check_vqv_near_equal(identity, vqv_inverse_mul(transform_a, transform_a), threshold);

		// The result moves the position of lhs exactly to the position of rhs
		CHECK(vector_all_near_equal3(translation_b, vqv_mul_point3(translation_a, inv_a_mul_b), threshold));

		// When the origin of the world moves, the result does not change
		const Vector4Type offset = vector_set(FloatType(8.0), FloatType(-5.0), FloatType(3.0));
		const TransformType moved_a = vqv_set(rotation_a, vector_add(translation_a, offset), scale_a);
		const TransformType moved_b = vqv_set(rotation_b, vector_add(translation_b, offset), scale_b);
		const TransformType moved_inv_a_mul_b = vqv_inverse_mul(moved_a, moved_b);
		CHECK(vector_all_near_equal3(vector_add(vqv_mul_point3(point, inv_a_mul_b), offset), vqv_mul_point3(vector_add(point, offset), moved_inv_a_mul_b), threshold));

		// When lhs and rhs have the same scale, inverse(lhs) * rhs has no scale. A VQV transform holds it and each point is exact.
		const TransformType transform_b_same_scale = vqv_set(rotation_b, translation_b, scale_a);
		const TransformType inv_a_mul_b_same_scale = vqv_inverse_mul(transform_a, transform_b_same_scale);
		CHECK(vector_all_near_equal3(vector_set(FloatType(1.0)), inv_a_mul_b_same_scale.scale, threshold));
		CHECK(vector_all_near_equal3(vqv_mul_point3(vqv_inverse_mul_point3(point, transform_a), transform_b_same_scale), vqv_mul_point3(point, inv_a_mul_b_same_scale), threshold));

		// When rhs has a uniform scale, a VQV transform holds lhs * inverse(rhs) and each point is exact
		const TransformType transform_b_uniform = vqv_set(rotation_b, translation_b, vector_set(FloatType(1.5)));
		const TransformType a_mul_inv_b_uniform = vqv_mul_inverse(transform_a, transform_b_uniform);
		CHECK(vector_all_near_equal3(vqv_inverse_mul_point3(vqv_mul_point3(point, transform_a), transform_b_uniform), vqv_mul_point3(point, a_mul_inv_b_uniform), threshold));
	}

	{
		// A negative scale: the functions go through a matrix
		const Vector4Type point = vector_set(FloatType(0.5), FloatType(-1.25), FloatType(2.0));
		const Vector4Type scale_a = vector_set(FloatType(-1.5), FloatType(0.25), FloatType(2.0));
		const Vector4Type scale_b = vector_set(FloatType(0.5), FloatType(3.0), FloatType(-1.25));
		const Vector4Type translation_a = vector_set(FloatType(1.0), FloatType(2.0), FloatType(3.0));
		const Vector4Type translation_b = vector_set(FloatType(-4.0), FloatType(0.5), FloatType(2.5));
		const QuatType rotation_a = quat_from_euler(scalar_deg_to_rad(FloatType(30.0)), scalar_deg_to_rad(FloatType(-45.0)), scalar_deg_to_rad(FloatType(90.0)));
		const QuatType rotation_b = quat_from_euler(scalar_deg_to_rad(FloatType(45.0)), scalar_deg_to_rad(FloatType(60.0)), scalar_deg_to_rad(FloatType(120.0)));

		const TransformType transform_a = vqv_set(rotation_a, translation_a, scale_a);
		const TransformType transform_b = vqv_set(rotation_b, translation_b, scale_b);

		// The inverse of a point is exact
		CHECK(vector_all_near_equal3(matrix_mul_point3(point, matrix_inverse(matrix_from_vqv(transform_a))), vqv_inverse_mul_point3(point, transform_a), threshold));

		// The translation and the scale of the multiplication with the result are exact
		const TransformType a_mul_inv_b = vqv_mul_inverse(transform_a, transform_b);
		const TransformType a_mul_inv_b_mul_b = vqv_mul(a_mul_inv_b, transform_b);
		CHECK(vector_all_near_equal3(translation_a, a_mul_inv_b_mul_b.translation, threshold));
		CHECK(vector_all_near_equal3(scale_a, a_mul_inv_b_mul_b.scale, threshold));

		const TransformType inv_a_mul_b = vqv_inverse_mul(transform_a, transform_b);
		const TransformType a_mul_inv_a_mul_b = vqv_mul(transform_a, inv_a_mul_b);
		CHECK(vector_all_near_equal3(translation_b, a_mul_inv_a_mul_b.translation, threshold));
		CHECK(vector_all_near_equal3(scale_b, a_mul_inv_a_mul_b.scale, threshold));

		check_vqv_near_equal(identity, vqv_mul_inverse(transform_a, transform_a), threshold);
		check_vqv_near_equal(identity, vqv_inverse_mul(transform_a, transform_a), threshold);

		// When rhs has a uniform negative scale, a VQV transform holds lhs * inverse(rhs) and each point is exact
		const TransformType transform_b_uniform = vqv_set(rotation_b, translation_b, vector_set(FloatType(-1.5)));
		const TransformType a_mul_inv_b_uniform = vqv_mul_inverse(transform_a, transform_b_uniform);
		CHECK(vector_all_near_equal3(vqv_inverse_mul_point3(vqv_mul_point3(point, transform_a), transform_b_uniform), vqv_mul_point3(point, a_mul_inv_b_uniform), threshold));

		// When lhs and rhs have the same negative scale, inverse(lhs) * rhs has no scale and each point is exact
		const TransformType transform_b_same_scale = vqv_set(rotation_b, translation_b, scale_a);
		const TransformType inv_a_mul_b_same_scale = vqv_inverse_mul(transform_a, transform_b_same_scale);
		CHECK(vector_all_near_equal3(vector_set(FloatType(1.0)), inv_a_mul_b_same_scale.scale, threshold));
		CHECK(vector_all_near_equal3(vqv_mul_point3(vqv_inverse_mul_point3(point, transform_a), transform_b_same_scale), vqv_mul_point3(point, inv_a_mul_b_same_scale), threshold));
	}

	{
		// The functions replace each scale lane that is zero with the fallback lane
		const Vector4Type point = vector_set(FloatType(0.5), FloatType(-1.25), FloatType(2.0));
		const Vector4Type translation = vector_set(FloatType(1.0), FloatType(2.0), FloatType(3.0));
		const QuatType rotation = quat_from_euler(scalar_deg_to_rad(FloatType(30.0)), scalar_deg_to_rad(FloatType(-45.0)), scalar_deg_to_rad(FloatType(90.0)));
		const Vector4Type fallback_scale = vector_set(FloatType(2.0), FloatType(4.0), FloatType(0.5));

		const TransformType transform_zero = vqv_set(rotation, translation, vector_set(FloatType(1.5), FloatType(0.0), FloatType(-2.0)));
		const TransformType transform_ref = vqv_set(rotation, translation, vector_set(FloatType(1.5), FloatType(4.0), FloatType(-2.0)));
		const TransformType transform_other = vqv_set(quat_from_euler(scalar_deg_to_rad(FloatType(45.0)), scalar_deg_to_rad(FloatType(60.0)), scalar_deg_to_rad(FloatType(120.0))), vector_set(FloatType(-4.0), FloatType(0.5), FloatType(2.5)), vector_set(FloatType(0.5), FloatType(3.0), FloatType(1.25)));

		CHECK(vector_all_near_equal3(vqv_inverse_mul_point3(point, transform_ref), vqv_inverse_mul_point3(point, transform_zero, fallback_scale), threshold));

		const TransformType mul_inverse_result = vqv_mul_inverse(transform_other, transform_zero, fallback_scale);
		CHECK(vqv_is_finite(mul_inverse_result));
		check_vqv_near_equal(vqv_mul_inverse(transform_other, transform_ref), mul_inverse_result, threshold);

		const TransformType inverse_mul_result = vqv_inverse_mul(transform_zero, transform_other, fallback_scale);
		CHECK(vqv_is_finite(inverse_mul_result));
		check_vqv_near_equal(vqv_inverse_mul(transform_ref, transform_other), inverse_mul_result, threshold);

		// The reciprocal of this denormal scale is infinite: the functions use the fallback lane
		const FloatType denormal_scale = (std::numeric_limits<FloatType>::min)() * FloatType(0.125);
		const TransformType transform_denormal = vqv_set(rotation, translation, vector_set(FloatType(1.5), denormal_scale, FloatType(-2.0)));
		CHECK(vector_all_near_equal3(vqv_inverse_mul_point3(point, transform_ref), vqv_inverse_mul_point3(point, transform_denormal, fallback_scale), threshold));
		check_vqv_near_equal(vqv_mul_inverse(transform_other, transform_ref), vqv_mul_inverse(transform_other, transform_denormal, fallback_scale), threshold);
		check_vqv_near_equal(vqv_inverse_mul(transform_ref, transform_other), vqv_inverse_mul(transform_denormal, transform_other, fallback_scale), threshold);
	}

	{
		Vector4Type x_axis = vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0));
		QuatType rotation_around_z = quat_from_euler(scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(90.0)), scalar_deg_to_rad(FloatType(0.0)));
		TransformType transform_a = vqv_set(rotation_around_z, x_axis, vector_set(FloatType(1.0)));
		CHECK(quat_is_normalized(vqv_normalize(transform_a).rotation, threshold));

		QuatType quat = quat_set(FloatType(-0.001138), FloatType(0.91623), FloatType(-1.624598), FloatType(0.715671));
		TransformType transform_b = vqv_set(quat, x_axis, vector_set(FloatType(1.0)));
		CHECK(!quat_is_normalized(transform_b.rotation, threshold));
		CHECK(quat_is_normalized(vqv_normalize(transform_b).rotation, threshold));
	}

	test_vqv_interpolation<TransformType>(threshold);

	{
		const FloatType inf = std::numeric_limits<FloatType>::infinity();
		const FloatType nan = std::numeric_limits<FloatType>::quiet_NaN();

		const QuatType quat0 = quat_normalize(quat_from_euler(scalar_deg_to_rad(FloatType(30.0)), scalar_deg_to_rad(FloatType(-45.0)), scalar_deg_to_rad(FloatType(90.0))));
		const QuatType quat_inf = quat_set(inf, inf, inf, inf);
		const QuatType quat_nan = quat_set(nan, nan, nan, nan);

		const Vector4Type trans0 = vector_set(FloatType(-0.001138), FloatType(0.91623), FloatType(-1.624598));
		const Vector4Type scale0 = vector_set(FloatType(-1.915), FloatType(0.23656), FloatType(-3.7811));
		const Vector4Type vec_inf = vector_set(inf, inf, inf, inf);
		const Vector4Type vec_nan = vector_set(nan, nan, nan, nan);

		CHECK(vqv_is_finite(identity) == true);
		CHECK(vqv_is_finite(vqv_set(quat0, trans0, scale0)) == true);
		CHECK(vqv_is_finite(vqv_set(quat_inf, trans0, scale0)) == false);
		CHECK(vqv_is_finite(vqv_set(quat_nan, trans0, scale0)) == false);
		CHECK(vqv_is_finite(vqv_set(quat0, vec_inf, scale0)) == false);
		CHECK(vqv_is_finite(vqv_set(quat0, vec_nan, scale0)) == false);
		CHECK(vqv_is_finite(vqv_set(quat0, trans0, vec_inf)) == false);
		CHECK(vqv_is_finite(vqv_set(quat0, trans0, vec_nan)) == false);
		CHECK(vqv_is_finite(vqv_set(quat_inf, vec_inf, vec_inf)) == false);
		CHECK(vqv_is_finite(vqv_set(quat_nan, vec_nan, vec_nan)) == false);
	}
}

TEST_CASE("vqvf math", "[math][vqv]")
{
	test_vqv_impl<vqvf, float>(vqv_identity(), 1.0E-4F);

	const quatf src_rotation = quat_set(0.39564531008956383F, 0.044254239301713752F, 0.22768840967675355F, 0.88863059760894492F);
	const vector4f src_translation = vector_set(-2.65F, 2.996113F, 0.68123521F);
	const vector4f src_scale = vector_set(1.2F, 0.8F, 2.1F);
	const vqvf src = vqv_set(src_rotation, src_translation, src_scale);
	const vqvd dst = vqv_cast(src);
	CHECK(quat_near_equal(src.rotation, quat_cast(dst.rotation), 1.0E-6F));
	CHECK(vector_all_near_equal3(src.translation, vector_cast(dst.translation), 1.0E-6F));
	CHECK(vector_all_near_equal3(src.scale, vector_cast(dst.scale), 1.0E-6F));
}

TEST_CASE("vqvd math", "[math][vqv]")
{
	test_vqv_impl<vqvd, double>(vqv_identity(), 1.0E-6);

	const quatd src_rotation = quat_set(0.39564531008956383, 0.044254239301713752, 0.22768840967675355, 0.88863059760894492);
	const vector4d src_translation = vector_set(-2.65, 2.996113, 0.68123521);
	const vector4d src_scale = vector_set(1.2, 0.8, 2.1);
	const vqvd src = vqv_set(src_rotation, src_translation, src_scale);
	const vqvf dst = vqv_cast(src);
	CHECK(quat_near_equal(src.rotation, quat_cast(dst.rotation), 1.0E-6));
	CHECK(vector_all_near_equal3(src.translation, vector_cast(dst.translation), 1.0E-6));
	CHECK(vector_all_near_equal3(src.scale, vector_cast(dst.scale), 1.0E-6));
}
