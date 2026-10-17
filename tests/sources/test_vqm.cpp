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

#include "catch2.impl.h"

#include <rtm/matrix3x4f.h>
#include <rtm/matrix3x4d.h>
#include <rtm/type_traits.h>
#include <rtm/packing/quatf.h>
#include <rtm/packing/quatd.h>
#include <rtm/vqmf.h>
#include <rtm/vqmd.h>

using namespace rtm;

template<typename Matrix3x4Type, typename FloatType>
static void check_matrix_near_equal(const Matrix3x4Type& lhs, const Matrix3x4Type& rhs, const FloatType threshold)
{
	CHECK(vector_all_near_equal3(lhs.x_axis, rhs.x_axis, threshold));
	CHECK(vector_all_near_equal3(lhs.y_axis, rhs.y_axis, threshold));
	CHECK(vector_all_near_equal3(lhs.z_axis, rhs.z_axis, threshold));
	CHECK(vector_all_near_equal3(lhs.w_axis, rhs.w_axis, threshold));
}

template<typename TransformType, typename FloatType>
static void check_scale_shear_near_equal(const TransformType& lhs, const TransformType& rhs, const FloatType threshold)
{
	const auto lhs_scale_shear = vqm_get_scale_shear(lhs);
	const auto rhs_scale_shear = vqm_get_scale_shear(rhs);
	CHECK(vector_all_near_equal3(lhs_scale_shear.x_axis, rhs_scale_shear.x_axis, threshold));
	CHECK(vector_all_near_equal3(lhs_scale_shear.y_axis, rhs_scale_shear.y_axis, threshold));
	CHECK(vector_all_near_equal3(lhs_scale_shear.z_axis, rhs_scale_shear.z_axis, threshold));
}

template<typename TransformType, typename FloatType>
static void check_vqm_near_equal(const TransformType& lhs, const TransformType& rhs, const FloatType threshold)
{
	CHECK(quat_near_equal(vqm_get_rotation(lhs), vqm_get_rotation(rhs), threshold));
	CHECK(vector_all_near_equal3(vqm_get_translation(lhs), vqm_get_translation(rhs), threshold));
	check_scale_shear_near_equal(lhs, rhs, threshold);
}

template<typename TransformType, typename FloatType>
static void test_vqm_impl(const FloatType threshold)
{
	using QuatType = typename related_types<FloatType>::quat;
	using Vector4Type = typename related_types<FloatType>::vector4;
	using ScalarType = typename related_types<FloatType>::scalar;
	using Matrix3x3Type = typename related_types<FloatType>::matrix3x3;
	using Matrix3x4Type = typename related_types<FloatType>::matrix3x4;

	// Identity validation
	{
		Vector4Type point = vector_set(FloatType(12.0), FloatType(0.0), FloatType(-130.033));

		TransformType identity = vqm_identity();

		Vector4Type mul_point_result = vqm_mul_point3(point, identity);
		CHECK(vector_all_near_equal3(mul_point_result, point, threshold));

		TransformType mul_itself_result = vqm_mul(identity, identity);
		CHECK(quat_near_equal(mul_itself_result.rotation, identity.rotation, threshold));
		check_scale_shear_near_equal(mul_itself_result, identity, threshold);
		CHECK(vector_all_near_equal3(vqm_get_translation(mul_itself_result), vqm_get_translation(identity), threshold));

		TransformType inverse_result = vqm_inverse(identity);
		CHECK(quat_near_equal(inverse_result.rotation, identity.rotation, threshold));
		check_scale_shear_near_equal(inverse_result, identity, threshold);
		CHECK(vector_all_near_equal3(vqm_get_translation(inverse_result), vqm_get_translation(identity), threshold));
	}

	// Getters and setters
	{
		QuatType rotation = quat_from_euler(scalar_deg_to_rad(FloatType(10.1)), scalar_deg_to_rad(FloatType(41.6)), scalar_deg_to_rad(FloatType(-12.7)));
		Vector4Type translation = vector_set(FloatType(1.0), FloatType(2.0), FloatType(3.0));
		Vector4Type scale = vector_set(FloatType(4.0), FloatType(5.0), FloatType(6.0));

		TransformType identity = vqm_identity();
		TransformType tx = vqm_set_rotation(identity, rotation);
		CHECK(quat_near_equal(vqm_get_rotation(tx), rotation, threshold));
		check_scale_shear_near_equal(tx, identity, threshold);
		CHECK(vector_all_near_equal3(vqm_get_translation(tx), vqm_get_translation(identity), threshold));

		tx = vqm_set_translation(tx, translation);
		CHECK(quat_near_equal(vqm_get_rotation(tx), rotation, threshold));
		check_scale_shear_near_equal(tx, identity, threshold);
		CHECK(vector_all_near_equal3(vqm_get_translation(tx), translation, threshold));

		tx = vqm_set_scale(tx, scale);
		CHECK(quat_near_equal(vqm_get_rotation(tx), rotation, threshold));
		CHECK(vector_all_near_equal3(vqm_get_scale(tx), scale, threshold));
		CHECK(vector_all_near_equal3(vqm_get_translation(tx), translation, threshold));

		const Matrix3x3Type scale_shear = matrix_set(
			vector_set(FloatType(2.0), FloatType(0.5), FloatType(-0.25)),
			vector_set(FloatType(0.75), FloatType(-3.0), FloatType(0.125)),
			vector_set(FloatType(-1.5), FloatType(0.375), FloatType(1.25)));
		const Vector4Type scale_shear_diagonal = vector_set(FloatType(2.0), FloatType(-3.0), FloatType(1.25));

		tx = vqm_set_scale_shear(tx, scale_shear);
		CHECK(quat_near_equal(vqm_get_rotation(tx), rotation, threshold));
		CHECK(vector_all_near_equal3(vqm_get_translation(tx), translation, threshold));
		CHECK(vector_all_near_equal3(vqm_get_scale(tx), scale_shear_diagonal, threshold));

		Matrix3x3Type tx_scale_shear = vqm_get_scale_shear(tx);
		CHECK(vector_all_near_equal3(tx_scale_shear.x_axis, scale_shear.x_axis, threshold));
		CHECK(vector_all_near_equal3(tx_scale_shear.y_axis, scale_shear.y_axis, threshold));
		CHECK(vector_all_near_equal3(tx_scale_shear.z_axis, scale_shear.z_axis, threshold));

		// vqm_set_scale changes the diagonal and keeps the shear
		tx = vqm_set_scale(tx, scale);
		CHECK(vector_all_near_equal3(vqm_get_scale(tx), scale, threshold));
		tx_scale_shear = vqm_get_scale_shear(tx);
		CHECK(vector_all_near_equal3(tx_scale_shear.x_axis, vector_set(FloatType(4.0), FloatType(0.5), FloatType(-0.25)), threshold));
		CHECK(vector_all_near_equal3(tx_scale_shear.y_axis, vector_set(FloatType(0.75), FloatType(5.0), FloatType(0.125)), threshold));
		CHECK(vector_all_near_equal3(tx_scale_shear.z_axis, vector_set(FloatType(-1.5), FloatType(0.375), FloatType(6.0)), threshold));

		// vqm_set with a scale/shear matrix: p' = p * M * R + t
		tx = vqm_set(rotation, translation, scale_shear);
		CHECK(quat_near_equal(vqm_get_rotation(tx), rotation, threshold));
		CHECK(vector_all_near_equal3(vqm_get_translation(tx), translation, threshold));
		tx_scale_shear = vqm_get_scale_shear(tx);
		CHECK(vector_all_near_equal3(tx_scale_shear.x_axis, scale_shear.x_axis, threshold));
		CHECK(vector_all_near_equal3(tx_scale_shear.y_axis, scale_shear.y_axis, threshold));
		CHECK(vector_all_near_equal3(tx_scale_shear.z_axis, scale_shear.z_axis, threshold));

		const Vector4Type point = vector_set(FloatType(-2.5), FloatType(1.75), FloatType(0.5));
		const Vector4Type expected_point = vector_add(quat_mul_vector3(matrix_mul_vector3(point, scale_shear), rotation), translation);
		CHECK(vector_all_near_equal3(vqm_mul_point3(point, tx), expected_point, threshold));
	}

	// Matrix conversion validation
	{
		QuatType rotation = quat_from_euler(scalar_deg_to_rad(FloatType(10.1)), scalar_deg_to_rad(FloatType(41.6)), scalar_deg_to_rad(FloatType(-12.7)));
		Vector4Type translation = vector_set(FloatType(1.0), FloatType(2.0), FloatType(3.0));
		Vector4Type scale = vector_set(FloatType(4.0), FloatType(5.0), FloatType(6.0));

		Matrix3x4Type src_mtx = matrix_from_vqv(rotation, translation, scale);
		TransformType dst_tx = vqm_set(rotation, translation, scale);
		Matrix3x4Type dst_mtx = matrix_from_vqm(dst_tx);
		CHECK(vector_all_near_equal3(src_mtx.x_axis, dst_mtx.x_axis, threshold));
		CHECK(vector_all_near_equal3(src_mtx.y_axis, dst_mtx.y_axis, threshold));
		CHECK(vector_all_near_equal3(src_mtx.z_axis, dst_mtx.z_axis, threshold));
		CHECK(vector_all_near_equal3(src_mtx.w_axis, dst_mtx.w_axis, threshold));
	}

	// VQM from matrix validation
	{
		QuatType rotation = quat_from_euler(scalar_deg_to_rad(FloatType(10.1)), scalar_deg_to_rad(FloatType(41.6)), scalar_deg_to_rad(FloatType(-12.7)));
		Vector4Type translation = vector_set(FloatType(1.0), FloatType(2.0), FloatType(3.0));

		// Without shear and with a positive scale, the function finds the original rotation and scale
		{
			Vector4Type scale = vector_set(FloatType(4.0), FloatType(5.0), FloatType(6.0));
			TransformType tx = vqm_from_matrix(matrix_from_vqv(rotation, translation, scale));
			tx = vqm_set_rotation(tx, quat_ensure_positive_w(tx.rotation));
			check_vqm_near_equal(tx, vqm_set(quat_ensure_positive_w(rotation), translation, scale), threshold);
		}

		// The rows of each scale/shear part are: shear only (with Z or X as the largest row), shear and negative scale,
		// shear and zero scale, all the rows parallel, two parallel rows and a zero row, and all the rows zero
		const Vector4Type scale_shear_axes[][3] =
		{
			{ vector_set(FloatType(4.0), FloatType(0.5), FloatType(-0.25)), vector_set(FloatType(0.3), FloatType(5.0), FloatType(0.75)), vector_set(FloatType(-0.6), FloatType(0.2), FloatType(6.0)) },
			{ vector_set(FloatType(6.0), FloatType(0.5), FloatType(-0.25)), vector_set(FloatType(0.3), FloatType(5.0), FloatType(0.75)), vector_set(FloatType(-0.6), FloatType(0.2), FloatType(4.0)) },
			{ vector_set(FloatType(-4.0), FloatType(0.5), FloatType(-0.25)), vector_set(FloatType(0.3), FloatType(5.0), FloatType(0.75)), vector_set(FloatType(-0.6), FloatType(0.2), FloatType(6.0)) },
			{ vector_set(FloatType(-4.0), FloatType(0.5), FloatType(-0.25)), vector_set(FloatType(0.3), FloatType(-5.0), FloatType(0.75)), vector_set(FloatType(-0.6), FloatType(0.2), FloatType(6.0)) },
			{ vector_set(FloatType(-4.0), FloatType(0.5), FloatType(-0.25)), vector_set(FloatType(0.3), FloatType(-5.0), FloatType(0.75)), vector_set(FloatType(-0.6), FloatType(0.2), FloatType(-6.0)) },
			{ vector_set(FloatType(0.0), FloatType(0.0), FloatType(0.0)), vector_set(FloatType(0.3), FloatType(5.0), FloatType(0.75)), vector_set(FloatType(-0.6), FloatType(0.2), FloatType(6.0)) },
			{ vector_set(FloatType(0.0), FloatType(0.0), FloatType(0.0)), vector_set(FloatType(0.3), FloatType(5.0), FloatType(0.75)), vector_set(FloatType(0.0), FloatType(0.0), FloatType(0.0)) },
			{ vector_set(FloatType(1.0), FloatType(2.0), FloatType(3.0)), vector_set(FloatType(-2.0), FloatType(-4.0), FloatType(-6.0)), vector_set(FloatType(0.5), FloatType(1.0), FloatType(1.5)) },
			{ vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0)), vector_set(FloatType(0.0), FloatType(-3.0), FloatType(0.0)), vector_set(FloatType(0.0), FloatType(0.0), FloatType(0.0)) },
			{ vector_set(FloatType(0.0), FloatType(0.0), FloatType(0.0)), vector_set(FloatType(0.0), FloatType(0.0), FloatType(0.0)), vector_set(FloatType(0.0), FloatType(0.0), FloatType(0.0)) },
		};

		for (const Vector4Type* axes : scale_shear_axes)
		{
			const TransformType src_tx = vqm_set(rotation, translation, matrix_set(axes[0], axes[1], axes[2]));

			// The product of the two parts is equal to the input matrix
			const Matrix3x4Type src_mtx = matrix_from_vqm(src_tx);
			const TransformType dst_tx = vqm_from_matrix(src_mtx);
			CHECK(quat_is_normalized(dst_tx.rotation, threshold));
			check_matrix_near_equal(matrix_from_vqm(dst_tx), src_mtx, threshold);

			// The scale/shear part contains the reflection: it has the same determinant as the input
			const Matrix3x3Type src_mtx3x3 = matrix_set(src_mtx.x_axis, src_mtx.y_axis, src_mtx.z_axis);
			const Matrix3x3Type dst_scale_shear = vqm_get_scale_shear(dst_tx);
			CHECK(scalar_near_equal(scalar_cast(matrix_determinant(dst_scale_shear)), scalar_cast(matrix_determinant(src_mtx3x3)), threshold));

			// The 3x3 matrix variant returns the same rotation and scale/shear, and a zero translation
			const TransformType dst_tx3x3 = vqm_from_matrix(src_mtx3x3);
			check_vqm_near_equal(dst_tx3x3, vqm_set_translation(dst_tx, vector_zero()), threshold);
		}
	}

	// VQM + VQM validation
	{
		QuatType rotation0 = quat_from_euler(scalar_deg_to_rad(FloatType(10.1)), scalar_deg_to_rad(FloatType(41.6)), scalar_deg_to_rad(FloatType(-12.7)));
		Vector4Type translation0 = vector_set(FloatType(1.0), FloatType(2.0), FloatType(3.0));
		Vector4Type scale0 = vector_set(FloatType(4.0), FloatType(5.0), FloatType(6.0));
		TransformType tx0 = vqm_set(rotation0, translation0, scale0);

		QuatType rotation1 = quat_from_euler(scalar_deg_to_rad(FloatType(-35.2)), scalar_deg_to_rad(FloatType(5.3)), scalar_deg_to_rad(FloatType(72.4)));
		Vector4Type translation1 = vector_set(FloatType(-8.5), FloatType(0.25), FloatType(14.0));
		Vector4Type scale1 = vector_set(FloatType(0.5), FloatType(-2.0), FloatType(3.5));
		TransformType tx1 = vqm_set(rotation1, translation1, scale1);

		// The function adds each component of the two transforms
		TransformType add_result = vqm_add(tx0, tx1);
		CHECK(vector_all_near_equal(quat_to_vector(add_result.rotation), vector_add(quat_to_vector(tx0.rotation), quat_to_vector(tx1.rotation)), threshold));
		CHECK(vector_all_near_equal3(vqm_get_translation(add_result), vector_add(translation0, translation1), threshold));
		const Matrix3x3Type add_scale_shear = vqm_get_scale_shear(add_result);
		const Matrix3x3Type scale_shear0 = vqm_get_scale_shear(tx0);
		const Matrix3x3Type scale_shear1 = vqm_get_scale_shear(tx1);
		CHECK(vector_all_near_equal3(add_scale_shear.x_axis, vector_add(scale_shear0.x_axis, scale_shear1.x_axis), threshold));
		CHECK(vector_all_near_equal3(add_scale_shear.y_axis, vector_add(scale_shear0.y_axis, scale_shear1.y_axis), threshold));
		CHECK(vector_all_near_equal3(add_scale_shear.z_axis, vector_add(scale_shear0.z_axis, scale_shear1.z_axis), threshold));

		// The sum of a transform with itself is equal to the product of the transform and 2
		TransformType add_itself_result = vqm_add(tx0, tx0);
		TransformType mul_scalar_result = vqm_mul(tx0, FloatType(2.0));
		CHECK(vector_all_near_equal(quat_to_vector(add_itself_result.rotation), quat_to_vector(mul_scalar_result.rotation), threshold));
		CHECK(vector_all_near_equal3(vqm_get_translation(add_itself_result), vqm_get_translation(mul_scalar_result), threshold));
		check_scale_shear_near_equal(add_itself_result, mul_scalar_result, threshold);
	}

	// VQM * VQM validation
	{
		QuatType rotation = quat_from_euler(scalar_deg_to_rad(FloatType(10.1)), scalar_deg_to_rad(FloatType(41.6)), scalar_deg_to_rad(FloatType(-12.7)));
		Vector4Type translation = vector_set(FloatType(1.0), FloatType(2.0), FloatType(3.0));

		// All positive scale
		Vector4Type scale = vector_set(FloatType(4.0), FloatType(5.0), FloatType(6.0));

		Matrix3x4Type src_mtx = matrix_from_vqv(rotation, translation, scale);
		src_mtx = matrix_mul(src_mtx, src_mtx);

		TransformType dst_tx = vqm_set(rotation, translation, scale);
		dst_tx = vqm_mul(dst_tx, dst_tx);
		Matrix3x4Type dst_mtx = matrix_from_vqm(dst_tx);
		CHECK(vector_all_near_equal3(src_mtx.x_axis, dst_mtx.x_axis, threshold));
		CHECK(vector_all_near_equal3(src_mtx.y_axis, dst_mtx.y_axis, threshold));
		CHECK(vector_all_near_equal3(src_mtx.z_axis, dst_mtx.z_axis, threshold));
		CHECK(vector_all_near_equal3(src_mtx.w_axis, dst_mtx.w_axis, threshold));

		// Two different transforms, thus the checks find inputs in the wrong order
		{
			QuatType rotation1 = quat_from_euler(scalar_deg_to_rad(FloatType(-35.2)), scalar_deg_to_rad(FloatType(5.3)), scalar_deg_to_rad(FloatType(72.4)));
			Vector4Type translation1 = vector_set(FloatType(-8.5), FloatType(0.25), FloatType(14.0));
			Vector4Type scale1 = vector_set(FloatType(0.5), FloatType(-2.0), FloatType(3.5));

			Matrix3x4Type mtx0 = matrix_from_vqv(rotation, translation, scale);
			Matrix3x4Type mtx1 = matrix_from_vqv(rotation1, translation1, scale1);
			TransformType tx0 = vqm_set(rotation, translation, scale);
			TransformType tx1 = vqm_set(rotation1, translation1, scale1);

			Matrix3x4Type ref_01 = matrix_mul(mtx0, mtx1);
			Matrix3x4Type result_01 = matrix_from_vqm(vqm_mul(tx0, tx1));
			check_matrix_near_equal(ref_01, result_01, threshold);

			Matrix3x4Type ref_10 = matrix_mul(mtx1, mtx0);
			Matrix3x4Type result_10 = matrix_from_vqm(vqm_mul(tx1, tx0));
			check_matrix_near_equal(ref_10, result_10, threshold);
		}

		// One negative scale
		scale = vector_set(FloatType(-4.0), FloatType(5.0), FloatType(6.0));

		src_mtx = matrix_from_vqv(rotation, translation, scale);
		src_mtx = matrix_mul(src_mtx, src_mtx);

		dst_tx = vqm_set(rotation, translation, scale);
		dst_tx = vqm_mul(dst_tx, dst_tx);
		dst_mtx = matrix_from_vqm(dst_tx);
		CHECK(vector_all_near_equal3(src_mtx.x_axis, dst_mtx.x_axis, threshold));
		CHECK(vector_all_near_equal3(src_mtx.y_axis, dst_mtx.y_axis, threshold));
		CHECK(vector_all_near_equal3(src_mtx.z_axis, dst_mtx.z_axis, threshold));
		CHECK(vector_all_near_equal3(src_mtx.w_axis, dst_mtx.w_axis, threshold));

		// Two negative scale
		scale = vector_set(FloatType(-4.0), FloatType(-5.0), FloatType(6.0));

		src_mtx = matrix_from_vqv(rotation, translation, scale);
		src_mtx = matrix_mul(src_mtx, src_mtx);

		dst_tx = vqm_set(rotation, translation, scale);
		dst_tx = vqm_mul(dst_tx, dst_tx);
		dst_mtx = matrix_from_vqm(dst_tx);
		CHECK(vector_all_near_equal3(src_mtx.x_axis, dst_mtx.x_axis, threshold));
		CHECK(vector_all_near_equal3(src_mtx.y_axis, dst_mtx.y_axis, threshold));
		CHECK(vector_all_near_equal3(src_mtx.z_axis, dst_mtx.z_axis, threshold));
		CHECK(vector_all_near_equal3(src_mtx.w_axis, dst_mtx.w_axis, threshold));

		// Three negative scale
		scale = vector_set(FloatType(-4.0), FloatType(-5.0), FloatType(-6.0));

		src_mtx = matrix_from_vqv(rotation, translation, scale);
		src_mtx = matrix_mul(src_mtx, src_mtx);

		dst_tx = vqm_set(rotation, translation, scale);
		dst_tx = vqm_mul(dst_tx, dst_tx);
		dst_mtx = matrix_from_vqm(dst_tx);
		CHECK(vector_all_near_equal3(src_mtx.x_axis, dst_mtx.x_axis, threshold));
		CHECK(vector_all_near_equal3(src_mtx.y_axis, dst_mtx.y_axis, threshold));
		CHECK(vector_all_near_equal3(src_mtx.z_axis, dst_mtx.z_axis, threshold));
		CHECK(vector_all_near_equal3(src_mtx.w_axis, dst_mtx.w_axis, threshold));

		// One zero scale
		scale = vector_set(FloatType(0.0), FloatType(5.0), FloatType(6.0));

		src_mtx = matrix_from_vqv(rotation, translation, scale);
		src_mtx = matrix_mul(src_mtx, src_mtx);

		dst_tx = vqm_set(rotation, translation, scale);
		dst_tx = vqm_mul(dst_tx, dst_tx);
		dst_mtx = matrix_from_vqm(dst_tx);
		CHECK(vector_all_near_equal3(src_mtx.x_axis, dst_mtx.x_axis, threshold));
		CHECK(vector_all_near_equal3(src_mtx.y_axis, dst_mtx.y_axis, threshold));
		CHECK(vector_all_near_equal3(src_mtx.z_axis, dst_mtx.z_axis, threshold));
		CHECK(vector_all_near_equal3(src_mtx.w_axis, dst_mtx.w_axis, threshold));

		// Two zero scale
		scale = vector_set(FloatType(0.0), FloatType(0.0), FloatType(6.0));

		src_mtx = matrix_from_vqv(rotation, translation, scale);
		src_mtx = matrix_mul(src_mtx, src_mtx);

		dst_tx = vqm_set(rotation, translation, scale);
		dst_tx = vqm_mul(dst_tx, dst_tx);
		dst_mtx = matrix_from_vqm(dst_tx);
		CHECK(vector_all_near_equal3(src_mtx.x_axis, dst_mtx.x_axis, threshold));
		CHECK(vector_all_near_equal3(src_mtx.y_axis, dst_mtx.y_axis, threshold));
		CHECK(vector_all_near_equal3(src_mtx.z_axis, dst_mtx.z_axis, threshold));
		CHECK(vector_all_near_equal3(src_mtx.w_axis, dst_mtx.w_axis, threshold));

		// Three zero scale
		scale = vector_set(FloatType(0.0), FloatType(0.0), FloatType(0.0));

		src_mtx = matrix_from_vqv(rotation, translation, scale);
		src_mtx = matrix_mul(src_mtx, src_mtx);

		dst_tx = vqm_set(rotation, translation, scale);
		dst_tx = vqm_mul(dst_tx, dst_tx);
		dst_mtx = matrix_from_vqm(dst_tx);
		CHECK(vector_all_near_equal3(src_mtx.x_axis, dst_mtx.x_axis, threshold));
		CHECK(vector_all_near_equal3(src_mtx.y_axis, dst_mtx.y_axis, threshold));
		CHECK(vector_all_near_equal3(src_mtx.z_axis, dst_mtx.z_axis, threshold));
		CHECK(vector_all_near_equal3(src_mtx.w_axis, dst_mtx.w_axis, threshold));
	}

	// VQM * scalar validation
	{
		QuatType rotation = quat_from_euler(scalar_deg_to_rad(FloatType(10.1)), scalar_deg_to_rad(FloatType(41.6)), scalar_deg_to_rad(FloatType(-12.7)));
		Vector4Type translation = vector_set(FloatType(1.0), FloatType(2.0), FloatType(3.0));
		Vector4Type scale = vector_set(FloatType(4.0), FloatType(5.0), FloatType(6.0));
		TransformType tx = vqm_set(rotation, translation, scale);

		// The function multiplies each component of the transform by the scalar
		const FloatType factor = FloatType(-1.5);
		TransformType mul_result = vqm_mul(tx, factor);
		CHECK(vector_all_near_equal(quat_to_vector(mul_result.rotation), vector_mul(quat_to_vector(tx.rotation), factor), threshold));
		CHECK(vector_all_near_equal3(vqm_get_translation(mul_result), vector_mul(vqm_get_translation(tx), factor), threshold));
		const Matrix3x3Type mul_scale_shear = vqm_get_scale_shear(mul_result);
		const Matrix3x3Type tx_scale_shear = vqm_get_scale_shear(tx);
		CHECK(vector_all_near_equal3(mul_scale_shear.x_axis, vector_mul(tx_scale_shear.x_axis, factor), threshold));
		CHECK(vector_all_near_equal3(mul_scale_shear.y_axis, vector_mul(tx_scale_shear.y_axis, factor), threshold));
		CHECK(vector_all_near_equal3(mul_scale_shear.z_axis, vector_mul(tx_scale_shear.z_axis, factor), threshold));

		// A product with 1.0 does not change the transform
		TransformType mul_one_result = vqm_mul(tx, FloatType(1.0));
		CHECK(quat_near_equal(mul_one_result.rotation, tx.rotation, threshold));
		CHECK(vector_all_near_equal3(vqm_get_translation(mul_one_result), vqm_get_translation(tx), threshold));
		check_scale_shear_near_equal(mul_one_result, tx, threshold);
	}

	// point/vec3 * VQM validation
	{
		Vector4Type point = vector_set(FloatType(12.0), FloatType(0.0), FloatType(-130.033));

		QuatType rotation = quat_from_euler(scalar_deg_to_rad(FloatType(10.1)), scalar_deg_to_rad(FloatType(41.6)), scalar_deg_to_rad(FloatType(-12.7)));
		Vector4Type translation = vector_set(FloatType(1.0), FloatType(2.0), FloatType(3.0));

		// All positive scale
		Vector4Type scale = vector_set(FloatType(4.0), FloatType(5.0), FloatType(6.0));

		Matrix3x4Type src_mtx = matrix_from_vqv(rotation, translation, scale);
		Vector4Type src_point = matrix_mul_point3(point, src_mtx);

		TransformType dst_tx = vqm_set(rotation, translation, scale);
		Vector4Type dst_point = vqm_mul_point3(point, dst_tx);
		CHECK(vector_all_near_equal3(src_point, dst_point, threshold));

		src_point = matrix_mul_vector3(point, src_mtx);
		dst_point = vqm_mul_vector3(point, dst_tx);
		CHECK(vector_all_near_equal3(src_point, dst_point, threshold));

		// One negative scale
		scale = vector_set(FloatType(-4.0), FloatType(5.0), FloatType(6.0));

		src_mtx = matrix_from_vqv(rotation, translation, scale);
		src_point = matrix_mul_point3(point, src_mtx);

		dst_tx = vqm_set(rotation, translation, scale);
		dst_point = vqm_mul_point3(point, dst_tx);
		CHECK(vector_all_near_equal3(src_point, dst_point, threshold));

		src_point = matrix_mul_vector3(point, src_mtx);
		dst_point = vqm_mul_vector3(point, dst_tx);
		CHECK(vector_all_near_equal3(src_point, dst_point, threshold));

		// Two negative scale
		scale = vector_set(FloatType(-4.0), FloatType(-5.0), FloatType(6.0));

		src_mtx = matrix_from_vqv(rotation, translation, scale);
		src_point = matrix_mul_point3(point, src_mtx);

		dst_tx = vqm_set(rotation, translation, scale);
		dst_point = vqm_mul_point3(point, dst_tx);
		CHECK(vector_all_near_equal3(src_point, dst_point, threshold));

		src_point = matrix_mul_vector3(point, src_mtx);
		dst_point = vqm_mul_vector3(point, dst_tx);
		CHECK(vector_all_near_equal3(src_point, dst_point, threshold));

		// Three negative scale
		scale = vector_set(FloatType(-4.0), FloatType(-5.0), FloatType(-6.0));

		src_mtx = matrix_from_vqv(rotation, translation, scale);
		src_point = matrix_mul_point3(point, src_mtx);

		dst_tx = vqm_set(rotation, translation, scale);
		dst_point = vqm_mul_point3(point, dst_tx);
		CHECK(vector_all_near_equal3(src_point, dst_point, threshold));

		src_point = matrix_mul_vector3(point, src_mtx);
		dst_point = vqm_mul_vector3(point, dst_tx);
		CHECK(vector_all_near_equal3(src_point, dst_point, threshold));

		// One zero scale
		scale = vector_set(FloatType(0.0), FloatType(5.0), FloatType(6.0));

		src_mtx = matrix_from_vqv(rotation, translation, scale);
		src_point = matrix_mul_point3(point, src_mtx);

		dst_tx = vqm_set(rotation, translation, scale);
		dst_point = vqm_mul_point3(point, dst_tx);
		CHECK(vector_all_near_equal3(src_point, dst_point, threshold));

		src_point = matrix_mul_vector3(point, src_mtx);
		dst_point = vqm_mul_vector3(point, dst_tx);
		CHECK(vector_all_near_equal3(src_point, dst_point, threshold));

		// Two zero scale
		scale = vector_set(FloatType(0.0), FloatType(0.0), FloatType(6.0));

		src_mtx = matrix_from_vqv(rotation, translation, scale);
		src_point = matrix_mul_point3(point, src_mtx);

		dst_tx = vqm_set(rotation, translation, scale);
		dst_point = vqm_mul_point3(point, dst_tx);
		CHECK(vector_all_near_equal3(src_point, dst_point, threshold));

		src_point = matrix_mul_vector3(point, src_mtx);
		dst_point = vqm_mul_vector3(point, dst_tx);
		CHECK(vector_all_near_equal3(src_point, dst_point, threshold));

		// Three zero scale
		scale = vector_set(FloatType(0.0), FloatType(0.0), FloatType(0.0));

		src_mtx = matrix_from_vqv(rotation, translation, scale);
		src_point = matrix_mul_point3(point, src_mtx);

		dst_tx = vqm_set(rotation, translation, scale);
		dst_point = vqm_mul_point3(point, dst_tx);
		CHECK(vector_all_near_equal3(src_point, dst_point, threshold));

		src_point = matrix_mul_vector3(point, src_mtx);
		dst_point = vqm_mul_vector3(point, dst_tx);
		CHECK(vector_all_near_equal3(src_point, dst_point, threshold));
	}

	// VQM inverse validation
	{
		QuatType rotation = quat_from_euler(scalar_deg_to_rad(FloatType(10.1)), scalar_deg_to_rad(FloatType(41.6)), scalar_deg_to_rad(FloatType(-12.7)));
		Vector4Type translation = vector_set(FloatType(1.0), FloatType(2.0), FloatType(3.0));

		// All positive scale
		Vector4Type scale = vector_set(FloatType(4.0), FloatType(5.0), FloatType(6.0));

		Matrix3x4Type src_mtx = matrix_from_vqv(rotation, translation, scale);
		Matrix3x4Type inv_src_mtx = matrix_inverse(src_mtx);

		TransformType dst_tx = vqm_set(rotation, translation, scale);
		TransformType inv_dst_tx = vqm_inverse(dst_tx);

		Matrix3x4Type inv_dst_mtx = matrix_from_vqm(inv_dst_tx);
		CHECK(vector_all_near_equal3(inv_src_mtx.x_axis, inv_dst_mtx.x_axis, threshold));
		CHECK(vector_all_near_equal3(inv_src_mtx.y_axis, inv_dst_mtx.y_axis, threshold));
		CHECK(vector_all_near_equal3(inv_src_mtx.z_axis, inv_dst_mtx.z_axis, threshold));
		CHECK(vector_all_near_equal3(inv_src_mtx.w_axis, inv_dst_mtx.w_axis, threshold));

		// T * T^-1 = identity
		TransformType identity = vqm_identity();
		TransformType inverse_mul_result = vqm_mul(dst_tx, vqm_inverse(dst_tx));
		CHECK(quat_near_equal(inverse_mul_result.rotation, identity.rotation, threshold));
		check_scale_shear_near_equal(inverse_mul_result, identity, threshold);
		CHECK(vector_all_near_equal3(vqm_get_translation(inverse_mul_result), vqm_get_translation(identity), threshold));

		// T^-1 * T = identity
		inverse_mul_result = vqm_mul(vqm_inverse(dst_tx), dst_tx);
		CHECK(quat_near_equal(inverse_mul_result.rotation, identity.rotation, threshold));
		check_scale_shear_near_equal(inverse_mul_result, identity, threshold);
		CHECK(vector_all_near_equal3(vqm_get_translation(inverse_mul_result), vqm_get_translation(identity), threshold));

		// One negative scale
		scale = vector_set(FloatType(-4.0), FloatType(5.0), FloatType(6.0));

		src_mtx = matrix_from_vqv(rotation, translation, scale);
		inv_src_mtx = matrix_inverse(src_mtx);

		dst_tx = vqm_set(rotation, translation, scale);
		inv_dst_tx = vqm_inverse(dst_tx);

		inv_dst_mtx = matrix_from_vqm(inv_dst_tx);
		CHECK(vector_all_near_equal3(inv_src_mtx.x_axis, inv_dst_mtx.x_axis, threshold));
		CHECK(vector_all_near_equal3(inv_src_mtx.y_axis, inv_dst_mtx.y_axis, threshold));
		CHECK(vector_all_near_equal3(inv_src_mtx.z_axis, inv_dst_mtx.z_axis, threshold));
		CHECK(vector_all_near_equal3(inv_src_mtx.w_axis, inv_dst_mtx.w_axis, threshold));

		// T * T^-1 = identity
		identity = vqm_identity();
		inverse_mul_result = vqm_mul(dst_tx, vqm_inverse(dst_tx));
		CHECK(quat_near_equal(inverse_mul_result.rotation, identity.rotation, threshold));
		check_scale_shear_near_equal(inverse_mul_result, identity, threshold);
		CHECK(vector_all_near_equal3(vqm_get_translation(inverse_mul_result), vqm_get_translation(identity), threshold));

		// T^-1 * T = identity
		inverse_mul_result = vqm_mul(vqm_inverse(dst_tx), dst_tx);
		CHECK(quat_near_equal(inverse_mul_result.rotation, identity.rotation, threshold));
		check_scale_shear_near_equal(inverse_mul_result, identity, threshold);
		CHECK(vector_all_near_equal3(vqm_get_translation(inverse_mul_result), vqm_get_translation(identity), threshold));

		// Two negative scale
		scale = vector_set(FloatType(-4.0), FloatType(-5.0), FloatType(6.0));

		src_mtx = matrix_from_vqv(rotation, translation, scale);
		inv_src_mtx = matrix_inverse(src_mtx);

		dst_tx = vqm_set(rotation, translation, scale);
		inv_dst_tx = vqm_inverse(dst_tx);

		inv_dst_mtx = matrix_from_vqm(inv_dst_tx);
		CHECK(vector_all_near_equal3(inv_src_mtx.x_axis, inv_dst_mtx.x_axis, threshold));
		CHECK(vector_all_near_equal3(inv_src_mtx.y_axis, inv_dst_mtx.y_axis, threshold));
		CHECK(vector_all_near_equal3(inv_src_mtx.z_axis, inv_dst_mtx.z_axis, threshold));
		CHECK(vector_all_near_equal3(inv_src_mtx.w_axis, inv_dst_mtx.w_axis, threshold));

		// T * T^-1 = identity
		identity = vqm_identity();
		inverse_mul_result = vqm_mul(dst_tx, vqm_inverse(dst_tx));
		CHECK(quat_near_equal(inverse_mul_result.rotation, identity.rotation, threshold));
		check_scale_shear_near_equal(inverse_mul_result, identity, threshold);
		CHECK(vector_all_near_equal3(vqm_get_translation(inverse_mul_result), vqm_get_translation(identity), threshold));

		// T^-1 * T = identity
		inverse_mul_result = vqm_mul(vqm_inverse(dst_tx), dst_tx);
		CHECK(quat_near_equal(inverse_mul_result.rotation, identity.rotation, threshold));
		check_scale_shear_near_equal(inverse_mul_result, identity, threshold);
		CHECK(vector_all_near_equal3(vqm_get_translation(inverse_mul_result), vqm_get_translation(identity), threshold));

		// Three negative scale
		scale = vector_set(FloatType(-4.0), FloatType(-5.0), FloatType(-6.0));

		src_mtx = matrix_from_vqv(rotation, translation, scale);
		inv_src_mtx = matrix_inverse(src_mtx);

		dst_tx = vqm_set(rotation, translation, scale);
		inv_dst_tx = vqm_inverse(dst_tx);

		inv_dst_mtx = matrix_from_vqm(inv_dst_tx);
		CHECK(vector_all_near_equal3(inv_src_mtx.x_axis, inv_dst_mtx.x_axis, threshold));
		CHECK(vector_all_near_equal3(inv_src_mtx.y_axis, inv_dst_mtx.y_axis, threshold));
		CHECK(vector_all_near_equal3(inv_src_mtx.z_axis, inv_dst_mtx.z_axis, threshold));
		CHECK(vector_all_near_equal3(inv_src_mtx.w_axis, inv_dst_mtx.w_axis, threshold));

		// T * T^-1 = identity
		identity = vqm_identity();
		inverse_mul_result = vqm_mul(dst_tx, vqm_inverse(dst_tx));
		CHECK(quat_near_equal(inverse_mul_result.rotation, identity.rotation, threshold));
		check_scale_shear_near_equal(inverse_mul_result, identity, threshold);
		CHECK(vector_all_near_equal3(vqm_get_translation(inverse_mul_result), vqm_get_translation(identity), threshold));

		// T^-1 * T = identity
		inverse_mul_result = vqm_mul(vqm_inverse(dst_tx), dst_tx);
		CHECK(quat_near_equal(inverse_mul_result.rotation, identity.rotation, threshold));
		check_scale_shear_near_equal(inverse_mul_result, identity, threshold);
		CHECK(vector_all_near_equal3(vqm_get_translation(inverse_mul_result), vqm_get_translation(identity), threshold));
	}

	// VQM inverse with fallback validation
	{
		QuatType rotation = quat_from_euler(scalar_deg_to_rad(FloatType(10.1)), scalar_deg_to_rad(FloatType(41.6)), scalar_deg_to_rad(FloatType(-12.7)));
		Vector4Type translation = vector_set(FloatType(1.0), FloatType(2.0), FloatType(3.0));
		Vector4Type fallback_scale = vector_set(FloatType(2.0), FloatType(4.0), FloatType(0.5));

		// The scale/shear part is invertible: the function ignores the fallback
		Matrix3x3Type scale_shear = matrix_set(
			vector_set(FloatType(-4.0), FloatType(0.5), FloatType(-0.25)),
			vector_set(FloatType(0.3), FloatType(5.0), FloatType(0.75)),
			vector_set(FloatType(-0.6), FloatType(0.2), FloatType(6.0)));
		TransformType tx = vqm_set(rotation, translation, scale_shear);
		check_vqm_near_equal(vqm_inverse(tx, fallback_scale), vqm_inverse(tx), threshold);

		// The scale/shear part is not invertible: the function uses the fallback scale without shear
		scale_shear.x_axis = vector_zero();
		tx = vqm_set_scale_shear(tx, scale_shear);
		TransformType inverse_result = vqm_inverse(tx, fallback_scale);
		CHECK(vqm_is_finite(inverse_result));
		check_vqm_near_equal(inverse_result, vqm_inverse(vqm_set(rotation, translation, fallback_scale)), threshold);

		// All the rows are parallel: the determinant is zero
		scale_shear = matrix_set(
			vector_set(FloatType(1.0), FloatType(2.0), FloatType(3.0)),
			vector_set(FloatType(-2.0), FloatType(-4.0), FloatType(-6.0)),
			vector_set(FloatType(0.5), FloatType(1.0), FloatType(1.5)));
		tx = vqm_set_scale_shear(tx, scale_shear);
		inverse_result = vqm_inverse(tx, fallback_scale);
		CHECK(vqm_is_finite(inverse_result));
		check_vqm_near_equal(inverse_result, vqm_inverse(vqm_set(rotation, translation, fallback_scale)), threshold);
	}

	// VQM lerp and slerp validation
	{
		const FloatType alpha = FloatType(0.33);
		const ScalarType alpha_s = scalar_set(alpha);

		QuatType rotation0 = quat_normalize(quat_from_euler(scalar_deg_to_rad(FloatType(30.0)), scalar_deg_to_rad(FloatType(-45.0)), scalar_deg_to_rad(FloatType(90.0))));
		QuatType rotation1 = quat_normalize(quat_from_euler(scalar_deg_to_rad(FloatType(45.0)), scalar_deg_to_rad(FloatType(60.0)), scalar_deg_to_rad(FloatType(120.0))));
		Vector4Type translation0 = vector_set(FloatType(-0.001138), FloatType(0.91623), FloatType(-1.624598));
		Vector4Type translation1 = vector_set(FloatType(-8.5), FloatType(0.25), FloatType(14.0));

		const Matrix3x3Type scale_shear0 = matrix_set(
			vector_set(FloatType(-4.0), FloatType(0.5), FloatType(-0.25)),
			vector_set(FloatType(0.3), FloatType(5.0), FloatType(0.75)),
			vector_set(FloatType(-0.6), FloatType(0.2), FloatType(6.0)));
		const Matrix3x3Type scale_shear1 = matrix_set(
			vector_set(FloatType(0.5), FloatType(-1.5), FloatType(2.0)),
			vector_set(FloatType(1.25), FloatType(-2.0), FloatType(0.1)),
			vector_set(FloatType(3.0), FloatType(0.4), FloatType(3.5)));

		TransformType tx0 = vqm_set(rotation0, translation0, scale_shear0);
		TransformType tx1 = vqm_set(rotation1, translation1, scale_shear1);

		// vqm_lerp interpolates each component of the two transforms
		const Matrix3x3Type lerp_scale_shear = matrix_set(
			vector_lerp(scale_shear0.x_axis, scale_shear1.x_axis, alpha),
			vector_lerp(scale_shear0.y_axis, scale_shear1.y_axis, alpha),
			vector_lerp(scale_shear0.z_axis, scale_shear1.z_axis, alpha));
		TransformType lerp_ref = vqm_set(quat_lerp(rotation0, rotation1, alpha), vector_lerp(translation0, translation1, alpha), lerp_scale_shear);

		// vqm_slerp divides each column by its scale, interpolates the scale with eerp and the shear with lerp,
		// and multiplies each column by the new scale. The scales of the X and Y columns have a different sign:
		// these columns use lerp. The scales of the Z column are 6.0 and 3.5.
		TransformType slerp_ref = vqm_set_rotation(lerp_ref, quat_slerp(rotation0, rotation1, alpha));
		const FloatType z_scale = scalar_eerp(FloatType(6.0), FloatType(3.5), alpha);
		const Matrix3x3Type slerp_scale_shear = matrix_set(
			vector_set_z(lerp_scale_shear.x_axis, scalar_lerp(FloatType(-0.25) / FloatType(6.0), FloatType(2.0) / FloatType(3.5), alpha) * z_scale),
			vector_set_z(lerp_scale_shear.y_axis, scalar_lerp(FloatType(0.75) / FloatType(6.0), FloatType(0.1) / FloatType(3.5), alpha) * z_scale),
			vector_set_z(lerp_scale_shear.z_axis, z_scale));
		slerp_ref = vqm_set_scale_shear(slerp_ref, slerp_scale_shear);

		check_vqm_near_equal(vqm_lerp(tx0, tx1, alpha), lerp_ref, threshold);
		check_vqm_near_equal(vqm_lerp(tx0, tx1, alpha_s), lerp_ref, threshold);
		check_vqm_near_equal(vqm_slerp(tx0, tx1, alpha), slerp_ref, threshold);
		check_vqm_near_equal(vqm_slerp(tx0, tx1, alpha_s), slerp_ref, threshold);

		// The result is the start transform when alpha is 0.0 and the end transform when alpha is 1.0
		check_vqm_near_equal(vqm_lerp(tx0, tx1, FloatType(0.0)), tx0, threshold);
		check_vqm_near_equal(vqm_lerp(tx0, tx1, FloatType(1.0)), tx1, threshold);
		check_vqm_near_equal(vqm_slerp(tx0, tx1, FloatType(0.0)), tx0, threshold);
		check_vqm_near_equal(vqm_slerp(tx0, tx1, FloatType(1.0)), tx1, threshold);

		// The end transform is the start transform with the X column scaled by 4.0: the result at alpha 0.5
		// is the start transform with the X column scaled by 2.0. The shear does not change.
		const Vector4Type x_scale4 = vector_set(FloatType(4.0), FloatType(1.0), FloatType(1.0), FloatType(1.0));
		const Vector4Type x_scale2 = vector_set(FloatType(2.0), FloatType(1.0), FloatType(1.0), FloatType(1.0));
		const TransformType scaled_tx = vqm_set_scale_shear(tx1, matrix_set(vector_mul(scale_shear1.x_axis, x_scale4), vector_mul(scale_shear1.y_axis, x_scale4), vector_mul(scale_shear1.z_axis, x_scale4)));
		const TransformType half_scaled_tx = vqm_set_scale_shear(tx1, matrix_set(vector_mul(scale_shear1.x_axis, x_scale2), vector_mul(scale_shear1.y_axis, x_scale2), vector_mul(scale_shear1.z_axis, x_scale2)));
		check_vqm_near_equal(vqm_slerp(tx1, scaled_tx, FloatType(0.5)), half_scaled_tx, threshold);
	}

	// Transforms with the identity scale/shear part: the m22 slot holds a flag and the functions skip the scale/shear part
	{
		const QuatType rotation0 = quat_from_euler(scalar_deg_to_rad(FloatType(10.1)), scalar_deg_to_rad(FloatType(41.6)), scalar_deg_to_rad(FloatType(-12.7)));
		const QuatType rotation1 = quat_from_euler(scalar_deg_to_rad(FloatType(-35.2)), scalar_deg_to_rad(FloatType(5.3)), scalar_deg_to_rad(FloatType(72.4)));
		const Vector4Type translation0 = vector_set(FloatType(1.0), FloatType(2.0), FloatType(3.0));
		const Vector4Type translation1 = vector_set(FloatType(-8.5), FloatType(0.25), FloatType(14.0));
		const Vector4Type one = vector_set(FloatType(1.0));
		const Vector4Type scale = vector_set(FloatType(2.0), FloatType(-3.0), FloatType(0.5));
		const Matrix3x3Type identity3x3 = matrix_identity();
		const Matrix3x3Type scale_shear0 = matrix_set(
			vector_set(FloatType(2.0), FloatType(0.5), FloatType(-0.25)),
			vector_set(FloatType(0.75), FloatType(-3.0), FloatType(0.125)),
			vector_set(FloatType(-1.5), FloatType(0.375), FloatType(1.25)));
		const Matrix3x3Type scale_shear1 = matrix_set(
			vector_set(FloatType(0.5), FloatType(-1.5), FloatType(2.0)),
			vector_set(FloatType(1.25), FloatType(-2.0), FloatType(0.1)),
			vector_set(FloatType(3.0), FloatType(0.4), FloatType(3.5)));

		const TransformType rigid0 = vqm_set(rotation0, translation0, one);
		const TransformType rigid1 = vqm_set(rotation1, translation1, identity3x3);
		const TransformType general0 = vqm_set(rotation0, translation0, scale_shear0);
		const TransformType general1 = vqm_set(rotation1, translation1, scale_shear1);

		// The setters set the flag when the scale/shear part is exactly the identity
		CHECK(rtm_impl::vqm_has_identity_scale_shear(TransformType(vqm_identity())));
		CHECK(rtm_impl::vqm_has_identity_scale_shear(rigid0));
		CHECK(rtm_impl::vqm_has_identity_scale_shear(rigid1));
		CHECK(!rtm_impl::vqm_has_identity_scale_shear(general0));
		CHECK(!rtm_impl::vqm_has_identity_scale_shear(vqm_set(rotation0, translation0, scale)));
		CHECK(rtm_impl::vqm_has_identity_scale_shear(vqm_set_scale_shear(general0, identity3x3)));
		CHECK(rtm_impl::vqm_has_identity_scale_shear(vqm_set_scale(vqm_set(rotation0, translation0, scale), one)));
		CHECK(rtm_impl::vqm_has_identity_scale_shear(vqm_from_matrix(identity3x3)));

		// The functions that do not change the scale/shear part keep the flag
		CHECK(rtm_impl::vqm_has_identity_scale_shear(vqm_set_translation(rigid0, translation1)));
		CHECK(rtm_impl::vqm_has_identity_scale_shear(vqm_set_rotation(rigid0, rotation1)));
		CHECK(rtm_impl::vqm_has_identity_scale_shear(vqm_normalize(rigid0)));
		CHECK(rtm_impl::vqm_has_identity_scale_shear(vqm_cast(vqm_cast(rigid0))));

		// The accessors return the identity values, not the flag
		CHECK(vqm_is_finite(rigid0));
		CHECK(vqm_is_finite(vqm_cast(rigid0)));
		CHECK(vector_all_equal3(vqm_get_scale(rigid0), one));
		CHECK(vector_all_equal3(vqm_get_translation(rigid0), translation0));
		CHECK(quat_near_equal(vqm_get_rotation(rigid0), rotation0, threshold));
		const Matrix3x3Type rigid_scale_shear = vqm_get_scale_shear(rigid0);
		CHECK(vector_all_equal3(rigid_scale_shear.x_axis, identity3x3.x_axis));
		CHECK(vector_all_equal3(rigid_scale_shear.y_axis, identity3x3.y_axis));
		CHECK(vector_all_equal3(rigid_scale_shear.z_axis, identity3x3.z_axis));
		check_matrix_near_equal(matrix_from_vqm(rigid0), matrix_from_vqv(rotation0, translation0, one), threshold);

		// vqm_set_scale replaces the flag with the new m22
		const TransformType scaled_rigid = vqm_set_scale(rigid0, scale);
		CHECK(!rtm_impl::vqm_has_identity_scale_shear(scaled_rigid));
		check_vqm_near_equal(scaled_rigid, vqm_set(rotation0, translation0, scale), threshold);

		const TransformType inputs[] = { rigid0, rigid1, general0, general1 };
		const Vector4Type point = vector_set(FloatType(-2.5), FloatType(1.75), FloatType(0.5));

		for (const TransformType& lhs : inputs)
		{
			const Matrix3x4Type lhs_mtx = matrix_from_vqm(lhs);

			// The product is equal to the product of the matrices for each combination of the two kinds
			for (const TransformType& rhs : inputs)
				check_matrix_near_equal(matrix_from_vqm(vqm_mul(lhs, rhs)), matrix_mul(lhs_mtx, matrix_from_vqm(rhs)), threshold);

			CHECK(vector_all_near_equal3(vqm_mul_point3(point, lhs), matrix_mul_point3(point, lhs_mtx), threshold));
			CHECK(vector_all_near_equal3(vqm_mul_vector3(point, lhs), matrix_mul_vector3(point, lhs_mtx), threshold));

			check_matrix_near_equal(matrix_from_vqm(vqm_inverse(lhs)), matrix_inverse(lhs_mtx), threshold);
			check_vqm_near_equal(vqm_inverse(lhs, scale), vqm_inverse(lhs), threshold);
		}

		// The product and the inverse keep the flag when no scale/shear part is involved
		CHECK(rtm_impl::vqm_has_identity_scale_shear(vqm_mul(rigid0, rigid1)));
		CHECK(rtm_impl::vqm_has_identity_scale_shear(vqm_inverse(rigid0)));
		CHECK(rtm_impl::vqm_has_identity_scale_shear(vqm_inverse(rigid0, scale)));
		CHECK(!rtm_impl::vqm_has_identity_scale_shear(vqm_mul(rigid0, general1)));
		CHECK(!rtm_impl::vqm_has_identity_scale_shear(vqm_mul(general0, rigid1)));

		// When the scale/shear part of rhs is the identity, the result has the scale/shear part of lhs
		check_scale_shear_near_equal(vqm_mul(general0, rigid1), general0, threshold);

		// The sum and the product with a scalar do not keep the flag
		const TransformType add_result = vqm_add(rigid0, rigid1);
		CHECK(!rtm_impl::vqm_has_identity_scale_shear(add_result));
		CHECK(vector_all_near_equal3(vqm_get_translation(add_result), vector_add(translation0, translation1), threshold));
		check_scale_shear_near_equal(add_result, vqm_set(rotation0, translation0, vector_set(FloatType(2.0))), threshold);

		const TransformType add_general_result = vqm_add(rigid0, general1);
		const Matrix3x3Type add_general_scale_shear = vqm_get_scale_shear(add_general_result);
		CHECK(vector_all_near_equal3(add_general_scale_shear.x_axis, vector_add(identity3x3.x_axis, scale_shear1.x_axis), threshold));
		CHECK(vector_all_near_equal3(add_general_scale_shear.y_axis, vector_add(identity3x3.y_axis, scale_shear1.y_axis), threshold));
		CHECK(vector_all_near_equal3(add_general_scale_shear.z_axis, vector_add(identity3x3.z_axis, scale_shear1.z_axis), threshold));

		const TransformType mul_scalar_result = vqm_mul(rigid0, FloatType(3.0));
		CHECK(!rtm_impl::vqm_has_identity_scale_shear(mul_scalar_result));
		CHECK(vector_all_near_equal3(vqm_get_translation(mul_scalar_result), vector_mul(translation0, FloatType(3.0)), threshold));
		check_scale_shear_near_equal(mul_scalar_result, vqm_set(rotation0, translation0, vector_set(FloatType(3.0))), threshold);

		// The interpolation of two transforms with the flag keeps the flag
		const FloatType alpha = FloatType(0.33);
		const TransformType lerp_result = vqm_lerp(rigid0, rigid1, alpha);
		CHECK(rtm_impl::vqm_has_identity_scale_shear(lerp_result));
		check_vqm_near_equal(lerp_result, vqm_set(quat_lerp(rotation0, rotation1, alpha), vector_lerp(translation0, translation1, alpha), one), threshold);

		const TransformType slerp_result = vqm_slerp(rigid0, rigid1, alpha);
		CHECK(rtm_impl::vqm_has_identity_scale_shear(slerp_result));
		check_vqm_near_equal(slerp_result, vqm_set(quat_slerp(rotation0, rotation1, alpha), vector_lerp(translation0, translation1, alpha), one), threshold);

		// The interpolation of a transform with the flag and a transform without it interpolates from the identity
		const TransformType lerp_mixed_result = vqm_lerp(rigid0, general1, alpha);
		CHECK(!rtm_impl::vqm_has_identity_scale_shear(lerp_mixed_result));
		const Matrix3x3Type lerp_mixed_ref = matrix_set(
			vector_lerp(identity3x3.x_axis, scale_shear1.x_axis, alpha),
			vector_lerp(identity3x3.y_axis, scale_shear1.y_axis, alpha),
			vector_lerp(identity3x3.z_axis, scale_shear1.z_axis, alpha));
		check_vqm_near_equal(lerp_mixed_result, vqm_set(quat_lerp(rotation0, rotation1, alpha), vector_lerp(translation0, translation1, alpha), lerp_mixed_ref), threshold);

		// The scale is positive on the three axes: vqm_slerp interpolates it with eerp from 1.0
		const Vector4Type positive_scale = vector_set(FloatType(2.0), FloatType(3.0), FloatType(0.5));
		const TransformType slerp_mixed_result = vqm_slerp(rigid0, vqm_set(rotation1, translation1, positive_scale), alpha);
		CHECK(!rtm_impl::vqm_has_identity_scale_shear(slerp_mixed_result));
		check_vqm_near_equal(slerp_mixed_result, vqm_set(quat_slerp(rotation0, rotation1, alpha), vector_lerp(translation0, translation1, alpha), vector_eerp(one, positive_scale, alpha)), threshold);
	}

	// VQM normalize validation
	{
		Vector4Type translation = vector_set(FloatType(1.0), FloatType(2.0), FloatType(3.0));
		Vector4Type scale = vector_set(FloatType(4.0), FloatType(-5.0), FloatType(6.0));

		QuatType rotation = quat_from_euler(scalar_deg_to_rad(FloatType(10.1)), scalar_deg_to_rad(FloatType(41.6)), scalar_deg_to_rad(FloatType(-12.7)));
		TransformType tx = vqm_set(rotation, translation, scale);
		CHECK(quat_is_normalized(vqm_normalize(tx).rotation, threshold));

		QuatType quat = quat_set(FloatType(-0.001138), FloatType(0.91623), FloatType(-1.624598), FloatType(0.715671));
		tx = vqm_set(quat, translation, scale);
		CHECK(!quat_is_normalized(tx.rotation, threshold));

		TransformType normalize_result = vqm_normalize(tx);
		CHECK(quat_is_normalized(normalize_result.rotation, threshold));
		CHECK(quat_near_equal(normalize_result.rotation, quat_normalize(quat), threshold));
		check_scale_shear_near_equal(normalize_result, tx, threshold);
		CHECK(vector_all_near_equal3(vqm_get_translation(normalize_result), vqm_get_translation(tx), threshold));
	}

	// VQM is_finite validation
	{
		const FloatType inf = std::numeric_limits<FloatType>::infinity();
		const FloatType nan = std::numeric_limits<FloatType>::quiet_NaN();
		const FloatType one = FloatType(1.0);

		const QuatType quat0 = quat_from_euler(scalar_deg_to_rad(FloatType(30.0)), scalar_deg_to_rad(FloatType(-45.0)), scalar_deg_to_rad(FloatType(90.0)));
		const QuatType quat_inf = quat_set(inf, inf, inf, inf);
		const QuatType quat_nan = quat_set(nan, nan, nan, nan);

		const Vector4Type trans0 = vector_set(FloatType(-0.001138), FloatType(0.91623), FloatType(-1.624598));
		const Vector4Type scale0 = vector_set(FloatType(-1.915), FloatType(0.23656), FloatType(-3.7811));
		const Vector4Type vec_inf = vector_set(inf, inf, inf, inf);
		const Vector4Type vec_nan = vector_set(nan, nan, nan, nan);

		const TransformType identity = vqm_identity();
		CHECK(vqm_is_finite(identity) == true);
		CHECK(vqm_is_finite(vqm_set(quat0, trans0, scale0)) == true);
		CHECK(vqm_is_finite(vqm_set(quat_inf, trans0, scale0)) == false);
		CHECK(vqm_is_finite(vqm_set(quat_nan, trans0, scale0)) == false);
		CHECK(vqm_is_finite(vqm_set(quat0, vec_inf, scale0)) == false);
		CHECK(vqm_is_finite(vqm_set(quat0, vec_nan, scale0)) == false);
		CHECK(vqm_is_finite(vqm_set(quat0, trans0, vec_inf)) == false);
		CHECK(vqm_is_finite(vqm_set(quat0, trans0, vec_nan)) == false);
		CHECK(vqm_is_finite(vqm_set(quat_inf, vec_inf, vec_inf)) == false);
		CHECK(vqm_is_finite(vqm_set(quat_nan, vec_nan, vec_nan)) == false);

		// vqm_set puts the scale [x], [y], and [z] in the x_axis, y_axis, and z_axis: test each axis
		CHECK(vqm_is_finite(vqm_set(quat0, trans0, vector_set(inf, one, one))) == false);
		CHECK(vqm_is_finite(vqm_set(quat0, trans0, vector_set(one, inf, one))) == false);
		CHECK(vqm_is_finite(vqm_set(quat0, trans0, vector_set(one, one, inf))) == false);
		CHECK(vqm_is_finite(vqm_set(quat0, trans0, vector_set(nan, one, one))) == false);
		CHECK(vqm_is_finite(vqm_set(quat0, trans0, vector_set(one, nan, one))) == false);
		CHECK(vqm_is_finite(vqm_set(quat0, trans0, vector_set(one, one, nan))) == false);

		// The translation [w] is undefined: it does not change the result
		CHECK(vqm_is_finite(vqm_set(quat0, vector_set(FloatType(1.0), FloatType(2.0), FloatType(3.0), inf), scale0)) == true);
	}
}

TEST_CASE("vqmf math", "[math][vqm]")
{
	// The transform uses 4 registers: the [w] lanes hold the third row of the scale/shear part
	static_assert(sizeof(vqmf) == 4 * sizeof(vector4f), "vqmf must use 4 registers");

	test_vqm_impl<vqmf, float>(1.0E-3F);

	const quatf src_rotation = quat_set(0.39564531008956383F, 0.044254239301713752F, 0.22768840967675355F, 0.88863059760894492F);
	const vector4f src_translation = vector_set(-2.65F, 2.996113F, 0.68123521F);
	const matrix3x3f src_scale_shear = matrix_set(vector_set(1.2F, 0.5F, -0.25F), vector_set(0.3F, 0.8F, 0.75F), vector_set(-0.6F, 0.2F, 2.1F));
	const vqmf src = vqm_set(src_rotation, src_translation, src_scale_shear);
	const vqmd dst = vqm_cast(src);

	// A cast back to the input type gives the input
	check_vqm_near_equal(src, vqm_cast(dst), 1.0E-6F);
}

TEST_CASE("vqmd math", "[math][vqm]")
{
	// The transform uses 4 registers: the [w] lanes hold the third row of the scale/shear part
	static_assert(sizeof(vqmd) == 4 * sizeof(vector4d), "vqmd must use 4 registers");

	test_vqm_impl<vqmd, double>(1.0E-8);

	const quatd src_rotation = quat_set(0.39564531008956383, 0.044254239301713752, 0.22768840967675355, 0.88863059760894492);
	const vector4d src_translation = vector_set(-2.65, 2.996113, 0.68123521);
	const matrix3x3d src_scale_shear = matrix_set(vector_set(1.2, 0.5, -0.25), vector_set(0.3, 0.8, 0.75), vector_set(-0.6, 0.2, 2.1));
	const vqmd src = vqm_set(src_rotation, src_translation, src_scale_shear);
	const vqmf dst = vqm_cast(src);

	// A cast back to the input type gives the input
	check_vqm_near_equal(src, vqm_cast(dst), 1.0E-6);
}
