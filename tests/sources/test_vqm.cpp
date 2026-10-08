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
#include <rtm/experimental/vqmf.h>
#include <rtm/experimental/vqmd.h>

using namespace rtm;

template<typename TransformType, typename FloatType>
static void test_vqm_impl(const FloatType threshold)
{
	using QuatType = typename related_types<FloatType>::quat;
	using Vector4Type = typename related_types<FloatType>::vector4;
	using Matrix3x4Type = typename related_types<FloatType>::matrix3x4;

	// Identity validation
	{
		Vector4Type point = vector_set(FloatType(12.0), FloatType(0.0), FloatType(-130.033));

		TransformType identity = vqm_identity();

		Vector4Type mul_point_result = vqm_mul_point3(point, identity);
		CHECK(vector_all_near_equal3(mul_point_result, point, threshold));

		TransformType mul_itself_result = vqm_mul(identity, identity);
		CHECK(quat_near_equal(mul_itself_result.rotation, identity.rotation, threshold));
		CHECK(vector_all_near_equal3(mul_itself_result.x_axis, identity.x_axis, threshold));
		CHECK(vector_all_near_equal3(mul_itself_result.y_axis, identity.y_axis, threshold));
		CHECK(vector_all_near_equal3(mul_itself_result.z_axis, identity.z_axis, threshold));
		CHECK(vector_all_near_equal3(mul_itself_result.translation, identity.translation, threshold));

		TransformType inverse_result = vqm_inverse(identity);
		CHECK(quat_near_equal(inverse_result.rotation, identity.rotation, threshold));
		CHECK(vector_all_near_equal3(inverse_result.x_axis, identity.x_axis, threshold));
		CHECK(vector_all_near_equal3(inverse_result.y_axis, identity.y_axis, threshold));
		CHECK(vector_all_near_equal3(inverse_result.z_axis, identity.z_axis, threshold));
		CHECK(vector_all_near_equal3(inverse_result.translation, identity.translation, threshold));
	}

	// Getters and setters
	{
		QuatType rotation = quat_from_euler(scalar_deg_to_rad(FloatType(10.1)), scalar_deg_to_rad(FloatType(41.6)), scalar_deg_to_rad(FloatType(-12.7)));
		Vector4Type translation = vector_set(FloatType(1.0), FloatType(2.0), FloatType(3.0));
		Vector4Type scale = vector_set(FloatType(4.0), FloatType(5.0), FloatType(6.0));

		TransformType identity = vqm_identity();
		TransformType tx = vqm_set_rotation(identity, rotation);
		CHECK(quat_near_equal(vqm_get_rotation(tx), rotation, threshold));
		CHECK(vector_all_near_equal3(tx.x_axis, identity.x_axis, threshold));
		CHECK(vector_all_near_equal3(tx.y_axis, identity.y_axis, threshold));
		CHECK(vector_all_near_equal3(tx.z_axis, identity.z_axis, threshold));
		CHECK(vector_all_near_equal3(tx.translation, identity.translation, threshold));

		tx = vqm_set_translation(tx, translation);
		CHECK(quat_near_equal(vqm_get_rotation(tx), rotation, threshold));
		CHECK(vector_all_near_equal3(tx.x_axis, identity.x_axis, threshold));
		CHECK(vector_all_near_equal3(tx.y_axis, identity.y_axis, threshold));
		CHECK(vector_all_near_equal3(tx.z_axis, identity.z_axis, threshold));
		CHECK(vector_all_near_equal3(vqm_get_translation(tx), translation, threshold));

		tx = vqm_set_scale(tx, scale);
		CHECK(quat_near_equal(vqm_get_rotation(tx), rotation, threshold));
		CHECK(vector_all_near_equal3(vqm_get_scale(tx), scale, threshold));
		CHECK(vector_all_near_equal3(vqm_get_translation(tx), translation, threshold));
	}

	// Addition
	{
		QuatType rotation0 = quat_from_euler(scalar_deg_to_rad(FloatType(10.1)), scalar_deg_to_rad(FloatType(41.6)), scalar_deg_to_rad(FloatType(-12.7)));
		Vector4Type translation0 = vector_set(FloatType(1.0), FloatType(2.0), FloatType(3.0));
		Vector4Type scale0 = vector_set(FloatType(4.0), FloatType(5.0), FloatType(6.0));
		TransformType tx0 = vqm_set(translation0, rotation0, scale0);

		QuatType rotation1 = quat_from_euler(scalar_deg_to_rad(FloatType(-35.2)), scalar_deg_to_rad(FloatType(5.3)), scalar_deg_to_rad(FloatType(72.4)));
		Vector4Type translation1 = vector_set(FloatType(-8.5), FloatType(0.25), FloatType(14.0));
		Vector4Type scale1 = vector_set(FloatType(0.5), FloatType(-2.0), FloatType(3.5));
		TransformType tx1 = vqm_set(translation1, rotation1, scale1);

		// The function adds each component of the two transforms
		TransformType add_result = vqm_add(tx0, tx1);
		CHECK(vector_all_near_equal(quat_to_vector(add_result.rotation), vector_add(quat_to_vector(tx0.rotation), quat_to_vector(tx1.rotation)), threshold));
		CHECK(vector_all_near_equal3(add_result.translation, vector_add(translation0, translation1), threshold));
		CHECK(vector_all_near_equal3(add_result.x_axis, vector_add(tx0.x_axis, tx1.x_axis), threshold));
		CHECK(vector_all_near_equal3(add_result.y_axis, vector_add(tx0.y_axis, tx1.y_axis), threshold));
		CHECK(vector_all_near_equal3(add_result.z_axis, vector_add(tx0.z_axis, tx1.z_axis), threshold));

		// The sum of a transform with itself is equal to the product of the transform and 2
		TransformType add_itself_result = vqm_add(tx0, tx0);
		TransformType mul_scalar_result = vqm_mul(tx0, FloatType(2.0));
		CHECK(vector_all_near_equal(quat_to_vector(add_itself_result.rotation), quat_to_vector(mul_scalar_result.rotation), threshold));
		CHECK(vector_all_near_equal3(add_itself_result.translation, mul_scalar_result.translation, threshold));
		CHECK(vector_all_near_equal3(add_itself_result.x_axis, mul_scalar_result.x_axis, threshold));
		CHECK(vector_all_near_equal3(add_itself_result.y_axis, mul_scalar_result.y_axis, threshold));
		CHECK(vector_all_near_equal3(add_itself_result.z_axis, mul_scalar_result.z_axis, threshold));
	}

	// Matrix conversion validation
	{
		QuatType rotation = quat_from_euler(scalar_deg_to_rad(FloatType(10.1)), scalar_deg_to_rad(FloatType(41.6)), scalar_deg_to_rad(FloatType(-12.7)));
		Vector4Type translation = vector_set(FloatType(1.0), FloatType(2.0), FloatType(3.0));
		Vector4Type scale = vector_set(FloatType(4.0), FloatType(5.0), FloatType(6.0));

		Matrix3x4Type src_mtx = matrix_from_qvv(rotation, translation, scale);
		TransformType dst_tx = vqm_set(translation, rotation, scale);
		Matrix3x4Type dst_mtx = vqm_to_matrix(dst_tx);
		CHECK(vector_all_near_equal3(src_mtx.x_axis, dst_mtx.x_axis, threshold));
		CHECK(vector_all_near_equal3(src_mtx.y_axis, dst_mtx.y_axis, threshold));
		CHECK(vector_all_near_equal3(src_mtx.z_axis, dst_mtx.z_axis, threshold));
		CHECK(vector_all_near_equal3(src_mtx.w_axis, dst_mtx.w_axis, threshold));
	}

	// VQM + VQM validation
	{
		// TODO
	}

	// VQM * VQM validation
	{
		QuatType rotation = quat_from_euler(scalar_deg_to_rad(FloatType(10.1)), scalar_deg_to_rad(FloatType(41.6)), scalar_deg_to_rad(FloatType(-12.7)));
		Vector4Type translation = vector_set(FloatType(1.0), FloatType(2.0), FloatType(3.0));

		// All positive scale
		Vector4Type scale = vector_set(FloatType(4.0), FloatType(5.0), FloatType(6.0));

		Matrix3x4Type src_mtx = matrix_from_qvv(rotation, translation, scale);
		src_mtx = matrix_mul(src_mtx, src_mtx);

		TransformType dst_tx = vqm_set(translation, rotation, scale);
		dst_tx = vqm_mul(dst_tx, dst_tx);
		Matrix3x4Type dst_mtx = vqm_to_matrix(dst_tx);
		CHECK(vector_all_near_equal3(src_mtx.x_axis, dst_mtx.x_axis, threshold));
		CHECK(vector_all_near_equal3(src_mtx.y_axis, dst_mtx.y_axis, threshold));
		CHECK(vector_all_near_equal3(src_mtx.z_axis, dst_mtx.z_axis, threshold));
		CHECK(vector_all_near_equal3(src_mtx.w_axis, dst_mtx.w_axis, threshold));

		// One negative scale
		scale = vector_set(FloatType(-4.0), FloatType(5.0), FloatType(6.0));

		src_mtx = matrix_from_qvv(rotation, translation, scale);
		src_mtx = matrix_mul(src_mtx, src_mtx);

		dst_tx = vqm_set(translation, rotation, scale);
		dst_tx = vqm_mul(dst_tx, dst_tx);
		dst_mtx = vqm_to_matrix(dst_tx);
		CHECK(vector_all_near_equal3(src_mtx.x_axis, dst_mtx.x_axis, threshold));
		CHECK(vector_all_near_equal3(src_mtx.y_axis, dst_mtx.y_axis, threshold));
		CHECK(vector_all_near_equal3(src_mtx.z_axis, dst_mtx.z_axis, threshold));
		CHECK(vector_all_near_equal3(src_mtx.w_axis, dst_mtx.w_axis, threshold));

		// Two negative scale
		scale = vector_set(FloatType(-4.0), FloatType(-5.0), FloatType(6.0));

		src_mtx = matrix_from_qvv(rotation, translation, scale);
		src_mtx = matrix_mul(src_mtx, src_mtx);

		dst_tx = vqm_set(translation, rotation, scale);
		dst_tx = vqm_mul(dst_tx, dst_tx);
		dst_mtx = vqm_to_matrix(dst_tx);
		CHECK(vector_all_near_equal3(src_mtx.x_axis, dst_mtx.x_axis, threshold));
		CHECK(vector_all_near_equal3(src_mtx.y_axis, dst_mtx.y_axis, threshold));
		CHECK(vector_all_near_equal3(src_mtx.z_axis, dst_mtx.z_axis, threshold));
		CHECK(vector_all_near_equal3(src_mtx.w_axis, dst_mtx.w_axis, threshold));

		// Three negative scale
		scale = vector_set(FloatType(-4.0), FloatType(-5.0), FloatType(-6.0));

		src_mtx = matrix_from_qvv(rotation, translation, scale);
		src_mtx = matrix_mul(src_mtx, src_mtx);

		dst_tx = vqm_set(translation, rotation, scale);
		dst_tx = vqm_mul(dst_tx, dst_tx);
		dst_mtx = vqm_to_matrix(dst_tx);
		CHECK(vector_all_near_equal3(src_mtx.x_axis, dst_mtx.x_axis, threshold));
		CHECK(vector_all_near_equal3(src_mtx.y_axis, dst_mtx.y_axis, threshold));
		CHECK(vector_all_near_equal3(src_mtx.z_axis, dst_mtx.z_axis, threshold));
		CHECK(vector_all_near_equal3(src_mtx.w_axis, dst_mtx.w_axis, threshold));

		// One zero scale
		scale = vector_set(FloatType(0.0), FloatType(5.0), FloatType(6.0));

		src_mtx = matrix_from_qvv(rotation, translation, scale);
		src_mtx = matrix_mul(src_mtx, src_mtx);

		dst_tx = vqm_set(translation, rotation, scale);
		dst_tx = vqm_mul(dst_tx, dst_tx);
		dst_mtx = vqm_to_matrix(dst_tx);
		CHECK(vector_all_near_equal3(src_mtx.x_axis, dst_mtx.x_axis, threshold));
		CHECK(vector_all_near_equal3(src_mtx.y_axis, dst_mtx.y_axis, threshold));
		CHECK(vector_all_near_equal3(src_mtx.z_axis, dst_mtx.z_axis, threshold));
		CHECK(vector_all_near_equal3(src_mtx.w_axis, dst_mtx.w_axis, threshold));

		// Two zero scale
		scale = vector_set(FloatType(0.0), FloatType(0.0), FloatType(6.0));

		src_mtx = matrix_from_qvv(rotation, translation, scale);
		src_mtx = matrix_mul(src_mtx, src_mtx);

		dst_tx = vqm_set(translation, rotation, scale);
		dst_tx = vqm_mul(dst_tx, dst_tx);
		dst_mtx = vqm_to_matrix(dst_tx);
		CHECK(vector_all_near_equal3(src_mtx.x_axis, dst_mtx.x_axis, threshold));
		CHECK(vector_all_near_equal3(src_mtx.y_axis, dst_mtx.y_axis, threshold));
		CHECK(vector_all_near_equal3(src_mtx.z_axis, dst_mtx.z_axis, threshold));
		CHECK(vector_all_near_equal3(src_mtx.w_axis, dst_mtx.w_axis, threshold));

		// Three zero scale
		scale = vector_set(FloatType(0.0), FloatType(0.0), FloatType(0.0));

		src_mtx = matrix_from_qvv(rotation, translation, scale);
		src_mtx = matrix_mul(src_mtx, src_mtx);

		dst_tx = vqm_set(translation, rotation, scale);
		dst_tx = vqm_mul(dst_tx, dst_tx);
		dst_mtx = vqm_to_matrix(dst_tx);
		CHECK(vector_all_near_equal3(src_mtx.x_axis, dst_mtx.x_axis, threshold));
		CHECK(vector_all_near_equal3(src_mtx.y_axis, dst_mtx.y_axis, threshold));
		CHECK(vector_all_near_equal3(src_mtx.z_axis, dst_mtx.z_axis, threshold));
		CHECK(vector_all_near_equal3(src_mtx.w_axis, dst_mtx.w_axis, threshold));
	}

	// VQM * scalar validation
	{
		// TODO
	}

	// point/vec3 * VQM validation
	{
		Vector4Type point = vector_set(FloatType(12.0), FloatType(0.0), FloatType(-130.033));

		QuatType rotation = quat_from_euler(scalar_deg_to_rad(FloatType(10.1)), scalar_deg_to_rad(FloatType(41.6)), scalar_deg_to_rad(FloatType(-12.7)));
		Vector4Type translation = vector_set(FloatType(1.0), FloatType(2.0), FloatType(3.0));

		// All positive scale
		Vector4Type scale = vector_set(FloatType(4.0), FloatType(5.0), FloatType(6.0));

		Matrix3x4Type src_mtx = matrix_from_qvv(rotation, translation, scale);
		Vector4Type src_point = matrix_mul_point3(point, src_mtx);

		TransformType dst_tx = vqm_set(translation, rotation, scale);
		Vector4Type dst_point = vqm_mul_point3(point, dst_tx);
		CHECK(vector_all_near_equal3(src_point, dst_point, threshold));

		src_point = matrix_mul_vector3(point, src_mtx);
		dst_point = vqm_mul_vector3(point, dst_tx);
		CHECK(vector_all_near_equal3(src_point, dst_point, threshold));

		// One negative scale
		scale = vector_set(FloatType(-4.0), FloatType(5.0), FloatType(6.0));

		src_mtx = matrix_from_qvv(rotation, translation, scale);
		src_point = matrix_mul_point3(point, src_mtx);

		dst_tx = vqm_set(translation, rotation, scale);
		dst_point = vqm_mul_point3(point, dst_tx);
		CHECK(vector_all_near_equal3(src_point, dst_point, threshold));

		src_point = matrix_mul_vector3(point, src_mtx);
		dst_point = vqm_mul_vector3(point, dst_tx);
		CHECK(vector_all_near_equal3(src_point, dst_point, threshold));

		// Two negative scale
		scale = vector_set(FloatType(-4.0), FloatType(-5.0), FloatType(6.0));

		src_mtx = matrix_from_qvv(rotation, translation, scale);
		src_point = matrix_mul_point3(point, src_mtx);

		dst_tx = vqm_set(translation, rotation, scale);
		dst_point = vqm_mul_point3(point, dst_tx);
		CHECK(vector_all_near_equal3(src_point, dst_point, threshold));

		src_point = matrix_mul_vector3(point, src_mtx);
		dst_point = vqm_mul_vector3(point, dst_tx);
		CHECK(vector_all_near_equal3(src_point, dst_point, threshold));

		// Three negative scale
		scale = vector_set(FloatType(-4.0), FloatType(-5.0), FloatType(-6.0));

		src_mtx = matrix_from_qvv(rotation, translation, scale);
		src_point = matrix_mul_point3(point, src_mtx);

		dst_tx = vqm_set(translation, rotation, scale);
		dst_point = vqm_mul_point3(point, dst_tx);
		CHECK(vector_all_near_equal3(src_point, dst_point, threshold));

		src_point = matrix_mul_vector3(point, src_mtx);
		dst_point = vqm_mul_vector3(point, dst_tx);
		CHECK(vector_all_near_equal3(src_point, dst_point, threshold));

		// One zero scale
		scale = vector_set(FloatType(0.0), FloatType(5.0), FloatType(6.0));

		src_mtx = matrix_from_qvv(rotation, translation, scale);
		src_point = matrix_mul_point3(point, src_mtx);

		dst_tx = vqm_set(translation, rotation, scale);
		dst_point = vqm_mul_point3(point, dst_tx);
		CHECK(vector_all_near_equal3(src_point, dst_point, threshold));

		src_point = matrix_mul_vector3(point, src_mtx);
		dst_point = vqm_mul_vector3(point, dst_tx);
		CHECK(vector_all_near_equal3(src_point, dst_point, threshold));

		// Two zero scale
		scale = vector_set(FloatType(0.0), FloatType(0.0), FloatType(6.0));

		src_mtx = matrix_from_qvv(rotation, translation, scale);
		src_point = matrix_mul_point3(point, src_mtx);

		dst_tx = vqm_set(translation, rotation, scale);
		dst_point = vqm_mul_point3(point, dst_tx);
		CHECK(vector_all_near_equal3(src_point, dst_point, threshold));

		src_point = matrix_mul_vector3(point, src_mtx);
		dst_point = vqm_mul_vector3(point, dst_tx);
		CHECK(vector_all_near_equal3(src_point, dst_point, threshold));

		// Three zero scale
		scale = vector_set(FloatType(0.0), FloatType(0.0), FloatType(0.0));

		src_mtx = matrix_from_qvv(rotation, translation, scale);
		src_point = matrix_mul_point3(point, src_mtx);

		dst_tx = vqm_set(translation, rotation, scale);
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

		Matrix3x4Type src_mtx = matrix_from_qvv(rotation, translation, scale);
		Matrix3x4Type inv_src_mtx = matrix_inverse(src_mtx);

		TransformType dst_tx = vqm_set(translation, rotation, scale);
		TransformType inv_dst_tx = vqm_inverse(dst_tx);

		Matrix3x4Type inv_dst_mtx = vqm_to_matrix(inv_dst_tx);
		CHECK(vector_all_near_equal3(inv_src_mtx.x_axis, inv_dst_mtx.x_axis, threshold));
		CHECK(vector_all_near_equal3(inv_src_mtx.y_axis, inv_dst_mtx.y_axis, threshold));
		CHECK(vector_all_near_equal3(inv_src_mtx.z_axis, inv_dst_mtx.z_axis, threshold));
		CHECK(vector_all_near_equal3(inv_src_mtx.w_axis, inv_dst_mtx.w_axis, threshold));

		// T * T^-1 = identity
		TransformType identity = vqm_identity();
		TransformType inverse_mul_result = vqm_mul(dst_tx, vqm_inverse(dst_tx));
		CHECK(quat_near_equal(inverse_mul_result.rotation, identity.rotation, threshold));
		CHECK(vector_all_near_equal3(inverse_mul_result.x_axis, identity.x_axis, threshold));
		CHECK(vector_all_near_equal3(inverse_mul_result.y_axis, identity.y_axis, threshold));
		CHECK(vector_all_near_equal3(inverse_mul_result.z_axis, identity.z_axis, threshold));
		CHECK(vector_all_near_equal3(inverse_mul_result.translation, identity.translation, threshold));

		// T^-1 * T = identity
		inverse_mul_result = vqm_mul(vqm_inverse(dst_tx), dst_tx);
		CHECK(quat_near_equal(inverse_mul_result.rotation, identity.rotation, threshold));
		CHECK(vector_all_near_equal3(inverse_mul_result.x_axis, identity.x_axis, threshold));
		CHECK(vector_all_near_equal3(inverse_mul_result.y_axis, identity.y_axis, threshold));
		CHECK(vector_all_near_equal3(inverse_mul_result.z_axis, identity.z_axis, threshold));
		CHECK(vector_all_near_equal3(inverse_mul_result.translation, identity.translation, threshold));

		// One negative scale
		scale = vector_set(FloatType(-4.0), FloatType(5.0), FloatType(6.0));

		src_mtx = matrix_from_qvv(rotation, translation, scale);
		inv_src_mtx = matrix_inverse(src_mtx);

		dst_tx = vqm_set(translation, rotation, scale);
		inv_dst_tx = vqm_inverse(dst_tx);

		inv_dst_mtx = vqm_to_matrix(inv_dst_tx);
		CHECK(vector_all_near_equal3(inv_src_mtx.x_axis, inv_dst_mtx.x_axis, threshold));
		CHECK(vector_all_near_equal3(inv_src_mtx.y_axis, inv_dst_mtx.y_axis, threshold));
		CHECK(vector_all_near_equal3(inv_src_mtx.z_axis, inv_dst_mtx.z_axis, threshold));
		CHECK(vector_all_near_equal3(inv_src_mtx.w_axis, inv_dst_mtx.w_axis, threshold));

		// T * T^-1 = identity
		identity = vqm_identity();
		inverse_mul_result = vqm_mul(dst_tx, vqm_inverse(dst_tx));
		CHECK(quat_near_equal(inverse_mul_result.rotation, identity.rotation, threshold));
		CHECK(vector_all_near_equal3(inverse_mul_result.x_axis, identity.x_axis, threshold));
		CHECK(vector_all_near_equal3(inverse_mul_result.y_axis, identity.y_axis, threshold));
		CHECK(vector_all_near_equal3(inverse_mul_result.z_axis, identity.z_axis, threshold));
		CHECK(vector_all_near_equal3(inverse_mul_result.translation, identity.translation, threshold));

		// T^-1 * T = identity
		inverse_mul_result = vqm_mul(vqm_inverse(dst_tx), dst_tx);
		CHECK(quat_near_equal(inverse_mul_result.rotation, identity.rotation, threshold));
		CHECK(vector_all_near_equal3(inverse_mul_result.x_axis, identity.x_axis, threshold));
		CHECK(vector_all_near_equal3(inverse_mul_result.y_axis, identity.y_axis, threshold));
		CHECK(vector_all_near_equal3(inverse_mul_result.z_axis, identity.z_axis, threshold));
		CHECK(vector_all_near_equal3(inverse_mul_result.translation, identity.translation, threshold));

		// Two negative scale
		scale = vector_set(FloatType(-4.0), FloatType(-5.0), FloatType(6.0));

		src_mtx = matrix_from_qvv(rotation, translation, scale);
		inv_src_mtx = matrix_inverse(src_mtx);

		dst_tx = vqm_set(translation, rotation, scale);
		inv_dst_tx = vqm_inverse(dst_tx);

		inv_dst_mtx = vqm_to_matrix(inv_dst_tx);
		CHECK(vector_all_near_equal3(inv_src_mtx.x_axis, inv_dst_mtx.x_axis, threshold));
		CHECK(vector_all_near_equal3(inv_src_mtx.y_axis, inv_dst_mtx.y_axis, threshold));
		CHECK(vector_all_near_equal3(inv_src_mtx.z_axis, inv_dst_mtx.z_axis, threshold));
		CHECK(vector_all_near_equal3(inv_src_mtx.w_axis, inv_dst_mtx.w_axis, threshold));

		// T * T^-1 = identity
		identity = vqm_identity();
		inverse_mul_result = vqm_mul(dst_tx, vqm_inverse(dst_tx));
		CHECK(quat_near_equal(inverse_mul_result.rotation, identity.rotation, threshold));
		CHECK(vector_all_near_equal3(inverse_mul_result.x_axis, identity.x_axis, threshold));
		CHECK(vector_all_near_equal3(inverse_mul_result.y_axis, identity.y_axis, threshold));
		CHECK(vector_all_near_equal3(inverse_mul_result.z_axis, identity.z_axis, threshold));
		CHECK(vector_all_near_equal3(inverse_mul_result.translation, identity.translation, threshold));

		// T^-1 * T = identity
		inverse_mul_result = vqm_mul(vqm_inverse(dst_tx), dst_tx);
		CHECK(quat_near_equal(inverse_mul_result.rotation, identity.rotation, threshold));
		CHECK(vector_all_near_equal3(inverse_mul_result.x_axis, identity.x_axis, threshold));
		CHECK(vector_all_near_equal3(inverse_mul_result.y_axis, identity.y_axis, threshold));
		CHECK(vector_all_near_equal3(inverse_mul_result.z_axis, identity.z_axis, threshold));
		CHECK(vector_all_near_equal3(inverse_mul_result.translation, identity.translation, threshold));

		// Three negative scale
		scale = vector_set(FloatType(-4.0), FloatType(-5.0), FloatType(-6.0));

		src_mtx = matrix_from_qvv(rotation, translation, scale);
		inv_src_mtx = matrix_inverse(src_mtx);

		dst_tx = vqm_set(translation, rotation, scale);
		inv_dst_tx = vqm_inverse(dst_tx);

		inv_dst_mtx = vqm_to_matrix(inv_dst_tx);
		CHECK(vector_all_near_equal3(inv_src_mtx.x_axis, inv_dst_mtx.x_axis, threshold));
		CHECK(vector_all_near_equal3(inv_src_mtx.y_axis, inv_dst_mtx.y_axis, threshold));
		CHECK(vector_all_near_equal3(inv_src_mtx.z_axis, inv_dst_mtx.z_axis, threshold));
		CHECK(vector_all_near_equal3(inv_src_mtx.w_axis, inv_dst_mtx.w_axis, threshold));

		// T * T^-1 = identity
		identity = vqm_identity();
		inverse_mul_result = vqm_mul(dst_tx, vqm_inverse(dst_tx));
		CHECK(quat_near_equal(inverse_mul_result.rotation, identity.rotation, threshold));
		CHECK(vector_all_near_equal3(inverse_mul_result.x_axis, identity.x_axis, threshold));
		CHECK(vector_all_near_equal3(inverse_mul_result.y_axis, identity.y_axis, threshold));
		CHECK(vector_all_near_equal3(inverse_mul_result.z_axis, identity.z_axis, threshold));
		CHECK(vector_all_near_equal3(inverse_mul_result.translation, identity.translation, threshold));

		// T^-1 * T = identity
		inverse_mul_result = vqm_mul(vqm_inverse(dst_tx), dst_tx);
		CHECK(quat_near_equal(inverse_mul_result.rotation, identity.rotation, threshold));
		CHECK(vector_all_near_equal3(inverse_mul_result.x_axis, identity.x_axis, threshold));
		CHECK(vector_all_near_equal3(inverse_mul_result.y_axis, identity.y_axis, threshold));
		CHECK(vector_all_near_equal3(inverse_mul_result.z_axis, identity.z_axis, threshold));
		CHECK(vector_all_near_equal3(inverse_mul_result.translation, identity.translation, threshold));
	}
}

TEST_CASE("vqmf math", "[math][vqm]")
{
	test_vqm_impl<vqmf, float>(1.0E-3F);
}

TEST_CASE("vqmd math", "[math][vqm]")
{
	test_vqm_impl<vqmd, double>(1.0E-8);
}
