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

#include <catch2/catch.hpp>

#include <rtm/type_traits.h>
#include <rtm/matrix3x4f.h>
#include <rtm/matrix3x4d.h>
#include <rtm/qvvf.h>
#include <rtm/qvvd.h>

using namespace rtm;

template<typename FloatType>
static void test_affine_matrix_setters(const FloatType threshold)
{
	using QuatType = typename float_traits<FloatType>::quat;
	using Vector4Type = typename float_traits<FloatType>::vector4;
	using QVVType = typename float_traits<FloatType>::qvv;
	using Matrix3x3Type = typename float_traits<FloatType>::matrix3x3;
	using Matrix3x4Type = typename float_traits<FloatType>::matrix3x4;

	const Matrix3x4Type identity = matrix_identity();

	{
		Vector4Type x_axis = vector_set(FloatType(1.0), FloatType(2.0), FloatType(3.0), FloatType(0.0));
		Vector4Type y_axis = vector_set(FloatType(4.0), FloatType(5.0), FloatType(6.0), FloatType(0.0));
		Vector4Type z_axis = vector_set(FloatType(7.0), FloatType(8.0), FloatType(9.0), FloatType(0.0));
		Vector4Type w_axis = vector_set(FloatType(10.0), FloatType(11.0), FloatType(12.0), FloatType(1.0));
		Matrix3x4Type mtx = matrix_set(x_axis, y_axis, z_axis, w_axis);
		CHECK(vector_all_near_equal3(x_axis, mtx.x_axis, threshold));
		CHECK(vector_all_near_equal3(y_axis, mtx.y_axis, threshold));
		CHECK(vector_all_near_equal3(z_axis, mtx.z_axis, threshold));
		CHECK(vector_all_near_equal3(w_axis, mtx.w_axis, threshold));
	}

	{
		CHECK(vector_all_near_equal3(vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0), FloatType(0.0)), identity.x_axis, threshold));
		CHECK(vector_all_near_equal3(vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0), FloatType(0.0)), identity.y_axis, threshold));
		CHECK(vector_all_near_equal3(vector_set(FloatType(0.0), FloatType(0.0), FloatType(1.0), FloatType(0.0)), identity.z_axis, threshold));
		CHECK(vector_all_near_equal3(vector_set(FloatType(0.0), FloatType(0.0), FloatType(0.0), FloatType(1.0)), identity.w_axis, threshold));
	}

	{
		QuatType rotation_around_z = quat_from_euler(scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(90.0)), scalar_deg_to_rad(FloatType(0.0)));
		Vector4Type translation = vector_set(FloatType(1.0), FloatType(2.0), FloatType(3.0));
		Matrix3x4Type mtx = matrix_from_qvv(rotation_around_z, translation, vector_set(FloatType(1.0)));
		CHECK(vector_all_near_equal3(vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0), FloatType(0.0)), mtx.x_axis, threshold));
		CHECK(vector_all_near_equal3(vector_set(FloatType(-1.0), FloatType(0.0), FloatType(0.0), FloatType(0.0)), mtx.y_axis, threshold));
		CHECK(vector_all_near_equal3(vector_set(FloatType(0.0), FloatType(0.0), FloatType(1.0), FloatType(0.0)), mtx.z_axis, threshold));
		CHECK(vector_all_near_equal3(vector_set(FloatType(1.0), FloatType(2.0), FloatType(3.0), FloatType(1.0)), mtx.w_axis, threshold));

		Vector4Type scale = vector_set(FloatType(4.0), FloatType(5.0), FloatType(6.0));
		mtx = matrix_from_qvv(rotation_around_z, translation, scale);
		CHECK(vector_all_near_equal3(vector_set(FloatType(0.0), FloatType(4.0), FloatType(0.0), FloatType(0.0)), mtx.x_axis, threshold));
		CHECK(vector_all_near_equal3(vector_set(FloatType(-5.0), FloatType(0.0), FloatType(0.0), FloatType(0.0)), mtx.y_axis, threshold));
		CHECK(vector_all_near_equal3(vector_set(FloatType(0.0), FloatType(0.0), FloatType(6.0), FloatType(0.0)), mtx.z_axis, threshold));
		CHECK(vector_all_near_equal3(vector_set(FloatType(1.0), FloatType(2.0), FloatType(3.0), FloatType(1.0)), mtx.w_axis, threshold));
	}

	{
		QuatType rotation_around_z = quat_from_euler(scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(90.0)), scalar_deg_to_rad(FloatType(0.0)));
		Matrix3x4Type mtx = matrix_from_quat(rotation_around_z);
		CHECK(vector_all_near_equal3(vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0), FloatType(0.0)), mtx.x_axis, threshold));
		CHECK(vector_all_near_equal3(vector_set(FloatType(-1.0), FloatType(0.0), FloatType(0.0), FloatType(0.0)), mtx.y_axis, threshold));
		CHECK(vector_all_near_equal3(vector_set(FloatType(0.0), FloatType(0.0), FloatType(1.0), FloatType(0.0)), mtx.z_axis, threshold));
		CHECK(vector_all_near_equal3(vector_set(FloatType(0.0), FloatType(0.0), FloatType(0.0), FloatType(1.0)), mtx.w_axis, threshold));
	}

	{
		QuatType rotation_around_z = quat_from_euler(scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(90.0)), scalar_deg_to_rad(FloatType(0.0)));
		Matrix3x4Type mtx = matrix_from_rotation(rotation_around_z);
		CHECK(vector_all_near_equal3(vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0), FloatType(0.0)), mtx.x_axis, threshold));
		CHECK(vector_all_near_equal3(vector_set(FloatType(-1.0), FloatType(0.0), FloatType(0.0), FloatType(0.0)), mtx.y_axis, threshold));
		CHECK(vector_all_near_equal3(vector_set(FloatType(0.0), FloatType(0.0), FloatType(1.0), FloatType(0.0)), mtx.z_axis, threshold));
		CHECK(vector_all_near_equal3(vector_set(FloatType(0.0), FloatType(0.0), FloatType(0.0), FloatType(1.0)), mtx.w_axis, threshold));
	}

	{
		QuatType rotation_around_z = quat_from_euler(scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(90.0)), scalar_deg_to_rad(FloatType(0.0)));
		Matrix3x3Type rotation_mtx = matrix_from_quat(rotation_around_z);
		Matrix3x4Type mtx = matrix_from_rotation(rotation_mtx);
		CHECK(vector_all_near_equal3(vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0), FloatType(0.0)), mtx.x_axis, threshold));
		CHECK(vector_all_near_equal3(vector_set(FloatType(-1.0), FloatType(0.0), FloatType(0.0), FloatType(0.0)), mtx.y_axis, threshold));
		CHECK(vector_all_near_equal3(vector_set(FloatType(0.0), FloatType(0.0), FloatType(1.0), FloatType(0.0)), mtx.z_axis, threshold));
		CHECK(vector_all_near_equal3(vector_set(FloatType(0.0), FloatType(0.0), FloatType(0.0), FloatType(1.0)), mtx.w_axis, threshold));
	}

	{
		Matrix3x4Type mtx = matrix_from_translation(vector_set(FloatType(1.0), FloatType(2.0), FloatType(3.0)));
		CHECK(vector_all_near_equal3(vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0), FloatType(0.0)), mtx.x_axis, threshold));
		CHECK(vector_all_near_equal3(vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0), FloatType(0.0)), mtx.y_axis, threshold));
		CHECK(vector_all_near_equal3(vector_set(FloatType(0.0), FloatType(0.0), FloatType(1.0), FloatType(0.0)), mtx.z_axis, threshold));
		CHECK(vector_all_near_equal3(vector_set(FloatType(1.0), FloatType(2.0), FloatType(3.0), FloatType(1.0)), mtx.w_axis, threshold));
	}

	{
		Matrix3x4Type mtx = matrix_from_scale(vector_set(FloatType(4.0), FloatType(5.0), FloatType(6.0)));
		CHECK(vector_all_near_equal3(vector_set(FloatType(4.0), FloatType(0.0), FloatType(0.0), FloatType(0.0)), mtx.x_axis, threshold));
		CHECK(vector_all_near_equal3(vector_set(FloatType(0.0), FloatType(5.0), FloatType(0.0), FloatType(0.0)), mtx.y_axis, threshold));
		CHECK(vector_all_near_equal3(vector_set(FloatType(0.0), FloatType(0.0), FloatType(6.0), FloatType(0.0)), mtx.z_axis, threshold));
		CHECK(vector_all_near_equal3(vector_set(FloatType(0.0), FloatType(0.0), FloatType(0.0), FloatType(1.0)), mtx.w_axis, threshold));
	}

	{
		QuatType rotation_around_z = quat_from_euler(scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(90.0)), scalar_deg_to_rad(FloatType(0.0)));
		Vector4Type translation = vector_set(FloatType(1.0), FloatType(2.0), FloatType(3.0));
		Vector4Type scale = vector_set(FloatType(4.0), FloatType(5.0), FloatType(6.0));
		QVVType transform = qvv_set(rotation_around_z, translation, scale);
		Matrix3x4Type mtx = matrix_from_qvv(transform);
		CHECK(vector_all_near_equal3(vector_set(FloatType(0.0), FloatType(4.0), FloatType(0.0), FloatType(0.0)), mtx.x_axis, threshold));
		CHECK(vector_all_near_equal3(vector_set(FloatType(-5.0), FloatType(0.0), FloatType(0.0), FloatType(0.0)), mtx.y_axis, threshold));
		CHECK(vector_all_near_equal3(vector_set(FloatType(0.0), FloatType(0.0), FloatType(6.0), FloatType(0.0)), mtx.z_axis, threshold));
		CHECK(vector_all_near_equal3(vector_set(FloatType(1.0), FloatType(2.0), FloatType(3.0), FloatType(1.0)), mtx.w_axis, threshold));
	}

	{
		QuatType rotation_around_z = quat_from_euler(scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(90.0)), scalar_deg_to_rad(FloatType(0.0)));
		Vector4Type translation = vector_set(FloatType(1.0), FloatType(2.0), FloatType(3.0));
		Vector4Type scale = vector_set(FloatType(4.0), FloatType(5.0), FloatType(6.0));
		Matrix3x4Type mtx = matrix_from_qvv(rotation_around_z, translation, scale);
		CHECK(vector_all_near_equal3(matrix_get_axis(mtx, axis4::x), mtx.x_axis, threshold));
		CHECK(vector_all_near_equal3(matrix_get_axis(mtx, axis4::y), mtx.y_axis, threshold));
		CHECK(vector_all_near_equal3(matrix_get_axis(mtx, axis4::z), mtx.z_axis, threshold));
		CHECK(vector_all_near_equal3(matrix_get_axis(mtx, axis4::w), mtx.w_axis, threshold));

		const Matrix3x4Type mtx2 = mtx;
		CHECK(vector_all_near_equal3(matrix_get_axis(mtx2, axis4::x), mtx2.x_axis, threshold));
		CHECK(vector_all_near_equal3(matrix_get_axis(mtx2, axis4::y), mtx2.y_axis, threshold));
		CHECK(vector_all_near_equal3(matrix_get_axis(mtx2, axis4::z), mtx2.z_axis, threshold));
		CHECK(vector_all_near_equal3(matrix_get_axis(mtx2, axis4::w), mtx2.w_axis, threshold));
	}
}

template<typename FloatType>
static void test_affine_matrix_arithmetic(const FloatType threshold)
{
	using QuatType = typename float_traits<FloatType>::quat;
	using Vector4Type = typename float_traits<FloatType>::vector4;
	using Matrix3x4Type = typename float_traits<FloatType>::matrix3x4;

	{
		Vector4Type x_axis = vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0));
		Vector4Type y_axis = vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0));

		QuatType rotation_around_z = quat_from_euler(scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(90.0)), scalar_deg_to_rad(FloatType(0.0)));
		Matrix3x4Type mtx_a = matrix_from_qvv(rotation_around_z, x_axis, vector_set(FloatType(1.0)));
		Vector4Type result = matrix_mul_point3(x_axis, mtx_a);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(1.0), FloatType(1.0), FloatType(0.0)), threshold));
		result = matrix_mul_point3(y_axis, mtx_a);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(0.0), FloatType(0.0), FloatType(0.0)), threshold));

		QuatType rotation_around_x = quat_from_euler(scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(90.0)));
		Matrix3x4Type mtx_b = matrix_from_qvv(rotation_around_x, y_axis, vector_set(FloatType(1.0)));
		result = matrix_mul_point3(x_axis, mtx_b);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(1.0), FloatType(1.0), FloatType(0.0)), threshold));
		result = matrix_mul_point3(y_axis, mtx_b);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(0.0), FloatType(1.0), FloatType(-1.0)), threshold));

		Matrix3x4Type mtx_ab = matrix_mul(mtx_a, mtx_b);
		Matrix3x4Type mtx_ba = matrix_mul(mtx_b, mtx_a);
		result = matrix_mul_point3(x_axis, mtx_ab);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(1.0), FloatType(1.0), FloatType(-1.0)), threshold));
		CHECK(vector_all_near_equal3(result, matrix_mul_point3(matrix_mul_point3(x_axis, mtx_a), mtx_b), threshold));
		result = matrix_mul_point3(y_axis, mtx_ab);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0)), threshold));
		CHECK(vector_all_near_equal3(result, matrix_mul_point3(matrix_mul_point3(y_axis, mtx_a), mtx_b), threshold));
		result = matrix_mul_point3(x_axis, mtx_ba);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0)), threshold));
		CHECK(vector_all_near_equal3(result, matrix_mul_point3(matrix_mul_point3(x_axis, mtx_b), mtx_a), threshold));
		result = matrix_mul_point3(y_axis, mtx_ba);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(0.0), FloatType(0.0), FloatType(-1.0)), threshold));
		CHECK(vector_all_near_equal3(result, matrix_mul_point3(matrix_mul_point3(y_axis, mtx_b), mtx_a), threshold));
	}

	{
		Vector4Type x_axis = vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0));
		Vector4Type y_axis = vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0));

		QuatType rotation_around_z = quat_from_euler(scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(90.0)), scalar_deg_to_rad(FloatType(0.0)));
		Matrix3x4Type mtx_a = matrix_from_qvv(rotation_around_z, x_axis, vector_set(FloatType(1.0)));
		Vector4Type result = matrix_mul_vector3(x_axis, mtx_a);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0)), threshold));
		result = matrix_mul_vector3(y_axis, mtx_a);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(-1.0), FloatType(0.0), FloatType(0.0)), threshold));

		QuatType rotation_around_x = quat_from_euler(scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(90.0)));
		Matrix3x4Type mtx_b = matrix_from_qvv(rotation_around_x, y_axis, vector_set(FloatType(1.0)));
		result = matrix_mul_vector3(x_axis, mtx_b);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0)), threshold));
		result = matrix_mul_vector3(y_axis, mtx_b);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(0.0), FloatType(0.0), FloatType(-1.0)), threshold));

		Matrix3x4Type mtx_ab = matrix_mul(mtx_a, mtx_b);
		Matrix3x4Type mtx_ba = matrix_mul(mtx_b, mtx_a);
		result = matrix_mul_vector3(x_axis, mtx_ab);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(0.0), FloatType(0.0), FloatType(-1.0)), threshold));
		CHECK(vector_all_near_equal3(result, matrix_mul_vector3(matrix_mul_vector3(x_axis, mtx_a), mtx_b), threshold));
		result = matrix_mul_vector3(y_axis, mtx_ab);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(-1.0), FloatType(0.0), FloatType(0.0)), threshold));
		CHECK(vector_all_near_equal3(result, matrix_mul_vector3(matrix_mul_vector3(y_axis, mtx_a), mtx_b), threshold));
		result = matrix_mul_vector3(x_axis, mtx_ba);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0)), threshold));
		CHECK(vector_all_near_equal3(result, matrix_mul_vector3(matrix_mul_vector3(x_axis, mtx_b), mtx_a), threshold));
		result = matrix_mul_vector3(y_axis, mtx_ba);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(0.0), FloatType(0.0), FloatType(-1.0)), threshold));
		CHECK(vector_all_near_equal3(result, matrix_mul_vector3(matrix_mul_vector3(y_axis, mtx_b), mtx_a), threshold));
	}
}

template<typename FloatType>
static void test_affine_matrix_transformations(const FloatType threshold)
{
	using QuatType = typename float_traits<FloatType>::quat;
	using Vector4Type = typename float_traits<FloatType>::vector4;
	using Matrix3x3Type = typename float_traits<FloatType>::matrix3x3;
	using Matrix3x4Type = typename float_traits<FloatType>::matrix3x4;

	const Matrix3x4Type identity = matrix_identity();
	const Vector4Type zero = vector_zero();

	{
		Vector4Type x_axis = vector_set(FloatType(1.0), FloatType(2.0), FloatType(3.0), FloatType(0.0));
		Vector4Type y_axis = vector_set(FloatType(4.0), FloatType(5.0), FloatType(6.0), FloatType(0.0));
		Vector4Type z_axis = vector_set(FloatType(7.0), FloatType(8.0), FloatType(9.0), FloatType(0.0));
		Vector4Type w_axis = vector_set(FloatType(10.0), FloatType(11.0), FloatType(12.0), FloatType(1.0));
		Matrix3x4Type mtx0 = matrix_set(x_axis, y_axis, z_axis, w_axis);
		Matrix3x3Type mtx1 = matrix_transpose(mtx0);
		CHECK(vector_all_near_equal3(vector_set(FloatType(1.0), FloatType(4.0), FloatType(7.0), FloatType(10.0)), mtx1.x_axis, threshold));
		CHECK(vector_all_near_equal3(vector_set(FloatType(2.0), FloatType(5.0), FloatType(8.0), FloatType(11.0)), mtx1.y_axis, threshold));
		CHECK(vector_all_near_equal3(vector_set(FloatType(3.0), FloatType(6.0), FloatType(9.0), FloatType(12.0)), mtx1.z_axis, threshold));
	}

	{
		QuatType rotation_around_z = quat_from_euler(scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(90.0)), scalar_deg_to_rad(FloatType(0.0)));
		Vector4Type translation = vector_set(FloatType(1.0), FloatType(2.0), FloatType(3.0));
		Vector4Type scale = vector_set(FloatType(4.0), FloatType(5.0), FloatType(6.0));
		Matrix3x4Type mtx = matrix_from_qvv(rotation_around_z, translation, scale);
		Matrix3x4Type inv_mtx = matrix_inverse(mtx);
		Matrix3x4Type result = matrix_mul(mtx, inv_mtx);
		CHECK(vector_all_near_equal3(vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0), FloatType(0.0)), result.x_axis, threshold));
		CHECK(vector_all_near_equal3(vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0), FloatType(0.0)), result.y_axis, threshold));
		CHECK(vector_all_near_equal3(vector_set(FloatType(0.0), FloatType(0.0), FloatType(1.0), FloatType(0.0)), result.z_axis, threshold));
		CHECK(vector_all_near_equal3(vector_set(FloatType(0.0), FloatType(0.0), FloatType(0.0), FloatType(1.0)), result.w_axis, threshold));
	}

	{
		QuatType rotation_around_z = quat_from_euler(scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(90.0)), scalar_deg_to_rad(FloatType(0.0)));
		Vector4Type translation = vector_set(FloatType(1.0), FloatType(2.0), FloatType(3.0));
		Vector4Type scale = vector_set(FloatType(4.0), FloatType(5.0), FloatType(6.0));
		Matrix3x4Type mtx = matrix_from_qvv(rotation_around_z, translation, scale);
		Matrix3x4Type inv_mtx = matrix_inverse(mtx, mtx);
		Matrix3x4Type result = matrix_mul(mtx, inv_mtx);
		CHECK(vector_all_near_equal3(vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0), FloatType(0.0)), result.x_axis, threshold));
		CHECK(vector_all_near_equal3(vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0), FloatType(0.0)), result.y_axis, threshold));
		CHECK(vector_all_near_equal3(vector_set(FloatType(0.0), FloatType(0.0), FloatType(1.0), FloatType(0.0)), result.z_axis, threshold));
		CHECK(vector_all_near_equal3(vector_set(FloatType(0.0), FloatType(0.0), FloatType(0.0), FloatType(1.0)), result.w_axis, threshold));
	}

	{
		Matrix3x4Type mtx = matrix_set(zero, zero, zero, zero);
		Matrix3x4Type inv_mtx = matrix_inverse(mtx, identity);
		CHECK(vector_all_near_equal(identity.x_axis, inv_mtx.x_axis, threshold));
		CHECK(vector_all_near_equal(identity.y_axis, inv_mtx.y_axis, threshold));
		CHECK(vector_all_near_equal(identity.z_axis, inv_mtx.z_axis, threshold));
		CHECK(vector_all_near_equal(identity.w_axis, inv_mtx.w_axis, threshold));
	}

	{
		QuatType rotation = quat_from_euler(scalar_deg_to_rad(FloatType(12.3)), scalar_deg_to_rad(FloatType(42.8)), scalar_deg_to_rad(FloatType(33.41)));
		Vector4Type translation = vector_set(FloatType(1.0), FloatType(2.0), FloatType(3.0));
		Vector4Type scale = vector_set(FloatType(4.0), FloatType(5.0), FloatType(6.0));
		Matrix3x4Type mtx = matrix_from_qvv(rotation, translation, scale);
		Matrix3x4Type inv_mtx = matrix_inverse(mtx);
		Matrix3x4Type result = matrix_mul(mtx, inv_mtx);
		CHECK(vector_all_near_equal3(vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0), FloatType(0.0)), result.x_axis, threshold));
		CHECK(vector_all_near_equal3(vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0), FloatType(0.0)), result.y_axis, threshold));
		CHECK(vector_all_near_equal3(vector_set(FloatType(0.0), FloatType(0.0), FloatType(1.0), FloatType(0.0)), result.z_axis, threshold));
		CHECK(vector_all_near_equal3(vector_set(FloatType(0.0), FloatType(0.0), FloatType(0.0), FloatType(1.0)), result.w_axis, threshold));
	}
}

template<typename FloatType>
static void test_affine_matrix_misc(const FloatType threshold)
{
	using QuatType = typename float_traits<FloatType>::quat;
	using Vector4Type = typename float_traits<FloatType>::vector4;
	using Matrix3x3Type = typename float_traits<FloatType>::matrix3x3;
	using Matrix3x4Type = typename float_traits<FloatType>::matrix3x4;

	{
		QuatType rotation_around_z = quat_from_euler(scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(90.0)), scalar_deg_to_rad(FloatType(0.0)));
		Matrix3x4Type mtx = matrix_from_quat(rotation_around_z);
		QuatType rotation = quat_from_matrix(mtx);
		CHECK(quat_near_equal(rotation_around_z, rotation, threshold));
	}

	{
		QuatType rotation_around_z = quat_from_euler(scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(90.0)), scalar_deg_to_rad(FloatType(0.0)));
		Vector4Type translation = vector_set(FloatType(1.0), FloatType(2.0), FloatType(3.0));
		Vector4Type scale = vector_set(FloatType(4.0), FloatType(5.0), FloatType(6.0));
		Matrix3x4Type mtx0 = matrix_from_qvv(rotation_around_z, translation, scale);
		Matrix3x4Type mtx0_no_scale = matrix_remove_scale(mtx0);
		CHECK(vector_all_near_equal3(vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0), FloatType(0.0)), mtx0_no_scale.x_axis, threshold));
		CHECK(vector_all_near_equal3(vector_set(FloatType(-1.0), FloatType(0.0), FloatType(0.0), FloatType(0.0)), mtx0_no_scale.y_axis, threshold));
		CHECK(vector_all_near_equal3(vector_set(FloatType(0.0), FloatType(0.0), FloatType(1.0), FloatType(0.0)), mtx0_no_scale.z_axis, threshold));
		CHECK(vector_all_near_equal3(vector_set(FloatType(1.0), FloatType(2.0), FloatType(3.0), FloatType(1.0)), mtx0_no_scale.w_axis, threshold));
	}

	{
		Vector4Type x_axis = vector_set(FloatType(5.62565), FloatType(7.90751), FloatType(4.37048), FloatType(99999.9999));
		Vector4Type y_axis = vector_set(FloatType(0.36345), FloatType(7.87300), FloatType(7.23000), FloatType(99999.9999));
		Vector4Type z_axis = vector_set(FloatType(8.06413), FloatType(3.91970), FloatType(8.48928), FloatType(99999.9999));
		Matrix3x4Type mtx = matrix_set(x_axis, y_axis, z_axis, x_axis);
		FloatType det = scalar_cast(matrix_determinant(mtx));
		CHECK(scalar_near_equal(det, FloatType(381.95681179092484), threshold));
	}

	{
		Vector4Type x_axis = vector_set(FloatType(5.62565), FloatType(7.90751), FloatType(4.37048), FloatType(99999.9999));
		Vector4Type y_axis = vector_set(FloatType(0.36345), FloatType(7.87300), FloatType(7.23000), FloatType(99999.9999));
		Vector4Type z_axis = vector_set(FloatType(8.06413), FloatType(3.91970), FloatType(8.48928), FloatType(99999.9999));
		Matrix3x4Type mtx = matrix_set(x_axis, y_axis, z_axis, x_axis);

		FloatType det_xx = scalar_cast(matrix_minor(mtx, axis3::x, axis3::x));
		CHECK(scalar_near_equal(det_xx, FloatType(38.4967), threshold));
		FloatType det_xy = scalar_cast(matrix_minor(mtx, axis3::x, axis3::y));
		CHECK(scalar_near_equal(det_xy, FloatType(-55.2182), threshold));
		FloatType det_xz = scalar_cast(matrix_minor(mtx, axis3::x, axis3::z));
		CHECK(scalar_near_equal(det_xz, FloatType(-62.0643), threshold));

		FloatType det_yx = scalar_cast(matrix_minor(mtx, axis3::y, axis3::x));
		CHECK(scalar_near_equal(det_yx, FloatType(49.9981), threshold));
		FloatType det_yy = scalar_cast(matrix_minor(mtx, axis3::y, axis3::y));
		CHECK(scalar_near_equal(det_yy, FloatType(12.5136), threshold));
		FloatType det_yz = scalar_cast(matrix_minor(mtx, axis3::y, axis3::z));
		CHECK(scalar_near_equal(det_yz, FloatType(-41.7163), threshold));

		FloatType det_zx = scalar_cast(matrix_minor(mtx, axis3::z, axis3::x));
		CHECK(scalar_near_equal(det_zx, FloatType(22.7625), threshold));
		FloatType det_zy = scalar_cast(matrix_minor(mtx, axis3::z, axis3::y));
		CHECK(scalar_near_equal(det_zy, FloatType(39.085), threshold));
		FloatType det_zz = scalar_cast(matrix_minor(mtx, axis3::z, axis3::z));
		CHECK(scalar_near_equal(det_zz, FloatType(41.4168), threshold));
	}

	{
		Vector4Type x_axis = vector_set(FloatType(5.62565), FloatType(7.90751), FloatType(4.37048), FloatType(99999.9999));
		Vector4Type y_axis = vector_set(FloatType(0.36345), FloatType(7.87300), FloatType(7.23000), FloatType(99999.9999));
		Vector4Type z_axis = vector_set(FloatType(8.06413), FloatType(3.91970), FloatType(8.48928), FloatType(99999.9999));
		Matrix3x4Type mtx = matrix_set(x_axis, y_axis, z_axis, x_axis);
		Matrix3x3Type cof = matrix_cofactor(mtx);

		FloatType cof_xx = scalar_cast(vector_get_x(cof.x_axis));
		CHECK(scalar_near_equal(cof_xx, FloatType(38.4967), threshold));
		FloatType cof_xy = scalar_cast(vector_get_y(cof.x_axis));
		CHECK(scalar_near_equal(cof_xy, FloatType(55.2182), threshold));
		FloatType cof_xz = scalar_cast(vector_get_z(cof.x_axis));
		CHECK(scalar_near_equal(cof_xz, FloatType(-62.0643), threshold));

		FloatType cof_yx = scalar_cast(vector_get_x(cof.y_axis));
		CHECK(scalar_near_equal(cof_yx, FloatType(-49.9981), threshold));
		FloatType cof_yy = scalar_cast(vector_get_y(cof.y_axis));
		CHECK(scalar_near_equal(cof_yy, FloatType(12.5136), threshold));
		FloatType cof_yz = scalar_cast(vector_get_z(cof.y_axis));
		CHECK(scalar_near_equal(cof_yz, FloatType(41.7163), threshold));

		FloatType cof_zx = scalar_cast(vector_get_x(cof.z_axis));
		CHECK(scalar_near_equal(cof_zx, FloatType(22.7625), threshold));
		FloatType cof_zy = scalar_cast(vector_get_y(cof.z_axis));
		CHECK(scalar_near_equal(cof_zy, FloatType(-39.085), threshold));
		FloatType cof_zz = scalar_cast(vector_get_z(cof.z_axis));
		CHECK(scalar_near_equal(cof_zz, FloatType(41.4168), threshold));
	}

	{
		Vector4Type x_axis = vector_set(FloatType(5.62565), FloatType(7.90751), FloatType(4.37048), FloatType(99999.9999));
		Vector4Type y_axis = vector_set(FloatType(0.36345), FloatType(7.87300), FloatType(7.23000), FloatType(99999.9999));
		Vector4Type z_axis = vector_set(FloatType(8.06413), FloatType(3.91970), FloatType(8.48928), FloatType(99999.9999));
		Matrix3x4Type mtx = matrix_set(x_axis, y_axis, z_axis, x_axis);
		Matrix3x3Type adj = matrix_adjugate(mtx);

		FloatType adj_xx = scalar_cast(vector_get_x(adj.x_axis));
		CHECK(scalar_near_equal(adj_xx, FloatType(38.4967), threshold));
		FloatType adj_xy = scalar_cast(vector_get_y(adj.x_axis));
		CHECK(scalar_near_equal(adj_xy, FloatType(-49.9981), threshold));
		FloatType adj_xz = scalar_cast(vector_get_z(adj.x_axis));
		CHECK(scalar_near_equal(adj_xz, FloatType(22.7625), threshold));

		FloatType adj_yx = scalar_cast(vector_get_x(adj.y_axis));
		CHECK(scalar_near_equal(adj_yx, FloatType(55.2182), threshold));
		FloatType adj_yy = scalar_cast(vector_get_y(adj.y_axis));
		CHECK(scalar_near_equal(adj_yy, FloatType(12.5136), threshold));
		FloatType adj_yz = scalar_cast(vector_get_z(adj.y_axis));
		CHECK(scalar_near_equal(adj_yz, FloatType(-39.085), threshold));

		FloatType adj_zx = scalar_cast(vector_get_x(adj.z_axis));
		CHECK(scalar_near_equal(adj_zx, FloatType(-62.0643), threshold));
		FloatType adj_zy = scalar_cast(vector_get_y(adj.z_axis));
		CHECK(scalar_near_equal(adj_zy, FloatType(41.7163), threshold));
		FloatType adj_zz = scalar_cast(vector_get_z(adj.z_axis));
		CHECK(scalar_near_equal(adj_zz, FloatType(41.4168), threshold));
	}
}

TEST_CASE("matrix3x4f math", "[math][matrix3x4]")
{
	test_affine_matrix_setters<float>(1.0E-4F);
	test_affine_matrix_arithmetic<float>(1.0E-4F);
	test_affine_matrix_transformations<float>(1.0E-4F);
	test_affine_matrix_misc<float>(1.0E-4F);

	{
		quatf rotation_around_z = quat_from_euler(scalar_deg_to_rad(0.0F), scalar_deg_to_rad(90.0F), scalar_deg_to_rad(0.0F));
		vector4f translation = vector_set(1.0F, 2.0F, 3.0F);
		vector4f scale = vector_set(4.0F, 5.0F, 6.0F);
		matrix3x4f src = matrix_from_qvv(rotation_around_z, translation, scale);
		matrix3x4d dst = matrix_cast(src);
		CHECK(vector_all_near_equal3(vector_cast(src.x_axis), dst.x_axis, 1.0E-4));
		CHECK(vector_all_near_equal3(vector_cast(src.y_axis), dst.y_axis, 1.0E-4));
		CHECK(vector_all_near_equal3(vector_cast(src.z_axis), dst.z_axis, 1.0E-4));
		CHECK(vector_all_near_equal3(vector_cast(src.w_axis), dst.w_axis, 1.0E-4));
	}
}

TEST_CASE("matrix3x4d math", "[math][matrix3x4]")
{
	test_affine_matrix_setters<double>(1.0E-4);
	test_affine_matrix_arithmetic<double>(1.0E-4F);
	test_affine_matrix_transformations<double>(1.0E-4F);
	test_affine_matrix_misc<double>(1.0E-4F);

	{
		quatd rotation_around_z = quat_from_euler(scalar_deg_to_rad(0.0), scalar_deg_to_rad(90.0), scalar_deg_to_rad(0.0));
		vector4d translation = vector_set(1.0, 2.0, 3.0);
		vector4d scale = vector_set(4.0, 5.0, 6.0);
		matrix3x4d src = matrix_from_qvv(rotation_around_z, translation, scale);
		matrix3x4f dst = matrix_cast(src);
		CHECK(vector_all_near_equal3(vector_cast(src.x_axis), dst.x_axis, 1.0E-4F));
		CHECK(vector_all_near_equal3(vector_cast(src.y_axis), dst.y_axis, 1.0E-4F));
		CHECK(vector_all_near_equal3(vector_cast(src.z_axis), dst.z_axis, 1.0E-4F));
		CHECK(vector_all_near_equal3(vector_cast(src.w_axis), dst.w_axis, 1.0E-4F));
	}
}
