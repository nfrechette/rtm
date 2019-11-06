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

#include <catch.hpp>

#include <rtm/type_traits.h>
#include <rtm/matrix3x4f.h>
#include <rtm/matrix3x4d.h>
#include <rtm/matrix4x4f.h>
#include <rtm/matrix4x4d.h>

using namespace rtm;

template<typename FloatType>
static void test_matrix_impl(const FloatType threshold)
{
	using QuatType = typename float_traits<FloatType>::quat;
	using Vector4Type = typename float_traits<FloatType>::vector4;
	using Matrix3x4Type = typename float_traits<FloatType>::matrix3x4;
	using Matrix4x4Type = typename float_traits<FloatType>::matrix4x4;

	const Matrix4x4Type identity = matrix_identity();

	{
		Vector4Type x_axis = vector_set(FloatType(1.0), FloatType(2.0), FloatType(3.0), FloatType(0.0));
		Vector4Type y_axis = vector_set(FloatType(4.0), FloatType(5.0), FloatType(6.0), FloatType(0.0));
		Vector4Type z_axis = vector_set(FloatType(7.0), FloatType(8.0), FloatType(9.0), FloatType(0.0));
		Vector4Type w_axis = vector_set(FloatType(10.0), FloatType(11.0), FloatType(12.0), FloatType(1.0));
		Matrix4x4Type mtx = matrix_set(x_axis, y_axis, z_axis, w_axis);
		REQUIRE(vector_all_near_equal(x_axis, mtx.x_axis, threshold));
		REQUIRE(vector_all_near_equal(y_axis, mtx.y_axis, threshold));
		REQUIRE(vector_all_near_equal(z_axis, mtx.z_axis, threshold));
		REQUIRE(vector_all_near_equal(w_axis, mtx.w_axis, threshold));
	}

	{
		REQUIRE(vector_all_near_equal(vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0), FloatType(0.0)), identity.x_axis, threshold));
		REQUIRE(vector_all_near_equal(vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0), FloatType(0.0)), identity.y_axis, threshold));
		REQUIRE(vector_all_near_equal(vector_set(FloatType(0.0), FloatType(0.0), FloatType(1.0), FloatType(0.0)), identity.z_axis, threshold));
		REQUIRE(vector_all_near_equal(vector_set(FloatType(0.0), FloatType(0.0), FloatType(0.0), FloatType(1.0)), identity.w_axis, threshold));
	}

	{
		Vector4Type x_axis = vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0));
		Vector4Type y_axis = vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0));

		QuatType rotation_around_z = quat_from_euler(degrees(FloatType(0.0)), degrees(FloatType(90.0)), degrees(FloatType(0.0)));
		Matrix3x4Type mtx_a3x4 = matrix_from_qvv(rotation_around_z, x_axis, vector_set(FloatType(1.0)));
		Matrix4x4Type mtx_a = matrix_cast(mtx_a3x4);
		Vector4Type result = matrix_mul_vector(x_axis, mtx_a);
		REQUIRE(vector_all_near_equal3(result, vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0), FloatType(0.0)), threshold));
		result = matrix_mul_vector(y_axis, mtx_a);
		REQUIRE(vector_all_near_equal3(result, vector_set(FloatType(-1.0), FloatType(0.0), FloatType(0.0), FloatType(0.0)), threshold));

		QuatType rotation_around_x = quat_from_euler(degrees(FloatType(0.0)), degrees(FloatType(0.0)), degrees(FloatType(90.0)));
		Matrix3x4Type mtx_b3x4 = matrix_from_qvv(rotation_around_x, y_axis, vector_set(FloatType(1.0)));
		Matrix4x4Type mtx_b = matrix_cast(mtx_b3x4);
		result = matrix_mul_vector(x_axis, mtx_b);
		REQUIRE(vector_all_near_equal3(result, vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0), FloatType(0.0)), threshold));
		result = matrix_mul_vector(y_axis, mtx_b);
		REQUIRE(vector_all_near_equal3(result, vector_set(FloatType(0.0), FloatType(0.0), FloatType(-1.0), FloatType(0.0)), threshold));

		Matrix4x4Type mtx_ab = matrix_mul(mtx_a, mtx_b);
		Matrix4x4Type mtx_ba = matrix_mul(mtx_b, mtx_a);
		result = matrix_mul_vector(x_axis, mtx_ab);
		REQUIRE(vector_all_near_equal3(result, vector_set(FloatType(0.0), FloatType(0.0), FloatType(-1.0), FloatType(0.0)), threshold));
		REQUIRE(vector_all_near_equal3(result, matrix_mul_vector(matrix_mul_vector(x_axis, mtx_a), mtx_b), threshold));
		result = matrix_mul_vector(y_axis, mtx_ab);
		REQUIRE(vector_all_near_equal3(result, vector_set(FloatType(-1.0), FloatType(0.0), FloatType(0.0), FloatType(0.0)), threshold));
		REQUIRE(vector_all_near_equal3(result, matrix_mul_vector(matrix_mul_vector(y_axis, mtx_a), mtx_b), threshold));
		result = matrix_mul_vector(x_axis, mtx_ba);
		REQUIRE(vector_all_near_equal3(result, vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0), FloatType(0.0)), threshold));
		REQUIRE(vector_all_near_equal3(result, matrix_mul_vector(matrix_mul_vector(x_axis, mtx_b), mtx_a), threshold));
		result = matrix_mul_vector(y_axis, mtx_ba);
		REQUIRE(vector_all_near_equal3(result, vector_set(FloatType(0.0), FloatType(0.0), FloatType(-1.0), FloatType(0.0)), threshold));
		REQUIRE(vector_all_near_equal3(result, matrix_mul_vector(matrix_mul_vector(y_axis, mtx_b), mtx_a), threshold));
	}

	{
		Vector4Type x_axis = vector_set(FloatType(1.0), FloatType(2.0), FloatType(3.0), FloatType(0.0));
		Vector4Type y_axis = vector_set(FloatType(4.0), FloatType(5.0), FloatType(6.0), FloatType(0.0));
		Vector4Type z_axis = vector_set(FloatType(7.0), FloatType(8.0), FloatType(9.0), FloatType(0.0));
		Vector4Type w_axis = vector_set(FloatType(10.0), FloatType(11.0), FloatType(12.0), FloatType(1.0));
		Matrix4x4Type mtx0 = matrix_set(x_axis, y_axis, z_axis, w_axis);
		Matrix4x4Type mtx1 = matrix_transpose(mtx0);
		REQUIRE(vector_all_near_equal(vector_set(FloatType(1.0), FloatType(4.0), FloatType(7.0), FloatType(10.0)), mtx1.x_axis, threshold));
		REQUIRE(vector_all_near_equal(vector_set(FloatType(2.0), FloatType(5.0), FloatType(8.0), FloatType(11.0)), mtx1.y_axis, threshold));
		REQUIRE(vector_all_near_equal(vector_set(FloatType(3.0), FloatType(6.0), FloatType(9.0), FloatType(12.0)), mtx1.z_axis, threshold));
		REQUIRE(vector_all_near_equal(vector_set(FloatType(0.0), FloatType(0.0), FloatType(0.0), FloatType(1.0)), mtx1.w_axis, threshold));
	}

	{
		QuatType rotation_around_z = quat_from_euler(degrees(FloatType(0.0)), degrees(FloatType(90.0)), degrees(FloatType(0.0)));
		Vector4Type translation = vector_set(FloatType(1.0), FloatType(2.0), FloatType(3.0));
		Vector4Type scale = vector_set(FloatType(4.0), FloatType(5.0), FloatType(6.0));
		Matrix3x4Type mtx3x4 = matrix_from_qvv(rotation_around_z, translation, scale);
		mtx3x4.x_axis = vector_set(vector_get_x(mtx3x4.x_axis), vector_get_y(mtx3x4.x_axis), vector_get_z(mtx3x4.x_axis), FloatType(0.0));
		mtx3x4.y_axis = vector_set(vector_get_x(mtx3x4.y_axis), vector_get_y(mtx3x4.y_axis), vector_get_z(mtx3x4.y_axis), FloatType(0.0));
		mtx3x4.z_axis = vector_set(vector_get_x(mtx3x4.z_axis), vector_get_y(mtx3x4.z_axis), vector_get_z(mtx3x4.z_axis), FloatType(0.0));
		mtx3x4.w_axis = vector_set(vector_get_x(mtx3x4.w_axis), vector_get_y(mtx3x4.w_axis), vector_get_z(mtx3x4.w_axis), FloatType(1.0));
		Matrix4x4Type mtx = matrix_cast(mtx3x4);
		Matrix4x4Type inv_mtx = matrix_inverse(mtx);
		Matrix4x4Type result = matrix_mul(mtx, inv_mtx);
		REQUIRE(vector_all_near_equal(vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0), FloatType(0.0)), result.x_axis, threshold));
		REQUIRE(vector_all_near_equal(vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0), FloatType(0.0)), result.y_axis, threshold));
		REQUIRE(vector_all_near_equal(vector_set(FloatType(0.0), FloatType(0.0), FloatType(1.0), FloatType(0.0)), result.z_axis, threshold));
		REQUIRE(vector_all_near_equal(vector_set(FloatType(0.0), FloatType(0.0), FloatType(0.0), FloatType(1.0)), result.w_axis, threshold));
	}
}

TEST_CASE("matrix4x4f math", "[math][matrix4x4]")
{
	test_matrix_impl<float>(1.0E-4F);

	{
		quatf rotation_around_z = quat_from_euler(degrees(0.0F), degrees(90.0F), degrees(0.0F));
		vector4f translation = vector_set(1.0F, 2.0F, 3.0F);
		vector4f scale = vector_set(4.0F, 5.0F, 6.0F);
		matrix3x4f src3x4 = matrix_from_qvv(rotation_around_z, translation, scale);
		matrix4x4f src = matrix_cast(src3x4);
		matrix4x4d dst = matrix_cast(src);
		REQUIRE(vector_all_near_equal(vector_cast(src.x_axis), dst.x_axis, 1.0E-4));
		REQUIRE(vector_all_near_equal(vector_cast(src.y_axis), dst.y_axis, 1.0E-4));
		REQUIRE(vector_all_near_equal(vector_cast(src.z_axis), dst.z_axis, 1.0E-4));
		REQUIRE(vector_all_near_equal(vector_cast(src.w_axis), dst.w_axis, 1.0E-4));
	}
}

TEST_CASE("matrix4x4d math", "[math][matrix4x4]")
{
	test_matrix_impl<double>(1.0E-4);

	{
		quatd rotation_around_z = quat_from_euler(degrees(0.0), degrees(90.0), degrees(0.0));
		vector4d translation = vector_set(1.0, 2.0, 3.0);
		vector4d scale = vector_set(4.0, 5.0, 6.0);
		matrix3x4d src3x4 = matrix_from_qvv(rotation_around_z, translation, scale);
		matrix4x4d src = matrix_cast(src3x4);
		matrix4x4f dst = matrix_cast(src);
		REQUIRE(vector_all_near_equal(vector_cast(src.x_axis), dst.x_axis, 1.0E-4F));
		REQUIRE(vector_all_near_equal(vector_cast(src.y_axis), dst.y_axis, 1.0E-4F));
		REQUIRE(vector_all_near_equal(vector_cast(src.z_axis), dst.z_axis, 1.0E-4F));
		REQUIRE(vector_all_near_equal(vector_cast(src.w_axis), dst.w_axis, 1.0E-4F));
	}
}
