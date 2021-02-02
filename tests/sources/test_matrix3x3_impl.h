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
#include <rtm/matrix3x3f.h>
#include <rtm/matrix3x3d.h>
#include <rtm/matrix3x4f.h>
#include <rtm/matrix3x4d.h>

using namespace rtm;

template<typename FloatType>
static void test_matrix3x3_setters(const FloatType threshold)
{
	using QuatType = typename float_traits<FloatType>::quat;
	using Vector4Type = typename float_traits<FloatType>::vector4;
	using Matrix3x3Type = typename float_traits<FloatType>::matrix3x3;

	const Matrix3x3Type identity = matrix_identity();

	{
		Vector4Type x_axis = vector_set(FloatType(1.0), FloatType(2.0), FloatType(3.0), FloatType(0.0));
		Vector4Type y_axis = vector_set(FloatType(4.0), FloatType(5.0), FloatType(6.0), FloatType(0.0));
		Vector4Type z_axis = vector_set(FloatType(7.0), FloatType(8.0), FloatType(9.0), FloatType(0.0));
		Matrix3x3Type mtx = matrix_set(x_axis, y_axis, z_axis);
		CHECK(vector_all_near_equal3(x_axis, mtx.x_axis, threshold));
		CHECK(vector_all_near_equal3(y_axis, mtx.y_axis, threshold));
		CHECK(vector_all_near_equal3(z_axis, mtx.z_axis, threshold));
	}

	{
		CHECK(vector_all_near_equal3(vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0)), identity.x_axis, threshold));
		CHECK(vector_all_near_equal3(vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0)), identity.y_axis, threshold));
		CHECK(vector_all_near_equal3(vector_set(FloatType(0.0), FloatType(0.0), FloatType(1.0)), identity.z_axis, threshold));
	}

	{
		QuatType rotation_around_z = quat_from_euler(scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(90.0)), scalar_deg_to_rad(FloatType(0.0)));
		Matrix3x3Type mtx = matrix_from_quat(rotation_around_z);
		CHECK(vector_all_near_equal3(vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0)), mtx.x_axis, threshold));
		CHECK(vector_all_near_equal3(vector_set(FloatType(-1.0), FloatType(0.0), FloatType(0.0)), mtx.y_axis, threshold));
		CHECK(vector_all_near_equal3(vector_set(FloatType(0.0), FloatType(0.0), FloatType(1.0)), mtx.z_axis, threshold));
	}

	{
		QuatType rotation_around_z = quat_from_euler(scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(90.0)), scalar_deg_to_rad(FloatType(0.0)));
		Matrix3x3Type mtx = matrix_from_rotation(rotation_around_z);
		CHECK(vector_all_near_equal3(vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0)), mtx.x_axis, threshold));
		CHECK(vector_all_near_equal3(vector_set(FloatType(-1.0), FloatType(0.0), FloatType(0.0)), mtx.y_axis, threshold));
		CHECK(vector_all_near_equal3(vector_set(FloatType(0.0), FloatType(0.0), FloatType(1.0)), mtx.z_axis, threshold));
	}

	{
		Matrix3x3Type mtx = matrix_from_scale(vector_set(FloatType(4.0), FloatType(5.0), FloatType(6.0)));
		CHECK(vector_all_near_equal3(vector_set(FloatType(4.0), FloatType(0.0), FloatType(0.0)), mtx.x_axis, threshold));
		CHECK(vector_all_near_equal3(vector_set(FloatType(0.0), FloatType(5.0), FloatType(0.0)), mtx.y_axis, threshold));
		CHECK(vector_all_near_equal3(vector_set(FloatType(0.0), FloatType(0.0), FloatType(6.0)), mtx.z_axis, threshold));
	}
}

template<typename FloatType>
static void test_matrix3x3_arithmetic(const FloatType threshold)
{
	using QuatType = typename float_traits<FloatType>::quat;
	using Vector4Type = typename float_traits<FloatType>::vector4;
	using Matrix3x3Type = typename float_traits<FloatType>::matrix3x3;

	{
		Vector4Type x_axis = vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0));
		Vector4Type y_axis = vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0));

		QuatType rotation_around_z = quat_from_euler(scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(90.0)), scalar_deg_to_rad(FloatType(0.0)));
		Matrix3x3Type mtx_a = matrix_from_quat(rotation_around_z);
		Vector4Type result = matrix_mul_vector3(x_axis, mtx_a);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0)), threshold));
		result = matrix_mul_vector3(y_axis, mtx_a);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(-1.0), FloatType(0.0), FloatType(0.0)), threshold));

		QuatType rotation_around_x = quat_from_euler(scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(90.0)));
		Matrix3x3Type mtx_b = matrix_from_quat(rotation_around_x);
		result = matrix_mul_vector3(x_axis, mtx_b);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0)), threshold));
		result = matrix_mul_vector3(y_axis, mtx_b);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(0.0), FloatType(0.0), FloatType(-1.0)), threshold));

		Matrix3x3Type mtx_ab = matrix_mul(mtx_a, mtx_b);
		Matrix3x3Type mtx_ba = matrix_mul(mtx_b, mtx_a);
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
static void test_matrix3x3_transformations(const FloatType threshold)
{
	using QuatType = typename float_traits<FloatType>::quat;
	using Vector4Type = typename float_traits<FloatType>::vector4;
	using Matrix3x3Type = typename float_traits<FloatType>::matrix3x3;

	const Matrix3x3Type identity = matrix_identity();
	const Vector4Type zero = vector_zero();

	{
		Vector4Type x_axis = vector_set(FloatType(1.0), FloatType(2.0), FloatType(3.0), FloatType(0.0));
		Vector4Type y_axis = vector_set(FloatType(4.0), FloatType(5.0), FloatType(6.0), FloatType(0.0));
		Vector4Type z_axis = vector_set(FloatType(7.0), FloatType(8.0), FloatType(9.0), FloatType(0.0));
		Matrix3x3Type mtx0 = matrix_set(x_axis, y_axis, z_axis);
		Matrix3x3Type mtx1 = matrix_transpose(mtx0);
		CHECK(vector_all_near_equal3(vector_set(FloatType(1.0), FloatType(4.0), FloatType(7.0)), mtx1.x_axis, threshold));
		CHECK(vector_all_near_equal3(vector_set(FloatType(2.0), FloatType(5.0), FloatType(8.0)), mtx1.y_axis, threshold));
		CHECK(vector_all_near_equal3(vector_set(FloatType(3.0), FloatType(6.0), FloatType(9.0)), mtx1.z_axis, threshold));
	}

	{
		QuatType rotation_around_z = quat_from_euler(scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(90.0)), scalar_deg_to_rad(FloatType(0.0)));
		Matrix3x3Type mtx = matrix_from_quat(rotation_around_z);
		Matrix3x3Type inv_mtx = matrix_inverse(mtx);
		Matrix3x3Type result = matrix_mul(mtx, inv_mtx);
		CHECK(vector_all_near_equal3(vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0)), result.x_axis, threshold));
		CHECK(vector_all_near_equal3(vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0)), result.y_axis, threshold));
		CHECK(vector_all_near_equal3(vector_set(FloatType(0.0), FloatType(0.0), FloatType(1.0)), result.z_axis, threshold));
	}

	{
		QuatType rotation = quat_from_euler(scalar_deg_to_rad(FloatType(12.3)), scalar_deg_to_rad(FloatType(42.8)), scalar_deg_to_rad(FloatType(33.41)));
		Matrix3x3Type mtx = matrix_from_quat(rotation);
		Matrix3x3Type inv_mtx = matrix_inverse(mtx);
		Matrix3x3Type result = matrix_mul(mtx, inv_mtx);
		CHECK(vector_all_near_equal3(vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0)), result.x_axis, threshold));
		CHECK(vector_all_near_equal3(vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0)), result.y_axis, threshold));
		CHECK(vector_all_near_equal3(vector_set(FloatType(0.0), FloatType(0.0), FloatType(1.0)), result.z_axis, threshold));
	}

	{
		QuatType rotation = quat_from_euler(scalar_deg_to_rad(FloatType(12.3)), scalar_deg_to_rad(FloatType(42.8)), scalar_deg_to_rad(FloatType(33.41)));
		Matrix3x3Type mtx = matrix_from_quat(rotation);
		Matrix3x3Type inv_mtx = matrix_inverse(mtx, mtx);
		Matrix3x3Type result = matrix_mul(mtx, inv_mtx);
		CHECK(vector_all_near_equal3(vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0)), result.x_axis, threshold));
		CHECK(vector_all_near_equal3(vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0)), result.y_axis, threshold));
		CHECK(vector_all_near_equal3(vector_set(FloatType(0.0), FloatType(0.0), FloatType(1.0)), result.z_axis, threshold));
	}

	{
		Matrix3x3Type mtx = matrix_set(zero, zero, zero);
		Matrix3x3Type inv_mtx = matrix_inverse(mtx, identity);
		CHECK(vector_all_near_equal(identity.x_axis, inv_mtx.x_axis, threshold));
		CHECK(vector_all_near_equal(identity.y_axis, inv_mtx.y_axis, threshold));
		CHECK(vector_all_near_equal(identity.z_axis, inv_mtx.z_axis, threshold));
	}
}

template<typename FloatType>
static void test_matrix3x3_misc(const FloatType threshold)
{
	using QuatType = typename float_traits<FloatType>::quat;
	using Vector4Type = typename float_traits<FloatType>::vector4;
	using Matrix3x3Type = typename float_traits<FloatType>::matrix3x3;
	using Matrix3x4Type = typename float_traits<FloatType>::matrix3x4;

	{
		QuatType rotation_around_z = quat_from_euler(scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(90.0)), scalar_deg_to_rad(FloatType(0.0)));
		Matrix3x3Type mtx = matrix_from_quat(rotation_around_z);
		QuatType rotation = quat_from_matrix(mtx);
		CHECK(quat_near_equal(rotation_around_z, rotation, threshold));
	}

	{
		QuatType rotation_around_z = quat_from_euler(scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(90.0)), scalar_deg_to_rad(FloatType(0.0)));
		Vector4Type translation = vector_set(FloatType(1.0), FloatType(2.0), FloatType(3.0));
		Vector4Type scale = vector_set(FloatType(4.0), FloatType(5.0), FloatType(6.0));
		Matrix3x4Type mtx0_3x4 = matrix_from_qvv(rotation_around_z, translation, scale);
		Matrix3x3Type mtx0 = matrix_cast(mtx0_3x4);
		Matrix3x3Type mtx0_no_scale = matrix_remove_scale(mtx0);
		CHECK(vector_all_near_equal3(vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0)), mtx0_no_scale.x_axis, threshold));
		CHECK(vector_all_near_equal3(vector_set(FloatType(-1.0), FloatType(0.0), FloatType(0.0)), mtx0_no_scale.y_axis, threshold));
		CHECK(vector_all_near_equal3(vector_set(FloatType(0.0), FloatType(0.0), FloatType(1.0)), mtx0_no_scale.z_axis, threshold));
	}

	{
		Vector4Type x_axis = vector_set(FloatType(5.62565), FloatType(7.90751), FloatType(4.37048), FloatType(99999.9999));
		Vector4Type y_axis = vector_set(FloatType(0.36345), FloatType(7.87300), FloatType(7.23000), FloatType(99999.9999));
		Vector4Type z_axis = vector_set(FloatType(8.06413), FloatType(3.91970), FloatType(8.48928), FloatType(99999.9999));
		Matrix3x3Type mtx = matrix_set(x_axis, y_axis, z_axis);
		FloatType det = scalar_cast(matrix_determinant(mtx));
		CHECK(scalar_near_equal(det, FloatType(381.95681179092484), threshold));
	}

	{
		Vector4Type x_axis = vector_set(FloatType(5.62565), FloatType(7.90751), FloatType(4.37048), FloatType(99999.9999));
		Vector4Type y_axis = vector_set(FloatType(0.36345), FloatType(7.87300), FloatType(7.23000), FloatType(99999.9999));
		Vector4Type z_axis = vector_set(FloatType(8.06413), FloatType(3.91970), FloatType(8.48928), FloatType(99999.9999));
		Matrix3x3Type mtx = matrix_set(x_axis, y_axis, z_axis);

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
