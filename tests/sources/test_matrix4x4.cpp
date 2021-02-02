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
#include <rtm/matrix4x4f.h>
#include <rtm/matrix4x4d.h>

using namespace rtm;

template<typename FloatType>
static void test_matrix4x4_setters(const FloatType threshold)
{
	using Vector4Type = typename float_traits<FloatType>::vector4;
	using Matrix4x4Type = typename float_traits<FloatType>::matrix4x4;

	const Matrix4x4Type identity = matrix_identity();

	{
		Vector4Type x_axis = vector_set(FloatType(1.0), FloatType(2.0), FloatType(3.0), FloatType(0.0));
		Vector4Type y_axis = vector_set(FloatType(4.0), FloatType(5.0), FloatType(6.0), FloatType(0.0));
		Vector4Type z_axis = vector_set(FloatType(7.0), FloatType(8.0), FloatType(9.0), FloatType(0.0));
		Vector4Type w_axis = vector_set(FloatType(10.0), FloatType(11.0), FloatType(12.0), FloatType(1.0));
		Matrix4x4Type mtx = matrix_set(x_axis, y_axis, z_axis, w_axis);
		CHECK(vector_all_near_equal(x_axis, mtx.x_axis, threshold));
		CHECK(vector_all_near_equal(y_axis, mtx.y_axis, threshold));
		CHECK(vector_all_near_equal(z_axis, mtx.z_axis, threshold));
		CHECK(vector_all_near_equal(w_axis, mtx.w_axis, threshold));
	}

	{
		CHECK(vector_all_near_equal(vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0), FloatType(0.0)), identity.x_axis, threshold));
		CHECK(vector_all_near_equal(vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0), FloatType(0.0)), identity.y_axis, threshold));
		CHECK(vector_all_near_equal(vector_set(FloatType(0.0), FloatType(0.0), FloatType(1.0), FloatType(0.0)), identity.z_axis, threshold));
		CHECK(vector_all_near_equal(vector_set(FloatType(0.0), FloatType(0.0), FloatType(0.0), FloatType(1.0)), identity.w_axis, threshold));
	}
}

template<typename FloatType>
static void test_matrix4x4_arithmetic(const FloatType threshold)
{
	using QuatType = typename float_traits<FloatType>::quat;
	using Vector4Type = typename float_traits<FloatType>::vector4;
	using Matrix3x4Type = typename float_traits<FloatType>::matrix3x4;
	using Matrix4x4Type = typename float_traits<FloatType>::matrix4x4;

	{
		Vector4Type x_axis = vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0));
		Vector4Type y_axis = vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0));

		QuatType rotation_around_z = quat_from_euler(scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(90.0)), scalar_deg_to_rad(FloatType(0.0)));
		Matrix3x4Type mtx_a3x4 = matrix_from_qvv(rotation_around_z, x_axis, vector_set(FloatType(1.0)));
		Matrix4x4Type mtx_a = matrix_cast(mtx_a3x4);
		Vector4Type result = matrix_mul_vector(x_axis, mtx_a);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0), FloatType(0.0)), threshold));
		result = matrix_mul_vector(y_axis, mtx_a);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(-1.0), FloatType(0.0), FloatType(0.0), FloatType(0.0)), threshold));

		QuatType rotation_around_x = quat_from_euler(scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(90.0)));
		Matrix3x4Type mtx_b3x4 = matrix_from_qvv(rotation_around_x, y_axis, vector_set(FloatType(1.0)));
		Matrix4x4Type mtx_b = matrix_cast(mtx_b3x4);
		result = matrix_mul_vector(x_axis, mtx_b);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0), FloatType(0.0)), threshold));
		result = matrix_mul_vector(y_axis, mtx_b);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(0.0), FloatType(0.0), FloatType(-1.0), FloatType(0.0)), threshold));

		Matrix4x4Type mtx_ab = matrix_mul(mtx_a, mtx_b);
		Matrix4x4Type mtx_ba = matrix_mul(mtx_b, mtx_a);
		result = matrix_mul_vector(x_axis, mtx_ab);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(0.0), FloatType(0.0), FloatType(-1.0), FloatType(0.0)), threshold));
		CHECK(vector_all_near_equal3(result, matrix_mul_vector(matrix_mul_vector(x_axis, mtx_a), mtx_b), threshold));
		result = matrix_mul_vector(y_axis, mtx_ab);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(-1.0), FloatType(0.0), FloatType(0.0), FloatType(0.0)), threshold));
		CHECK(vector_all_near_equal3(result, matrix_mul_vector(matrix_mul_vector(y_axis, mtx_a), mtx_b), threshold));
		result = matrix_mul_vector(x_axis, mtx_ba);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0), FloatType(0.0)), threshold));
		CHECK(vector_all_near_equal3(result, matrix_mul_vector(matrix_mul_vector(x_axis, mtx_b), mtx_a), threshold));
		result = matrix_mul_vector(y_axis, mtx_ba);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(0.0), FloatType(0.0), FloatType(-1.0), FloatType(0.0)), threshold));
		CHECK(vector_all_near_equal3(result, matrix_mul_vector(matrix_mul_vector(y_axis, mtx_b), mtx_a), threshold));
	}
}

template<typename FloatType>
static void test_matrix4x4_transformations(const FloatType threshold)
{
	using QuatType = typename float_traits<FloatType>::quat;
	using Vector4Type = typename float_traits<FloatType>::vector4;
	using Matrix3x4Type = typename float_traits<FloatType>::matrix3x4;
	using Matrix4x4Type = typename float_traits<FloatType>::matrix4x4;

	const Matrix4x4Type identity = matrix_identity();
	const Vector4Type zero = vector_zero();

	{
		Vector4Type x_axis = vector_set(FloatType(1.0), FloatType(2.0), FloatType(3.0), FloatType(0.0));
		Vector4Type y_axis = vector_set(FloatType(4.0), FloatType(5.0), FloatType(6.0), FloatType(0.0));
		Vector4Type z_axis = vector_set(FloatType(7.0), FloatType(8.0), FloatType(9.0), FloatType(0.0));
		Vector4Type w_axis = vector_set(FloatType(10.0), FloatType(11.0), FloatType(12.0), FloatType(1.0));
		Matrix4x4Type mtx0 = matrix_set(x_axis, y_axis, z_axis, w_axis);
		Matrix4x4Type mtx1 = matrix_transpose(mtx0);
		CHECK(vector_all_near_equal(vector_set(FloatType(1.0), FloatType(4.0), FloatType(7.0), FloatType(10.0)), mtx1.x_axis, threshold));
		CHECK(vector_all_near_equal(vector_set(FloatType(2.0), FloatType(5.0), FloatType(8.0), FloatType(11.0)), mtx1.y_axis, threshold));
		CHECK(vector_all_near_equal(vector_set(FloatType(3.0), FloatType(6.0), FloatType(9.0), FloatType(12.0)), mtx1.z_axis, threshold));
		CHECK(vector_all_near_equal(vector_set(FloatType(0.0), FloatType(0.0), FloatType(0.0), FloatType(1.0)), mtx1.w_axis, threshold));
	}

	{
		QuatType rotation_around_z = quat_from_euler(scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(90.0)), scalar_deg_to_rad(FloatType(0.0)));
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
		CHECK(vector_all_near_equal(vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0), FloatType(0.0)), result.x_axis, threshold));
		CHECK(vector_all_near_equal(vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0), FloatType(0.0)), result.y_axis, threshold));
		CHECK(vector_all_near_equal(vector_set(FloatType(0.0), FloatType(0.0), FloatType(1.0), FloatType(0.0)), result.z_axis, threshold));
		CHECK(vector_all_near_equal(vector_set(FloatType(0.0), FloatType(0.0), FloatType(0.0), FloatType(1.0)), result.w_axis, threshold));
	}

	{
		QuatType rotation_around_z = quat_from_euler(scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(90.0)), scalar_deg_to_rad(FloatType(0.0)));
		Vector4Type translation = vector_set(FloatType(1.0), FloatType(2.0), FloatType(3.0));
		Vector4Type scale = vector_set(FloatType(4.0), FloatType(5.0), FloatType(6.0));
		Matrix3x4Type mtx3x4 = matrix_from_qvv(rotation_around_z, translation, scale);
		mtx3x4.x_axis = vector_set(vector_get_x(mtx3x4.x_axis), vector_get_y(mtx3x4.x_axis), vector_get_z(mtx3x4.x_axis), FloatType(0.0));
		mtx3x4.y_axis = vector_set(vector_get_x(mtx3x4.y_axis), vector_get_y(mtx3x4.y_axis), vector_get_z(mtx3x4.y_axis), FloatType(0.0));
		mtx3x4.z_axis = vector_set(vector_get_x(mtx3x4.z_axis), vector_get_y(mtx3x4.z_axis), vector_get_z(mtx3x4.z_axis), FloatType(0.0));
		mtx3x4.w_axis = vector_set(vector_get_x(mtx3x4.w_axis), vector_get_y(mtx3x4.w_axis), vector_get_z(mtx3x4.w_axis), FloatType(1.0));
		Matrix4x4Type mtx = matrix_cast(mtx3x4);
		Matrix4x4Type inv_mtx = matrix_inverse(mtx, mtx);
		Matrix4x4Type result = matrix_mul(mtx, inv_mtx);
		CHECK(vector_all_near_equal(vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0), FloatType(0.0)), result.x_axis, threshold));
		CHECK(vector_all_near_equal(vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0), FloatType(0.0)), result.y_axis, threshold));
		CHECK(vector_all_near_equal(vector_set(FloatType(0.0), FloatType(0.0), FloatType(1.0), FloatType(0.0)), result.z_axis, threshold));
		CHECK(vector_all_near_equal(vector_set(FloatType(0.0), FloatType(0.0), FloatType(0.0), FloatType(1.0)), result.w_axis, threshold));
	}

	{
		Matrix4x4Type mtx = matrix_set(zero, zero, zero, zero);
		Matrix4x4Type inv_mtx = matrix_inverse(mtx, identity);
		CHECK(vector_all_near_equal(identity.x_axis, inv_mtx.x_axis, threshold));
		CHECK(vector_all_near_equal(identity.y_axis, inv_mtx.y_axis, threshold));
		CHECK(vector_all_near_equal(identity.z_axis, inv_mtx.z_axis, threshold));
		CHECK(vector_all_near_equal(identity.w_axis, inv_mtx.w_axis, threshold));
	}
}

template<typename FloatType>
static void test_matrix4x4_misc(const FloatType threshold)
{
	using Vector4Type = typename float_traits<FloatType>::vector4;
	using Matrix4x4Type = typename float_traits<FloatType>::matrix4x4;

	{
		Vector4Type x_axis = vector_set(FloatType(1.65424), FloatType(0.22921), FloatType(5.73038), FloatType(4.46541));
		Vector4Type y_axis = vector_set(FloatType(1.90220), FloatType(0.82590), FloatType(6.61556), FloatType(4.46383));
		Vector4Type z_axis = vector_set(FloatType(7.36288), FloatType(7.09841), FloatType(0.33519), FloatType(7.43985));
		Vector4Type w_axis = vector_set(FloatType(4.42391), FloatType(4.03858), FloatType(2.49537), FloatType(0.11255));
		Matrix4x4Type mtx = matrix_set(x_axis, y_axis, z_axis, w_axis);
		FloatType det = scalar_cast(matrix_determinant(mtx));
		CHECK(scalar_near_equal(det, FloatType(120.68779956246105324363), threshold));
	}

	{
		FloatType threshold2 = FloatType(1.0E-3F);
		Vector4Type x_axis = vector_set(FloatType(1.65424), FloatType(0.22921), FloatType(5.73038), FloatType(4.46541));
		Vector4Type y_axis = vector_set(FloatType(1.90220), FloatType(0.82590), FloatType(6.61556), FloatType(4.46383));
		Vector4Type z_axis = vector_set(FloatType(7.36288), FloatType(7.09841), FloatType(0.33519), FloatType(7.43985));
		Vector4Type w_axis = vector_set(FloatType(4.42391), FloatType(4.03858), FloatType(2.49537), FloatType(0.11255));
		Matrix4x4Type mtx = matrix_set(x_axis, y_axis, z_axis, w_axis);

		FloatType det_xx = scalar_cast(matrix_minor(mtx, axis4::x, axis4::x));
		CHECK(scalar_near_equal(det_xx, FloatType(251.213), threshold2));
		FloatType det_xy = scalar_cast(matrix_minor(mtx, axis4::x, axis4::y));
		CHECK(scalar_near_equal(det_xy, FloatType(252.409), threshold2));
		FloatType det_xz = scalar_cast(matrix_minor(mtx, axis4::x, axis4::z));
		CHECK(scalar_near_equal(det_xz, FloatType(-36.5778), threshold2));
		FloatType det_xw = scalar_cast(matrix_minor(mtx, axis4::x, axis4::w));
		CHECK(scalar_near_equal(det_xw, FloatType(6.1402), threshold2));

		FloatType det_yx = scalar_cast(matrix_minor(mtx, axis4::y, axis4::x));
		CHECK(scalar_near_equal(det_yx, FloatType(236.404), threshold2));
		FloatType det_yy = scalar_cast(matrix_minor(mtx, axis4::y, axis4::y));
		CHECK(scalar_near_equal(det_yy, FloatType(228.63), threshold2));
		FloatType det_yz = scalar_cast(matrix_minor(mtx, axis4::y, axis4::z));
		CHECK(scalar_near_equal(det_yz, FloatType(-48.4728), threshold2));
		FloatType det_yw = scalar_cast(matrix_minor(mtx, axis4::y, axis4::w));
		CHECK(scalar_near_equal(det_yw, FloatType(13.6377), threshold2));

		FloatType det_zx = scalar_cast(matrix_minor(mtx, axis4::z, axis4::x));
		CHECK(scalar_near_equal(det_zx, FloatType(-9.7121), threshold2));
		FloatType det_zy = scalar_cast(matrix_minor(mtx, axis4::z, axis4::y));
		CHECK(scalar_near_equal(det_zy, FloatType(-14.752), threshold2));
		FloatType det_zz = scalar_cast(matrix_minor(mtx, axis4::z, axis4::z));
		CHECK(scalar_near_equal(det_zz, FloatType(-7.20201), threshold2));
		FloatType det_zw = scalar_cast(matrix_minor(mtx, axis4::z, axis4::w));
		CHECK(scalar_near_equal(det_zw, FloatType(-12.0829), threshold2));

		FloatType det_wx = scalar_cast(matrix_minor(mtx, axis4::w, axis4::x));
		CHECK(scalar_near_equal(det_wx, FloatType(-51.1582), threshold2));
		FloatType det_wy = scalar_cast(matrix_minor(mtx, axis4::w, axis4::y));
		CHECK(scalar_near_equal(det_wy, FloatType(-28.475), threshold2));
		FloatType det_wz = scalar_cast(matrix_minor(mtx, axis4::w, axis4::z));
		CHECK(scalar_near_equal(det_wz, FloatType(-4.82179), threshold2));
		FloatType det_ww = scalar_cast(matrix_minor(mtx, axis4::w, axis4::w));
		CHECK(scalar_near_equal(det_ww, FloatType(-23.678), threshold2));
	}

	{
		FloatType threshold2 = FloatType(1.0E-3F);
		Vector4Type x_axis = vector_set(FloatType(1.65424), FloatType(0.22921), FloatType(5.73038), FloatType(4.46541));
		Vector4Type y_axis = vector_set(FloatType(1.90220), FloatType(0.82590), FloatType(6.61556), FloatType(4.46383));
		Vector4Type z_axis = vector_set(FloatType(7.36288), FloatType(7.09841), FloatType(0.33519), FloatType(7.43985));
		Vector4Type w_axis = vector_set(FloatType(4.42391), FloatType(4.03858), FloatType(2.49537), FloatType(0.11255));
		Matrix4x4Type mtx = matrix_set(x_axis, y_axis, z_axis, w_axis);
		Matrix4x4Type cof = matrix_cofactor(mtx);

		FloatType cof_xx = scalar_cast(vector_get_x(cof.x_axis));
		CHECK(scalar_near_equal(cof_xx, FloatType(251.213), threshold2));
		FloatType cof_xy = scalar_cast(vector_get_y(cof.x_axis));
		CHECK(scalar_near_equal(cof_xy, FloatType(-252.409), threshold2));
		FloatType cof_xz = scalar_cast(vector_get_z(cof.x_axis));
		CHECK(scalar_near_equal(cof_xz, FloatType(-36.5778), threshold2));
		FloatType cof_xw = scalar_cast(vector_get_w(cof.x_axis));
		CHECK(scalar_near_equal(cof_xw, FloatType(-6.1402), threshold2));

		FloatType cof_yx = scalar_cast(vector_get_x(cof.y_axis));
		CHECK(scalar_near_equal(cof_yx, FloatType(-236.404), threshold2));
		FloatType cof_yy = scalar_cast(vector_get_y(cof.y_axis));
		CHECK(scalar_near_equal(cof_yy, FloatType(228.63), threshold2));
		FloatType cof_yz = scalar_cast(vector_get_z(cof.y_axis));
		CHECK(scalar_near_equal(cof_yz, FloatType(48.4728), threshold2));
		FloatType cof_yw = scalar_cast(vector_get_w(cof.y_axis));
		CHECK(scalar_near_equal(cof_yw, FloatType(13.6377), threshold2));

		FloatType cof_zx = scalar_cast(vector_get_x(cof.z_axis));
		CHECK(scalar_near_equal(cof_zx, FloatType(-9.7121), threshold2));
		FloatType cof_zy = scalar_cast(vector_get_y(cof.z_axis));
		CHECK(scalar_near_equal(cof_zy, FloatType(14.752), threshold2));
		FloatType cof_zz = scalar_cast(vector_get_z(cof.z_axis));
		CHECK(scalar_near_equal(cof_zz, FloatType(-7.20201), threshold2));
		FloatType cof_zw = scalar_cast(vector_get_w(cof.z_axis));
		CHECK(scalar_near_equal(cof_zw, FloatType(12.0829), threshold2));

		FloatType cof_wx = scalar_cast(vector_get_x(cof.w_axis));
		CHECK(scalar_near_equal(cof_wx, FloatType(51.1582), threshold2));
		FloatType cof_wy = scalar_cast(vector_get_y(cof.w_axis));
		CHECK(scalar_near_equal(cof_wy, FloatType(-28.475), threshold2));
		FloatType cof_wz = scalar_cast(vector_get_z(cof.w_axis));
		CHECK(scalar_near_equal(cof_wz, FloatType(4.82179), threshold2));
		FloatType cof_ww = scalar_cast(vector_get_w(cof.w_axis));
		CHECK(scalar_near_equal(cof_ww, FloatType(-23.678), threshold2));
	}

	{
		FloatType threshold2 = FloatType(1.0E-3F);
		Vector4Type x_axis = vector_set(FloatType(1.65424), FloatType(0.22921), FloatType(5.73038), FloatType(4.46541));
		Vector4Type y_axis = vector_set(FloatType(1.90220), FloatType(0.82590), FloatType(6.61556), FloatType(4.46383));
		Vector4Type z_axis = vector_set(FloatType(7.36288), FloatType(7.09841), FloatType(0.33519), FloatType(7.43985));
		Vector4Type w_axis = vector_set(FloatType(4.42391), FloatType(4.03858), FloatType(2.49537), FloatType(0.11255));
		Matrix4x4Type mtx = matrix_set(x_axis, y_axis, z_axis, w_axis);
		Matrix4x4Type adj = matrix_adjugate(mtx);

		FloatType adj_xx = scalar_cast(vector_get_x(adj.x_axis));
		CHECK(scalar_near_equal(adj_xx, FloatType(251.213), threshold2));
		FloatType adj_xy = scalar_cast(vector_get_y(adj.x_axis));
		CHECK(scalar_near_equal(adj_xy, FloatType(-236.404), threshold2));
		FloatType adj_xz = scalar_cast(vector_get_z(adj.x_axis));
		CHECK(scalar_near_equal(adj_xz, FloatType(-9.7121), threshold2));
		FloatType adj_xw = scalar_cast(vector_get_w(adj.x_axis));
		CHECK(scalar_near_equal(adj_xw, FloatType(51.1582), threshold2));

		FloatType adj_yx = scalar_cast(vector_get_x(adj.y_axis));
		CHECK(scalar_near_equal(adj_yx, FloatType(-252.409), threshold2));
		FloatType adj_yy = scalar_cast(vector_get_y(adj.y_axis));
		CHECK(scalar_near_equal(adj_yy, FloatType(228.63), threshold2));
		FloatType adj_yz = scalar_cast(vector_get_z(adj.y_axis));
		CHECK(scalar_near_equal(adj_yz, FloatType(14.752), threshold2));
		FloatType adj_yw = scalar_cast(vector_get_w(adj.y_axis));
		CHECK(scalar_near_equal(adj_yw, FloatType(-28.475), threshold2));

		FloatType adj_zx = scalar_cast(vector_get_x(adj.z_axis));
		CHECK(scalar_near_equal(adj_zx, FloatType(-36.5778), threshold2));
		FloatType adj_zy = scalar_cast(vector_get_y(adj.z_axis));
		CHECK(scalar_near_equal(adj_zy, FloatType(48.4728), threshold2));
		FloatType adj_zz = scalar_cast(vector_get_z(adj.z_axis));
		CHECK(scalar_near_equal(adj_zz, FloatType(-7.20201), threshold2));
		FloatType adj_zw = scalar_cast(vector_get_w(adj.z_axis));
		CHECK(scalar_near_equal(adj_zw, FloatType(4.82179), threshold2));

		FloatType adj_wx = scalar_cast(vector_get_x(adj.w_axis));
		CHECK(scalar_near_equal(adj_wx, FloatType(-6.1402), threshold2));
		FloatType adj_wy = scalar_cast(vector_get_y(adj.w_axis));
		CHECK(scalar_near_equal(adj_wy, FloatType(13.6377), threshold2));
		FloatType adj_wz = scalar_cast(vector_get_z(adj.w_axis));
		CHECK(scalar_near_equal(adj_wz, FloatType(12.0829), threshold2));
		FloatType adj_ww = scalar_cast(vector_get_w(adj.w_axis));
		CHECK(scalar_near_equal(adj_ww, FloatType(-23.678), threshold2));
	}
}

TEST_CASE("matrix4x4f math", "[math][matrix4x4]")
{
	test_matrix4x4_setters<float>(1.0E-4F);
	test_matrix4x4_arithmetic<float>(1.0E-4F);
	test_matrix4x4_transformations<float>(1.0E-4F);
	test_matrix4x4_misc<float>(1.0E-4F);

	{
		quatf rotation_around_z = quat_from_euler(scalar_deg_to_rad(0.0F), scalar_deg_to_rad(90.0F), scalar_deg_to_rad(0.0F));
		vector4f translation = vector_set(1.0F, 2.0F, 3.0F);
		vector4f scale = vector_set(4.0F, 5.0F, 6.0F);
		matrix3x4f src3x4 = matrix_from_qvv(rotation_around_z, translation, scale);

		matrix4x4f src = matrix_cast(src3x4);
		CHECK((float)vector_get_w(src.x_axis) == 0.0F);
		CHECK((float)vector_get_w(src.y_axis) == 0.0F);
		CHECK((float)vector_get_w(src.z_axis) == 0.0F);
		CHECK((float)vector_get_w(src.w_axis) == 1.0F);

		matrix4x4d dst = matrix_cast(src);
		CHECK(vector_all_near_equal(vector_cast(src.x_axis), dst.x_axis, 1.0E-4));
		CHECK(vector_all_near_equal(vector_cast(src.y_axis), dst.y_axis, 1.0E-4));
		CHECK(vector_all_near_equal(vector_cast(src.z_axis), dst.z_axis, 1.0E-4));
		CHECK(vector_all_near_equal(vector_cast(src.w_axis), dst.w_axis, 1.0E-4));
	}
}

TEST_CASE("matrix4x4d math", "[math][matrix4x4]")
{
	test_matrix4x4_setters<double>(1.0E-4);
	test_matrix4x4_arithmetic<double>(1.0E-4);
	test_matrix4x4_transformations<double>(1.0E-4);
	test_matrix4x4_misc<double>(1.0E-4);

	{
		quatd rotation_around_z = quat_from_euler(scalar_deg_to_rad(0.0), scalar_deg_to_rad(90.0), scalar_deg_to_rad(0.0));
		vector4d translation = vector_set(1.0, 2.0, 3.0);
		vector4d scale = vector_set(4.0, 5.0, 6.0);
		matrix3x4d src3x4 = matrix_from_qvv(rotation_around_z, translation, scale);

		matrix4x4d src = matrix_cast(src3x4);
		CHECK((double)vector_get_w(src.x_axis) == 0.0);
		CHECK((double)vector_get_w(src.y_axis) == 0.0);
		CHECK((double)vector_get_w(src.z_axis) == 0.0);
		CHECK((double)vector_get_w(src.w_axis) == 1.0);

		matrix4x4f dst = matrix_cast(src);
		CHECK(vector_all_near_equal(vector_cast(src.x_axis), dst.x_axis, 1.0E-4F));
		CHECK(vector_all_near_equal(vector_cast(src.y_axis), dst.y_axis, 1.0E-4F));
		CHECK(vector_all_near_equal(vector_cast(src.z_axis), dst.z_axis, 1.0E-4F));
		CHECK(vector_all_near_equal(vector_cast(src.w_axis), dst.w_axis, 1.0E-4F));
	}
}
