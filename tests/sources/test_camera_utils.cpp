////////////////////////////////////////////////////////////////////////////////
// The MIT License (MIT)
//
// Copyright (c) 2026 Nicholas Frechette & Realtime Math contributors
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

#include <rtm/type_traits.h>
#include <rtm/camera_utilsd.h>
#include <rtm/camera_utilsf.h>
#include <rtm/matrix3x4d.h>
#include <rtm/matrix3x4f.h>
#include <rtm/matrix4x4d.h>
#include <rtm/matrix4x4f.h>
#include <rtm/scalard.h>
#include <rtm/scalarf.h>
#include <rtm/vector4d.h>
#include <rtm/vector4f.h>

using namespace rtm;

template<typename FloatType>
static void test_camera_look(const FloatType threshold)
{
	using Vector4Type = typename related_types<FloatType>::vector4;
	using Matrix3x4Type = typename related_types<FloatType>::matrix3x4;

	const Vector4Type zero = vector_zero();

	{
		// The look direction and the up direction are not normalized
		const Vector4Type position = vector_set(FloatType(1.0), FloatType(2.0), FloatType(3.0));
		const Vector4Type direction = vector_set(FloatType(0.0), FloatType(0.0), FloatType(5.0));
		const Vector4Type up = vector_set(FloatType(0.0), FloatType(3.0), FloatType(0.0));

		const Matrix3x4Type mtx = matrix_look_to(position, direction, up);
		CHECK(vector_all_near_equal3(vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0)), mtx.x_axis, threshold));
		CHECK(vector_all_near_equal3(vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0)), mtx.y_axis, threshold));
		CHECK(vector_all_near_equal3(vector_set(FloatType(0.0), FloatType(0.0), FloatType(1.0)), mtx.z_axis, threshold));
		CHECK(vector_all_near_equal3(position, mtx.w_axis, threshold));
	}

	{
		const Vector4Type position = vector_set(FloatType(-4.0), FloatType(0.5), FloatType(2.0));
		const Vector4Type direction = vector_set(FloatType(2.0), FloatType(0.0), FloatType(0.0));
		const Vector4Type up = vector_set(FloatType(0.0), FloatType(0.0), FloatType(1.0));

		const Matrix3x4Type mtx = matrix_look_to(position, direction, up);
		CHECK(vector_all_near_equal3(vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0)), mtx.x_axis, threshold));
		CHECK(vector_all_near_equal3(vector_set(FloatType(0.0), FloatType(0.0), FloatType(1.0)), mtx.y_axis, threshold));
		CHECK(vector_all_near_equal3(vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0)), mtx.z_axis, threshold));
		CHECK(vector_all_near_equal3(position, mtx.w_axis, threshold));
	}

	{
		// The up direction is not perpendicular to the look direction
		const Vector4Type position = vector_set(FloatType(1.5), FloatType(-2.0), FloatType(7.0));
		const Vector4Type location = vector_set(FloatType(-3.0), FloatType(4.0), FloatType(2.5));
		const Vector4Type up = vector_set(FloatType(0.2), FloatType(1.0), FloatType(0.3));
		const Vector4Type direction = vector_sub(location, position);

		const Matrix3x4Type mtx = matrix_look_to(position, direction, up);

		// The function returns an ortho-normal frame
		CHECK(scalar_near_equal(FloatType(vector_length3(mtx.x_axis)), FloatType(1.0), threshold));
		CHECK(scalar_near_equal(FloatType(vector_length3(mtx.y_axis)), FloatType(1.0), threshold));
		CHECK(scalar_near_equal(FloatType(vector_length3(mtx.z_axis)), FloatType(1.0), threshold));
		CHECK(scalar_near_equal(FloatType(vector_dot3(mtx.x_axis, mtx.y_axis)), FloatType(0.0), threshold));
		CHECK(scalar_near_equal(FloatType(vector_dot3(mtx.x_axis, mtx.z_axis)), FloatType(0.0), threshold));
		CHECK(scalar_near_equal(FloatType(vector_dot3(mtx.y_axis, mtx.z_axis)), FloatType(0.0), threshold));
		CHECK(vector_all_near_equal3(vector_cross3(mtx.x_axis, mtx.y_axis), mtx.z_axis, threshold));

		// The forward axis points towards the location and the up axis is on the side of the up input
		CHECK(vector_all_near_equal3(vector_normalize3(direction), mtx.z_axis, threshold));
		CHECK(FloatType(vector_dot3(mtx.y_axis, up)) > FloatType(0.0));
		CHECK(vector_all_near_equal3(position, mtx.w_axis, threshold));

		const Matrix3x4Type mtx_at = matrix_look_at(position, location, up);
		CHECK(vector_all_near_equal3(mtx.x_axis, mtx_at.x_axis, threshold));
		CHECK(vector_all_near_equal3(mtx.y_axis, mtx_at.y_axis, threshold));
		CHECK(vector_all_near_equal3(mtx.z_axis, mtx_at.z_axis, threshold));
		CHECK(vector_all_near_equal3(mtx.w_axis, mtx_at.w_axis, threshold));

		// The view transform is the inverse of the camera transform
		const Matrix3x4Type view = view_look_to(position, direction, up);
		const Matrix3x4Type inv_mtx = matrix_inverse(mtx);
		CHECK(vector_all_near_equal3(inv_mtx.x_axis, view.x_axis, threshold));
		CHECK(vector_all_near_equal3(inv_mtx.y_axis, view.y_axis, threshold));
		CHECK(vector_all_near_equal3(inv_mtx.z_axis, view.z_axis, threshold));
		CHECK(vector_all_near_equal3(inv_mtx.w_axis, view.w_axis, threshold));

		const Matrix3x4Type view_at = view_look_at(position, location, up);
		CHECK(vector_all_near_equal3(view.x_axis, view_at.x_axis, threshold));
		CHECK(vector_all_near_equal3(view.y_axis, view_at.y_axis, threshold));
		CHECK(vector_all_near_equal3(view.z_axis, view_at.z_axis, threshold));
		CHECK(vector_all_near_equal3(view.w_axis, view_at.w_axis, threshold));

		// The view transform moves the camera position to the origin
		// and the location to the forward axis
		CHECK(vector_all_near_equal3(zero, matrix_mul_point3(position, view), threshold));
		const FloatType distance = vector_length3(direction);
		CHECK(vector_all_near_equal3(vector_set(FloatType(0.0), FloatType(0.0), distance), matrix_mul_point3(location, view), threshold));

		// A point local to the camera goes to world space and back again
		const Vector4Type local_point = vector_set(FloatType(0.5), FloatType(-1.25), FloatType(3.0));
		const Vector4Type world_point = matrix_mul_point3(local_point, mtx);
		CHECK(vector_all_near_equal3(local_point, matrix_mul_point3(world_point, view), threshold));
	}
}

template<typename FloatType>
static void test_camera_projection(const FloatType threshold)
{
	using Vector4Type = typename related_types<FloatType>::vector4;
	using Matrix4x4Type = typename related_types<FloatType>::matrix4x4;

	const FloatType view_width = FloatType(16.0);
	const FloatType view_height = FloatType(9.0);
	const FloatType near_distance = FloatType(0.5);
	const FloatType far_distance = FloatType(100.0);

	{
		const Matrix4x4Type mtx = proj_perspective(view_width, view_height, near_distance, far_distance);

		const FloatType scaled_far_distance = far_distance / (far_distance - near_distance);
		CHECK(vector_all_near_equal(vector_set(FloatType(2.0) * near_distance / view_width, FloatType(0.0), FloatType(0.0), FloatType(0.0)), mtx.x_axis, threshold));
		CHECK(vector_all_near_equal(vector_set(FloatType(0.0), FloatType(2.0) * near_distance / view_height, FloatType(0.0), FloatType(0.0)), mtx.y_axis, threshold));
		CHECK(vector_all_near_equal(vector_set(FloatType(0.0), FloatType(0.0), scaled_far_distance, FloatType(1.0)), mtx.z_axis, threshold));
		CHECK(vector_all_near_equal(vector_set(FloatType(0.0), FloatType(0.0), -scaled_far_distance * near_distance, FloatType(0.0)), mtx.w_axis, threshold));

		// The corner of the near plane goes to [1, 1, 0] after the division by W
		const Vector4Type near_corner = matrix_mul_vector(vector_set(view_width * FloatType(0.5), view_height * FloatType(0.5), near_distance, FloatType(1.0)), mtx);
		CHECK(vector_all_near_equal3(vector_set(FloatType(1.0), FloatType(1.0), FloatType(0.0)), vector_div(near_corner, vector_dup_w(near_corner)), threshold));

		// A point on the far plane goes to a depth of 1 after the division by W
		const Vector4Type far_point = matrix_mul_vector(vector_set(FloatType(0.0), FloatType(0.0), far_distance, FloatType(1.0)), mtx);
		CHECK(scalar_near_equal(FloatType(vector_get_z(far_point)) / FloatType(vector_get_w(far_point)), FloatType(1.0), threshold));
	}

	{
		const FloatType fov_angle_y = scalar_deg_to_rad(FloatType(60.0));
		const FloatType aspect_ratio = view_width / view_height;
		const Matrix4x4Type mtx = proj_perspective_fov(fov_angle_y, aspect_ratio, near_distance, far_distance);

		// The same projection with the dimensions of the near plane
		const FloatType near_height = FloatType(2.0) * near_distance * scalar_tan(fov_angle_y * FloatType(0.5));
		const FloatType near_width = near_height * aspect_ratio;
		const Matrix4x4Type ref = proj_perspective(near_width, near_height, near_distance, far_distance);
		CHECK(vector_all_near_equal(ref.x_axis, mtx.x_axis, threshold));
		CHECK(vector_all_near_equal(ref.y_axis, mtx.y_axis, threshold));
		CHECK(vector_all_near_equal(ref.z_axis, mtx.z_axis, threshold));
		CHECK(vector_all_near_equal(ref.w_axis, mtx.w_axis, threshold));

		// For a field of view of 90 degrees, the vertical scale is 1
		const Matrix4x4Type mtx90 = proj_perspective_fov(scalar_deg_to_rad(FloatType(90.0)), FloatType(2.0), near_distance, far_distance);
		CHECK(vector_all_near_equal(vector_set(FloatType(0.5), FloatType(0.0), FloatType(0.0), FloatType(0.0)), mtx90.x_axis, threshold));
		CHECK(vector_all_near_equal(vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0), FloatType(0.0)), mtx90.y_axis, threshold));
	}

	{
		const Matrix4x4Type mtx = proj_orthographic(view_width, view_height, near_distance, far_distance);

		const FloatType inv_range = FloatType(1.0) / (far_distance - near_distance);
		CHECK(vector_all_near_equal(vector_set(FloatType(2.0) / view_width, FloatType(0.0), FloatType(0.0), FloatType(0.0)), mtx.x_axis, threshold));
		CHECK(vector_all_near_equal(vector_set(FloatType(0.0), FloatType(2.0) / view_height, FloatType(0.0), FloatType(0.0)), mtx.y_axis, threshold));
		CHECK(vector_all_near_equal(vector_set(FloatType(0.0), FloatType(0.0), inv_range, FloatType(0.0)), mtx.z_axis, threshold));
		CHECK(vector_all_near_equal(vector_set(FloatType(0.0), FloatType(0.0), -inv_range * near_distance, FloatType(1.0)), mtx.w_axis, threshold));

		// The corners of the view volume go to [1, 1, 0] and [-1, -1, 1]
		const Vector4Type near_corner = matrix_mul_vector(vector_set(view_width * FloatType(0.5), view_height * FloatType(0.5), near_distance, FloatType(1.0)), mtx);
		CHECK(vector_all_near_equal(vector_set(FloatType(1.0), FloatType(1.0), FloatType(0.0), FloatType(1.0)), near_corner, threshold));

		const Vector4Type far_corner = matrix_mul_vector(vector_set(view_width * FloatType(-0.5), view_height * FloatType(-0.5), far_distance, FloatType(1.0)), mtx);
		CHECK(vector_all_near_equal(vector_set(FloatType(-1.0), FloatType(-1.0), FloatType(1.0), FloatType(1.0)), far_corner, threshold));
	}
}

TEST_CASE("camera_utilsf math", "[math][camera]")
{
	test_camera_look<float>(1.0E-4F);
	test_camera_projection<float>(1.0E-4F);
}

TEST_CASE("camera_utilsd math", "[math][camera]")
{
	test_camera_look<double>(1.0E-9);
	test_camera_projection<double>(1.0E-9);
}
