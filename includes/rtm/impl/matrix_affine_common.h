#pragma once

////////////////////////////////////////////////////////////////////////////////
// The MIT License (MIT)
//
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

#include "rtm/math.h"
#include "rtm/vector4f.h"
#include "rtm/vector4d.h"
#include "rtm/quatf.h"
#include "rtm/quatd.h"

namespace rtm
{
	namespace rtm_impl
	{
		//////////////////////////////////////////////////////////////////////////
		// A helper struct to convert a quaternion to matrices of similar width.
		//////////////////////////////////////////////////////////////////////////
		template<typename float_type, typename quat_type, typename vector4_type, typename matrix3x3_type, typename matrix3x4_type>
		struct matrix_from_quat_helper
		{
			constexpr explicit matrix_from_quat_helper(quat_type quat_) RTM_NO_EXCEPT : quat(quat_) {}

			inline RTM_SIMD_CALL operator matrix3x3_type() const RTM_NO_EXCEPT
			{
				RTM_ASSERT(quat_is_normalized(quat), "Quaternion is not normalized");

				const float_type x2 = quat_get_x(quat) + quat_get_x(quat);
				const float_type y2 = quat_get_y(quat) + quat_get_y(quat);
				const float_type z2 = quat_get_z(quat) + quat_get_z(quat);
				const float_type xx = quat_get_x(quat) * x2;
				const float_type xy = quat_get_x(quat) * y2;
				const float_type xz = quat_get_x(quat) * z2;
				const float_type yy = quat_get_y(quat) * y2;
				const float_type yz = quat_get_y(quat) * z2;
				const float_type zz = quat_get_z(quat) * z2;
				const float_type wx = quat_get_w(quat) * x2;
				const float_type wy = quat_get_w(quat) * y2;
				const float_type wz = quat_get_w(quat) * z2;

				const vector4_type x_axis = vector_set(float_type(1.0) - (yy + zz), xy + wz, xz - wy, float_type(0.0));
				const vector4_type y_axis = vector_set(xy - wz, float_type(1.0) - (xx + zz), yz + wx, float_type(0.0));
				const vector4_type z_axis = vector_set(xz + wy, yz - wx, float_type(1.0) - (xx + yy), float_type(0.0));
				return matrix3x3_type{ x_axis, y_axis, z_axis };
			}

			inline RTM_SIMD_CALL operator matrix3x4_type() const RTM_NO_EXCEPT
			{
				RTM_ASSERT(quat_is_normalized(quat), "Quaternion is not normalized");

				const float_type x2 = quat_get_x(quat) + quat_get_x(quat);
				const float_type y2 = quat_get_y(quat) + quat_get_y(quat);
				const float_type z2 = quat_get_z(quat) + quat_get_z(quat);
				const float_type xx = quat_get_x(quat) * x2;
				const float_type xy = quat_get_x(quat) * y2;
				const float_type xz = quat_get_x(quat) * z2;
				const float_type yy = quat_get_y(quat) * y2;
				const float_type yz = quat_get_y(quat) * z2;
				const float_type zz = quat_get_z(quat) * z2;
				const float_type wx = quat_get_w(quat) * x2;
				const float_type wy = quat_get_w(quat) * y2;
				const float_type wz = quat_get_w(quat) * z2;

				const vector4_type x_axis = vector_set(float_type(1.0) - (yy + zz), xy + wz, xz - wy, float_type(0.0));
				const vector4_type y_axis = vector_set(xy - wz, float_type(1.0) - (xx + zz), yz + wx, float_type(0.0));
				const vector4_type z_axis = vector_set(xz + wy, yz - wx, float_type(1.0) - (xx + yy), float_type(0.0));
				const vector4_type w_axis = vector_set(float_type(0.0), float_type(0.0), float_type(0.0), float_type(1.0));
				return matrix3x4_type{ x_axis, y_axis, z_axis, w_axis };
			}

			quat_type quat;
		};

		//////////////////////////////////////////////////////////////////////////
		// A helper struct to convert a 3D scale vector to matrices of similar width.
		//////////////////////////////////////////////////////////////////////////
		template<typename float_type, typename vector4_type, typename matrix3x3_type, typename matrix3x4_type>
		struct matrix_from_scale_helper
		{
			constexpr explicit matrix_from_scale_helper(vector4_type scale_) RTM_NO_EXCEPT : scale(scale_) {}

			inline RTM_SIMD_CALL operator matrix3x3_type() const RTM_NO_EXCEPT
			{
				RTM_ASSERT(!vector_any_near_equal3(scale, vector_zero()), "Scale cannot be zero");
				return matrix3x3_type{ vector_set(vector_get_x(scale), float_type(0.0), float_type(0.0), float_type(0.0)), vector_set(float_type(0.0), vector_get_y(scale), float_type(0.0), float_type(0.0)), vector_set(float_type(0.0), float_type(0.0), vector_get_z(scale), float_type(0.0)) };
			}

			inline RTM_SIMD_CALL operator matrix3x4_type() const RTM_NO_EXCEPT
			{
				RTM_ASSERT(!vector_any_near_equal3(scale, vector_zero()), "Scale cannot be zero");
				return matrix3x4_type{ vector_set(vector_get_x(scale), float_type(0.0), float_type(0.0), float_type(0.0)), vector_set(float_type(0.0), vector_get_y(scale), float_type(0.0), float_type(0.0)), vector_set(float_type(0.0), float_type(0.0), vector_get_z(scale), float_type(0.0)), vector_set(float_type(0.0), float_type(0.0), float_type(0.0), float_type(1.0)) };
			}

			vector4_type scale;
		};

		constexpr vector4f matrix_get_axis(vector4f_arg0 x_axis, vector4f_arg1 y_axis, vector4f_arg2 z_axis, axis4 axis)
		{
			return axis == axis4::x ? x_axis : (axis == axis4::y ? y_axis : z_axis);
		}

		constexpr const vector4d& matrix_get_axis(const vector4d& x_axis, const vector4d& y_axis, const vector4d& z_axis, axis4 axis)
		{
			return axis == axis4::x ? x_axis : (axis == axis4::y ? y_axis : z_axis);
		}

		//////////////////////////////////////////////////////////////////////////
		// Converts a 3x3 matrix into a rotation quaternion.
		//////////////////////////////////////////////////////////////////////////
		inline quatf RTM_SIMD_CALL quat_from_matrix(vector4f_arg0 x_axis, vector4f_arg1 y_axis, vector4f_arg2 z_axis) RTM_NO_EXCEPT
		{
			const vector4f zero = vector_zero();
			if (vector_all_near_equal3(x_axis, zero) || vector_all_near_equal3(y_axis, zero) || vector_all_near_equal3(z_axis, zero))
			{
				// Zero scale not supported, return the identity
				return quat_identity();
			}

			const float mtx_trace = vector_get_x(x_axis) + vector_get_y(y_axis) + vector_get_z(z_axis);
			if (mtx_trace > 0.0f)
			{
				const float inv_trace = scalar_sqrt_reciprocal(mtx_trace + 1.0f);
				const float half_inv_trace = inv_trace * 0.5f;

				const float x = (vector_get_z(y_axis) - vector_get_y(z_axis)) * half_inv_trace;
				const float y = (vector_get_x(z_axis) - vector_get_z(x_axis)) * half_inv_trace;
				const float z = (vector_get_y(x_axis) - vector_get_x(y_axis)) * half_inv_trace;
				const float w = scalar_reciprocal(inv_trace) * 0.5f;

				return quat_normalize(quat_set(x, y, z, w));
			}
			else
			{
				// Note that axis4::xyzw have the same values as mix4::xyzw
				int8_t best_axis = (int8_t)axis4::x;
				if (vector_get_y(y_axis) > vector_get_x(x_axis))
					best_axis = (int8_t)axis4::y;
				if (vector_get_z(z_axis) > vector_get_component(matrix_get_axis(x_axis, y_axis, z_axis, axis4(best_axis)), mix4(best_axis)))
					best_axis = (int8_t)axis4::z;

				const int8_t next_best_axis = (best_axis + 1) % 3;
				const int8_t next_next_best_axis = (next_best_axis + 1) % 3;

				const float mtx_pseudo_trace = 1.0f +
					vector_get_component(matrix_get_axis(x_axis, y_axis, z_axis, axis4(best_axis)), mix4(best_axis)) -
					vector_get_component(matrix_get_axis(x_axis, y_axis, z_axis, axis4(next_best_axis)), mix4(next_best_axis)) -
					vector_get_component(matrix_get_axis(x_axis, y_axis, z_axis, axis4(next_next_best_axis)), mix4(next_next_best_axis));

				const float inv_pseudo_trace = scalar_sqrt_reciprocal(mtx_pseudo_trace);
				const float half_inv_pseudo_trace = inv_pseudo_trace * 0.5f;

				float quat_values[4];
				quat_values[best_axis] = scalar_reciprocal(inv_pseudo_trace) * 0.5f;
				quat_values[next_best_axis] = half_inv_pseudo_trace *
					(vector_get_component(matrix_get_axis(x_axis, y_axis, z_axis, axis4(best_axis)), mix4(next_best_axis)) +
						vector_get_component(matrix_get_axis(x_axis, y_axis, z_axis, axis4(next_best_axis)), mix4(best_axis)));
				quat_values[next_next_best_axis] = half_inv_pseudo_trace *
					(vector_get_component(matrix_get_axis(x_axis, y_axis, z_axis, axis4(best_axis)), mix4(next_next_best_axis)) +
						vector_get_component(matrix_get_axis(x_axis, y_axis, z_axis, axis4(next_next_best_axis)), mix4(best_axis)));
				quat_values[3] = half_inv_pseudo_trace *
					(vector_get_component(matrix_get_axis(x_axis, y_axis, z_axis, axis4(next_best_axis)), mix4(next_next_best_axis)) -
						vector_get_component(matrix_get_axis(x_axis, y_axis, z_axis, axis4(next_next_best_axis)), mix4(next_best_axis)));

				return quat_normalize(quat_unaligned_load(&quat_values[0]));
			}
		}

		//////////////////////////////////////////////////////////////////////////
		// Converts a 3x3 matrix into a rotation quaternion.
		//////////////////////////////////////////////////////////////////////////
		inline quatd RTM_SIMD_CALL quat_from_matrix(const vector4d& x_axis, const vector4d& y_axis, const vector4d& z_axis) RTM_NO_EXCEPT
		{
			const vector4d zero = vector_zero();
			if (vector_all_near_equal3(x_axis, zero) || vector_all_near_equal3(y_axis, zero) || vector_all_near_equal3(z_axis, zero))
			{
				// Zero scale not supported, return the identity
				return quat_identity();
			}

			const double mtx_trace = vector_get_x(x_axis) + vector_get_y(y_axis) + vector_get_z(z_axis);
			if (mtx_trace > 0.0)
			{
				const double inv_trace = scalar_sqrt_reciprocal(mtx_trace + 1.0);
				const double half_inv_trace = inv_trace * 0.5;

				const double x = (vector_get_z(y_axis) - vector_get_y(z_axis)) * half_inv_trace;
				const double y = (vector_get_x(z_axis) - vector_get_z(x_axis)) * half_inv_trace;
				const double z = (vector_get_y(x_axis) - vector_get_x(y_axis)) * half_inv_trace;
				const double w = scalar_reciprocal(inv_trace) * 0.5;

				return quat_normalize(quat_set(x, y, z, w));
			}
			else
			{
				// Note that axis4::xyzw have the same values as mix4::xyzw
				int8_t best_axis = (int8_t)axis4::x;
				if (vector_get_y(y_axis) > vector_get_x(x_axis))
					best_axis = (int8_t)axis4::y;
				if (vector_get_z(z_axis) > vector_get_component(matrix_get_axis(x_axis, y_axis, z_axis, axis4(best_axis)), mix4(best_axis)))
					best_axis = (int8_t)axis4::z;

				const int8_t next_best_axis = (best_axis + 1) % 3;
				const int8_t next_next_best_axis = (next_best_axis + 1) % 3;

				const double mtx_pseudo_trace = 1.0 +
					vector_get_component(matrix_get_axis(x_axis, y_axis, z_axis, axis4(best_axis)), mix4(best_axis)) -
					vector_get_component(matrix_get_axis(x_axis, y_axis, z_axis, axis4(next_best_axis)), mix4(next_best_axis)) -
					vector_get_component(matrix_get_axis(x_axis, y_axis, z_axis, axis4(next_next_best_axis)), mix4(next_next_best_axis));

				const double inv_pseudo_trace = scalar_sqrt_reciprocal(mtx_pseudo_trace);
				const double half_inv_pseudo_trace = inv_pseudo_trace * 0.5;

				double quat_values[4];
				quat_values[best_axis] = scalar_reciprocal(inv_pseudo_trace) * 0.5;
				quat_values[next_best_axis] = half_inv_pseudo_trace *
					(vector_get_component(matrix_get_axis(x_axis, y_axis, z_axis, axis4(best_axis)), mix4(next_best_axis)) +
						vector_get_component(matrix_get_axis(x_axis, y_axis, z_axis, axis4(next_best_axis)), mix4(best_axis)));
				quat_values[next_next_best_axis] = half_inv_pseudo_trace *
					(vector_get_component(matrix_get_axis(x_axis, y_axis, z_axis, axis4(best_axis)), mix4(next_next_best_axis)) +
						vector_get_component(matrix_get_axis(x_axis, y_axis, z_axis, axis4(next_next_best_axis)), mix4(best_axis)));
				quat_values[3] = half_inv_pseudo_trace *
					(vector_get_component(matrix_get_axis(x_axis, y_axis, z_axis, axis4(next_best_axis)), mix4(next_next_best_axis)) -
						vector_get_component(matrix_get_axis(x_axis, y_axis, z_axis, axis4(next_next_best_axis)), mix4(next_best_axis)));

				return quat_normalize(quat_unaligned_load(&quat_values[0]));
			}
		}
	}

	//////////////////////////////////////////////////////////////////////////
	// Converts a rotation quaternion into a 3x3 or 3x4 affine matrix.
	//////////////////////////////////////////////////////////////////////////
	constexpr rtm_impl::matrix_from_quat_helper<float, quatf, vector4f, matrix3x3f, matrix3x4f> RTM_SIMD_CALL matrix_from_quat(quatf_arg0 quat) RTM_NO_EXCEPT
	{
		return rtm_impl::matrix_from_quat_helper<float, quatf, vector4f, matrix3x3f, matrix3x4f>(quat);
	}

	//////////////////////////////////////////////////////////////////////////
	// Converts a rotation quaternion into a 3x3 or 3x4 affine matrix.
	//////////////////////////////////////////////////////////////////////////
	constexpr rtm_impl::matrix_from_quat_helper<double, quatd, vector4d, matrix3x3d, matrix3x4d> RTM_SIMD_CALL matrix_from_quat(const quatd& quat) RTM_NO_EXCEPT
	{
		return rtm_impl::matrix_from_quat_helper<double, quatd, vector4d, matrix3x3d, matrix3x4d>(quat);
	}

	//////////////////////////////////////////////////////////////////////////
	// Converts a 3D scale vector into a 3x3 or 3x4 affine matrix.
	//////////////////////////////////////////////////////////////////////////
	constexpr rtm_impl::matrix_from_scale_helper<float, vector4f, matrix3x3f, matrix3x4f> RTM_SIMD_CALL matrix_from_scale(vector4f_arg0 scale) RTM_NO_EXCEPT
	{
		return rtm_impl::matrix_from_scale_helper<float, vector4f, matrix3x3f, matrix3x4f>(scale);
	}

	//////////////////////////////////////////////////////////////////////////
	// Converts a 3D scale vector into a 3x3 or 3x4 affine matrix.
	//////////////////////////////////////////////////////////////////////////
	constexpr rtm_impl::matrix_from_scale_helper<double, vector4d, matrix3x3d, matrix3x4d> RTM_SIMD_CALL matrix_from_scale(const vector4d& scale) RTM_NO_EXCEPT
	{
		return rtm_impl::matrix_from_scale_helper<double, vector4d, matrix3x3d, matrix3x4d>(scale);
	}
}
