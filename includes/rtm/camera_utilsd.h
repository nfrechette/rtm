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

#include "rtm/macros.h"
#include "rtm/math.h"
#include "rtm/vector4d.h"
#include "rtm/version.h"
#include "rtm/impl/compiler_utils.h"
#include "rtm/impl/error.h"

RTM_IMPL_FILE_PRAGMA_PUSH

namespace rtm
{
	RTM_IMPL_VERSION_NAMESPACE_BEGIN

	//////////////////////////////////////////////////////////////////////////
	// Returns a left-handed affine 3x4 matrix representing a camera transform.
	// A camera transform multiplied by a point3 transforms a point3 local to the camera
	// into world space.
	// The camera transform is located at the specified position,
	// looking towards the specified direction, and using the specified up direction.
	//
	// The look to direction and up inputs do not need to be normalized.
	//
	// In this frame of reference:
	//    - X axis: right direction
	//    - Y axis: up direction
	//    - Z axis: forward/look direction
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline matrix3x4d RTM_SIMD_CALL matrix_look_to_lh(vector4d_arg0 position, vector4d_arg1 direction, vector4d_arg2 up) RTM_NO_EXCEPT
	{
		RTM_ASSERT(vector_is_finite3(position), "Look position must be finite");
		RTM_ASSERT(!vector_all_equal3(direction, vector_zero()) && vector_is_finite3(direction), "Look towards direction must be non-zero and finite");
		RTM_ASSERT(!vector_all_equal3(up, vector_zero()) && vector_is_finite3(up), "Look up direction must be non-zero and finite");

		// Constructs our orientation ortho-normal frame
		vector4d z_axis = vector_normalize3(direction);
		vector4d x_axis = vector_normalize3(vector_cross3(up, z_axis));
		vector4d y_axis = vector_cross3(z_axis, x_axis);	// already normalized since inputs are normalized

		return matrix3x4d{ x_axis, y_axis, z_axis, position };
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns a right-handed affine 3x4 matrix representing a camera transform.
	// A camera transform multiplied by a point3 transforms a point3 local to the camera
	// into world space.
	// The camera transform is located at the specified position,
	// looking towards the specified direction, and using the specified up direction.
	//
	// The look to direction and up inputs do not need to be normalized.
	//
	// In this frame of reference:
	//    - X axis: left direction
	//    - Y axis: up direction
	//    - Z axis: forward/look direction
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline matrix3x4d RTM_SIMD_CALL matrix_look_to_rh(vector4d_arg0 position, vector4d_arg1 direction, vector4d_arg2 up) RTM_NO_EXCEPT
	{
		// A right-handed camera transform is equivalent to a left-handed one pointing in the opposite direction
		return matrix_look_to_lh(position, vector_neg(direction), up);
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns a left-handed affine 3x4 matrix representing a camera view transform.
	// A camera view transform is the inverse of a camera transform.
	// A camera transform multiplied by a point3 transforms a point3 local to the camera
	// into world space while a camera view transform multiplied by a point3 does the
	// opposite: it transforms a world space point3 into local space of the camera.
	// This is typically used before applying a projection matrix.
	// The camera transform is located at the specified position,
	// looking towards the specified direction, and using the specified up direction.
	// The camera view transform is then constructed from its inverse.
	//
	// The look to direction and up inputs do not need to be normalized.
	//
	// In this frame of reference:
	//    - X axis: right direction
	//    - Y axis: up direction
	//    - Z axis: forward/look direction
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline matrix3x4d RTM_SIMD_CALL view_look_to_lh(vector4d_arg0 position, vector4d_arg1 direction, vector4d_arg2 up) RTM_NO_EXCEPT
	{
		RTM_ASSERT(vector_is_finite3(position), "Look position must be finite");
		RTM_ASSERT(!vector_all_equal3(direction, vector_zero()) && vector_is_finite3(direction), "Look towards direction must be non-zero and finite");
		RTM_ASSERT(!vector_all_equal3(up, vector_zero()) && vector_is_finite3(up), "Look up direction must be non-zero and finite");

		// Constructs our orientation ortho-normal frame
		vector4d z_axis = vector_normalize3(direction);
		vector4d x_axis = vector_normalize3(vector_cross3(up, z_axis));
		vector4d y_axis = vector_cross3(z_axis, x_axis);	// already normalized since inputs are normalized

		// The look-to matrix converts world space inputs into the view space or the look to camera
		// To that end, we thus need the inverse of the world space camera transform
		// Because our affine matrix is ortho-normalized, we can calculate its inverse using a transpose
		// We wish for the translation part to be in the W axis, and so we would have to set the camera
		// position in the W column before we perform a 4x4 transpose and negation of the W axis
		// To avoid this unnecessary 4x4 step, we instead transpose the upper 3x3 portion (in place) and set
		// the W axis directly

		// Negate the position to invert it
		vector4d neg_position = vector_neg(position);

		scalard pos_x = vector_dot3(x_axis, neg_position);
		scalard pos_y = vector_dot3(y_axis, neg_position);
		scalard pos_z = vector_dot3(z_axis, neg_position);
		vector4d w_axis = vector_set(pos_x, pos_y, pos_z);

		// Transpose in place to invert the rotation
		RTM_MATRIXD_TRANSPOSE_3X3(x_axis, y_axis, z_axis, x_axis, y_axis, z_axis);

		return matrix3x4d{ x_axis, y_axis, z_axis, w_axis };
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns a right-handed affine 3x4 matrix representing a camera view transform.
	// A camera view transform is the inverse of a camera transform.
	// A camera transform multiplied by a point3 transforms a point3 local to the camera
	// into world space while a camera view transform multiplied by a point3 does the
	// opposite: it transforms a world space point3 into local space of the camera.
	// This is typically used before applying a projection matrix.
	// The camera transform is located at the specified position,
	// looking towards the specified direction, and using the specified up direction.
	// The camera view transform is then constructed from its inverse.
	//
	// The look to direction and up inputs do not need to be normalized.
	//
	// In this frame of reference:
	//    - X axis: left direction
	//    - Y axis: up direction
	//    - Z axis: forward/look direction
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline matrix3x4d RTM_SIMD_CALL view_look_to_rh(vector4d_arg0 position, vector4d_arg1 direction, vector4d_arg2 up) RTM_NO_EXCEPT
	{
		// A right-handed camera view transform is equivalent to a left-handed one pointing in the opposite direction
		return view_look_to_lh(position, vector_neg(direction), up);
	}

	RTM_IMPL_VERSION_NAMESPACE_END
}

RTM_IMPL_FILE_PRAGMA_POP
