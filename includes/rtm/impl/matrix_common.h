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

namespace rtm
{
	//////////////////////////////////////////////////////////////////////////
	// Sets all 4 axes and creates a 3x4 affine matrix.
	//////////////////////////////////////////////////////////////////////////
	inline matrix3x4f RTM_SIMD_CALL matrix_set(vector4f_arg0 x_axis, vector4f_arg1 y_axis, vector4f_arg2 z_axis, vector4f_arg3 w_axis) RTM_NO_EXCEPT
	{
		RTM_ASSERT(vector_get_w(x_axis) == 0.0f, "X axis does not have a W component == 0.0");
		RTM_ASSERT(vector_get_w(y_axis) == 0.0f, "Y axis does not have a W component == 0.0");
		RTM_ASSERT(vector_get_w(z_axis) == 0.0f, "Z axis does not have a W component == 0.0");
		RTM_ASSERT(vector_get_w(w_axis) == 1.0f, "W axis does not have a W component == 1.0");
		return matrix3x4f{ x_axis, y_axis, z_axis, w_axis };
	}

	//////////////////////////////////////////////////////////////////////////
	// Sets all 4 axes and creates a 3x4 affine matrix.
	//////////////////////////////////////////////////////////////////////////
	inline matrix3x4d RTM_SIMD_CALL matrix_set(const vector4d& x_axis, const vector4d& y_axis, const vector4d& z_axis, const vector4d& w_axis) RTM_NO_EXCEPT
	{
		RTM_ASSERT(vector_get_w(x_axis) == 0.0, "X axis does not have a W component == 0.0");
		RTM_ASSERT(vector_get_w(y_axis) == 0.0, "Y axis does not have a W component == 0.0");
		RTM_ASSERT(vector_get_w(z_axis) == 0.0, "Z axis does not have a W component == 0.0");
		RTM_ASSERT(vector_get_w(w_axis) == 1.0, "W axis does not have a W component == 1.0");
		return matrix3x4d{ x_axis, y_axis, z_axis, w_axis };
	}

	namespace rtm_impl
	{
		//////////////////////////////////////////////////////////////////////////
		// Various matrix constants
		//////////////////////////////////////////////////////////////////////////
		enum class matrix_constants
		{
			identity
		};

		//////////////////////////////////////////////////////////////////////////
		// This is a helper struct to allow a single consistent API between
		// various matrix types when the semantics are identical but the return
		// type differs. Implicit coercion is used to return the desired value
		// at the call site.
		//////////////////////////////////////////////////////////////////////////
		template<matrix_constants constant>
		struct matrix_constant
		{
			inline RTM_SIMD_CALL operator matrix3x4d() RTM_NO_EXCEPT
			{
				switch (constant)
				{
				case matrix_constants::identity:
				default:
					return matrix3x4d{ vector_set(1.0, 0.0, 0.0, 0.0), vector_set(0.0, 1.0, 0.0, 0.0), vector_set(0.0, 0.0, 1.0, 0.0), vector_set(0.0, 0.0, 0.0, 1.0) };
				}
			}

			inline RTM_SIMD_CALL operator matrix3x4f() RTM_NO_EXCEPT
			{
				switch (constant)
				{
				case matrix_constants::identity:
				default:
					return matrix3x4f{ vector_set(1.0f, 0.0f, 0.0f, 0.0f), vector_set(0.0f, 1.0f, 0.0f, 0.0f), vector_set(0.0f, 0.0f, 1.0f, 0.0f), vector_set(0.0f, 0.0f, 0.0f, 1.0f) };
				}
			}
		};
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the identity matrix.
	//////////////////////////////////////////////////////////////////////////
	inline rtm_impl::matrix_constant<rtm_impl::matrix_constants::identity> RTM_SIMD_CALL matrix_identity() RTM_NO_EXCEPT
	{
		return rtm_impl::matrix_constant<rtm_impl::matrix_constants::identity>();
	}
}