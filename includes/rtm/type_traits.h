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

#include "rtm/types.h"

namespace rtm
{
	//////////////////////////////////////////////////////////////////////////
	// Returns the proper vector4 type for a floating point type.
	//////////////////////////////////////////////////////////////////////////
	template<typename FloatType>
	struct vector4_type {};
	template<> struct vector4_type<float> { using type = vector4f; };
	template<> struct vector4_type<double> { using type = vector4d; };

	//////////////////////////////////////////////////////////////////////////
	// Returns the proper quaternion type for a floating point type.
	//////////////////////////////////////////////////////////////////////////
	template<typename FloatType>
	struct quat_type {};
	template<> struct quat_type<float> { using type = quatf; };
	template<> struct quat_type<double> { using type = quatd; };

	//////////////////////////////////////////////////////////////////////////
	// Returns the proper QVV type for a floating point type.
	//////////////////////////////////////////////////////////////////////////
	template<typename FloatType>
	struct qvv_type {};
	template<> struct qvv_type<float> { using type = qvvf; };
	template<> struct qvv_type<double> { using type = qvvd; };

	//////////////////////////////////////////////////////////////////////////
	// Returns the proper matrix3x3 type for a floating point type.
	//////////////////////////////////////////////////////////////////////////
	template<typename FloatType>
	struct matrix3x3_type {};
	template<> struct matrix3x3_type<float> { using type = matrix3x3f; };
	template<> struct matrix3x3_type<double> { using type = matrix3x3d; };

	//////////////////////////////////////////////////////////////////////////
	// Returns the proper matrix3x4 type for a floating point type.
	//////////////////////////////////////////////////////////////////////////
	template<typename FloatType>
	struct matrix3x4_type {};
	template<> struct matrix3x4_type<float> { using type = matrix3x4f; };
	template<> struct matrix3x4_type<double> { using type = matrix3x4d; };

	//////////////////////////////////////////////////////////////////////////
	// Returns the proper matrix4x4 type for a floating point type.
	//////////////////////////////////////////////////////////////////////////
	template<typename FloatType>
	struct matrix4x4_type {};
	template<> struct matrix4x4_type<float> { using type = matrix4x4f; };
	template<> struct matrix4x4_type<double> { using type = matrix4x4d; };

	//////////////////////////////////////////////////////////////////////////
	// Returns the proper angle type for a floating point type.
	//////////////////////////////////////////////////////////////////////////
	template<typename FloatType>
	struct angle_type {};
	template<> struct angle_type<float> { using type = anglef; };
	template<> struct angle_type<double> { using type = angled; };
}
