#pragma once

////////////////////////////////////////////////////////////////////////////////
// The MIT License (MIT)
//
// Copyright (c) 2017 Nicholas Frechette & Animation Compression Library contributors
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

#include <cstdint>

namespace rtm
{
#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// A quaternion (4D complex number) where the imaginary part is the W component.
	// It accurately represents a 3D rotation with no gimbal lock as long as it is kept normalized.
	//////////////////////////////////////////////////////////////////////////
	using quatf = __m128;

	//////////////////////////////////////////////////////////////////////////
	// A quaternion (4D complex number) where the imaginary part is the W component.
	// It accurately represents a 3D rotation with no gimbal lock as long as it is kept normalized.
	//////////////////////////////////////////////////////////////////////////
	struct quatd
	{
		__m128d xy;
		__m128d zw;
	};

	//////////////////////////////////////////////////////////////////////////
	// A 4D vector.
	//////////////////////////////////////////////////////////////////////////
	using vector4f = __m128;

	//////////////////////////////////////////////////////////////////////////
	// A 4D vector.
	//////////////////////////////////////////////////////////////////////////
	struct vector4d
	{
		__m128d xy;
		__m128d zw;
	};
#elif defined(RTM_NEON_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// A quaternion (4D complex number) where the imaginary part is the W component.
	// It accurately represents a 3D rotation with no gimbal lock as long as it is kept normalized.
	//////////////////////////////////////////////////////////////////////////
	using quatf = float32x4_t;

	//////////////////////////////////////////////////////////////////////////
	// A quaternion (4D complex number) where the imaginary part is the W component.
	// It accurately represents a 3D rotation with no gimbal lock as long as it is kept normalized.
	//////////////////////////////////////////////////////////////////////////
	struct alignas(16) quatd
	{
		double x;
		double y;
		double z;
		double w;
	};

	//////////////////////////////////////////////////////////////////////////
	// A 4D vector.
	//////////////////////////////////////////////////////////////////////////
	using vector4f = float32x4_t;

	//////////////////////////////////////////////////////////////////////////
	// A 4D vector.
	//////////////////////////////////////////////////////////////////////////
	struct alignas(16) vector4d
	{
		double x;
		double y;
		double z;
		double w;
	};
#else
	//////////////////////////////////////////////////////////////////////////
	// A quaternion (4D complex number) where the imaginary part is the W component.
	// It accurately represents a 3D rotation with no gimbal lock as long as it is kept normalized.
	//////////////////////////////////////////////////////////////////////////
	struct alignas(16) quatf
	{
		float x;
		float y;
		float z;
		float w;
	};

	//////////////////////////////////////////////////////////////////////////
	// A quaternion (4D complex number) where the imaginary part is the W component.
	// It accurately represents a 3D rotation with no gimbal lock as long as it is kept normalized.
	//////////////////////////////////////////////////////////////////////////
	struct alignas(16) quatd
	{
		double x;
		double y;
		double z;
		double w;
	};

	//////////////////////////////////////////////////////////////////////////
	// A 4D vector.
	//////////////////////////////////////////////////////////////////////////
	struct alignas(16) vector4f
	{
		float x;
		float y;
		float z;
		float w;
	};

	//////////////////////////////////////////////////////////////////////////
	// A 4D vector.
	//////////////////////////////////////////////////////////////////////////
	struct alignas(16) vector4d
	{
		double x;
		double y;
		double z;
		double w;
	};
#endif

	//////////////////////////////////////////////////////////////////////////
	// A QVV transform represents a 3D rotation (quaternion), 3D translation (vector), and 3D scale (vector).
	// It properly handles positive scaling but negative scaling is a bit more problematic.
	// A best effort is made by converting the quaternion to a matrix during those operations.
	// If scale fidelity is important, consider using an affine matrix 3x4 instead.
	//////////////////////////////////////////////////////////////////////////
	struct qvvf
	{
		quatf		rotation;
		vector4f	translation;
		vector4f	scale;
	};

	//////////////////////////////////////////////////////////////////////////
	// A QVV transform represents a 3D rotation (quaternion), 3D translation (vector), and 3D scale (vector).
	// It properly handles positive scaling but negative scaling is a bit more problematic.
	// A best effort is made by converting the quaternion to a matrix during those operations.
	// If scale fidelity is important, consider using an affine matrix 3x4 instead.
	//////////////////////////////////////////////////////////////////////////
	struct qvvd
	{
		quatd		rotation;
		vector4d	translation;
		vector4d	scale;
	};

	//////////////////////////////////////////////////////////////////////////
	// An 3x4 affine matrix represents a 3D rotation, 3D translation, and 3D scale.
	// It properly deals with skew/shear when present but once scale with mirroring is combined,
	// it cannot be safely extracted back.
	//
	// Affine matrices have their last row always equal to [0, 0, 0, 1] which is why it is 3x4.
	//
	// Left handed coordinate system:
	// X axis == forward
	// Y axis == right
	// Z axis == up
	//////////////////////////////////////////////////////////////////////////
	struct matrix3x4f
	{
		vector4f	x_axis;
		vector4f	y_axis;
		vector4f	z_axis;
		vector4f	w_axis;
	};

	//////////////////////////////////////////////////////////////////////////
	// An 3x4 affine matrix represents a 3D rotation, 3D translation, and 3D scale.
	// It properly deals with skew/shear when present but once scale with mirroring is combined,
	// it cannot be safely extracted back.
	//
	// Affine matrices have their last row always equal to [0, 0, 0, 1] which is why it is 3x4.
	//
	// Left handed coordinate system:
	// X axis == forward
	// Y axis == right
	// Z axis == up
	//////////////////////////////////////////////////////////////////////////
	struct matrix3x4d
	{
		vector4d	x_axis;
		vector4d	y_axis;
		vector4d	z_axis;
		vector4d	w_axis;
	};

	//////////////////////////////////////////////////////////////////////////
	// A generic 4x4 matrix.
	//////////////////////////////////////////////////////////////////////////
	struct matrix4x4f
	{
		vector4f	x_axis;
		vector4f	y_axis;
		vector4f	z_axis;
		vector4f	w_axis;
	};

	//////////////////////////////////////////////////////////////////////////
	// A generic 4x4 matrix.
	//////////////////////////////////////////////////////////////////////////
	struct matrix4x4d
	{
		vector4d	x_axis;
		vector4d	y_axis;
		vector4d	z_axis;
		vector4d	w_axis;
	};

	//////////////////////////////////////////////////////////////////////////
	// Represents a component when mixing/shuffling/permuting vectors.
	// [xyzw] are used to refer to the first input while [abcd] refer to the second input.
	//////////////////////////////////////////////////////////////////////////
	enum class mix4
	{
		x = 0,
		y = 1,
		z = 2,
		w = 3,

		a = 4,
		b = 5,
		c = 6,
		d = 7,
	};

	//////////////////////////////////////////////////////////////////////////
	// Represents an axis in 4D.
	//////////////////////////////////////////////////////////////////////////
	enum class axis4
	{
		x = 0,
		y = 1,
		z = 2,
		w = 3,
	};



	//////////////////////////////////////////////////////////////////////////
	// Type traits
	//////////////////////////////////////////////////////////////////////////


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
	// Register passing typedefs
	//////////////////////////////////////////////////////////////////////////



#if defined(RTM_USE_VECTORCALL)
	// On x64 with __vectorcall, the first 6x vector4 arguments can be passed by value in a register, everything else afterwards is passed by const&
	using vector4f_arg0 = const vector4f;
	using vector4f_arg1 = const vector4f;
	using vector4f_arg2 = const vector4f;
	using vector4f_arg3 = const vector4f;
	using vector4f_arg4 = const vector4f;
	using vector4f_arg5 = const vector4f;
	using vector4f_argn = const vector4f&;

	using quatf_arg0 = const quatf;
	using quatf_arg1 = const quatf;
	using quatf_arg2 = const quatf;
	using quatf_arg3 = const quatf;
	using quatf_arg4 = const quatf;
	using quatf_arg5 = const quatf;
	using quatf_argn = const quatf&;

	// With __vectorcall, vector aggregates are also passed by register
	using qvvf_arg0 = const qvvf;
	using qvvf_arg1 = const qvvf;
	using qvvf_argn = const qvvf&;

	using matrix3x4f_arg0 = const matrix3x4f;
	using matrix3x4f_argn = const matrix3x4f&;
#elif defined(RTM_NEON_INTRINSICS)
	// On ARM NEON, the first 4x vector4 arguments can be passed by value in a register, everything else afterwards is passed by const&
	using vector4f_arg0 = const vector4f;
	using vector4f_arg1 = const vector4f;
	using vector4f_arg2 = const vector4f;
	using vector4f_arg3 = const vector4f;
	using vector4f_arg4 = const vector4f&;
	using vector4f_arg5 = const vector4f&;
	using vector4f_argn = const vector4f&;

	using quatf_arg0 = const quatf;
	using quatf_arg1 = const quatf;
	using quatf_arg2 = const quatf;
	using quatf_arg3 = const quatf;
	using quatf_arg4 = const quatf&;
	using quatf_arg5 = const quatf&;
	using quatf_argn = const quatf&;

	using qvvf_arg0 = const qvvf&;
	using qvvf_arg1 = const qvvf&;
	using qvvf_argn = const qvvf&;

	using matrix3x4f_arg0 = const matrix3x4f&;
	using matrix3x4f_argn = const matrix3x4f&;
#else
	// On every other platform, everything is passed by const&
	using vector4f_arg0 = const vector4f&;
	using vector4f_arg1 = const vector4f&;
	using vector4f_arg2 = const vector4f&;
	using vector4f_arg3 = const vector4f&;
	using vector4f_arg4 = const vector4f&;
	using vector4f_arg5 = const vector4f&;
	using vector4f_argn = const vector4f&;

	using quatf_arg0 = const quatf&;
	using quatf_arg1 = const quatf&;
	using quatf_arg2 = const quatf&;
	using quatf_arg3 = const quatf&;
	using quatf_arg4 = const quatf&;
	using quatf_arg5 = const quatf&;
	using quatf_argn = const quatf&;

	using qvvf_arg0 = const qvvf&;
	using qvvf_arg1 = const qvvf&;
	using qvvf_argn = const qvvf&;

	using matrix3x4f_arg0 = const matrix3x4f&;
	using matrix3x4f_argn = const matrix3x4f&;
#endif
}
