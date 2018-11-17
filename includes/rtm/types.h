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
	namespace rtm_impl
	{
		union mask_converter
		{
			double dbl;
			uint64_t u64;
			float flt[2];

			constexpr mask_converter(uint64_t value) : u64(value) {}
			constexpr mask_converter(double value) : dbl(value) {}
			constexpr mask_converter(float value) : flt{value, value} {}

			constexpr operator double() const { return dbl; }
			constexpr operator float() const { return flt[0]; }
		};

		constexpr mask_converter get_mask_value(bool is_true)
		{
			return mask_converter(is_true ? uint64_t(0xFFFFFFFFFFFFFFFFull) : uint64_t(0));
		}

		constexpr double select(double mask, double if_true, double if_false)
		{
			return mask_converter(mask).u64 == 0 ? if_false : if_true;
		}

		constexpr float select(float mask, float if_true, float if_false)
		{
			return mask_converter(mask).u64 == 0 ? if_false : if_true;
		}
	}

#if defined(RTM_SSE2_INTRINSICS)
	typedef __m128 quatf;
	typedef __m128 vector4f;

	struct quatd
	{
		__m128d xy;
		__m128d zw;
	};

	struct vector4d
	{
		__m128d xy;
		__m128d zw;
	};
#elif defined(RTM_NEON_INTRINSICS)
	typedef float32x4_t quatf;
	typedef float32x4_t vector4f;

	struct alignas(16) quatd
	{
		double x;
		double y;
		double z;
		double w;
	};

	struct alignas(16) vector4d
	{
		double x;
		double y;
		double z;
		double w;
	};
#else
	struct alignas(16) quatf
	{
		float x;
		float y;
		float z;
		float w;
	};

	struct alignas(16) vector4f
	{
		float x;
		float y;
		float z;
		float w;
	};

	struct alignas(16) quatd
	{
		double x;
		double y;
		double z;
		double w;
	};

	struct alignas(16) vector4d
	{
		double x;
		double y;
		double z;
		double w;
	};
#endif

	struct qvvf
	{
		quatf		rotation;
		vector4f	translation;
		vector4f	scale;
	};

	struct qvvd
	{
		quatd		rotation;
		vector4d	translation;
		vector4d	scale;
	};

	struct matrix3x4f
	{
		vector4f	x_axis;
		vector4f	y_axis;
		vector4f	z_axis;
		vector4f	w_axis;
	};

	struct matrix3x4d
	{
		vector4d	x_axis;
		vector4d	y_axis;
		vector4d	z_axis;
		vector4d	w_axis;
	};

	//////////////////////////////////////////////////////////////////////////
	// Represents a component when mixing/shuffling/permuting vectors
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
