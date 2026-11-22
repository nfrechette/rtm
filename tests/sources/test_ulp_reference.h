#pragma once

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

// These functions run the scalar code path of RTM (RTM_NO_INTRINSICS).
// test_ulp_reference.cpp defines them. test_ulp.cpp compares the code path of the build with them.
//
// The two files see different RTM types (for example, vector4f is __m128 in one file and a struct
// in the other). Thus the functions use only arrays of float and double.
// Each function reads 4 values from each input and writes 4 values to the output.

namespace rtm_ulp_ref
{
	// The functions that the tests measure
	enum class ulp_function
	{
		// Element-wise functions with one input (atan2 has two)
		exp,
		log,
		sqrt,
		reciprocal,
		sqrt_reciprocal,
		sin,
		cos,
		tan,
		asin,
		acos,
		atan,
		atan2,

		// Vector functions
		dot,
		dot3,
		dot2,
		length,
		length3,
		length_squared,
		length_squared3,
		length_squared2,
		length_reciprocal,
		length_reciprocal3,
		length_reciprocal2,
		normalize,
		normalize3,
		normalize2,
		distance3,
		cross3,
		mul_add,
		neg_mul_sub,
		lerp,

		// Quaternion functions
		quat_mul,
		quat_mul_vector3,
		quat_normalize,
		quat_dot,
		quat_length,
		quat_length_squared,
		quat_length_reciprocal,
		quat_lerp,
		quat_slerp,
		quat_from_euler,
		quat_get_angle,
		quat_rotation_log,
		quat_rotation_exp,
		quat_from_matrix,
		quat_ensure_positive_w,		// float32 only
		quat_from_positive_w,		// float32 only
	};

	// Computes the function with the scalar code path. See compute_vector_function_impl for the inputs and the output.
	void compute_vector_function(ulp_function function, const float* input0, const float* input1, const float* input2, float* output);
	void compute_vector_function(ulp_function function, const double* input0, const double* input1, const double* input2, double* output);

	void quat_normalize_deterministic(const float* input, float* output);
	void quat_normalize_deterministic(const double* input, double* output);

	// Returns true if the scalar code path computes with more precision than the float type (FLT_EVAL_METHOD is not 0).
	// For example, GCC uses the x87 registers for 32-bit x86 and does not round each operation to float32 or float64.
	bool has_excess_precision();
}
