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

	// The functions that must give the same bits on all the code paths (see compute_bit_exact_function_impl)
	enum class bit_exact_function
	{
		add, sub, mul, div, mul_scalar, neg, abs, min, max, clamp, sign, copy_sign,
		floor, ceil, round_bankers, round_symmetric, fraction,
		logical_and, logical_or, logical_xor,
		equal, not_equal, less_than, less_equal, greater_than, greater_equal, finite, select,
		all_equal, all_not_equal, all_less_than, all_less_equal, all_greater_than, all_greater_equal,
		any_equal, any_not_equal, any_less_than, any_less_equal, any_greater_than, any_greater_equal,
		is_finite, all_near_equal, any_near_equal,
		mask_and, mask_or, mask_xor, mask_not, mask_all_true, mask_any_true, mask_all_equal, mask_any_equal, mask_get,
		load1, load2, load3, broadcast, store1, store2, store3,
		set_components, set_component, get_components, get_component, dup_x, dup_y, dup_z, dup_w, min_max_component,
		mix0, mix1, mix2,
		count,
	};

	// Computes the function with the scalar code path. See compute_bit_exact_function_impl for the inputs and the output.
	void compute_bit_exact_function(bit_exact_function function, const float* input0, const float* input1, const float* input2, float* output);
	void compute_bit_exact_function(bit_exact_function function, const double* input0, const double* input1, const double* input2, double* output);

	// The matrix and transform functions (see compute_transform_function_impl)
	enum class transform_function
	{
		matrix3x3_mul, matrix3x3_mul_vector3, matrix3x3_transpose, matrix3x3_inverse, matrix3x3_determinant,
		matrix3x4_mul, matrix3x4_mul_point3, matrix3x4_mul_vector3, matrix3x4_transpose, matrix3x4_inverse, matrix3x4_determinant,
		matrix4x4_mul, matrix4x4_mul_vector, matrix4x4_transpose, matrix4x4_inverse, matrix4x4_determinant,
		vq_mul, vq_mul_point3, vq_inverse,
		vqs_mul, vqs_mul_point3, vqs_inverse,
		vqv_mul, vqv_mul_point3, vqv_mul_inverse, vqv_inverse_mul, vqv_inverse_mul_point3,
		count,
	};

	// Computes the function with the scalar code path. The input has 8 vectors (32 values) and the output has 4 vectors (16 values).
	void compute_transform_function(transform_function function, const float* input, float* output);
	void compute_transform_function(transform_function function, const double* input, double* output);

	void quat_normalize_deterministic(const float* input, float* output);
	void quat_normalize_deterministic(const double* input, double* output);

	// Returns true if the scalar code path computes with more precision than the float type (FLT_EVAL_METHOD is not 0).
	// For example, GCC uses the x87 registers for 32-bit x86 and does not round each operation to float32 or float64.
	bool has_excess_precision();
}
