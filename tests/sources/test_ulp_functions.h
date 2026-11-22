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

// test_ulp.cpp and test_ulp_reference.cpp both include this file after the RTM headers.
// In test_ulp_reference.cpp, the macro "rtm" changes the namespace to rtm_ref: thus the same code
// calls the scalar code path there and the code path of the build in test_ulp.cpp.
// The function is in an unnamed namespace: each file has its own copy, and the linker does not merge them.

#include "test_ulp_reference.h"

namespace
{
	// Writes a scalar result to the 4 lanes of the output
	template<typename FloatType>
	void store_scalar_result(FloatType value, FloatType* output)
	{
		output[0] = value;
		output[1] = value;
		output[2] = value;
		output[3] = value;
	}

	// The packing functions exist only for float32
	inline void compute_float32_only_function(rtm_ulp_ref::ulp_function function, const float* input0, float* output)
	{
		using rtm_ulp_ref::ulp_function;

		if (function == ulp_function::quat_ensure_positive_w)
			rtm::quat_store(rtm::quat_ensure_positive_w(rtm::quat_load(input0)), output);
		else if (function == ulp_function::quat_from_positive_w)
			rtm::quat_store(rtm::quat_from_positive_w(rtm::vector_load(input0)), output);
	}

	inline void compute_float32_only_function(rtm_ulp_ref::ulp_function, const double*, double* output)
	{
		store_scalar_result(0.0, output);
	}

	// Computes a function on 4 values of each input.
	//    - The element-wise functions compute each lane. atan2 reads y from input0 and x from input1.
	//    - The vector functions read their vector arguments from input0, input1, and input2, in order.
	//      lerp reads alpha from input2. A function with a scalar result writes it to the 4 lanes.
	//    - The quaternion functions read their quaternion arguments from input0 and input1.
	//      quat_mul_vector3 reads the vector from input0 and the quaternion from input1.
	//      quat_lerp and quat_slerp read alpha from input2[0].
	//      quat_from_euler reads the pitch, the yaw, and the roll from input0[0], input0[1], and input0[2].
	//      quat_from_matrix reads the X, Y, and Z axes of the matrix from input0, input1, and input2.
	template<typename FloatType>
	void compute_vector_function_impl(rtm_ulp_ref::ulp_function function, const FloatType* input0, const FloatType* input1, const FloatType* input2, FloatType* output)
	{
		using rtm_ulp_ref::ulp_function;

		const auto v0 = rtm::vector_load(input0);
		const auto v1 = rtm::vector_load(input1);
		const auto v2 = rtm::vector_load(input2);
		const auto q0 = rtm::quat_load(input0);
		const auto q1 = rtm::quat_load(input1);

		switch (function)
		{
		case ulp_function::exp:					rtm::vector_store(rtm::vector_exp(v0), output); break;
		case ulp_function::log:					rtm::vector_store(rtm::vector_log(v0), output); break;
		case ulp_function::sqrt:				rtm::vector_store(rtm::vector_sqrt(v0), output); break;
		case ulp_function::reciprocal:			rtm::vector_store(rtm::vector_reciprocal(v0), output); break;
		case ulp_function::sqrt_reciprocal:		rtm::vector_store(rtm::vector_sqrt_reciprocal(v0), output); break;
		case ulp_function::sin:					rtm::vector_store(rtm::vector_sin(v0), output); break;
		case ulp_function::cos:					rtm::vector_store(rtm::vector_cos(v0), output); break;
		case ulp_function::tan:					rtm::vector_store(rtm::vector_tan(v0), output); break;
		case ulp_function::asin:				rtm::vector_store(rtm::vector_asin(v0), output); break;
		case ulp_function::acos:				rtm::vector_store(rtm::vector_acos(v0), output); break;
		case ulp_function::atan:				rtm::vector_store(rtm::vector_atan(v0), output); break;
		case ulp_function::atan2:				rtm::vector_store(rtm::vector_atan2(v0, v1), output); break;

		case ulp_function::dot:					rtm::vector_store(rtm::vector_dot_as_vector(v0, v1), output); break;
		case ulp_function::dot3:				rtm::vector_store(rtm::vector_dot3_as_vector(v0, v1), output); break;
		case ulp_function::dot2:				rtm::vector_store(rtm::vector_dot2_as_vector(v0, v1), output); break;
		case ulp_function::length:				store_scalar_result(FloatType(rtm::scalar_cast(rtm::vector_length_as_scalar(v0))), output); break;
		case ulp_function::length3:				store_scalar_result(FloatType(rtm::scalar_cast(rtm::vector_length3_as_scalar(v0))), output); break;
		case ulp_function::length_squared:		rtm::vector_store(rtm::vector_length_squared_as_vector(v0), output); break;
		case ulp_function::length_squared3:		rtm::vector_store(rtm::vector_length_squared3_as_vector(v0), output); break;
		case ulp_function::length_squared2:		rtm::vector_store(rtm::vector_length_squared2_as_vector(v0), output); break;
		case ulp_function::length_reciprocal:	store_scalar_result(FloatType(rtm::scalar_cast(rtm::vector_length_reciprocal_as_scalar(v0))), output); break;
		case ulp_function::length_reciprocal3:	store_scalar_result(FloatType(rtm::scalar_cast(rtm::vector_length_reciprocal3_as_scalar(v0))), output); break;
		case ulp_function::length_reciprocal2:	store_scalar_result(FloatType(rtm::scalar_cast(rtm::vector_length_reciprocal2_as_scalar(v0))), output); break;
		case ulp_function::normalize:			rtm::vector_store(rtm::vector_normalize(v0), output); break;
		case ulp_function::normalize3:			rtm::vector_store(rtm::vector_normalize3(v0), output); break;
		case ulp_function::normalize2:			rtm::vector_store(rtm::vector_normalize2(v0), output); break;
		case ulp_function::distance3:			store_scalar_result(FloatType(rtm::scalar_cast(rtm::vector_distance3_as_scalar(v0, v1))), output); break;
		case ulp_function::cross3:				rtm::vector_store(rtm::vector_cross3(v0, v1), output); break;
		case ulp_function::mul_add:				rtm::vector_store(rtm::vector_mul_add(v0, v1, v2), output); break;
		case ulp_function::neg_mul_sub:			rtm::vector_store(rtm::vector_neg_mul_sub(v0, v1, v2), output); break;
		case ulp_function::lerp:				rtm::vector_store(rtm::vector_lerp(v0, v1, v2), output); break;

		case ulp_function::quat_mul:			rtm::quat_store(rtm::quat_mul(q0, q1), output); break;
		case ulp_function::quat_mul_vector3:	rtm::vector_store(rtm::quat_mul_vector3(v0, q1), output); break;
		case ulp_function::quat_normalize:		rtm::quat_store(rtm::quat_normalize(q0), output); break;
		case ulp_function::quat_dot:			store_scalar_result(FloatType(rtm::scalar_cast(rtm::quat_dot_as_scalar(q0, q1))), output); break;
		case ulp_function::quat_length:			store_scalar_result(FloatType(rtm::scalar_cast(rtm::quat_length_as_scalar(q0))), output); break;
		case ulp_function::quat_length_squared:	store_scalar_result(FloatType(rtm::scalar_cast(rtm::quat_length_squared_as_scalar(q0))), output); break;
		case ulp_function::quat_length_reciprocal:	store_scalar_result(FloatType(rtm::scalar_cast(rtm::quat_length_reciprocal_as_scalar(q0))), output); break;
		case ulp_function::quat_lerp:			rtm::quat_store(rtm::quat_lerp(q0, q1, input2[0]), output); break;
		case ulp_function::quat_slerp:			rtm::quat_store(rtm::quat_slerp(q0, q1, input2[0]), output); break;
		case ulp_function::quat_from_euler:		rtm::quat_store(rtm::quat_from_euler(input0[0], input0[1], input0[2]), output); break;
		case ulp_function::quat_get_angle:		store_scalar_result(FloatType(rtm::quat_get_angle(q0)), output); break;
		case ulp_function::quat_rotation_log:	rtm::quat_store(rtm::quat_rotation_log(q0), output); break;
		case ulp_function::quat_rotation_exp:	rtm::quat_store(rtm::quat_rotation_exp(q0), output); break;
		case ulp_function::quat_from_matrix:	rtm::quat_store(rtm::quat_from_matrix(rtm::matrix_set(v0, v1, v2)), output); break;
		case ulp_function::quat_ensure_positive_w:
		case ulp_function::quat_from_positive_w:
			compute_float32_only_function(function, input0, output);
			break;
		}
	}
}
