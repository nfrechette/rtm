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

#include <cstring>

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

	// Returns 1.0 for true and 0.0 for false
	template<typename FloatType>
	FloatType get_bool_value(bool value)
	{
		return value ? FloatType(1.0) : FloatType(0.0);
	}

	// Writes 4 bool values
	template<typename FloatType>
	void store_bool_results(bool value0, bool value1, bool value2, bool value3, FloatType* output)
	{
		output[0] = get_bool_value<FloatType>(value0);
		output[1] = get_bool_value<FloatType>(value1);
		output[2] = get_bool_value<FloatType>(value2);
		output[3] = get_bool_value<FloatType>(value3);
	}

	// Computes a function that must give the same bits on all the code paths. The function writes the 4 lanes of the output.
	//    - The vector arguments come from input0, input1, and input2, in order. mul_scalar and the set functions read
	//      their scalars from input1. all_near_equal and any_near_equal read the threshold from input2[0].
	//    - A mask result is 1.0 or 0.0 in each lane (with vector_select).
	//    - The all_* and any_* functions write the result for 4, 3, and 2 components in the lanes 0, 1, and 2.
	//    - The mask_* functions use the masks less_than(input0, input1) and less_equal(input1, input2).
	//    - The store functions write over a copy of input2.
	template<typename FloatType>
	void compute_bit_exact_function_impl(rtm_ulp_ref::bit_exact_function function, const FloatType* input0, const FloatType* input1, const FloatType* input2, FloatType* output)
	{
		using rtm_ulp_ref::bit_exact_function;
		using rtm::mix4;
		using rtm::component4;

		const auto v0 = rtm::vector_load(input0);
		const auto v1 = rtm::vector_load(input1);
		const auto v2 = rtm::vector_load(input2);
		const auto one = rtm::vector_set(FloatType(1.0));
		const auto zero = rtm::vector_set(FloatType(0.0));
		const auto mask0 = rtm::vector_less_than(v0, v1);
		const auto mask1 = rtm::vector_less_equal(v1, v2);
		const FloatType threshold = input2[0];

		switch (function)
		{
		case bit_exact_function::add:				rtm::vector_store(rtm::vector_add(v0, v1), output); break;
		case bit_exact_function::sub:				rtm::vector_store(rtm::vector_sub(v0, v1), output); break;
		case bit_exact_function::mul:				rtm::vector_store(rtm::vector_mul(v0, v1), output); break;
		case bit_exact_function::div:				rtm::vector_store(rtm::vector_div(v0, v1), output); break;
		case bit_exact_function::mul_scalar:		rtm::vector_store(rtm::vector_mul(v0, input1[0]), output); break;
		case bit_exact_function::neg:				rtm::vector_store(rtm::vector_neg(v0), output); break;
		case bit_exact_function::abs:				rtm::vector_store(rtm::vector_abs(v0), output); break;
		case bit_exact_function::min:				rtm::vector_store(rtm::vector_min(v0, v1), output); break;
		case bit_exact_function::max:				rtm::vector_store(rtm::vector_max(v0, v1), output); break;
		case bit_exact_function::clamp:				rtm::vector_store(rtm::vector_clamp(v0, v1, v2), output); break;
		case bit_exact_function::sign:				rtm::vector_store(rtm::vector_sign(v0), output); break;
		case bit_exact_function::copy_sign:			rtm::vector_store(rtm::vector_copy_sign(v0, v1), output); break;
		case bit_exact_function::floor:				rtm::vector_store(rtm::vector_floor(v0), output); break;
		case bit_exact_function::ceil:				rtm::vector_store(rtm::vector_ceil(v0), output); break;
		case bit_exact_function::round_bankers:		rtm::vector_store(rtm::vector_round_bankers(v0), output); break;
		case bit_exact_function::round_symmetric:	rtm::vector_store(rtm::vector_round_symmetric(v0), output); break;
		case bit_exact_function::fraction:			rtm::vector_store(rtm::vector_fraction(v0), output); break;
		case bit_exact_function::logical_and:		rtm::vector_store(rtm::vector_and(v0, v1), output); break;
		case bit_exact_function::logical_or:		rtm::vector_store(rtm::vector_or(v0, v1), output); break;
		case bit_exact_function::logical_xor:		rtm::vector_store(rtm::vector_xor(v0, v1), output); break;
		case bit_exact_function::equal:				rtm::vector_store(rtm::vector_select(rtm::vector_equal(v0, v1), one, zero), output); break;
		case bit_exact_function::not_equal:			rtm::vector_store(rtm::vector_select(rtm::vector_not_equal(v0, v1), one, zero), output); break;
		case bit_exact_function::less_than:			rtm::vector_store(rtm::vector_select(rtm::vector_less_than(v0, v1), one, zero), output); break;
		case bit_exact_function::less_equal:		rtm::vector_store(rtm::vector_select(rtm::vector_less_equal(v0, v1), one, zero), output); break;
		case bit_exact_function::greater_than:		rtm::vector_store(rtm::vector_select(rtm::vector_greater_than(v0, v1), one, zero), output); break;
		case bit_exact_function::greater_equal:		rtm::vector_store(rtm::vector_select(rtm::vector_greater_equal(v0, v1), one, zero), output); break;
		case bit_exact_function::finite:			rtm::vector_store(rtm::vector_select(rtm::vector_finite(v0), one, zero), output); break;
		case bit_exact_function::select:			rtm::vector_store(rtm::vector_select(mask0, v1, v2), output); break;
		case bit_exact_function::all_equal:			store_bool_results(rtm::vector_all_equal(v0, v1), rtm::vector_all_equal3(v0, v1), rtm::vector_all_equal2(v0, v1), false, output); break;
		case bit_exact_function::all_not_equal:		store_bool_results(rtm::vector_all_not_equal(v0, v1), rtm::vector_all_not_equal3(v0, v1), rtm::vector_all_not_equal2(v0, v1), false, output); break;
		case bit_exact_function::all_less_than:		store_bool_results(rtm::vector_all_less_than(v0, v1), rtm::vector_all_less_than3(v0, v1), rtm::vector_all_less_than2(v0, v1), false, output); break;
		case bit_exact_function::all_less_equal:	store_bool_results(rtm::vector_all_less_equal(v0, v1), rtm::vector_all_less_equal3(v0, v1), rtm::vector_all_less_equal2(v0, v1), false, output); break;
		case bit_exact_function::all_greater_than:	store_bool_results(rtm::vector_all_greater_than(v0, v1), rtm::vector_all_greater_than3(v0, v1), rtm::vector_all_greater_than2(v0, v1), false, output); break;
		case bit_exact_function::all_greater_equal:	store_bool_results(rtm::vector_all_greater_equal(v0, v1), rtm::vector_all_greater_equal3(v0, v1), rtm::vector_all_greater_equal2(v0, v1), false, output); break;
		case bit_exact_function::any_equal:			store_bool_results(rtm::vector_any_equal(v0, v1), rtm::vector_any_equal3(v0, v1), rtm::vector_any_equal2(v0, v1), false, output); break;
		case bit_exact_function::any_not_equal:		store_bool_results(rtm::vector_any_not_equal(v0, v1), rtm::vector_any_not_equal3(v0, v1), rtm::vector_any_not_equal2(v0, v1), false, output); break;
		case bit_exact_function::any_less_than:		store_bool_results(rtm::vector_any_less_than(v0, v1), rtm::vector_any_less_than3(v0, v1), rtm::vector_any_less_than2(v0, v1), false, output); break;
		case bit_exact_function::any_less_equal:	store_bool_results(rtm::vector_any_less_equal(v0, v1), rtm::vector_any_less_equal3(v0, v1), rtm::vector_any_less_equal2(v0, v1), false, output); break;
		case bit_exact_function::any_greater_than:	store_bool_results(rtm::vector_any_greater_than(v0, v1), rtm::vector_any_greater_than3(v0, v1), rtm::vector_any_greater_than2(v0, v1), false, output); break;
		case bit_exact_function::any_greater_equal:	store_bool_results(rtm::vector_any_greater_equal(v0, v1), rtm::vector_any_greater_equal3(v0, v1), rtm::vector_any_greater_equal2(v0, v1), false, output); break;
		case bit_exact_function::is_finite:			store_bool_results(rtm::vector_is_finite(v0), rtm::vector_is_finite3(v0), rtm::vector_is_finite2(v0), false, output); break;
		case bit_exact_function::all_near_equal:	store_bool_results(rtm::vector_all_near_equal(v0, v1, threshold), rtm::vector_all_near_equal3(v0, v1, threshold), rtm::vector_all_near_equal2(v0, v1, threshold), false, output); break;
		case bit_exact_function::any_near_equal:	store_bool_results(rtm::vector_any_near_equal(v0, v1, threshold), rtm::vector_any_near_equal3(v0, v1, threshold), rtm::vector_any_near_equal2(v0, v1, threshold), false, output); break;
		case bit_exact_function::mask_and:			rtm::vector_store(rtm::vector_select(rtm::mask_and(mask0, mask1), one, zero), output); break;
		case bit_exact_function::mask_or:			rtm::vector_store(rtm::vector_select(rtm::mask_or(mask0, mask1), one, zero), output); break;
		case bit_exact_function::mask_xor:			rtm::vector_store(rtm::vector_select(rtm::mask_xor(mask0, mask1), one, zero), output); break;
		case bit_exact_function::mask_not:			rtm::vector_store(rtm::vector_select(rtm::mask_not(mask0), one, zero), output); break;
		case bit_exact_function::mask_all_true:		store_bool_results(rtm::mask_all_true(mask0), rtm::mask_all_true3(mask0), rtm::mask_all_true2(mask0), false, output); break;
		case bit_exact_function::mask_any_true:		store_bool_results(rtm::mask_any_true(mask0), rtm::mask_any_true3(mask0), rtm::mask_any_true2(mask0), false, output); break;
		case bit_exact_function::mask_all_equal:	store_bool_results(rtm::mask_all_equal(mask0, mask1), rtm::mask_all_equal3(mask0, mask1), rtm::mask_all_equal2(mask0, mask1), false, output); break;
		case bit_exact_function::mask_any_equal:	store_bool_results(rtm::mask_any_equal(mask0, mask1), rtm::mask_any_equal3(mask0, mask1), rtm::mask_any_equal2(mask0, mask1), false, output); break;
		case bit_exact_function::mask_get:			store_bool_results(rtm::mask_get_x(mask0) != 0, rtm::mask_get_y(mask0) != 0, rtm::mask_get_z(mask0) != 0, rtm::mask_get_w(mask0) != 0, output); break;
		case bit_exact_function::load1:				rtm::vector_store(rtm::vector_load1(input0), output); break;
		case bit_exact_function::load2:				rtm::vector_store(rtm::vector_load2(input0), output); break;
		case bit_exact_function::load3:				rtm::vector_store(rtm::vector_load3(input0), output); break;
		case bit_exact_function::broadcast:			rtm::vector_store(rtm::vector_broadcast(input0), output); break;
		case bit_exact_function::store1:			std::memcpy(output, input2, sizeof(FloatType) * 4); rtm::vector_store1(v0, output); break;
		case bit_exact_function::store2:			std::memcpy(output, input2, sizeof(FloatType) * 4); rtm::vector_store2(v0, output); break;
		case bit_exact_function::store3:			std::memcpy(output, input2, sizeof(FloatType) * 4); rtm::vector_store3(v0, output); break;
		case bit_exact_function::set_components:	rtm::vector_store(rtm::vector_set_w(rtm::vector_set_z(rtm::vector_set_y(rtm::vector_set_x(v0, input1[0]), input1[1]), input1[2]), input1[3]), output); break;
		case bit_exact_function::set_component:		rtm::vector_store(rtm::vector_set_component(rtm::vector_set_component(v0, input1[1], component4::y), input1[3], component4::w), output); break;
		case bit_exact_function::get_components:
			output[0] = FloatType(rtm::scalar_cast(rtm::vector_get_x_as_scalar(v0)));
			output[1] = FloatType(rtm::scalar_cast(rtm::vector_get_y_as_scalar(v0)));
			output[2] = FloatType(rtm::scalar_cast(rtm::vector_get_z_as_scalar(v0)));
			output[3] = FloatType(rtm::scalar_cast(rtm::vector_get_w_as_scalar(v0)));
			break;
		case bit_exact_function::get_component:
			output[0] = FloatType(rtm::scalar_cast(rtm::vector_get_component_as_scalar(v0, component4::x)));
			output[1] = FloatType(rtm::scalar_cast(rtm::vector_get_component_as_scalar(v0, component4::y)));
			output[2] = FloatType(rtm::scalar_cast(rtm::vector_get_component_as_scalar(v0, component4::z)));
			output[3] = FloatType(rtm::scalar_cast(rtm::vector_get_component_as_scalar(v0, component4::w)));
			break;
		case bit_exact_function::dup_x:				rtm::vector_store(rtm::vector_dup_x(v0), output); break;
		case bit_exact_function::dup_y:				rtm::vector_store(rtm::vector_dup_y(v0), output); break;
		case bit_exact_function::dup_z:				rtm::vector_store(rtm::vector_dup_z(v0), output); break;
		case bit_exact_function::dup_w:				rtm::vector_store(rtm::vector_dup_w(v0), output); break;
		case bit_exact_function::min_max_component:
			output[0] = FloatType(rtm::scalar_cast(rtm::vector_get_min_component_as_scalar(v0)));
			output[1] = FloatType(rtm::scalar_cast(rtm::vector_get_max_component_as_scalar(v0)));
			output[2] = FloatType(0.0);
			output[3] = FloatType(0.0);
			break;
		case bit_exact_function::mix0:				rtm::vector_store(rtm::vector_mix<mix4::x, mix4::b, mix4::w, mix4::a>(v0, v1), output); break;
		case bit_exact_function::mix1:				rtm::vector_store(rtm::vector_mix<mix4::c, mix4::y, mix4::d, mix4::z>(v0, v1), output); break;
		case bit_exact_function::mix2:				rtm::vector_store(rtm::vector_mix<mix4::w, mix4::w, mix4::a, mix4::d>(v0, v1), output); break;
		case bit_exact_function::count:				break;
		}
	}

	// The matrix and transform types of a float type
	template<typename FloatType>
	struct transform_types;

	template<>
	struct transform_types<float>
	{
		using matrix3x3 = rtm::matrix3x3f;
		using matrix3x4 = rtm::matrix3x4f;
		using matrix4x4 = rtm::matrix4x4f;
	};

	template<>
	struct transform_types<double>
	{
		using matrix3x3 = rtm::matrix3x3d;
		using matrix3x4 = rtm::matrix3x4d;
		using matrix4x4 = rtm::matrix4x4d;
	};

	template<typename MatrixType, typename FloatType>
	void store_matrix_axes(const MatrixType& matrix, FloatType* output)
	{
		rtm::vector_store(matrix.x_axis, output);
		rtm::vector_store(matrix.y_axis, output + 4);
		rtm::vector_store(matrix.z_axis, output + 8);
	}

	template<typename MatrixType, typename FloatType>
	void store_matrix_axes4(const MatrixType& matrix, FloatType* output)
	{
		store_matrix_axes(matrix, output);
		rtm::vector_store(matrix.w_axis, output + 12);
	}

	// Computes a matrix or a transform function. The input has 8 vectors and the output has 4 vectors.
	//    - A matrix function reads the axes of the first matrix from the vectors 0 to 3, and the axes of the second
	//      matrix or the vector from the vectors 4 to 7. It writes the axes of the result to the output vectors.
	//    - A VQ, VQS, or VQV function reads the first transform from the vectors 0 to 3: the rotation, the translation
	//      (VQS: the translation and the scale in [w]), and the scale (VQV). It reads the second transform or the point
	//      from the vectors 4 to 7. It writes the rotation, the translation, and the scale of the result in this order.
	//    - A determinant is in the lane 0 of the output vector 0.
	template<typename FloatType>
	void compute_transform_function_impl(rtm_ulp_ref::transform_function function, const FloatType* input, FloatType* output)
	{
		using rtm_ulp_ref::transform_function;
		using matrix3x3 = typename transform_types<FloatType>::matrix3x3;
		using matrix3x4 = typename transform_types<FloatType>::matrix3x4;
		using matrix4x4 = typename transform_types<FloatType>::matrix4x4;

		const auto v0 = rtm::vector_load(input + 0);
		const auto v1 = rtm::vector_load(input + 4);
		const auto v2 = rtm::vector_load(input + 8);
		const auto v3 = rtm::vector_load(input + 12);
		const auto v4 = rtm::vector_load(input + 16);
		const auto v5 = rtm::vector_load(input + 20);
		const auto v6 = rtm::vector_load(input + 24);
		const auto v7 = rtm::vector_load(input + 28);
		const auto q0 = rtm::quat_load(input + 0);
		const auto q4 = rtm::quat_load(input + 16);

		const matrix3x3 m33_lhs = rtm::matrix_set(v0, v1, v2);
		const matrix3x3 m33_rhs = rtm::matrix_set(v4, v5, v6);
		const matrix3x4 m34_lhs = rtm::matrix_set(v0, v1, v2, v3);
		const matrix3x4 m34_rhs = rtm::matrix_set(v4, v5, v6, v7);
		const matrix4x4 m44_lhs = rtm::matrix_set(v0, v1, v2, v3);
		const matrix4x4 m44_rhs = rtm::matrix_set(v4, v5, v6, v7);

		const auto vq_lhs = rtm::vq_set(q0, v1);
		const auto vq_rhs = rtm::vq_set(q4, v5);
		const auto vqs_lhs = rtm::vqs_set(q0, v1, input[7]);
		const auto vqs_rhs = rtm::vqs_set(q4, v5, input[23]);
		const auto vqv_lhs = rtm::vqv_set(q0, v1, v2);
		const auto vqv_rhs = rtm::vqv_set(q4, v5, v6);

		switch (function)
		{
		case transform_function::matrix3x3_mul:				store_matrix_axes(rtm::matrix_mul(m33_lhs, m33_rhs), output); break;
		case transform_function::matrix3x3_mul_vector3:		rtm::vector_store(rtm::matrix_mul_vector3(v4, m33_lhs), output); break;
		case transform_function::matrix3x3_transpose:		store_matrix_axes(rtm::matrix_transpose(m33_lhs), output); break;
		case transform_function::matrix3x3_inverse:			store_matrix_axes(rtm::matrix_inverse(m33_lhs), output); break;
		case transform_function::matrix3x3_determinant:		output[0] = FloatType(rtm::scalar_cast(rtm::matrix_determinant(m33_lhs))); break;
		case transform_function::matrix3x4_mul:				store_matrix_axes4(rtm::matrix_mul(m34_lhs, m34_rhs), output); break;
		case transform_function::matrix3x4_mul_point3:		rtm::vector_store(rtm::matrix_mul_point3(v4, m34_lhs), output); break;
		case transform_function::matrix3x4_mul_vector3:		rtm::vector_store(rtm::matrix_mul_vector3(v4, m34_lhs), output); break;
		case transform_function::matrix3x4_transpose:		store_matrix_axes(rtm::matrix_transpose(m34_lhs), output); break;
		case transform_function::matrix3x4_inverse:			store_matrix_axes4(rtm::matrix_inverse(m34_lhs), output); break;
		case transform_function::matrix3x4_determinant:		output[0] = FloatType(rtm::scalar_cast(rtm::matrix_determinant(m34_lhs))); break;
		case transform_function::matrix4x4_mul:				store_matrix_axes4(rtm::matrix_mul(m44_lhs, m44_rhs), output); break;
		case transform_function::matrix4x4_mul_vector:		rtm::vector_store(rtm::matrix_mul_vector(v4, m44_lhs), output); break;
		case transform_function::matrix4x4_transpose:		store_matrix_axes4(rtm::matrix_transpose(m44_lhs), output); break;
		case transform_function::matrix4x4_inverse:			store_matrix_axes4(rtm::matrix_inverse(m44_lhs), output); break;
		case transform_function::matrix4x4_determinant:		output[0] = FloatType(rtm::scalar_cast(rtm::matrix_determinant(m44_lhs))); break;
		case transform_function::vq_mul:					{ const auto r = rtm::vq_mul(vq_lhs, vq_rhs); rtm::quat_store(r.rotation, output); rtm::vector_store(r.translation, output + 4); break; }
		case transform_function::vq_mul_point3:				rtm::vector_store(rtm::vq_mul_point3(v4, vq_lhs), output); break;
		case transform_function::vq_inverse:				{ const auto r = rtm::vq_inverse(vq_lhs); rtm::quat_store(r.rotation, output); rtm::vector_store(r.translation, output + 4); break; }
		case transform_function::vqs_mul:					{ const auto r = rtm::vqs_mul(vqs_lhs, vqs_rhs); rtm::quat_store(r.rotation, output); rtm::vector_store(r.translation_scale, output + 4); break; }
		case transform_function::vqs_mul_point3:			rtm::vector_store(rtm::vqs_mul_point3(v4, vqs_lhs), output); break;
		case transform_function::vqs_inverse:				{ const auto r = rtm::vqs_inverse(vqs_lhs); rtm::quat_store(r.rotation, output); rtm::vector_store(r.translation_scale, output + 4); break; }
		case transform_function::vqv_mul:					{ const auto r = rtm::vqv_mul(vqv_lhs, vqv_rhs); rtm::quat_store(r.rotation, output); rtm::vector_store(r.translation, output + 4); rtm::vector_store(r.scale, output + 8); break; }
		case transform_function::vqv_mul_point3:			rtm::vector_store(rtm::vqv_mul_point3(v4, vqv_lhs), output); break;
		case transform_function::vqv_mul_inverse:			{ const auto r = rtm::vqv_mul_inverse(vqv_lhs, vqv_rhs); rtm::quat_store(r.rotation, output); rtm::vector_store(r.translation, output + 4); rtm::vector_store(r.scale, output + 8); break; }
		case transform_function::vqv_inverse_mul:			{ const auto r = rtm::vqv_inverse_mul(vqv_lhs, vqv_rhs); rtm::quat_store(r.rotation, output); rtm::vector_store(r.translation, output + 4); rtm::vector_store(r.scale, output + 8); break; }
		case transform_function::vqv_inverse_mul_point3:	rtm::vector_store(rtm::vqv_inverse_mul_point3(v4, vqv_lhs), output); break;
		case transform_function::count:						break;
		}
	}
}
