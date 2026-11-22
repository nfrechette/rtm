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

#include "catch2.impl.h"

#include <rtm/matrix3x3d.h>
#include <rtm/mask4d.h>
#include <rtm/mask4f.h>
#include <rtm/matrix3x3f.h>
#include <rtm/quatd.h>
#include <rtm/quatf.h>
#include <rtm/scalard.h>
#include <rtm/scalarf.h>
#include <rtm/vector4d.h>
#include <rtm/vector4f.h>
#include <rtm/packing/quatf.h>

#include "test_ulp_functions.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iterator>
#include <limits>
#include <random>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

using namespace rtm;

// These tests measure the error of a function against a reference with more precision.
// The tests use one of two units:
//    - ulp (units in the last place): the number of representable values between the result and the
//      correctly rounded value of the function. A correctly rounded result has an error of 0 ulp.
//    - scaled ulp: the error in ulp of max(|exact result|, scale). The scale of a function is the size of the
//      values that the function adds or subtracts. When the terms cancel, the error in ulp of the result is
//      very large, but the error is small in comparison with the terms. For example:
//         - The trigonometric functions and the functions with a unit result (for example a normalized vector)
//           use a scale of 1.0: their error is absolute below 1.0. RTM computes the trigonometric functions with
//           polynomial approximations. Near a zero of the function (for example sin(pi)), the error in ulp is
//           very large, but the absolute error is not.
//         - A dot product uses sum(|lhs[i] * rhs[i]|).
//         - A length uses the length: the terms do not cancel.
//
// The tests also compare the code path of the build (SSE2, AVX, NEON, WASM, or scalar) with the
// scalar code path. test_ulp_reference.cpp compiles the scalar code path in the same executable.
// Other functions of the API that compute the same value (for example the scalar functions, and the
// _as_scalar variants) must give the same result as the vector function.
//
// The inputs are edge values and random values. The random values are the same on all platforms:
// std::mt19937 gives the same sequence everywhere, and the tests do not use the std distributions.
//
// The float64 trigonometric functions call the C library (std::sin, std::cos, and so on).
// Thus the tests measure only the float32 trigonometric functions.

namespace
{
	using rtm_ulp_ref::ulp_function;

	// The number of random input values for each function and each float type.
	// A call of a function reads 4 values of each input.
	constexpr uint32_t k_num_random_inputs = 1 << 16;

	// The largest number of other functions of the API that compute the same value
	constexpr uint32_t k_max_num_alternatives = 2;

	constexpr double k_pi = 3.14159265358979323846;

	// The unit of the error of a function
	enum class error_unit
	{
		ulp,			// The error in ulp
		scaled_ulp,		// The error in ulp of max(|exact result|, scale)
	};

	const char* get_function_name(ulp_function function)
	{
		switch (function)
		{
		case ulp_function::exp:						return "exp";
		case ulp_function::log:						return "log";
		case ulp_function::sqrt:					return "sqrt";
		case ulp_function::reciprocal:				return "reciprocal";
		case ulp_function::sqrt_reciprocal:			return "sqrt_reciprocal";
		case ulp_function::sin:						return "sin";
		case ulp_function::cos:						return "cos";
		case ulp_function::tan:						return "tan";
		case ulp_function::asin:					return "asin";
		case ulp_function::acos:					return "acos";
		case ulp_function::atan:					return "atan";
		case ulp_function::atan2:					return "atan2";
		case ulp_function::dot:						return "vector_dot";
		case ulp_function::dot3:					return "vector_dot3";
		case ulp_function::dot2:					return "vector_dot2";
		case ulp_function::length:					return "vector_length";
		case ulp_function::length3:					return "vector_length3";
		case ulp_function::length_squared:			return "vector_length_squared";
		case ulp_function::length_squared3:			return "vector_length_squared3";
		case ulp_function::length_squared2:			return "vector_length_squared2";
		case ulp_function::length_reciprocal:		return "vector_length_reciprocal";
		case ulp_function::length_reciprocal3:		return "vector_length_reciprocal3";
		case ulp_function::length_reciprocal2:		return "vector_length_reciprocal2";
		case ulp_function::normalize:				return "vector_normalize";
		case ulp_function::normalize3:				return "vector_normalize3";
		case ulp_function::normalize2:				return "vector_normalize2";
		case ulp_function::distance3:				return "vector_distance3";
		case ulp_function::cross3:					return "vector_cross3";
		case ulp_function::mul_add:					return "vector_mul_add";
		case ulp_function::neg_mul_sub:				return "vector_neg_mul_sub";
		case ulp_function::lerp:					return "vector_lerp";
		case ulp_function::quat_mul:				return "quat_mul";
		case ulp_function::quat_mul_vector3:		return "quat_mul_vector3";
		case ulp_function::quat_normalize:			return "quat_normalize";
		case ulp_function::quat_dot:				return "quat_dot";
		case ulp_function::quat_length:				return "quat_length";
		case ulp_function::quat_length_squared:		return "quat_length_squared";
		case ulp_function::quat_length_reciprocal:	return "quat_length_reciprocal";
		case ulp_function::quat_lerp:				return "quat_lerp";
		case ulp_function::quat_slerp:				return "quat_slerp";
		case ulp_function::quat_from_euler:			return "quat_from_euler";
		case ulp_function::quat_get_angle:			return "quat_get_angle";
		case ulp_function::quat_rotation_log:		return "quat_rotation_log";
		case ulp_function::quat_rotation_exp:		return "quat_rotation_exp";
		case ulp_function::quat_from_matrix:		return "quat_from_matrix";
		case ulp_function::quat_ensure_positive_w:	return "quat_ensure_positive_w";
		case ulp_function::quat_from_positive_w:	return "quat_from_positive_w";
		}

		return "unknown";
	}

	// Returns true if the function computes each lane on its own and has a scalar variant
	bool is_element_wise(ulp_function function)
	{
		return uint32_t(function) <= uint32_t(ulp_function::atan2);
	}

	error_unit get_error_unit(ulp_function function)
	{
		switch (function)
		{
		case ulp_function::exp:
		case ulp_function::log:
		case ulp_function::sqrt:
		case ulp_function::reciprocal:
		case ulp_function::sqrt_reciprocal:
		case ulp_function::quat_ensure_positive_w:
			return error_unit::ulp;
		default:
			return error_unit::scaled_ulp;
		}
	}

	const char* get_unit_name(ulp_function function)
	{
		return get_error_unit(function) == error_unit::ulp ? "ulp" : "scaled ulp";
	}

	// Returns true if the result can be q or -q: both represent the same rotation
	bool has_sign_ambiguity(ulp_function function)
	{
		return function == ulp_function::quat_from_matrix;
	}

	// Returns true if the operations of the test round each result to float32 or float64 (FLT_EVAL_METHOD is 0)
	bool has_exact_operations()
	{
#if defined(FLT_EVAL_METHOD) && FLT_EVAL_METHOD == 0
		return true;
#else
		return false;
#endif
	}

	// The bounds are the largest values that we measured with 4M random inputs on ARM64 (NEON and scalar),
	// x64 (SSE4.1, AVX2 with FMA), WASM, and with 1M random inputs on 32-bit x86 (x87), rounded up to a multiple of 0.5.
	// The tests use the first random inputs of the same sequence.

	template<typename FloatType>
	struct ulp_traits;

	template<>
	struct ulp_traits<float>
	{
		using int_type = int32_t;

		// A float64 value has 29 more bits than a float32 value: it is a sufficient reference
		using reference_type = double;

		// Returns the largest error that we permit, in the unit of the function (see get_error_unit)
		static double get_max_error(ulp_function function)
		{
			// sqrt_reciprocal: 1.0 / sqrt(x) rounds two times.
			// tan: tan(x) = sin(x) / cos(x). Near 1.5, cos(x) is small and the error of cos(x) is large in comparison.
			switch (function)
			{
			case ulp_function::exp:				return 1.0;
			case ulp_function::log:				return 1.0;
			case ulp_function::sqrt:			return 0.0;
			case ulp_function::reciprocal:		return 0.0;
			case ulp_function::sqrt_reciprocal:	return 1.0;
			case ulp_function::sin:				return 2.0;
			case ulp_function::cos:				return 3.0;
			case ulp_function::tan:				return 25.0;
			case ulp_function::asin:			return 3.0;
			case ulp_function::acos:			return 3.0;
			case ulp_function::atan:			return 5.0;
			case ulp_function::atan2:			return 5.0;
			case ulp_function::dot:				return 2.5;
			case ulp_function::dot3:			return 2.0;
			case ulp_function::dot2:			return 1.5;
			case ulp_function::length:			return 2.0;
			case ulp_function::length3:			return 1.5;
			case ulp_function::length_squared:	return 2.5;
			case ulp_function::length_squared3:	return 2.0;
			case ulp_function::length_squared2:	return 1.5;
			case ulp_function::length_reciprocal:	return 3.0;
			case ulp_function::length_reciprocal3:	return 2.5;
			case ulp_function::length_reciprocal2:	return 2.5;
			case ulp_function::normalize:		return 1.5;
			case ulp_function::normalize3:		return 1.5;
			case ulp_function::normalize2:		return 1.5;
			case ulp_function::distance3:		return 2.0;
			case ulp_function::cross3:			return 1.5;
			case ulp_function::mul_add:			return 1.0;
			case ulp_function::neg_mul_sub:		return 1.0;
			case ulp_function::lerp:			return 1.5;
			case ulp_function::quat_mul:		return 2.5;
			case ulp_function::quat_mul_vector3:	return 3.0;
			case ulp_function::quat_normalize:	return 1.5;
			case ulp_function::quat_dot:		return 2.5;
			case ulp_function::quat_length:		return 2.0;
			case ulp_function::quat_length_squared:	return 3.0;
			case ulp_function::quat_length_reciprocal:	return 3.0;
			case ulp_function::quat_lerp:		return 2.0;
			case ulp_function::quat_slerp:		return 3.0;
			case ulp_function::quat_from_euler:	return 3.0;
			case ulp_function::quat_get_angle:	return 2.5;
			case ulp_function::quat_rotation_log:	return 4.5;
			case ulp_function::quat_rotation_exp:	return 4.0;
			case ulp_function::quat_from_matrix:	return 2.0;
			case ulp_function::quat_ensure_positive_w:	return 0.0;
			case ulp_function::quat_from_positive_w:	return 2.5;
			default:							return 0.0;
			}
		}

		// Returns the largest difference between the SSE2 or WASM code path and the scalar code path.
		// The vector code adds the terms of a dot product in a different order than the scalar code.
		static double get_simd_path_difference(ulp_function function)
		{
			switch (function)
			{
			case ulp_function::asin:			return 1.0;
			case ulp_function::dot:				return 2.0;
			case ulp_function::length:			return 2.0;
			case ulp_function::length_squared:	return 2.0;
			case ulp_function::length_reciprocal:	return 2.0;
			case ulp_function::normalize:		return 1.5;
			case ulp_function::normalize3:		return 0.5;
			case ulp_function::normalize2:		return 0.5;
			case ulp_function::quat_mul:		return 2.0;
			case ulp_function::quat_mul_vector3:	return 2.0;
			case ulp_function::quat_normalize:	return 1.0;
			case ulp_function::quat_dot:		return 2.0;
			case ulp_function::quat_length:		return 1.0;
			case ulp_function::quat_length_squared:	return 2.0;
			case ulp_function::quat_length_reciprocal:	return 1.0;
			case ulp_function::quat_lerp:		return 1.5;
			case ulp_function::quat_slerp:		return 3.0;
			case ulp_function::quat_from_matrix:	return 1.0;
			default:							return 0.0;
			}
		}

		// Returns the largest difference between two code paths that do not round the same operations.
		// For example, one code path uses FMA and the other does not, or one code path computes with x87 registers.
		static double get_rounding_path_difference(ulp_function function)
		{
			switch (function)
			{
			case ulp_function::exp:				return 1.0;
			case ulp_function::log:				return 1.0;
			case ulp_function::sqrt_reciprocal:	return 1.0;
			case ulp_function::sin:				return 1.5;
			case ulp_function::cos:				return 1.0;
			case ulp_function::tan:				return 22.0;
			case ulp_function::asin:			return 2.0;
			case ulp_function::acos:			return 2.0;
			case ulp_function::atan:			return 1.0;
			case ulp_function::atan2:			return 1.0;
			case ulp_function::dot:				return 2.0;
			case ulp_function::dot3:			return 2.0;
			case ulp_function::dot2:			return 1.0;
			case ulp_function::length:			return 2.0;
			case ulp_function::length3:			return 2.0;
			case ulp_function::length_squared:	return 2.0;
			case ulp_function::length_squared3:	return 2.0;
			case ulp_function::length_squared2:	return 1.0;
			case ulp_function::length_reciprocal:	return 2.0;
			case ulp_function::length_reciprocal3:	return 2.0;
			case ulp_function::length_reciprocal2:	return 2.0;
			case ulp_function::normalize:		return 2.0;
			case ulp_function::normalize3:		return 1.5;
			case ulp_function::normalize2:		return 1.5;
			case ulp_function::distance3:		return 2.0;
			case ulp_function::cross3:			return 1.0;
			case ulp_function::mul_add:			return 1.0;
			case ulp_function::neg_mul_sub:		return 1.0;
			case ulp_function::lerp:			return 2.0;
			case ulp_function::quat_mul:		return 3.0;
			case ulp_function::quat_mul_vector3:	return 3.0;
			case ulp_function::quat_normalize:	return 1.0;
			case ulp_function::quat_dot:		return 2.0;
			case ulp_function::quat_length:		return 1.0;
			case ulp_function::quat_length_squared:	return 1.0;
			case ulp_function::quat_length_reciprocal:	return 1.0;
			case ulp_function::quat_lerp:		return 2.0;
			case ulp_function::quat_slerp:		return 2.0;
			case ulp_function::quat_from_euler:	return 2.0;
			case ulp_function::quat_get_angle:	return 1.0;
			case ulp_function::quat_rotation_log:	return 3.0;
			case ulp_function::quat_rotation_exp:	return 3.0;
			case ulp_function::quat_from_matrix:	return 1.0;
			case ulp_function::quat_from_positive_w:	return 3.0;
			default:							return 0.0;
			}
		}

		// Returns the largest difference that we permit between the code path of the build and the scalar code path
		static double get_max_simd_path_difference(ulp_function function)
		{
#if defined(RTM_NEON_INTRINSICS)
			// The NEON code path uses FMA and the scalar code path does not
			return get_rounding_path_difference(function);
#elif defined(RTM_COMPILER_GCC) && defined(__FP_FAST_FMAF)
			// GCC uses -ffp-contract=fast by default and the headers do not disable contraction with GCC.
			// GCC merges a multiply and an add into one FMA instruction, thus the code paths can round different operations.
			return (std::max)(get_simd_path_difference(function), get_rounding_path_difference(function));
#else
			return get_simd_path_difference(function);
#endif
		}

		// Returns the largest difference between the scalar code path with excess precision (x87) and the other code paths
		static double get_excess_precision_path_difference(ulp_function function) { return get_rounding_path_difference(function); }

		// The largest difference of quat_normalize_deterministic with excess precision (x87)
		static double get_excess_precision_quat_difference() { return 1.0; }

		static bool flushes_denormals()
		{
#if defined(RTM_NEON_INTRINSICS) && !defined(RTM_NEON64_INTRINSICS)
			// ARMv7 NEON flushes denormal float32 values to zero
			return true;
#else
			return false;
#endif
		}
	};

	template<>
	struct ulp_traits<double>
	{
		using int_type = int64_t;

		// Only the x87 long double (64 bits of mantissa) has a reliable std::exp and std::log.
		// With MSVC and on ARM64 macOS, long double is a float64 value. With musl (WASM),
		// std::exp and std::log of a 128-bit long double call the float64 functions.
		using reference_type = long double;

		// Returns the largest error of the reference in ulp
		static double get_reference_error(ulp_function function)
		{
			switch (function)
			{
			case ulp_function::sqrt:
			case ulp_function::reciprocal:
				// Without excess precision, the float64 operations give the exact reference (see compute_exact_element).
				// With excess precision, the long double result rounds two times.
				return has_exact_operations() ? 0.0 : 1.0;
			case ulp_function::sqrt_reciprocal:
				// The reference rounds two times
				return 1.0;
			default:
#if LDBL_MANT_DIG == 64
				return 0.0;
#else
				// The float64 reference can have an error of 1 ulp
				return 1.0;
#endif
			}
		}

		// Returns the largest error that we permit, in the unit of the function (see get_error_unit)
		static double get_max_error(ulp_function function)
		{
			switch (function)
			{
			case ulp_function::exp:					return 2.0 + get_reference_error(function);
			case ulp_function::log:					return 2.0 + get_reference_error(function);
			case ulp_function::sqrt:				return 0.0 + get_reference_error(function);
			case ulp_function::reciprocal:			return 0.0 + get_reference_error(function);
			case ulp_function::sqrt_reciprocal:		return 1.0 + get_reference_error(function);	// 1.0 / sqrt(x) rounds two times
			default:
				// The bounds include the error of the reference when long double is a float64 value
				return get_vector_max_error(function);
			}
		}

		static double get_vector_max_error(ulp_function function)
		{
			switch (function)
			{
			case ulp_function::dot:				return 2.0;
			case ulp_function::dot3:			return 2.0;
			case ulp_function::dot2:			return 1.5;
			case ulp_function::length:			return 2.0;
			case ulp_function::length3:			return 2.0;
			case ulp_function::length_squared:	return 3.0;
			case ulp_function::length_squared3:	return 2.0;
			case ulp_function::length_squared2:	return 1.5;
			case ulp_function::length_reciprocal:	return 3.0;
			case ulp_function::length_reciprocal3:	return 2.5;
			case ulp_function::length_reciprocal2:	return 2.5;
			case ulp_function::normalize:		return 1.5;
			case ulp_function::normalize3:		return 1.5;
			case ulp_function::normalize2:		return 1.5;
			case ulp_function::distance3:		return 2.0;
			case ulp_function::cross3:			return 1.5;
			case ulp_function::mul_add:			return 1.0;
			case ulp_function::neg_mul_sub:		return 1.0;
			case ulp_function::lerp:			return 2.0;
			case ulp_function::quat_mul:		return 3.0;
			case ulp_function::quat_mul_vector3:	return 4.0;
			case ulp_function::quat_normalize:	return 1.5;
			case ulp_function::quat_dot:		return 2.0;
			case ulp_function::quat_length:		return 1.5;
			case ulp_function::quat_length_squared:	return 3.0;
			case ulp_function::quat_length_reciprocal:	return 2.5;
			case ulp_function::quat_lerp:		return 2.0;
			case ulp_function::quat_slerp:		return 2.5;
			case ulp_function::quat_from_euler:	return 1.5;
			case ulp_function::quat_get_angle:	return 1.0;
			case ulp_function::quat_rotation_log:	return 4.0;
			case ulp_function::quat_rotation_exp:	return 4.0;
			case ulp_function::quat_from_matrix:	return 2.0;
			default:							return 0.0;
			}
		}

		// Returns the largest difference between the SSE2 or WASM code path and the scalar code path
		static double get_simd_path_difference(ulp_function function)
		{
			switch (function)
			{
			case ulp_function::dot:				return 2.0;
			case ulp_function::length:			return 2.0;
			case ulp_function::length_squared:	return 2.0;
			case ulp_function::length_reciprocal:	return 2.0;
			case ulp_function::normalize:		return 2.0;
			case ulp_function::quat_mul:		return 2.0;
			case ulp_function::quat_mul_vector3:	return 4.0;
			case ulp_function::quat_normalize:	return 1.0;
			case ulp_function::quat_dot:		return 2.0;
			case ulp_function::quat_length:		return 1.0;
			case ulp_function::quat_length_squared:	return 2.0;
			case ulp_function::quat_length_reciprocal:	return 2.0;
			case ulp_function::quat_lerp:		return 2.0;
			case ulp_function::quat_slerp:		return 3.0;
			case ulp_function::quat_from_matrix:	return 1.0;
			default:							return 0.0;
			}
		}

		// Returns the largest difference between two code paths that do not round the same operations
		static double get_rounding_path_difference(ulp_function function)
		{
			// For a few inputs of exp, the results differ by 2 ulp
			switch (function)
			{
			case ulp_function::exp:				return 2.0;
			case ulp_function::log:				return 2.0;
			case ulp_function::dot:				return 2.0;
			case ulp_function::dot3:			return 2.0;
			case ulp_function::dot2:			return 1.0;
			case ulp_function::length:			return 2.0;
			case ulp_function::length3:			return 2.0;
			case ulp_function::length_squared:	return 2.0;
			case ulp_function::length_squared3:	return 2.0;
			case ulp_function::length_squared2:	return 1.0;
			case ulp_function::length_reciprocal:	return 2.0;
			case ulp_function::length_reciprocal3:	return 2.0;
			case ulp_function::length_reciprocal2:	return 2.0;
			case ulp_function::normalize:		return 2.0;
			case ulp_function::normalize3:		return 1.5;
			case ulp_function::normalize2:		return 1.5;
			case ulp_function::distance3:		return 2.0;
			case ulp_function::cross3:			return 1.0;
			case ulp_function::mul_add:			return 1.0;
			case ulp_function::neg_mul_sub:		return 1.0;
			case ulp_function::lerp:			return 2.0;
			case ulp_function::quat_mul:		return 3.0;
			case ulp_function::quat_mul_vector3:	return 4.0;
			case ulp_function::quat_normalize:	return 1.0;
			case ulp_function::quat_dot:		return 2.0;
			case ulp_function::quat_length:		return 1.0;
			case ulp_function::quat_length_squared:	return 2.0;
			case ulp_function::quat_length_reciprocal:	return 2.0;
			case ulp_function::quat_lerp:		return 2.0;
			case ulp_function::quat_slerp:		return 3.0;
			case ulp_function::quat_rotation_log:	return 3.0;
			case ulp_function::quat_rotation_exp:	return 4.0;
			case ulp_function::quat_from_matrix:	return 1.5;
			default:							return 0.0;
			}
		}

		// Returns the largest difference that we permit between the code path of the build and the scalar code path
		static double get_max_simd_path_difference(ulp_function function)
		{
#if defined(RTM_NEON_INTRINSICS)
			// The NEON code path uses FMA and the scalar code path does not
			return get_rounding_path_difference(function);
#elif defined(RTM_COMPILER_GCC) && defined(__FP_FAST_FMA)
			// GCC uses -ffp-contract=fast by default and the headers do not disable contraction with GCC.
			// GCC merges a multiply and an add into one FMA instruction, thus the code paths can round different operations.
			return (std::max)(get_simd_path_difference(function), get_rounding_path_difference(function));
#else
			return get_simd_path_difference(function);
#endif
		}

		// Returns the largest difference between the scalar code path with excess precision (x87) and the other code paths
		static double get_excess_precision_path_difference(ulp_function function)
		{
			switch (function)
			{
			case ulp_function::sqrt:				return 1.0;	// The x87 square root and division round two times
			case ulp_function::reciprocal:			return 1.0;
			case ulp_function::sqrt_reciprocal:		return 2.0;
			default:								return get_rounding_path_difference(function);
			}
		}

		// The largest difference of quat_normalize_deterministic with excess precision (x87)
		static double get_excess_precision_quat_difference() { return 2.0; }

		static bool flushes_denormals() { return false; }
	};

	// Returns the largest difference that we permit between the code path of the build and the scalar code path.
	// With excess precision (x87), the scalar code path rounds some results twice:
	// first to the precision of the register, then to float32 or float64. Thus the last bits can differ.
	template<typename FloatType>
	double get_max_path_difference(ulp_function function)
	{
		const double max_simd_path_difference = ulp_traits<FloatType>::get_max_simd_path_difference(function);
		if (!rtm_ulp_ref::has_excess_precision())
			return max_simd_path_difference;

		return (std::max)(max_simd_path_difference, ulp_traits<FloatType>::get_excess_precision_path_difference(function));
	}

	// Returns true if the function of the code path of the build calls the C library (std::sin, std::acos, and so on).
	// An alternative is another function of the API that computes the same value (see compute_alternatives).
	template<typename FloatType>
	bool calls_c_library(ulp_function function, bool is_alternative)
	{
		// The float64 trigonometric functions call the C library on all the code paths: the results are the same
		if (!std::is_same<FloatType, float>::value)
			return false;

#if defined(RTM_NEON_INTRINSICS)
		// With NEON, the float32 scalar trigonometric functions, vector_asin, and vector_acos call the C library.
		// The quaternion functions that call scalar_sincos or scalar_acos also call it.
		switch (function)
		{
		case ulp_function::sin:
		case ulp_function::cos:
		case ulp_function::tan:
		case ulp_function::atan:
		case ulp_function::atan2:
			return is_alternative;
		case ulp_function::asin:
		case ulp_function::acos:
		case ulp_function::quat_slerp:
		case ulp_function::quat_from_euler:
		case ulp_function::quat_get_angle:
		case ulp_function::quat_rotation_log:
		case ulp_function::quat_rotation_exp:
			return true;
		default:
			return false;
		}
#else
		(void)function;
		(void)is_alternative;
		return false;
#endif
	}

	// Returns the value as an integer with the same order as the float values.
	// -0.0 and +0.0 give 0.
	template<typename FloatType>
	int64_t get_ordered_bits(FloatType value)
	{
		using int_type = typename ulp_traits<FloatType>::int_type;

		int_type bits;
		std::memcpy(&bits, &value, sizeof(FloatType));

		// The float format uses a sign and a magnitude: we change a negative value to its two's complement
		if (bits < 0)
			bits = (std::numeric_limits<int_type>::min)() - bits;

		return int64_t(bits);
	}

	// Returns the number of representable values between two float values.
	// Two NaN values have a distance of 0.
	template<typename FloatType>
	uint64_t get_ulp_distance(FloatType lhs, FloatType rhs)
	{
		const bool is_lhs_nan = std::isnan(lhs);
		const bool is_rhs_nan = std::isnan(rhs);
		if (is_lhs_nan || is_rhs_nan)
			return (is_lhs_nan && is_rhs_nan) ? 0 : (std::numeric_limits<uint64_t>::max)();

		const int64_t lhs_bits = get_ordered_bits(lhs);
		const int64_t rhs_bits = get_ordered_bits(rhs);

		// The unsigned subtraction does not overflow
		return lhs_bits >= rhs_bits ? (uint64_t(lhs_bits) - uint64_t(rhs_bits)) : (uint64_t(rhs_bits) - uint64_t(lhs_bits));
	}

	// Rounds a reference value to the nearest float value
	template<typename FloatType, typename ReferenceType>
	FloatType round_reference(ReferenceType value)
	{
		// The conversion of a value above the largest float value is undefined behavior: we round it here
		const ReferenceType max_value = ReferenceType((std::numeric_limits<FloatType>::max)());
		const ReferenceType abs_value = std::fabs(value);
		if (abs_value > max_value)
		{
			// The mantissa of the largest value is odd: the value at the midpoint rounds to infinity
			const ReferenceType half_ulp = std::ldexp(ReferenceType(1.0), std::numeric_limits<FloatType>::max_exponent - std::numeric_limits<FloatType>::digits - 1);
			const FloatType abs_result = abs_value >= (max_value + half_ulp) ? std::numeric_limits<FloatType>::infinity() : (std::numeric_limits<FloatType>::max)();
			return value < ReferenceType(0.0) ? -abs_result : abs_result;
		}

		return FloatType(value);
	}

	// Returns the error of a result in the unit of the function.
	// If a value is not finite, returns the distance in ulp between the result and the rounded reference.
	template<typename FloatType, typename ReferenceType>
	double get_error(error_unit unit, FloatType result, ReferenceType reference, ReferenceType scale)
	{
		const FloatType rounded_reference = round_reference<FloatType>(reference);
		if (unit == error_unit::ulp || !std::isfinite(result) || !std::isfinite(rounded_reference))
			return double(get_ulp_distance(result, rounded_reference));

		// The ulp of a value in the range [2^(e-1), 2^e) is 2^(e - digits)
		int exponent;
		(void)std::frexp((std::max)(ReferenceType(std::fabs(reference)), scale), &exponent);
		const ReferenceType ulp = std::ldexp(ReferenceType(1.0), exponent - std::numeric_limits<FloatType>::digits);

		return double(ReferenceType(std::fabs(ReferenceType(result) - reference)) / ulp);
	}

	//////////////////////////////////////////////////////////////////////////
	// The exact results
	//////////////////////////////////////////////////////////////////////////

	// Computes the exact result of an element-wise function with the reference type
	template<typename FloatType, typename ReferenceType = typename ulp_traits<FloatType>::reference_type>
	ReferenceType compute_exact_element(ulp_function function, FloatType float_input0, FloatType float_input1)
	{
		// IEEE 754 requires a correctly rounded square root and division: the float type gives the exact rounded result.
		// With excess precision (x87), the result is rounded twice and we use the reference type.
		if (has_exact_operations())
		{
			if (function == ulp_function::sqrt)
				return ReferenceType(std::sqrt(float_input0));
			else if (function == ulp_function::reciprocal)
				return ReferenceType(FloatType(1.0) / float_input0);
		}

		const ReferenceType input0 = ReferenceType(float_input0);
		const ReferenceType input1 = ReferenceType(float_input1);

		switch (function)
		{
		case ulp_function::exp:				return std::exp(input0);
		case ulp_function::log:				return std::log(input0);
		case ulp_function::sqrt:			return std::sqrt(input0);
		case ulp_function::reciprocal:		return ReferenceType(1.0) / input0;
		case ulp_function::sqrt_reciprocal:	return ReferenceType(1.0) / std::sqrt(input0);
		case ulp_function::sin:				return std::sin(input0);
		case ulp_function::cos:				return std::cos(input0);
		case ulp_function::tan:				return std::tan(input0);
		case ulp_function::asin:			return std::asin(input0);
		case ulp_function::acos:			return std::acos(input0);
		case ulp_function::atan:			return std::atan(input0);
		case ulp_function::atan2:			return std::atan2(input0, input1);
		default:							return ReferenceType(0.0);
		}
	}

	// A quaternion with the reference type: [xyz] is the vector part and [w] is the scalar part
	template<typename ReferenceType>
	struct reference_quat
	{
		ReferenceType x;
		ReferenceType y;
		ReferenceType z;
		ReferenceType w;
	};

	// Returns the Hamilton product lhs * rhs
	template<typename ReferenceType>
	reference_quat<ReferenceType> hamilton_product(const reference_quat<ReferenceType>& lhs, const reference_quat<ReferenceType>& rhs)
	{
		reference_quat<ReferenceType> result;
		result.x = (lhs.w * rhs.x) + (lhs.x * rhs.w) + (lhs.y * rhs.z) - (lhs.z * rhs.y);
		result.y = (lhs.w * rhs.y) - (lhs.x * rhs.z) + (lhs.y * rhs.w) + (lhs.z * rhs.x);
		result.z = (lhs.w * rhs.z) + (lhs.x * rhs.y) - (lhs.y * rhs.x) + (lhs.z * rhs.w);
		result.w = (lhs.w * rhs.w) - (lhs.x * rhs.x) - (lhs.y * rhs.y) - (lhs.z * rhs.z);
		return result;
	}

	// Rotates a 3D vector with a quaternion: q * v * conjugate(q)
	template<typename ReferenceType>
	void rotate_vector(const reference_quat<ReferenceType>& rotation, const ReferenceType* vector, ReferenceType* output)
	{
		const reference_quat<ReferenceType> vector_quat = { vector[0], vector[1], vector[2], ReferenceType(0.0) };
		const reference_quat<ReferenceType> conjugate = { -rotation.x, -rotation.y, -rotation.z, rotation.w };
		const reference_quat<ReferenceType> result = hamilton_product(hamilton_product(rotation, vector_quat), conjugate);
		output[0] = result.x;
		output[1] = result.y;
		output[2] = result.z;
	}

	template<typename ReferenceType>
	ReferenceType get_dot(const ReferenceType* lhs, const ReferenceType* rhs, uint32_t num_components)
	{
		ReferenceType result = ReferenceType(0.0);
		for (uint32_t component_index = 0; component_index < num_components; ++component_index)
			result += lhs[component_index] * rhs[component_index];
		return result;
	}

	template<typename ReferenceType>
	ReferenceType get_abs_dot(const ReferenceType* lhs, const ReferenceType* rhs, uint32_t num_components)
	{
		ReferenceType result = ReferenceType(0.0);
		for (uint32_t component_index = 0; component_index < num_components; ++component_index)
			result += std::fabs(lhs[component_index] * rhs[component_index]);
		return result;
	}

	template<typename ReferenceType>
	ReferenceType clamp_unit(ReferenceType value)
	{
		return (std::min)((std::max)(value, ReferenceType(-1.0)), ReferenceType(1.0));
	}

	// Computes the exact result of a function with the reference type.
	// Writes the exact result and the error scale of each lane. Returns the number of lanes of the result.
	template<typename FloatType, typename ReferenceType = typename ulp_traits<FloatType>::reference_type>
	uint32_t compute_exact(ulp_function function, const FloatType* float_input0, const FloatType* float_input1, const FloatType* float_input2, ReferenceType* exact, ReferenceType* scale)
	{
		ReferenceType input0[4];
		ReferenceType input1[4];
		ReferenceType input2[4];
		for (uint32_t component_index = 0; component_index < 4; ++component_index)
		{
			input0[component_index] = ReferenceType(float_input0[component_index]);
			input1[component_index] = ReferenceType(float_input1[component_index]);
			input2[component_index] = ReferenceType(float_input2[component_index]);
			scale[component_index] = ReferenceType(1.0);
		}

		if (is_element_wise(function))
		{
			for (uint32_t lane_index = 0; lane_index < 4; ++lane_index)
				exact[lane_index] = compute_exact_element(function, float_input0[lane_index], float_input1[lane_index]);
			return 4;
		}

		const reference_quat<ReferenceType> quat0 = { input0[0], input0[1], input0[2], input0[3] };
		const reference_quat<ReferenceType> quat1 = { input1[0], input1[1], input1[2], input1[3] };
		const ReferenceType alpha = input2[0];

		switch (function)
		{
		case ulp_function::dot:
		case ulp_function::dot3:
		case ulp_function::dot2:
		case ulp_function::quat_dot:
		{
			const uint32_t num_components = function == ulp_function::dot3 ? 3 : (function == ulp_function::dot2 ? 2 : 4);
			exact[0] = get_dot(input0, input1, num_components);
			scale[0] = get_abs_dot(input0, input1, num_components);
			return 1;
		}
		case ulp_function::length:
		case ulp_function::length3:
		case ulp_function::length_squared:
		case ulp_function::length_squared3:
		case ulp_function::length_squared2:
		case ulp_function::length_reciprocal:
		case ulp_function::length_reciprocal3:
		case ulp_function::length_reciprocal2:
		case ulp_function::quat_length:
		case ulp_function::quat_length_squared:
		case ulp_function::quat_length_reciprocal:
		{
			uint32_t num_components = 4;
			if (function == ulp_function::length3 || function == ulp_function::length_squared3 || function == ulp_function::length_reciprocal3)
				num_components = 3;
			else if (function == ulp_function::length_squared2 || function == ulp_function::length_reciprocal2)
				num_components = 2;

			const ReferenceType length_squared = get_dot(input0, input0, num_components);
			if (function == ulp_function::length_squared || function == ulp_function::length_squared3 || function == ulp_function::length_squared2 || function == ulp_function::quat_length_squared)
				exact[0] = length_squared;
			else if (function == ulp_function::length_reciprocal || function == ulp_function::length_reciprocal3 || function == ulp_function::length_reciprocal2 || function == ulp_function::quat_length_reciprocal)
				exact[0] = ReferenceType(1.0) / std::sqrt(length_squared);
			else
				exact[0] = std::sqrt(length_squared);

			// The terms do not cancel: the scale is the result
			scale[0] = std::fabs(exact[0]);
			return 1;
		}
		case ulp_function::normalize:
		case ulp_function::normalize3:
		case ulp_function::normalize2:
		case ulp_function::quat_normalize:
		{
			const uint32_t num_components = function == ulp_function::normalize3 ? 3 : (function == ulp_function::normalize2 ? 2 : 4);
			const ReferenceType length = std::sqrt(get_dot(input0, input0, num_components));
			for (uint32_t component_index = 0; component_index < num_components; ++component_index)
				exact[component_index] = input0[component_index] / length;
			return num_components;
		}
		case ulp_function::distance3:
		{
			const ReferenceType difference[3] = { input1[0] - input0[0], input1[1] - input0[1], input1[2] - input0[2] };
			exact[0] = std::sqrt(get_dot(difference, difference, 3));
			scale[0] = exact[0];
			return 1;
		}
		case ulp_function::cross3:
		{
			const ReferenceType terms[3][2] =
			{
				{ input0[1] * input1[2], input0[2] * input1[1] },
				{ input0[2] * input1[0], input0[0] * input1[2] },
				{ input0[0] * input1[1], input0[1] * input1[0] },
			};

			for (uint32_t component_index = 0; component_index < 3; ++component_index)
			{
				exact[component_index] = terms[component_index][0] - terms[component_index][1];
				scale[component_index] = std::fabs(terms[component_index][0]) + std::fabs(terms[component_index][1]);
			}
			return 3;
		}
		case ulp_function::mul_add:
		case ulp_function::neg_mul_sub:
		{
			for (uint32_t component_index = 0; component_index < 4; ++component_index)
			{
				const ReferenceType product = input0[component_index] * input1[component_index];
				exact[component_index] = function == ulp_function::mul_add ? (input2[component_index] + product) : (input2[component_index] - product);
				scale[component_index] = std::fabs(product) + std::fabs(input2[component_index]);
			}
			return 4;
		}
		case ulp_function::lerp:
		{
			for (uint32_t component_index = 0; component_index < 4; ++component_index)
			{
				const ReferenceType start_term = (ReferenceType(1.0) - input2[component_index]) * input0[component_index];
				const ReferenceType end_term = input2[component_index] * input1[component_index];
				exact[component_index] = start_term + end_term;

				// The function computes (start - (alpha * start)) + (alpha * end): when alpha is near 1.0,
				// the subtraction has an error near the ulp of start, also when the result is small
				scale[component_index] = std::fabs(input0[component_index]) + std::fabs(input1[component_index]);
			}
			return 4;
		}
		case ulp_function::quat_mul:
		{
			// Multiplication order is as follow: local_to_world = quat_mul(local_to_object, object_to_world)
			// Thus quat_mul(lhs, rhs) is the Hamilton product rhs * lhs
			const reference_quat<ReferenceType> result = hamilton_product(quat1, quat0);
			exact[0] = result.x;
			exact[1] = result.y;
			exact[2] = result.z;
			exact[3] = result.w;

			// Each component is a sum of 4 products: the scale is the same sum with the absolute values
			const reference_quat<ReferenceType> abs_quat0 = { std::fabs(quat0.x), std::fabs(quat0.y), std::fabs(quat0.z), std::fabs(quat0.w) };
			const reference_quat<ReferenceType> abs_quat1 = { std::fabs(quat1.x), std::fabs(quat1.y), std::fabs(quat1.z), std::fabs(quat1.w) };
			scale[0] = (abs_quat1.w * abs_quat0.x) + (abs_quat1.x * abs_quat0.w) + (abs_quat1.y * abs_quat0.z) + (abs_quat1.z * abs_quat0.y);
			scale[1] = (abs_quat1.w * abs_quat0.y) + (abs_quat1.x * abs_quat0.z) + (abs_quat1.y * abs_quat0.w) + (abs_quat1.z * abs_quat0.x);
			scale[2] = (abs_quat1.w * abs_quat0.z) + (abs_quat1.x * abs_quat0.y) + (abs_quat1.y * abs_quat0.x) + (abs_quat1.z * abs_quat0.w);
			scale[3] = (abs_quat1.w * abs_quat0.w) + (abs_quat1.x * abs_quat0.x) + (abs_quat1.y * abs_quat0.y) + (abs_quat1.z * abs_quat0.z);
			return 4;
		}
		case ulp_function::quat_mul_vector3:
		{
			rotate_vector(quat1, input0, exact);

			// A rotation does not change the length
			const ReferenceType vector_length = std::sqrt(get_dot(input0, input0, 3));
			for (uint32_t component_index = 0; component_index < 3; ++component_index)
				scale[component_index] = vector_length;
			return 3;
		}
		case ulp_function::quat_lerp:
		{
			// If the inputs are on the opposite ends of the hypersphere, the function negates the end
			const ReferenceType end_sign = get_dot(input0, input1, 4) < ReferenceType(0.0) ? ReferenceType(-1.0) : ReferenceType(1.0);

			ReferenceType result[4];
			for (uint32_t component_index = 0; component_index < 4; ++component_index)
				result[component_index] = ((ReferenceType(1.0) - alpha) * input0[component_index]) + (alpha * end_sign * input1[component_index]);

			const ReferenceType length = std::sqrt(get_dot(result, result, 4));
			for (uint32_t component_index = 0; component_index < 4; ++component_index)
				exact[component_index] = result[component_index] / length;
			return 4;
		}
		case ulp_function::quat_slerp:
		{
			ReferenceType cos_half_angle = get_dot(input0, input1, 4);
			const ReferenceType end_sign = cos_half_angle < ReferenceType(0.0) ? ReferenceType(-1.0) : ReferenceType(1.0);
			cos_half_angle = clamp_unit(cos_half_angle * end_sign);

			const ReferenceType half_angle = std::acos(cos_half_angle);
			const ReferenceType sin_half_angle = std::sqrt((ReferenceType(1.0) - cos_half_angle) * (ReferenceType(1.0) + cos_half_angle));

			// When the angle is near zero, the function uses the linear weights (see quat_slerp)
			ReferenceType start_weight = ReferenceType(1.0) - alpha;
			ReferenceType end_weight = alpha;
			if (sin_half_angle >= ReferenceType(1.0E-4))
			{
				start_weight = std::sin((ReferenceType(1.0) - alpha) * half_angle) / sin_half_angle;
				end_weight = std::sin(alpha * half_angle) / sin_half_angle;
			}

			for (uint32_t component_index = 0; component_index < 4; ++component_index)
				exact[component_index] = (start_weight * input0[component_index]) + (end_weight * end_sign * input1[component_index]);
			return 4;
		}
		case ulp_function::quat_from_euler:
		{
			const ReferenceType half = ReferenceType(0.5);
			const ReferenceType sp = std::sin(input0[0] * half);
			const ReferenceType cp = std::cos(input0[0] * half);
			const ReferenceType sy = std::sin(input0[1] * half);
			const ReferenceType cy = std::cos(input0[1] * half);
			const ReferenceType sr = std::sin(input0[2] * half);
			const ReferenceType cr = std::cos(input0[2] * half);

			exact[0] = (cr * sp * sy) - (sr * cp * cy);
			exact[1] = (-cr * sp * cy) - (sr * cp * sy);
			exact[2] = (cr * cp * sy) - (sr * sp * cy);
			exact[3] = (cr * cp * cy) + (sr * sp * sy);
			return 4;
		}
		case ulp_function::quat_get_angle:
		{
			exact[0] = std::acos(clamp_unit(quat0.w)) * ReferenceType(2.0);
			return 1;
		}
		case ulp_function::quat_rotation_log:
		{
			// Near the identity, the function returns the input [xyz] (see quat_rotation_log)
			const ReferenceType w = clamp_unit(quat0.w);
			const ReferenceType xyz_length = std::sqrt(get_dot(input0, input0, 3));
			const ReferenceType xyz_scale = w > ReferenceType(1.0 - 1.0E-6) ? ReferenceType(1.0) : (std::acos(w) / xyz_length);
			for (uint32_t component_index = 0; component_index < 3; ++component_index)
				exact[component_index] = input0[component_index] * xyz_scale;
			exact[3] = ReferenceType(0.0);
			return 4;
		}
		case ulp_function::quat_rotation_exp:
		{
			// Near zero, the function returns the input [xyz] (see quat_rotation_exp)
			const ReferenceType xyz_length = std::sqrt(get_dot(input0, input0, 3));
			const ReferenceType xyz_scale = xyz_length < ReferenceType(1.0E-6) ? ReferenceType(1.0) : (std::sin(xyz_length) / xyz_length);
			for (uint32_t component_index = 0; component_index < 3; ++component_index)
				exact[component_index] = input0[component_index] * xyz_scale;
			exact[3] = std::cos(xyz_length);
			return 4;
		}
		case ulp_function::quat_from_matrix:
		{
			// Each axis is the image of a unit vector: axis[j][i] is the component i of the axis j.
			// We use the standard matrix m[i][j] = axis[j][i] and the method of Shepperd.
			const ReferenceType* axes[3] = { input0, input1, input2 };
			ReferenceType m[3][3];
			for (uint32_t row_index = 0; row_index < 3; ++row_index)
				for (uint32_t column_index = 0; column_index < 3; ++column_index)
					m[row_index][column_index] = axes[column_index][row_index];

			const ReferenceType trace = m[0][0] + m[1][1] + m[2][2];
			reference_quat<ReferenceType> result;
			if (trace > (std::max)((std::max)(m[0][0], m[1][1]), m[2][2]))
			{
				const ReferenceType s = std::sqrt(ReferenceType(1.0) + trace) * ReferenceType(2.0);
				result.w = s * ReferenceType(0.25);
				result.x = (m[2][1] - m[1][2]) / s;
				result.y = (m[0][2] - m[2][0]) / s;
				result.z = (m[1][0] - m[0][1]) / s;
			}
			else if (m[0][0] >= m[1][1] && m[0][0] >= m[2][2])
			{
				const ReferenceType s = std::sqrt(ReferenceType(1.0) + m[0][0] - m[1][1] - m[2][2]) * ReferenceType(2.0);
				result.w = (m[2][1] - m[1][2]) / s;
				result.x = s * ReferenceType(0.25);
				result.y = (m[0][1] + m[1][0]) / s;
				result.z = (m[0][2] + m[2][0]) / s;
			}
			else if (m[1][1] >= m[2][2])
			{
				const ReferenceType s = std::sqrt(ReferenceType(1.0) + m[1][1] - m[0][0] - m[2][2]) * ReferenceType(2.0);
				result.w = (m[0][2] - m[2][0]) / s;
				result.x = (m[0][1] + m[1][0]) / s;
				result.y = s * ReferenceType(0.25);
				result.z = (m[1][2] + m[2][1]) / s;
			}
			else
			{
				const ReferenceType s = std::sqrt(ReferenceType(1.0) + m[2][2] - m[0][0] - m[1][1]) * ReferenceType(2.0);
				result.w = (m[1][0] - m[0][1]) / s;
				result.x = (m[0][2] + m[2][0]) / s;
				result.y = (m[1][2] + m[2][1]) / s;
				result.z = s * ReferenceType(0.25);
			}

			const ReferenceType components[4] = { result.x, result.y, result.z, result.w };
			const ReferenceType length = std::sqrt(get_dot(components, components, 4));
			for (uint32_t component_index = 0; component_index < 4; ++component_index)
				exact[component_index] = components[component_index] / length;
			return 4;
		}
		case ulp_function::quat_ensure_positive_w:
		{
			// The function tests the sign bit of [w]: -0.0 is negative
			const ReferenceType sign = std::signbit(float_input0[3]) ? ReferenceType(-1.0) : ReferenceType(1.0);
			for (uint32_t component_index = 0; component_index < 4; ++component_index)
				exact[component_index] = input0[component_index] * sign;
			return 4;
		}
		case ulp_function::quat_from_positive_w:
		{
			for (uint32_t component_index = 0; component_index < 3; ++component_index)
				exact[component_index] = input0[component_index];
			exact[3] = std::sqrt(std::fabs(ReferenceType(1.0) - get_dot(input0, input0, 3)));

			// The function computes w^2 = 1.0 - (x^2 + y^2 + z^2) with an absolute error near epsilon.
			// The error of w = sqrt(w^2) is near epsilon / (2 * w): it is large when w is small.
			const ReferenceType min_w = std::sqrt(ReferenceType(std::numeric_limits<FloatType>::epsilon()));
			scale[3] = ReferenceType(0.5) / (std::max)(exact[3], min_w);
			return 4;
		}
		default:
			return 0;
		}
	}

	//////////////////////////////////////////////////////////////////////////
	// The other functions of the API that compute the same value
	//////////////////////////////////////////////////////////////////////////

	// Computes the scalar variant of an element-wise function
	template<typename FloatType>
	FloatType compute_scalar_function(ulp_function function, FloatType input0, FloatType input1, FloatType input2)
	{
		const auto input = scalar_set(input0);

		switch (function)
		{
		case ulp_function::exp:				return scalar_cast(scalar_exp(input));
		case ulp_function::log:				return scalar_cast(scalar_log(input));
		case ulp_function::sqrt:			return scalar_cast(scalar_sqrt(input));
		case ulp_function::reciprocal:		return scalar_cast(scalar_reciprocal(input));
		case ulp_function::sqrt_reciprocal:	return scalar_cast(scalar_sqrt_reciprocal(input));
		case ulp_function::sin:				return scalar_cast(scalar_sin(input));
		case ulp_function::cos:				return scalar_cast(scalar_cos(input));
		case ulp_function::tan:				return scalar_cast(scalar_tan(input));
		case ulp_function::asin:			return scalar_cast(scalar_asin(input));
		case ulp_function::acos:			return scalar_cast(scalar_acos(input));
		case ulp_function::atan:			return scalar_cast(scalar_atan(input));
		case ulp_function::atan2:			return scalar_cast(scalar_atan2(input, scalar_set(input1)));
		case ulp_function::mul_add:			return scalar_cast(scalar_mul_add(input, scalar_set(input1), scalar_set(input2)));
		case ulp_function::neg_mul_sub:		return scalar_cast(scalar_neg_mul_sub(input, scalar_set(input1), scalar_set(input2)));
		case ulp_function::lerp:			return scalar_cast(scalar_lerp(input, scalar_set(input1), scalar_set(input2)));
		default:							return FloatType(0.0);
		}
	}

	// Computes the other functions of the API that compute the same value as the vector function.
	// Writes 4 lanes for each alternative. Returns the number of alternatives.
	template<typename FloatType>
	uint32_t compute_alternatives(ulp_function function, const FloatType* input0, const FloatType* input1, const FloatType* input2, FloatType (*outputs)[4])
	{
		if (is_element_wise(function) || function == ulp_function::mul_add || function == ulp_function::neg_mul_sub || function == ulp_function::lerp)
		{
			for (uint32_t lane_index = 0; lane_index < 4; ++lane_index)
				outputs[0][lane_index] = compute_scalar_function(function, input0[lane_index], input1[lane_index], input2[lane_index]);
			return 1;
		}

		const auto v0 = vector_load(input0);
		const auto v1 = vector_load(input1);
		const auto q0 = quat_load(input0);
		const auto q1 = quat_load(input1);

		switch (function)
		{
		case ulp_function::dot:
			store_scalar_result(FloatType(scalar_cast(vector_dot_as_scalar(v0, v1))), outputs[0]);
			store_scalar_result(FloatType(vector_dot(v0, v1)), outputs[1]);
			return 2;
		case ulp_function::dot3:
			store_scalar_result(FloatType(scalar_cast(vector_dot3_as_scalar(v0, v1))), outputs[0]);
			store_scalar_result(FloatType(vector_dot3(v0, v1)), outputs[1]);
			return 2;
		case ulp_function::dot2:
			store_scalar_result(FloatType(scalar_cast(vector_dot2_as_scalar(v0, v1))), outputs[0]);
			store_scalar_result(FloatType(vector_dot2(v0, v1)), outputs[1]);
			return 2;
		case ulp_function::length:
			store_scalar_result(FloatType(vector_length(v0)), outputs[0]);
			return 1;
		case ulp_function::length3:
			store_scalar_result(FloatType(vector_length3(v0)), outputs[0]);
			return 1;
		case ulp_function::length_squared:
			store_scalar_result(FloatType(scalar_cast(vector_length_squared_as_scalar(v0))), outputs[0]);
			store_scalar_result(FloatType(vector_length_squared(v0)), outputs[1]);
			return 2;
		case ulp_function::length_squared3:
			store_scalar_result(FloatType(scalar_cast(vector_length_squared3_as_scalar(v0))), outputs[0]);
			store_scalar_result(FloatType(vector_length_squared3(v0)), outputs[1]);
			return 2;
		case ulp_function::length_squared2:
			store_scalar_result(FloatType(scalar_cast(vector_length_squared2_as_scalar(v0))), outputs[0]);
			store_scalar_result(FloatType(vector_length_squared2(v0)), outputs[1]);
			return 2;
		case ulp_function::length_reciprocal:
			store_scalar_result(FloatType(vector_length_reciprocal(v0)), outputs[0]);
			return 1;
		case ulp_function::length_reciprocal3:
			store_scalar_result(FloatType(vector_length_reciprocal3(v0)), outputs[0]);
			return 1;
		case ulp_function::length_reciprocal2:
			store_scalar_result(FloatType(vector_length_reciprocal2(v0)), outputs[0]);
			return 1;
		case ulp_function::distance3:
			store_scalar_result(FloatType(vector_distance3(v0, v1)), outputs[0]);
			return 1;
		case ulp_function::quat_dot:
			store_scalar_result(FloatType(quat_dot(q0, q1)), outputs[0]);
			return 1;
		case ulp_function::quat_length:
			store_scalar_result(FloatType(quat_length(q0)), outputs[0]);
			return 1;
		case ulp_function::quat_length_squared:
			store_scalar_result(FloatType(quat_length_squared(q0)), outputs[0]);
			return 1;
		case ulp_function::quat_length_reciprocal:
			store_scalar_result(FloatType(quat_length_reciprocal(q0)), outputs[0]);
			return 1;
		case ulp_function::quat_lerp:
			quat_store(quat_lerp(q0, q1, scalar_set(input2[0])), outputs[0]);
			return 1;
		case ulp_function::quat_slerp:
			quat_store(quat_slerp(q0, q1, scalar_set(input2[0])), outputs[0]);
			return 1;
		default:
			return 0;
		}
	}

	//////////////////////////////////////////////////////////////////////////
	// The inputs
	//////////////////////////////////////////////////////////////////////////

	// Returns true if the value is a denormal value
	template<typename FloatType>
	bool is_denormal(FloatType value)
	{
		return value != FloatType(0.0) && std::fabs(value) < (std::numeric_limits<FloatType>::min)();
	}

	// Returns a random value in the range [0.0, 1.0)
	double get_random_fraction(std::mt19937& generator)
	{
		const uint64_t high = uint64_t(generator());
		const uint64_t low = uint64_t(generator());
		const uint64_t bits = ((high << 32) | low) >> 11;	// 53 bits
		return double(bits) * (1.0 / 9007199254740992.0);	// 2^53
	}

	template<typename FloatType>
	FloatType get_random_value(std::mt19937& generator, double min_value, double max_value)
	{
		return FloatType(min_value + ((max_value - min_value) * get_random_fraction(generator)));
	}

	// Returns a random positive float value: each bit pattern of a finite value has the same probability.
	// The value can be 0.0 and a denormal value.
	float get_random_positive_bits(std::mt19937& generator, float)
	{
		const uint32_t bits = uint32_t(generator()) % 0x7F800000U;	// Below +inf
		float value;
		std::memcpy(&value, &bits, sizeof(float));
		return value;
	}

	double get_random_positive_bits(std::mt19937& generator, double)
	{
		const uint64_t high = uint64_t(generator());
		const uint64_t low = uint64_t(generator());
		const uint64_t bits = ((high << 32) | low) % 0x7FF0000000000000ULL;	// Below +inf
		double value;
		std::memcpy(&value, &bits, sizeof(double));
		return value;
	}

	// Returns a random float value with a random sign: each bit pattern of a finite value has the same probability
	template<typename FloatType>
	FloatType get_random_bits(std::mt19937& generator)
	{
		const FloatType value = get_random_positive_bits(generator, FloatType());
		return (generator() & 1) != 0 ? -value : value;
	}

	// Writes a random vector. All the components use the same random scale in the range [2^-30, 2^30].
	// The squares of the components do not overflow and are not denormal values.
	template<typename FloatType>
	void get_random_vector(std::mt19937& generator, FloatType* output)
	{
		const double scale = std::ldexp(1.0, int(generator() % 61) - 30);
		for (uint32_t component_index = 0; component_index < 4; ++component_index)
			output[component_index] = get_random_value<FloatType>(generator, -scale, scale);
	}

	// Writes a random unit quaternion, rounded to the float type
	template<typename FloatType>
	void get_random_quat(std::mt19937& generator, FloatType* output)
	{
		double components[4];
		double length_squared;
		do
		{
			for (double& component : components)
				component = get_random_value<double>(generator, -1.0, 1.0);
			length_squared = components[0] * components[0] + components[1] * components[1] + components[2] * components[2] + components[3] * components[3];
		} while (length_squared < 0.01 || length_squared > 1.0);

		const double length = std::sqrt(length_squared);
		for (uint32_t component_index = 0; component_index < 4; ++component_index)
			output[component_index] = FloatType(components[component_index] / length);
	}

	// The inputs of a function. A call of a function reads 4 values of each input.
	template<typename FloatType>
	struct input_set
	{
		std::vector<FloatType> input0;
		std::vector<FloatType> input1;
		std::vector<FloatType> input2;

		// Adds one lane of an element-wise function
		void add_lane(FloatType value0, FloatType value1 = FloatType(1.0), FloatType value2 = FloatType(1.0))
		{
			input0.push_back(value0);
			input1.push_back(value1);
			input2.push_back(value2);
		}

		// Adds a lane and the float values just below and just above it
		void add_lane_with_neighbors(FloatType value)
		{
			add_lane(value);
			add_lane(std::nextafter(value, -std::numeric_limits<FloatType>::infinity()));
			add_lane(std::nextafter(value, std::numeric_limits<FloatType>::infinity()));
		}

		// Adds one call: 4 values of each input
		void add_call(const FloatType* values0, const FloatType* values1, const FloatType* values2)
		{
			input0.insert(input0.end(), values0, values0 + 4);
			input1.insert(input1.end(), values1, values1 + 4);
			input2.insert(input2.end(), values2, values2 + 4);
		}

		size_t get_num_calls() const { return input0.size() / 4; }
	};

	template<typename FloatType>
	void add_element_wise_edge_inputs(ulp_function function, input_set<FloatType>& inputs)
	{
		using limits = std::numeric_limits<FloatType>;
		using reference_type = typename ulp_traits<FloatType>::reference_type;

		const FloatType infinity = limits::infinity();
		const FloatType nan = limits::quiet_NaN();
		const FloatType max_value = (limits::max)();
		const FloatType min_value = (limits::min)();
		const FloatType denormal_min_value = limits::denorm_min();
		const FloatType epsilon = limits::epsilon();
		const FloatType mantissa_shift = FloatType(std::ldexp(FloatType(1.0), limits::digits - 1));	// 2^23 or 2^52

		switch (function)
		{
		case ulp_function::exp:
		case ulp_function::log:
		case ulp_function::sqrt:
		case ulp_function::reciprocal:
		case ulp_function::sqrt_reciprocal:
		case ulp_function::atan:
		{
			const FloatType common_inputs[] =
			{
				FloatType(0.0), FloatType(-0.0), FloatType(1.0), FloatType(-1.0), FloatType(0.5), FloatType(2.0),
				infinity, -infinity, nan,
				max_value, -max_value, min_value, -min_value, denormal_min_value, -denormal_min_value,
				epsilon, FloatType(1.0) + epsilon, FloatType(1.0) - (epsilon * FloatType(0.5)),
				FloatType(0.70710678118654752440), FloatType(1.41421356237309504880), FloatType(0.69314718055994530942),
				mantissa_shift, mantissa_shift * FloatType(1.5),
			};

			for (const FloatType value : common_inputs)
				inputs.add_lane(value);
			break;
		}
		case ulp_function::sin:
		case ulp_function::cos:
		{
			const FloatType angles[] = { FloatType(0.0), FloatType(k_pi * 0.25), FloatType(k_pi * 0.5), FloatType(k_pi), FloatType(k_pi * 1.5), FloatType(k_pi * 2.0) };
			for (const FloatType angle : angles)
			{
				inputs.add_lane_with_neighbors(angle);
				inputs.add_lane_with_neighbors(-angle);
			}

			const FloatType special_inputs[] = { infinity, -infinity, nan, min_value, -min_value, denormal_min_value, -denormal_min_value };
			for (const FloatType value : special_inputs)
				inputs.add_lane(value);
			break;
		}
		case ulp_function::tan:
		{
			// The result of tan near pi/2 is very large: the tests do not use these inputs
			const FloatType angles[] = { FloatType(0.0), FloatType(k_pi * 0.25), FloatType(1.0), FloatType(1.5) };
			for (const FloatType angle : angles)
			{
				inputs.add_lane_with_neighbors(angle);
				inputs.add_lane_with_neighbors(-angle);
			}

			const FloatType special_inputs[] = { infinity, -infinity, nan, min_value, -min_value, denormal_min_value, -denormal_min_value };
			for (const FloatType value : special_inputs)
				inputs.add_lane(value);
			break;
		}
		case ulp_function::asin:
		case ulp_function::acos:
		{
			// The input must be in the range [-1.0, 1.0]
			const FloatType special_inputs[] =
			{
				FloatType(0.0), FloatType(-0.0), FloatType(1.0), FloatType(-1.0), FloatType(0.5), FloatType(-0.5),
				FloatType(1.0) - (epsilon * FloatType(0.5)), FloatType(-1.0) + (epsilon * FloatType(0.5)),
				FloatType(0.70710678118654752440), FloatType(-0.70710678118654752440),
				nan, min_value, -min_value, denormal_min_value, -denormal_min_value,
			};

			for (const FloatType value : special_inputs)
				inputs.add_lane(value);
			break;
		}
		case ulp_function::atan2:
		{
			// atan2(0.0, 0.0) returns 0.0 for all the signs of zero, unlike std::atan2: the tests do not use these inputs
			const FloatType values[] = { FloatType(1.0), FloatType(-1.0), FloatType(0.5), FloatType(-2.0), FloatType(0.0), FloatType(-0.0) };
			for (const FloatType y : values)
			{
				for (const FloatType x : values)
				{
					if (y != FloatType(0.0) || x != FloatType(0.0))
						inputs.add_lane(y, x);
				}
			}
			break;
		}
		default:
			break;
		}

		if (function == ulp_function::exp)
		{
			// The input values where the result overflows, becomes a denormal value, and becomes zero
			const FloatType thresholds[] =
			{
				FloatType(std::log(reference_type(max_value))),
				FloatType(std::log(reference_type(min_value))),
				FloatType(std::log(reference_type(denormal_min_value))),
				FloatType(std::is_same<FloatType, float>::value ? -104.0 : -746.0),	// The clamp of the input in exp_log_common.h
				FloatType(std::is_same<FloatType, float>::value ? 89.0 : 710.0),
			};

			for (const FloatType threshold : thresholds)
				inputs.add_lane_with_neighbors(threshold);
		}
	}

	template<typename FloatType>
	void add_element_wise_random_inputs(ulp_function function, std::mt19937& generator, input_set<FloatType>& inputs)
	{
		const bool is_float32 = std::is_same<FloatType, float>::value;

		for (uint32_t input_index = 0; input_index < k_num_random_inputs; ++input_index)
		{
			const bool is_even = (input_index % 2) == 0;

			switch (function)
			{
			case ulp_function::exp:
				// The range includes the inputs where the result overflows and where the result is zero
				inputs.add_lane(is_float32 ? get_random_value<FloatType>(generator, -110.0, 95.0) : get_random_value<FloatType>(generator, -750.0, 715.0));
				break;
			case ulp_function::log:
				// All the exponents, and the range near 1.0 where the result is near zero
				inputs.add_lane(is_even ? get_random_positive_bits(generator, FloatType()) : get_random_value<FloatType>(generator, 0.5, 2.0));
				break;
			case ulp_function::sqrt:
			case ulp_function::sqrt_reciprocal:
				inputs.add_lane(get_random_positive_bits(generator, FloatType()));
				break;
			case ulp_function::reciprocal:
				inputs.add_lane(get_random_bits<FloatType>(generator));
				break;
			case ulp_function::sin:
			case ulp_function::cos:
				// One full turn in each direction
				inputs.add_lane(get_random_value<FloatType>(generator, -2.0 * k_pi, 2.0 * k_pi));
				break;
			case ulp_function::tan:
				inputs.add_lane(get_random_value<FloatType>(generator, -1.5, 1.5));
				break;
			case ulp_function::asin:
			case ulp_function::acos:
			{
				// The full range, and the small values with all the exponents
				FloatType value = get_random_value<FloatType>(generator, -1.0, 1.0);
				if (!is_even)
					value = value * FloatType(std::ldexp(1.0, -int(generator() % 32)));
				inputs.add_lane(value);
				break;
			}
			case ulp_function::atan:
				inputs.add_lane(is_even ? get_random_bits<FloatType>(generator) : get_random_value<FloatType>(generator, -4.0, 4.0));
				break;
			case ulp_function::atan2:
			{
				// A random scale for each input gives a large range of ratios
				const double y_scale = std::ldexp(1.0, int(generator() % 17) - 8);
				const double x_scale = std::ldexp(1.0, int(generator() % 17) - 8);
				const FloatType y = get_random_value<FloatType>(generator, -y_scale, y_scale);
				const FloatType x = get_random_value<FloatType>(generator, -x_scale, x_scale);
				inputs.add_lane(y, x);
				break;
			}
			default:
				break;
			}
		}
	}

	template<typename FloatType>
	void add_vector_edge_inputs(ulp_function function, input_set<FloatType>& inputs)
	{
		const FloatType large = FloatType(std::ldexp(1.0, 30));
		const FloatType small = FloatType(std::ldexp(1.0, -30));

		const FloatType vectors[][4] =
		{
			{ FloatType(0.0), FloatType(0.0), FloatType(0.0), FloatType(0.0) },
			{ FloatType(1.0), FloatType(0.0), FloatType(0.0), FloatType(0.0) },
			{ FloatType(0.0), FloatType(1.0), FloatType(0.0), FloatType(0.0) },
			{ FloatType(0.0), FloatType(0.0), FloatType(-1.0), FloatType(0.0) },
			{ FloatType(0.0), FloatType(0.0), FloatType(0.0), FloatType(1.0) },
			{ FloatType(1.0), FloatType(1.0), FloatType(1.0), FloatType(1.0) },
			{ FloatType(-1.0), FloatType(2.0), FloatType(-3.0), FloatType(4.0) },
			{ large, -large, large, large },
			{ small, small, -small, small },
		};

		const FloatType alphas[] = { FloatType(0.0), FloatType(0.25), FloatType(0.5), FloatType(1.0) };

		// A zero vector has no direction: the result of the functions that divide by the length is not defined
		const bool divides_by_length = function == ulp_function::normalize || function == ulp_function::normalize3 || function == ulp_function::normalize2;

		for (const FloatType (&vector0)[4] : vectors)
		{
			if (divides_by_length && vector0[0] == FloatType(0.0) && vector0[1] == FloatType(0.0))
				continue;	// The first lanes are zero for normalize2 and they can be zero for normalize3

			for (const FloatType (&vector1)[4] : vectors)
			{
				const FloatType alpha_values[4] = { alphas[0], alphas[1], alphas[2], alphas[3] };
				const FloatType* vector2 = function == ulp_function::lerp ? alpha_values : &vectors[6][0];
				inputs.add_call(&vector0[0], &vector1[0], vector2);
			}
		}
	}

	template<typename FloatType>
	void add_quat_edge_inputs(ulp_function function, input_set<FloatType>& inputs)
	{
		const FloatType half_sqrt2 = FloatType(0.70710678118654752440);
		const FloatType quats[][4] =
		{
			{ FloatType(0.0), FloatType(0.0), FloatType(0.0), FloatType(1.0) },		// Identity
			{ FloatType(0.0), FloatType(0.0), FloatType(0.0), FloatType(-1.0) },
			{ FloatType(1.0), FloatType(0.0), FloatType(0.0), FloatType(0.0) },		// Half turns
			{ FloatType(0.0), FloatType(-1.0), FloatType(0.0), FloatType(0.0) },
			{ FloatType(0.0), FloatType(0.0), half_sqrt2, half_sqrt2 },				// Quarter turns
			{ half_sqrt2, FloatType(0.0), FloatType(0.0), -half_sqrt2 },
			{ FloatType(0.5), FloatType(-0.5), FloatType(0.5), FloatType(0.5) },
		};

		const FloatType alphas[4] = { FloatType(0.0), FloatType(0.0), FloatType(0.0), FloatType(0.0) };
		const FloatType alpha_values[] = { FloatType(0.0), FloatType(0.25), FloatType(0.5), FloatType(1.0) };

		switch (function)
		{
		case ulp_function::quat_from_euler:
		{
			// Pitch, yaw, and roll, with the pitch at +-pi/2 where the yaw and the roll turn around the same axis
			const FloatType angles[][4] =
			{
				{ FloatType(0.0), FloatType(0.0), FloatType(0.0), FloatType(0.0) },
				{ FloatType(k_pi * 0.5), FloatType(0.3), FloatType(-0.7), FloatType(0.0) },
				{ FloatType(-k_pi * 0.5), FloatType(1.1), FloatType(0.2), FloatType(0.0) },
				{ FloatType(k_pi), FloatType(-k_pi), FloatType(k_pi), FloatType(0.0) },
			};

			for (const FloatType (&angle)[4] : angles)
				inputs.add_call(&angle[0], &angle[0], alphas);
			break;
		}
		case ulp_function::quat_ensure_positive_w:
		{
			// The function tests the sign bit of [w]
			const FloatType values[][4] =
			{
				{ FloatType(0.5), FloatType(-0.5), FloatType(0.5), FloatType(0.5) },
				{ FloatType(0.5), FloatType(-0.5), FloatType(0.5), FloatType(-0.5) },
				{ FloatType(1.0), FloatType(0.0), FloatType(0.0), FloatType(0.0) },
				{ FloatType(1.0), FloatType(0.0), FloatType(0.0), FloatType(-0.0) },
			};

			for (const FloatType (&value)[4] : values)
				inputs.add_call(&value[0], &value[0], alphas);
			break;
		}
		case ulp_function::quat_rotation_exp:
		{
			// The input is the logarithm of a rotation: [w] is 0.0
			const FloatType values[][4] =
			{
				{ FloatType(0.0), FloatType(0.0), FloatType(0.0), FloatType(0.0) },
				{ FloatType(k_pi * 0.5), FloatType(0.0), FloatType(0.0), FloatType(0.0) },
				{ FloatType(0.0), FloatType(0.0), FloatType(-k_pi), FloatType(0.0) },
				{ FloatType(0.5), FloatType(-0.5), FloatType(0.5), FloatType(0.0) },
			};

			for (const FloatType (&value)[4] : values)
				inputs.add_call(&value[0], &value[0], alphas);
			break;
		}
		case ulp_function::quat_from_matrix:
		case ulp_function::quat_from_positive_w:
			// See add_quat_random_call: the inputs depend on a quaternion
			break;
		default:
		{
			for (const FloatType (&quat0)[4] : quats)
			{
				for (const FloatType (&quat1)[4] : quats)
				{
					for (const FloatType alpha : alpha_values)
					{
						const FloatType alpha_lanes[4] = { alpha, alpha, alpha, alpha };
						inputs.add_call(&quat0[0], &quat1[0], alpha_lanes);
					}
				}
			}
			break;
		}
		}
	}

	template<typename FloatType>
	void add_quat_random_call(ulp_function function, std::mt19937& generator, input_set<FloatType>& inputs)
	{
		FloatType values0[4];
		FloatType values1[4];
		FloatType values2[4];
		get_random_quat(generator, values0);
		get_random_quat(generator, values1);

		const FloatType alpha = get_random_value<FloatType>(generator, 0.0, 1.0);
		for (FloatType& value : values2)
			value = alpha;

		switch (function)
		{
		case ulp_function::quat_mul_vector3:
			get_random_vector(generator, values0);
			break;
		case ulp_function::quat_normalize:
		case ulp_function::quat_dot:
		case ulp_function::quat_length:
		case ulp_function::quat_length_squared:
		case ulp_function::quat_length_reciprocal:
		{
			// These functions do not need a unit quaternion
			const double scale = std::ldexp(1.0, int(generator() % 61) - 30);
			for (FloatType& value : values0)
				value = FloatType(double(value) * scale);
			break;
		}
		case ulp_function::quat_from_euler:
			for (uint32_t component_index = 0; component_index < 3; ++component_index)
				values0[component_index] = get_random_value<FloatType>(generator, -2.0 * k_pi, 2.0 * k_pi);
			values0[3] = FloatType(0.0);
			break;
		case ulp_function::quat_rotation_exp:
		{
			// The logarithm of a rotation: the axis multiplied by the half angle in the range [0, pi]
			const double half_angle = get_random_value<double>(generator, 0.0, k_pi);
			const double axis_length = std::sqrt(double(values1[0]) * double(values1[0]) + double(values1[1]) * double(values1[1]) + double(values1[2]) * double(values1[2]));
			for (uint32_t component_index = 0; component_index < 3; ++component_index)
				values0[component_index] = FloatType(double(values1[component_index]) / axis_length * half_angle);
			values0[3] = FloatType(0.0);
			break;
		}
		case ulp_function::quat_from_matrix:
		{
			// The axes of the rotation matrix: the images of the unit vectors
			using reference_type = typename ulp_traits<FloatType>::reference_type;
			const reference_quat<reference_type> rotation = { reference_type(values0[0]), reference_type(values0[1]), reference_type(values0[2]), reference_type(values0[3]) };
			FloatType* axes[3] = { values0, values1, values2 };
			for (uint32_t axis_index = 0; axis_index < 3; ++axis_index)
			{
				reference_type unit_vector[3] = { reference_type(0.0), reference_type(0.0), reference_type(0.0) };
				unit_vector[axis_index] = reference_type(1.0);

				reference_type axis[3];
				rotate_vector(rotation, unit_vector, axis);
				for (uint32_t component_index = 0; component_index < 3; ++component_index)
					axes[axis_index][component_index] = FloatType(axis[component_index]);
				axes[axis_index][3] = FloatType(0.0);
			}
			break;
		}
		case ulp_function::quat_from_positive_w:
			// The input is the [xyz] of a unit quaternion with a positive [w]
			values0[3] = FloatType(0.0);
			break;
		case ulp_function::quat_ensure_positive_w:
			if ((generator() & 1) != 0)
				values0[3] = -values0[3];
			break;
		default:
			break;
		}

		inputs.add_call(values0, values1, values2);
	}

	template<typename FloatType>
	void add_vector_random_call(ulp_function function, std::mt19937& generator, input_set<FloatType>& inputs)
	{
		FloatType values0[4];
		FloatType values1[4];
		FloatType values2[4];
		get_random_vector(generator, values0);
		get_random_vector(generator, values1);
		get_random_vector(generator, values2);

		if (function == ulp_function::lerp)
		{
			for (FloatType& value : values2)
				value = get_random_value<FloatType>(generator, 0.0, 1.0);
		}

		inputs.add_call(values0, values1, values2);
	}

	template<typename FloatType>
	input_set<FloatType> get_inputs(ulp_function function)
	{
		input_set<FloatType> inputs;

		// Each function has its own seed: a new function does not change the inputs of the other functions
		std::mt19937 generator(uint32_t(function) + 1);

		if (is_element_wise(function))
		{
			add_element_wise_edge_inputs(function, inputs);
			add_element_wise_random_inputs(function, generator, inputs);

			// The functions process 4 inputs at a time
			while ((inputs.input0.size() % 4) != 0)
				inputs.add_lane(FloatType(1.0));
		}
		else if (uint32_t(function) < uint32_t(ulp_function::quat_mul))
		{
			add_vector_edge_inputs(function, inputs);
			for (uint32_t call_index = 0; call_index < k_num_random_inputs / 4; ++call_index)
				add_vector_random_call(function, generator, inputs);
		}
		else
		{
			add_quat_edge_inputs(function, inputs);
			for (uint32_t call_index = 0; call_index < k_num_random_inputs / 4; ++call_index)
				add_quat_random_call(function, generator, inputs);
		}

		return inputs;
	}

	// Returns the inputs of a call as text, with all the digits
	template<typename FloatType>
	std::string get_input_string(ulp_function function, const input_set<FloatType>& inputs, size_t call_index, uint32_t lane_index)
	{
		std::ostringstream stream;
		stream << std::setprecision(std::numeric_limits<FloatType>::max_digits10);

		if (is_element_wise(function))
		{
			const size_t value_index = (call_index * 4) + lane_index;
			stream << inputs.input0[value_index];
			if (function == ulp_function::atan2)
				stream << ", " << inputs.input1[value_index];
			return stream.str();
		}

		const std::vector<FloatType>* input_arrays[3] = { &inputs.input0, &inputs.input1, &inputs.input2 };
		for (uint32_t input_index = 0; input_index < 3; ++input_index)
		{
			const FloatType* values = &(*input_arrays[input_index])[call_index * 4];
			stream << (input_index == 0 ? "" : ", ") << "[" << values[0] << ", " << values[1] << ", " << values[2] << ", " << values[3] << "]";
		}
		stream << " (lane " << lane_index << ")";
		return stream.str();
	}

	//////////////////////////////////////////////////////////////////////////
	// The tests
	//////////////////////////////////////////////////////////////////////////

	template<typename FloatType>
	void test_function_ulp_impl(ulp_function function)
	{
		using traits = ulp_traits<FloatType>;
		using reference_type = typename traits::reference_type;

		const error_unit unit = get_error_unit(function);
		const double max_error = traits::get_max_error(function);
		const double max_path_difference = get_max_path_difference<FloatType>(function);

		const input_set<FloatType> inputs = get_inputs<FloatType>(function);
		const size_t num_calls = inputs.get_num_calls();

		double largest_error = 0.0;
		size_t largest_error_call = 0;
		uint32_t largest_error_lane = 0;
		double largest_path_difference = 0.0;
		size_t largest_path_difference_call = 0;
		uint32_t largest_path_difference_lane = 0;
		size_t num_tested_calls = 0;

		for (size_t call_index = 0; call_index < num_calls; ++call_index)
		{
			const FloatType* input0 = &inputs.input0[call_index * 4];
			const FloatType* input1 = &inputs.input1[call_index * 4];
			const FloatType* input2 = &inputs.input2[call_index * 4];

			reference_type exact[4];
			reference_type scale[4];
			const uint32_t num_lanes = compute_exact(function, input0, input1, input2, exact, scale);

			if (traits::flushes_denormals())
			{
				// The SIMD code path flushes the denormal inputs and results to zero
				bool has_denormal = false;
				for (uint32_t lane_index = 0; lane_index < 4; ++lane_index)
				{
					has_denormal |= is_denormal(input0[lane_index]) || is_denormal(input1[lane_index]) || is_denormal(input2[lane_index]);
					if (lane_index < num_lanes)
					{
						const FloatType rounded_exact = round_reference<FloatType>(exact[lane_index]);
						has_denormal |= is_denormal(rounded_exact) || (rounded_exact == FloatType(0.0) && exact[lane_index] != reference_type(0.0));
					}
				}

				if (has_denormal)
					continue;
			}

			// The results: the vector function, the reference path, then the alternatives
			FloatType results[2 + k_max_num_alternatives][4];
			compute_vector_function_impl(function, input0, input1, input2, results[0]);
			rtm_ulp_ref::compute_vector_function(function, input0, input1, input2, results[1]);
			const uint32_t num_results = 2 + compute_alternatives(function, input0, input1, input2, &results[2]);
			const FloatType* reference_path_result = results[1];

			for (uint32_t result_index = 0; result_index < num_results; ++result_index)
			{
				const FloatType* result = results[result_index];

				// When the sign of a quaternion is ambiguous, we compare with the exact result of the same sign
				reference_type exact_sign = reference_type(1.0);
				if (has_sign_ambiguity(function))
				{
					reference_type dot = reference_type(0.0);
					for (uint32_t lane_index = 0; lane_index < num_lanes; ++lane_index)
						dot += reference_type(result[lane_index]) * exact[lane_index];
					exact_sign = dot < reference_type(0.0) ? reference_type(-1.0) : reference_type(1.0);
				}

				const bool is_alternative = result_index >= 2;
				const bool check_path_difference = result_index != 1 && !calls_c_library<FloatType>(function, is_alternative);

				for (uint32_t lane_index = 0; lane_index < num_lanes; ++lane_index)
				{
					const double error = get_error(unit, result[lane_index], exact[lane_index] * exact_sign, scale[lane_index]);
					if (error > largest_error)
					{
						largest_error = error;
						largest_error_call = call_index;
						largest_error_lane = lane_index;
					}

					if (check_path_difference)
					{
						const double path_difference = get_error(unit, result[lane_index], reference_type(reference_path_result[lane_index]), scale[lane_index]);
						if (path_difference > largest_path_difference)
						{
							largest_path_difference = path_difference;
							largest_path_difference_call = call_index;
							largest_path_difference_lane = lane_index;
						}
					}
				}
			}

			num_tested_calls++;
		}

		CHECK(num_tested_calls > num_calls / 2);

		{
			INFO(get_function_name(function) << ": the largest error is " << largest_error << " " << get_unit_name(function)
				<< " for the input " << get_input_string(function, inputs, largest_error_call, largest_error_lane));
			CHECK(largest_error <= max_error);
		}

		{
			INFO(get_function_name(function) << ": the largest difference with the scalar code path is " << largest_path_difference << " " << get_unit_name(function)
				<< " for the input " << get_input_string(function, inputs, largest_path_difference_call, largest_path_difference_lane));
			CHECK(largest_path_difference <= max_path_difference);
		}
	}

	// vector_sincos and scalar_sincos must give the same results as the sin and cos functions
	template<typename FloatType>
	void test_sincos_impl()
	{
		const input_set<FloatType> inputs = get_inputs<FloatType>(ulp_function::sin);

		uint64_t largest_difference = 0;
		FloatType largest_difference_input = FloatType(0.0);

		for (size_t call_index = 0; call_index < inputs.get_num_calls(); ++call_index)
		{
			const FloatType* input = &inputs.input0[call_index * 4];
			const auto input_v = vector_load(input);

			FloatType sin_results[4];
			FloatType cos_results[4];
			vector_store(vector_sin(input_v), sin_results);
			vector_store(vector_cos(input_v), cos_results);

			auto sincos_sin = input_v;
			auto sincos_cos = input_v;
			vector_sincos(input_v, sincos_sin, sincos_cos);

			FloatType sincos_sin_results[4];
			FloatType sincos_cos_results[4];
			vector_store(sincos_sin, sincos_sin_results);
			vector_store(sincos_cos, sincos_cos_results);

			for (uint32_t lane_index = 0; lane_index < 4; ++lane_index)
			{
				FloatType scalar_sin_result;
				FloatType scalar_cos_result;
				scalar_sincos(input[lane_index], scalar_sin_result, scalar_cos_result);

				const uint64_t differences[] =
				{
					get_ulp_distance(sincos_sin_results[lane_index], sin_results[lane_index]),
					get_ulp_distance(sincos_cos_results[lane_index], cos_results[lane_index]),
					get_ulp_distance(scalar_sin_result, scalar_cast(scalar_sin(scalar_set(input[lane_index])))),
					get_ulp_distance(scalar_cos_result, scalar_cast(scalar_cos(scalar_set(input[lane_index])))),
				};

				for (const uint64_t difference : differences)
				{
					if (difference > largest_difference)
					{
						largest_difference = difference;
						largest_difference_input = input[lane_index];
					}
				}
			}
		}

		INFO("sincos: the largest difference with sin and cos is " << largest_difference << " ulp for the input "
			<< std::setprecision(std::numeric_limits<FloatType>::max_digits10) << largest_difference_input);
		CHECK(largest_difference == 0);
	}

	template<typename FloatType>
	void test_quat_normalize_deterministic_impl()
	{
		std::mt19937 generator(1000);

		uint64_t largest_path_difference = 0;
		FloatType largest_path_difference_input[4] = { FloatType(0.0), FloatType(0.0), FloatType(0.0), FloatType(0.0) };

		for (uint32_t input_index = 0; input_index < k_num_random_inputs / 4; ++input_index)
		{
			// A random scale tests the inputs far from a length of 1.0
			const double scale = std::ldexp(1.0, int(generator() % 33) - 16);

			FloatType input[4];
			for (FloatType& component : input)
				component = get_random_value<FloatType>(generator, -scale, scale);

			FloatType vector_result[4];
			FloatType reference_path_result[4];
			quat_store(quat_normalize_deterministic(quat_load(&input[0])), &vector_result[0]);
			rtm_ulp_ref::quat_normalize_deterministic(&input[0], &reference_path_result[0]);

			for (size_t component_index = 0; component_index < 4; ++component_index)
			{
				const uint64_t path_difference = get_ulp_distance(vector_result[component_index], reference_path_result[component_index]);
				if (path_difference > largest_path_difference)
				{
					largest_path_difference = path_difference;
					std::memcpy(&largest_path_difference_input[0], &input[0], sizeof(input));
				}
			}
		}

		INFO("The largest difference with the scalar code path is " << largest_path_difference << " ulp for the input "
			<< std::setprecision(std::numeric_limits<FloatType>::max_digits10)
			<< "[" << largest_path_difference_input[0] << ", " << largest_path_difference_input[1]
			<< ", " << largest_path_difference_input[2] << ", " << largest_path_difference_input[3] << "]");

		// The function gives the same result on all the code paths that round each operation to the float type
		const uint64_t max_path_difference = rtm_ulp_ref::has_excess_precision() ? uint64_t(ulp_traits<FloatType>::get_excess_precision_quat_difference()) : 0;
		CHECK(largest_path_difference <= max_path_difference);
	}

	//////////////////////////////////////////////////////////////////////////
	// The functions that must give the same bits on all the code paths
	//////////////////////////////////////////////////////////////////////////

	using rtm_ulp_ref::bit_exact_function;

	const char* get_bit_exact_function_name(bit_exact_function function)
	{
		static const char* const names[] =
		{
			"add", "sub", "mul", "div", "mul_scalar", "neg", "abs", "min", "max", "clamp", "sign", "copy_sign",
			"floor", "ceil", "round_bankers", "round_symmetric", "fraction",
			"and", "or", "xor",
			"equal", "not_equal", "less_than", "less_equal", "greater_than", "greater_equal", "finite", "select",
			"all_equal", "all_not_equal", "all_less_than", "all_less_equal", "all_greater_than", "all_greater_equal",
			"any_equal", "any_not_equal", "any_less_than", "any_less_equal", "any_greater_than", "any_greater_equal",
			"is_finite", "all_near_equal", "any_near_equal",
			"mask_and", "mask_or", "mask_xor", "mask_not", "mask_all_true", "mask_any_true", "mask_all_equal", "mask_any_equal", "mask_get",
			"load1", "load2", "load3", "broadcast", "store1", "store2", "store3",
			"set_components", "set_component", "get_components", "get_component", "dup_x", "dup_y", "dup_z", "dup_w", "min_max_component",
			"mix0", "mix1", "mix2",
		};
		static_assert(sizeof(names) / sizeof(names[0]) == size_t(bit_exact_function::count), "Add the name of each function");

		return names[uint32_t(function)];
	}

	// Computes the scalar function of the code path of the build for each lane, when one exists. Returns false otherwise.
	template<typename FloatType>
	bool compute_bit_exact_alternative(bit_exact_function function, const FloatType* input0, const FloatType* input1, const FloatType* input2, FloatType* output)
	{
		if (function == bit_exact_function::mul_scalar)
		{
			vector_store(vector_mul(vector_load(input0), scalar_set(input1[0])), output);
			return true;
		}

		for (uint32_t lane_index = 0; lane_index < 4; ++lane_index)
		{
			const auto value0 = scalar_set(input0[lane_index]);
			const auto value1 = scalar_set(input1[lane_index]);
			const auto value2 = scalar_set(input2[lane_index]);

			FloatType result;
			switch (function)
			{
			case bit_exact_function::add:				result = scalar_cast(scalar_add(value0, value1)); break;
			case bit_exact_function::sub:				result = scalar_cast(scalar_sub(value0, value1)); break;
			case bit_exact_function::mul:				result = scalar_cast(scalar_mul(value0, value1)); break;
			case bit_exact_function::div:				result = scalar_cast(scalar_div(value0, value1)); break;
			case bit_exact_function::abs:				result = scalar_cast(scalar_abs(value0)); break;
			case bit_exact_function::min:				result = scalar_cast(scalar_min(value0, value1)); break;
			case bit_exact_function::max:				result = scalar_cast(scalar_max(value0, value1)); break;
			case bit_exact_function::clamp:				result = scalar_cast(scalar_clamp(value0, value1, value2)); break;
			case bit_exact_function::floor:				result = scalar_cast(scalar_floor(value0)); break;
			case bit_exact_function::ceil:				result = scalar_cast(scalar_ceil(value0)); break;
			case bit_exact_function::round_bankers:		result = scalar_cast(scalar_round_bankers(value0)); break;
			case bit_exact_function::round_symmetric:	result = scalar_cast(scalar_round_symmetric(value0)); break;
			case bit_exact_function::equal:				result = get_bool_value<FloatType>(scalar_equal(value0, value1)); break;
			case bit_exact_function::less_than:			result = get_bool_value<FloatType>(scalar_lower_than(value0, value1)); break;
			case bit_exact_function::less_equal:		result = get_bool_value<FloatType>(scalar_lower_equal(value0, value1)); break;
			case bit_exact_function::greater_than:		result = get_bool_value<FloatType>(scalar_greater_than(value0, value1)); break;
			case bit_exact_function::greater_equal:		result = get_bool_value<FloatType>(scalar_greater_equal(value0, value1)); break;
			case bit_exact_function::finite:			result = get_bool_value<FloatType>(scalar_is_finite(value0)); break;
			default:									return false;
			}

			output[lane_index] = result;
		}

		return true;
	}

	// Returns the edge values of the bit-exact functions
	template<typename FloatType>
	std::vector<FloatType> get_bit_exact_edge_values()
	{
		using limits = std::numeric_limits<FloatType>;
		const FloatType mantissa_shift = FloatType(std::ldexp(1.0, limits::digits - 1));	// 2^23 or 2^52

		const FloatType values[] =
		{
			FloatType(0.0), FloatType(-0.0), FloatType(1.0), FloatType(-1.0), FloatType(0.5), FloatType(-0.5),
			FloatType(1.5), FloatType(-1.5), FloatType(2.5), FloatType(-2.5), FloatType(-3.5),
			std::nextafter(FloatType(0.5), FloatType(0.0)), std::nextafter(FloatType(-0.5), FloatType(0.0)),
			limits::infinity(), -limits::infinity(), limits::quiet_NaN(), -limits::quiet_NaN(),
			(limits::max)(), -(limits::max)(), (limits::min)(), -(limits::min)(), limits::denorm_min(), -limits::denorm_min(),
			mantissa_shift, -mantissa_shift, mantissa_shift + FloatType(1.0), -(mantissa_shift + FloatType(1.0)), mantissa_shift - FloatType(0.5),
			FloatType(2147483648.0), FloatType(-2147483648.0), FloatType(4294967296.5),
		};

		return std::vector<FloatType>(std::begin(values), std::end(values));
	}

	// Returns a random input value: an edge value, a random bit pattern, a multiple of 0.25, or a value in [-10, 10]
	template<typename FloatType>
	FloatType get_bit_exact_random_value(std::mt19937& generator, const std::vector<FloatType>& edge_values)
	{
		switch (generator() % 4)
		{
		case 0:		return edge_values[generator() % edge_values.size()];
		case 1:		return get_random_bits<FloatType>(generator);
		case 2:		return FloatType(int32_t(generator() % 161) - 80) * FloatType(0.25);
		default:	return get_random_value<FloatType>(generator, -10.0, 10.0);
		}
	}

	// Changes the inputs of the functions that have conditions on their inputs
	template<typename FloatType>
	void fix_bit_exact_inputs(bit_exact_function function, FloatType* input0, FloatType* input1, FloatType* input2)
	{
		(void)input0;

		if (function == bit_exact_function::clamp)
		{
			// The minimum must not be larger than the maximum, and the limits must not be NaN
			for (uint32_t lane_index = 0; lane_index < 4; ++lane_index)
			{
				const FloatType min_value = std::isnan(input1[lane_index]) ? FloatType(-1.0) : input1[lane_index];
				const FloatType max_value = std::isnan(input2[lane_index]) ? FloatType(1.0) : input2[lane_index];
				input1[lane_index] = (std::min)(min_value, max_value);
				input2[lane_index] = (std::max)(min_value, max_value);
			}
		}
		else if (function == bit_exact_function::all_near_equal || function == bit_exact_function::any_near_equal)
		{
			// The threshold must be a positive value
			input2[0] = std::isnan(input2[0]) ? FloatType(0.5) : std::fabs(input2[0]);
		}
	}

	// Returns true if the two values have the same bits. All the NaN values are equal.
	template<typename FloatType>
	bool has_same_bits(FloatType lhs, FloatType rhs)
	{
		if (std::isnan(lhs) || std::isnan(rhs))
			return std::isnan(lhs) && std::isnan(rhs);

		return std::memcmp(&lhs, &rhs, sizeof(FloatType)) == 0;
	}

	// Returns the value and its bits as text
	template<typename FloatType>
	std::string get_bits_string(FloatType value)
	{
		using uint_type = typename std::make_unsigned<typename ulp_traits<FloatType>::int_type>::type;
		uint_type bits;
		std::memcpy(&bits, &value, sizeof(FloatType));

		std::ostringstream stream;
		stream << std::setprecision(std::numeric_limits<FloatType>::max_digits10) << value << " (0x" << std::hex << uint64_t(bits) << ")";
		return stream.str();
	}

	template<typename FloatType>
	std::string get_vector_bits_string(const FloatType* values)
	{
		return "[" + get_bits_string(values[0]) + ", " + get_bits_string(values[1]) + ", " + get_bits_string(values[2]) + ", " + get_bits_string(values[3]) + "]";
	}

	// The functions in the list must give the same bits as the scalar code path
	template<typename FloatType>
	void test_bit_exact_function_impl(bit_exact_function function)
	{
		const std::vector<FloatType> edge_values = get_bit_exact_edge_values<FloatType>();
		std::mt19937 generator(2000 + uint32_t(function));

		input_set<FloatType> inputs;

		// Each edge value with each edge value
		for (size_t index0 = 0; index0 < edge_values.size(); ++index0)
		{
			FloatType values0[4];
			FloatType values1[4];
			FloatType values2[4];
			for (uint32_t lane_index = 0; lane_index < 4; ++lane_index)
			{
				values0[lane_index] = edge_values[(index0 + lane_index) % edge_values.size()];
				values2[lane_index] = edge_values[(index0 + (lane_index * 7)) % edge_values.size()];
			}

			for (size_t index1 = 0; index1 < edge_values.size(); ++index1)
			{
				for (uint32_t lane_index = 0; lane_index < 4; ++lane_index)
					values1[lane_index] = edge_values[(index1 + (lane_index * 3)) % edge_values.size()];

				FloatType fixed_values1[4];
				FloatType fixed_values2[4];
				std::memcpy(fixed_values1, values1, sizeof(values1));
				std::memcpy(fixed_values2, values2, sizeof(values2));
				fix_bit_exact_inputs(function, values0, fixed_values1, fixed_values2);
				inputs.add_call(values0, fixed_values1, fixed_values2);
			}
		}

		// Random values. A quarter of the lanes of input1 are equal to input0.
		for (uint32_t call_index = 0; call_index < k_num_random_inputs / 4; ++call_index)
		{
			FloatType values0[4];
			FloatType values1[4];
			FloatType values2[4];
			for (uint32_t lane_index = 0; lane_index < 4; ++lane_index)
			{
				values0[lane_index] = get_bit_exact_random_value(generator, edge_values);
				values1[lane_index] = (generator() % 4) == 0 ? values0[lane_index] : get_bit_exact_random_value(generator, edge_values);
				values2[lane_index] = get_bit_exact_random_value(generator, edge_values);
			}

			fix_bit_exact_inputs(function, values0, values1, values2);
			inputs.add_call(values0, values1, values2);
		}

		// With excess precision (x87), the scalar code path does not round abs(lhs - rhs) to the float type before the
		// comparison with the threshold. Thus the result of near_equal can differ when the difference is near the threshold.
		if (rtm_ulp_ref::has_excess_precision() && (function == bit_exact_function::all_near_equal || function == bit_exact_function::any_near_equal))
			return;

		// With excess precision (x87), the scalar code path rounds float64 arithmetic twice
		const bool allows_one_ulp = rtm_ulp_ref::has_excess_precision() && std::is_same<FloatType, double>::value &&
			(function == bit_exact_function::add || function == bit_exact_function::sub || function == bit_exact_function::mul ||
			 function == bit_exact_function::div || function == bit_exact_function::mul_scalar || function == bit_exact_function::fraction);

		size_t num_differences = 0;
		std::string first_difference;

		for (size_t call_index = 0; call_index < inputs.get_num_calls(); ++call_index)
		{
			const FloatType* input0 = &inputs.input0[call_index * 4];
			const FloatType* input1 = &inputs.input1[call_index * 4];
			const FloatType* input2 = &inputs.input2[call_index * 4];

			if (ulp_traits<FloatType>::flushes_denormals())
			{
				// The SIMD code path flushes the denormal inputs to zero
				bool has_denormal = false;
				for (uint32_t lane_index = 0; lane_index < 4; ++lane_index)
					has_denormal |= is_denormal(input0[lane_index]) || is_denormal(input1[lane_index]) || is_denormal(input2[lane_index]);
				if (has_denormal)
					continue;
			}

			FloatType reference_path_result[4];
			FloatType results[2][4];
			rtm_ulp_ref::compute_bit_exact_function(function, input0, input1, input2, reference_path_result);
			compute_bit_exact_function_impl(function, input0, input1, input2, results[0]);
			const uint32_t num_results = compute_bit_exact_alternative(function, input0, input1, input2, results[1]) ? 2 : 1;

			for (uint32_t result_index = 0; result_index < num_results; ++result_index)
			{
				const FloatType* result = results[result_index];

				bool is_same = true;
				for (uint32_t lane_index = 0; lane_index < 4; ++lane_index)
				{
					FloatType lhs = result[lane_index];
					FloatType rhs = reference_path_result[lane_index];
					if (ulp_traits<FloatType>::flushes_denormals())
					{
						// The SIMD code path flushes the denormal results to zero
						lhs = is_denormal(lhs) ? std::copysign(FloatType(0.0), lhs) : lhs;
						rhs = is_denormal(rhs) ? std::copysign(FloatType(0.0), rhs) : rhs;
					}

					if (!has_same_bits(lhs, rhs) && !(allows_one_ulp && get_ulp_distance(lhs, rhs) <= 1))
						is_same = false;
				}

				if (is_same)
					continue;

				if (num_differences == 0)
				{
					first_difference = std::string(result_index == 0 ? "the vector function" : "the scalar function")
						+ " returns " + get_vector_bits_string(result) + " and the scalar code path returns " + get_vector_bits_string(reference_path_result)
						+ " for the inputs " + get_vector_bits_string(input0) + ", " + get_vector_bits_string(input1) + ", " + get_vector_bits_string(input2);
				}

				num_differences++;
			}
		}

		INFO(get_bit_exact_function_name(function) << ": " << num_differences << " differences with the scalar code path. The first: " << first_difference);
		CHECK(num_differences == 0);
	}

	template<typename FloatType>
	void test_bit_exact_impl()
	{
		for (uint32_t function_index = 0; function_index < uint32_t(bit_exact_function::count); ++function_index)
			test_bit_exact_function_impl<FloatType>(bit_exact_function(function_index));
	}

	template<typename FloatType>
	void test_vector_products_ulp_impl()
	{
		test_function_ulp_impl<FloatType>(ulp_function::dot);
		test_function_ulp_impl<FloatType>(ulp_function::dot3);
		test_function_ulp_impl<FloatType>(ulp_function::dot2);
		test_function_ulp_impl<FloatType>(ulp_function::cross3);
		test_function_ulp_impl<FloatType>(ulp_function::mul_add);
		test_function_ulp_impl<FloatType>(ulp_function::neg_mul_sub);
		test_function_ulp_impl<FloatType>(ulp_function::lerp);
	}

	template<typename FloatType>
	void test_vector_length_ulp_impl()
	{
		test_function_ulp_impl<FloatType>(ulp_function::length);
		test_function_ulp_impl<FloatType>(ulp_function::length3);
		test_function_ulp_impl<FloatType>(ulp_function::length_squared);
		test_function_ulp_impl<FloatType>(ulp_function::length_squared3);
		test_function_ulp_impl<FloatType>(ulp_function::length_squared2);
		test_function_ulp_impl<FloatType>(ulp_function::length_reciprocal);
		test_function_ulp_impl<FloatType>(ulp_function::length_reciprocal3);
		test_function_ulp_impl<FloatType>(ulp_function::length_reciprocal2);
		test_function_ulp_impl<FloatType>(ulp_function::normalize);
		test_function_ulp_impl<FloatType>(ulp_function::normalize3);
		test_function_ulp_impl<FloatType>(ulp_function::normalize2);
		test_function_ulp_impl<FloatType>(ulp_function::distance3);
	}

	template<typename FloatType>
	void test_quat_ulp_impl()
	{
		test_function_ulp_impl<FloatType>(ulp_function::quat_mul);
		test_function_ulp_impl<FloatType>(ulp_function::quat_mul_vector3);
		test_function_ulp_impl<FloatType>(ulp_function::quat_normalize);
		test_function_ulp_impl<FloatType>(ulp_function::quat_dot);
		test_function_ulp_impl<FloatType>(ulp_function::quat_length);
		test_function_ulp_impl<FloatType>(ulp_function::quat_length_squared);
		test_function_ulp_impl<FloatType>(ulp_function::quat_length_reciprocal);
		test_function_ulp_impl<FloatType>(ulp_function::quat_lerp);
		test_function_ulp_impl<FloatType>(ulp_function::quat_slerp);
		test_function_ulp_impl<FloatType>(ulp_function::quat_from_euler);
		test_function_ulp_impl<FloatType>(ulp_function::quat_get_angle);
		test_function_ulp_impl<FloatType>(ulp_function::quat_rotation_log);
		test_function_ulp_impl<FloatType>(ulp_function::quat_rotation_exp);
		test_function_ulp_impl<FloatType>(ulp_function::quat_from_matrix);
	}
}

TEST_CASE("vector4f math exp/log ulp", "[math][vector4][ulp]")
{
	test_function_ulp_impl<float>(ulp_function::exp);
	test_function_ulp_impl<float>(ulp_function::log);
}

TEST_CASE("vector4d math exp/log ulp", "[math][vector4][ulp]")
{
	test_function_ulp_impl<double>(ulp_function::exp);
	test_function_ulp_impl<double>(ulp_function::log);
}

TEST_CASE("vector4f math sqrt/reciprocal ulp", "[math][vector4][ulp]")
{
	test_function_ulp_impl<float>(ulp_function::sqrt);
	test_function_ulp_impl<float>(ulp_function::reciprocal);
	test_function_ulp_impl<float>(ulp_function::sqrt_reciprocal);
}

TEST_CASE("vector4d math sqrt/reciprocal ulp", "[math][vector4][ulp]")
{
	test_function_ulp_impl<double>(ulp_function::sqrt);
	test_function_ulp_impl<double>(ulp_function::reciprocal);
	test_function_ulp_impl<double>(ulp_function::sqrt_reciprocal);
}

TEST_CASE("vector4f math trigonometry ulp", "[math][vector4][ulp]")
{
	test_function_ulp_impl<float>(ulp_function::sin);
	test_function_ulp_impl<float>(ulp_function::cos);
	test_function_ulp_impl<float>(ulp_function::tan);
	test_function_ulp_impl<float>(ulp_function::asin);
	test_function_ulp_impl<float>(ulp_function::acos);
	test_function_ulp_impl<float>(ulp_function::atan);
	test_function_ulp_impl<float>(ulp_function::atan2);
	test_sincos_impl<float>();
}

TEST_CASE("vector4d math sincos code paths", "[math][vector4][ulp]")
{
	test_sincos_impl<double>();
}

TEST_CASE("vector4f math products ulp", "[math][vector4][ulp]")
{
	test_vector_products_ulp_impl<float>();
}

TEST_CASE("vector4d math products ulp", "[math][vector4][ulp]")
{
	test_vector_products_ulp_impl<double>();
}

TEST_CASE("vector4f math length ulp", "[math][vector4][ulp]")
{
	test_vector_length_ulp_impl<float>();
}

TEST_CASE("vector4d math length ulp", "[math][vector4][ulp]")
{
	test_vector_length_ulp_impl<double>();
}

TEST_CASE("quatf math ulp", "[math][quat][ulp]")
{
	test_quat_ulp_impl<float>();
	test_function_ulp_impl<float>(ulp_function::quat_ensure_positive_w);
	test_function_ulp_impl<float>(ulp_function::quat_from_positive_w);
}

TEST_CASE("quatd math ulp", "[math][quat][ulp]")
{
	test_quat_ulp_impl<double>();
}

TEST_CASE("vector4f math bit exact code paths", "[math][vector4][ulp]")
{
	test_bit_exact_impl<float>();
}

TEST_CASE("vector4d math bit exact code paths", "[math][vector4][ulp]")
{
	test_bit_exact_impl<double>();
}

TEST_CASE("quatf math normalize_deterministic code paths", "[math][quat][ulp]")
{
	test_quat_normalize_deterministic_impl<float>();
}

TEST_CASE("quatd math normalize_deterministic code paths", "[math][quat][ulp]")
{
	test_quat_normalize_deterministic_impl<double>();
}
