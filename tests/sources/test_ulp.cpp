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

#include <rtm/quatd.h>
#include <rtm/quatf.h>
#include <rtm/scalard.h>
#include <rtm/scalarf.h>
#include <rtm/vector4d.h>
#include <rtm/vector4f.h>

#include "test_ulp_functions.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
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
//    - ulp of max(|exact result|, 1.0): the same unit when the result is 1.0 or more, and the
//      absolute error in units of epsilon below 1.0. RTM computes the trigonometric functions with
//      polynomial approximations that have a small absolute error. Near a zero of the function
//      (for example sin(pi)), the error in ulp is very large, but the absolute error is not.
//
// The tests also compare the code path of the build (SSE2, AVX, NEON, WASM, or scalar) with the
// scalar code path. test_ulp_reference.cpp compiles the scalar code path in the same executable.
//
// The inputs are edge values and random values. The random values are the same on all platforms:
// std::mt19937 gives the same sequence everywhere, and the tests do not use the std distributions.
//
// The float64 trigonometric functions call the C library (std::sin, std::cos, and so on).
// Thus the tests measure only the float32 trigonometric functions.

namespace
{
	using rtm_ulp_ref::ulp_function;

	// The number of random inputs for each function and each float type
	constexpr uint32_t k_num_random_inputs = 1 << 16;

	constexpr double k_pi = 3.14159265358979323846;

	// The unit of the error of a function
	enum class error_unit
	{
		ulp,			// The error in ulp
		ulp_above_one,	// The error in ulp of max(|exact result|, 1.0)
	};

	const char* get_function_name(ulp_function function)
	{
		switch (function)
		{
		case ulp_function::exp:				return "exp";
		case ulp_function::log:				return "log";
		case ulp_function::sqrt:			return "sqrt";
		case ulp_function::reciprocal:		return "reciprocal";
		case ulp_function::sqrt_reciprocal:	return "sqrt_reciprocal";
		case ulp_function::sin:				return "sin";
		case ulp_function::cos:				return "cos";
		case ulp_function::tan:				return "tan";
		case ulp_function::asin:			return "asin";
		case ulp_function::acos:			return "acos";
		case ulp_function::atan:			return "atan";
		case ulp_function::atan2:			return "atan2";
		}

		return "unknown";
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
			return error_unit::ulp;
		default:
			return error_unit::ulp_above_one;
		}
	}

	const char* get_unit_name(ulp_function function)
	{
		return get_error_unit(function) == error_unit::ulp ? "ulp" : "ulp of max(|exact result|, 1.0)";
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
			switch (function)
			{
			case ulp_function::exp:				return 1.0;
			case ulp_function::log:				return 1.0;
			case ulp_function::sqrt:			return 0.0;
			case ulp_function::reciprocal:		return 0.0;
			case ulp_function::sqrt_reciprocal:	return 1.0;		// 1.0 / sqrt(x) rounds two times
			case ulp_function::sin:				return 2.0;
			case ulp_function::cos:				return 3.0;
			case ulp_function::tan:				return 25.0;	// tan(x) = sin(x) / cos(x): near 1.5, cos(x) is small and the error of cos(x) is large in comparison
			case ulp_function::asin:			return 3.0;
			case ulp_function::acos:			return 3.0;
			case ulp_function::atan:			return 5.0;
			case ulp_function::atan2:			return 5.0;
			}

			return 0.0;
		}

		// Returns the largest difference between two code paths that do not round the same operations.
		// For example, one code path uses FMA and the other does not, or one code path computes with x87 registers.
		static double get_rounding_path_difference(ulp_function function)
		{
			switch (function)
			{
			case ulp_function::exp:				return 1.0;
			case ulp_function::log:				return 1.0;
			case ulp_function::sqrt:			return 0.0;
			case ulp_function::reciprocal:		return 0.0;
			case ulp_function::sqrt_reciprocal:	return 0.0;
			case ulp_function::sin:				return 1.5;
			case ulp_function::cos:				return 1.0;
			case ulp_function::tan:				return 22.0;
			case ulp_function::asin:			return 2.0;
			case ulp_function::acos:			return 2.0;
			case ulp_function::atan:			return 1.0;
			case ulp_function::atan2:			return 1.0;
			}

			return 0.0;
		}

		// Returns the largest difference that we permit between the code path of the build and the scalar code path
		static double get_max_simd_path_difference(ulp_function function)
		{
#if defined(RTM_NEON_INTRINSICS) || (defined(RTM_FMA_INTRINSICS) && defined(RTM_COMPILER_CLANG))
			// The NEON code path uses FMA and the scalar code path does not.
			// With FMA on x86 and x64, Clang merges the multiply and the add intrinsics into one FMA instruction.
			return get_rounding_path_difference(function);
#else
			// The SSE2 and WASM vector_asin do the operations in a different order than the scalar code path
			return function == ulp_function::asin ? 1.0 : 0.0;
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
				// Without excess precision, the float64 operations give the exact reference (see compute_exact).
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

		// Returns the largest error that we permit in ulp
		static double get_max_error(ulp_function function)
		{
			double max_error = 0.0;
			switch (function)
			{
			case ulp_function::exp:				max_error = 2.0; break;
			case ulp_function::log:				max_error = 2.0; break;
			case ulp_function::sqrt:			max_error = 0.0; break;
			case ulp_function::reciprocal:		max_error = 0.0; break;
			case ulp_function::sqrt_reciprocal:	max_error = 1.0; break;	// 1.0 / sqrt(x) rounds two times
			default:							break;
			}

			return max_error + get_reference_error(function);
		}

		// Returns the largest difference between two code paths that do not round the same operations
		static double get_rounding_path_difference(ulp_function function)
		{
			// For a few inputs of exp, the results differ by 2 ulp
			return function == ulp_function::exp || function == ulp_function::log ? 2.0 : 0.0;
		}

		// Returns the largest difference that we permit between the code path of the build and the scalar code path
		static double get_max_simd_path_difference(ulp_function function)
		{
#if defined(RTM_NEON_INTRINSICS)
			// The NEON code path uses FMA and the scalar code path does not
			return get_rounding_path_difference(function);
#else
			(void)function;
			return 0.0;
#endif
		}

		// Returns the largest difference between the scalar code path with excess precision (x87) and the other code paths
		static double get_excess_precision_path_difference(ulp_function function)
		{
			switch (function)
			{
			case ulp_function::exp:				return 2.0;
			case ulp_function::log:				return 2.0;
			case ulp_function::sqrt:			return 1.0;	// The x87 square root and division round two times
			case ulp_function::reciprocal:		return 1.0;
			case ulp_function::sqrt_reciprocal:	return 2.0;
			default:							return 0.0;
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

	// Returns the distance between a result and a reference value in ulp of max(|reference|, 1.0).
	// If a value is not finite, returns the distance in ulp between the result and the rounded reference.
	template<typename FloatType, typename ReferenceType>
	double get_distance_above_one(FloatType result, ReferenceType reference, FloatType rounded_reference)
	{
		if (!std::isfinite(result) || !std::isfinite(rounded_reference))
			return double(get_ulp_distance(result, rounded_reference));

		// The ulp of a value in the range [2^(e-1), 2^e) is 2^(e - digits)
		int exponent;
		(void)std::frexp((std::max)(ReferenceType(std::fabs(reference)), ReferenceType(1.0)), &exponent);
		const ReferenceType ulp = std::ldexp(ReferenceType(1.0), exponent - std::numeric_limits<FloatType>::digits);

		return double(ReferenceType(std::fabs(ReferenceType(result) - reference)) / ulp);
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

	// Computes the exact result with the reference type
	template<typename FloatType, typename ReferenceType = typename ulp_traits<FloatType>::reference_type>
	ReferenceType compute_exact(ulp_function function, FloatType float_input0, FloatType float_input1)
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
		}

		return ReferenceType(0.0);
	}

	// Computes the scalar function of the code path of the build
	template<typename FloatType>
	FloatType compute_scalar_function(ulp_function function, FloatType input0, FloatType input1)
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
		}

		return FloatType(0.0);
	}

	// Returns true if the function of the code path of the build calls the C library (std::sin, std::acos, and so on)
	template<typename FloatType>
	bool calls_c_library(ulp_function function, bool is_scalar_function)
	{
		// The float64 trigonometric functions call the C library on all the code paths, but the tests do not measure them
		if (!std::is_same<FloatType, float>::value)
			return false;

#if defined(RTM_NEON_INTRINSICS)
		// With NEON, the float32 scalar trigonometric functions, vector_asin, and vector_acos call the C library
		switch (function)
		{
		case ulp_function::sin:
		case ulp_function::cos:
		case ulp_function::tan:
		case ulp_function::atan:
		case ulp_function::atan2:
			return is_scalar_function;
		case ulp_function::asin:
		case ulp_function::acos:
			return true;
		default:
			return false;
		}
#else
		(void)function;
		(void)is_scalar_function;
		return false;
#endif
	}

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

	// The inputs of a function. The functions with one input do not read the second input.
	template<typename FloatType>
	struct input_set
	{
		std::vector<FloatType> input0;
		std::vector<FloatType> input1;

		void add(FloatType value0, FloatType value1 = FloatType(1.0))
		{
			input0.push_back(value0);
			input1.push_back(value1);
		}

		// Adds the value and the float values just below and just above it
		void add_with_neighbors(FloatType value)
		{
			add(value);
			add(std::nextafter(value, -std::numeric_limits<FloatType>::infinity()));
			add(std::nextafter(value, std::numeric_limits<FloatType>::infinity()));
		}
	};

	template<typename FloatType>
	void add_edge_inputs(ulp_function function, input_set<FloatType>& inputs)
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
				inputs.add(value);
			break;
		}
		case ulp_function::sin:
		case ulp_function::cos:
		{
			const FloatType angles[] = { FloatType(0.0), FloatType(k_pi * 0.25), FloatType(k_pi * 0.5), FloatType(k_pi), FloatType(k_pi * 1.5), FloatType(k_pi * 2.0) };
			for (const FloatType angle : angles)
			{
				inputs.add_with_neighbors(angle);
				inputs.add_with_neighbors(-angle);
			}

			const FloatType special_inputs[] = { infinity, -infinity, nan, min_value, -min_value, denormal_min_value, -denormal_min_value };
			for (const FloatType value : special_inputs)
				inputs.add(value);
			break;
		}
		case ulp_function::tan:
		{
			// The result of tan near pi/2 is very large: the tests do not use these inputs
			const FloatType angles[] = { FloatType(0.0), FloatType(k_pi * 0.25), FloatType(1.0), FloatType(1.5) };
			for (const FloatType angle : angles)
			{
				inputs.add_with_neighbors(angle);
				inputs.add_with_neighbors(-angle);
			}

			const FloatType special_inputs[] = { infinity, -infinity, nan, min_value, -min_value, denormal_min_value, -denormal_min_value };
			for (const FloatType value : special_inputs)
				inputs.add(value);
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
				inputs.add(value);
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
						inputs.add(y, x);
				}
			}
			break;
		}
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
				inputs.add_with_neighbors(threshold);
		}
	}

	template<typename FloatType>
	input_set<FloatType> get_inputs(ulp_function function)
	{
		input_set<FloatType> inputs;
		add_edge_inputs(function, inputs);

		// Each function has its own seed: a new function does not change the inputs of the other functions
		std::mt19937 generator(uint32_t(function) + 1);
		const bool is_float32 = std::is_same<FloatType, float>::value;

		for (uint32_t input_index = 0; input_index < k_num_random_inputs; ++input_index)
		{
			const bool is_even = (input_index % 2) == 0;

			switch (function)
			{
			case ulp_function::exp:
				// The range includes the inputs where the result overflows and where the result is zero
				inputs.add(is_float32 ? get_random_value<FloatType>(generator, -110.0, 95.0) : get_random_value<FloatType>(generator, -750.0, 715.0));
				break;
			case ulp_function::log:
				// All the exponents, and the range near 1.0 where the result is near zero
				inputs.add(is_even ? get_random_positive_bits(generator, FloatType()) : get_random_value<FloatType>(generator, 0.5, 2.0));
				break;
			case ulp_function::sqrt:
			case ulp_function::sqrt_reciprocal:
				inputs.add(get_random_positive_bits(generator, FloatType()));
				break;
			case ulp_function::reciprocal:
				inputs.add(get_random_bits<FloatType>(generator));
				break;
			case ulp_function::sin:
			case ulp_function::cos:
				// One full turn in each direction
				inputs.add(get_random_value<FloatType>(generator, -2.0 * k_pi, 2.0 * k_pi));
				break;
			case ulp_function::tan:
				inputs.add(get_random_value<FloatType>(generator, -1.5, 1.5));
				break;
			case ulp_function::asin:
			case ulp_function::acos:
			{
				// The full range, and the small values with all the exponents
				FloatType value = get_random_value<FloatType>(generator, -1.0, 1.0);
				if (!is_even)
					value = value * FloatType(std::ldexp(1.0, -int(generator() % 32)));
				inputs.add(value);
				break;
			}
			case ulp_function::atan:
				inputs.add(is_even ? get_random_bits<FloatType>(generator) : get_random_value<FloatType>(generator, -4.0, 4.0));
				break;
			case ulp_function::atan2:
			{
				// A random scale for each input gives a large range of ratios
				const double y_scale = std::ldexp(1.0, int(generator() % 17) - 8);
				const double x_scale = std::ldexp(1.0, int(generator() % 17) - 8);
				const FloatType y = get_random_value<FloatType>(generator, -y_scale, y_scale);
				const FloatType x = get_random_value<FloatType>(generator, -x_scale, x_scale);
				inputs.add(y, x);
				break;
			}
			}
		}

		// The functions process 4 inputs at a time
		while ((inputs.input0.size() % 4) != 0)
			inputs.add(FloatType(1.0));

		return inputs;
	}

	// Returns the inputs of a function as text, with all the digits
	template<typename FloatType>
	std::string get_input_string(ulp_function function, const FloatType* inputs)
	{
		std::ostringstream stream;
		stream << std::setprecision(std::numeric_limits<FloatType>::max_digits10) << inputs[0];
		if (function == ulp_function::atan2)
			stream << ", " << inputs[1];
		return stream.str();
	}

	template<typename FloatType>
	void test_function_ulp_impl(ulp_function function)
	{
		using traits = ulp_traits<FloatType>;
		using reference_type = typename traits::reference_type;

		const error_unit unit = get_error_unit(function);
		const double max_error = traits::get_max_error(function);
		const double max_path_difference = get_max_path_difference<FloatType>(function);

		const input_set<FloatType> inputs = get_inputs<FloatType>(function);

		double largest_error = 0.0;
		FloatType largest_error_input[2] = { FloatType(0.0), FloatType(0.0) };
		double largest_path_difference = 0.0;
		FloatType largest_path_difference_input[2] = { FloatType(0.0), FloatType(0.0) };
		size_t num_tested_inputs = 0;

		for (size_t input_index = 0; input_index < inputs.input0.size(); input_index += 4)
		{
			const FloatType* input0 = &inputs.input0[input_index];
			const FloatType* input1 = &inputs.input1[input_index];

			FloatType vector_results[4];
			FloatType reference_path_results[4];
			compute_vector_function_impl(function, input0, input1, &vector_results[0]);
			rtm_ulp_ref::compute_vector_function(function, input0, input1, &reference_path_results[0]);

			for (size_t lane_index = 0; lane_index < 4; ++lane_index)
			{
				const FloatType lane_input0 = input0[lane_index];
				const FloatType lane_input1 = input1[lane_index];
				const reference_type exact_result = compute_exact(function, lane_input0, lane_input1);
				const FloatType rounded_result = round_reference<FloatType>(exact_result);

				if (traits::flushes_denormals())
				{
					const bool has_denormal_input = is_denormal(lane_input0) || (function == ulp_function::atan2 && is_denormal(lane_input1));
					const bool has_denormal_result = is_denormal(rounded_result) || (rounded_result == FloatType(0.0) && exact_result != reference_type(0.0));
					if (has_denormal_input || has_denormal_result)
						continue;	// The SIMD code path flushes the input or the result to zero
				}

				const FloatType results[] =
				{
					compute_scalar_function(function, lane_input0, lane_input1),
					vector_results[lane_index],
					reference_path_results[lane_index],
				};

				double error = 0.0;
				for (const FloatType result : results)
				{
					const double result_error = unit == error_unit::ulp ? double(get_ulp_distance(result, rounded_result)) : get_distance_above_one(result, exact_result, rounded_result);
					error = (std::max)(error, result_error);
				}

				// We compare the scalar and the vector results of the build with the result of the scalar code path
				const FloatType reference_path_result = reference_path_results[lane_index];
				double path_difference = 0.0;
				for (size_t result_index = 0; result_index < 2; ++result_index)
				{
					if (calls_c_library<FloatType>(function, result_index == 0))
						continue;	// The result does not come from the code of RTM

					const FloatType result = results[result_index];
					const double result_difference = unit == error_unit::ulp ? double(get_ulp_distance(result, reference_path_result)) : get_distance_above_one(result, reference_type(reference_path_result), reference_path_result);
					path_difference = (std::max)(path_difference, result_difference);
				}

				if (error > largest_error)
				{
					largest_error = error;
					largest_error_input[0] = lane_input0;
					largest_error_input[1] = lane_input1;
				}

				if (path_difference > largest_path_difference)
				{
					largest_path_difference = path_difference;
					largest_path_difference_input[0] = lane_input0;
					largest_path_difference_input[1] = lane_input1;
				}

				num_tested_inputs++;
			}
		}

		CHECK(num_tested_inputs > k_num_random_inputs / 2);

		{
			INFO(get_function_name(function) << ": the largest error is " << largest_error << " " << get_unit_name(function)
				<< " for the input " << get_input_string(function, largest_error_input));
			CHECK(largest_error <= max_error);
		}

		{
			INFO(get_function_name(function) << ": the largest difference with the scalar code path is " << largest_path_difference << " " << get_unit_name(function)
				<< " for the input " << get_input_string(function, largest_path_difference_input));
			CHECK(largest_path_difference <= max_path_difference);
		}
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
}

TEST_CASE("quatf math normalize_deterministic code paths", "[math][quat][ulp]")
{
	test_quat_normalize_deterministic_impl<float>();
}

TEST_CASE("quatd math normalize_deterministic code paths", "[math][quat][ulp]")
{
	test_quat_normalize_deterministic_impl<double>();
}
