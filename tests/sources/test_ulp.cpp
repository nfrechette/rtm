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
#include "test_ulp_reference.h"

#include <rtm/quatd.h>
#include <rtm/quatf.h>
#include <rtm/scalard.h>
#include <rtm/scalarf.h>
#include <rtm/vector4d.h>
#include <rtm/vector4f.h>

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iterator>
#include <limits>
#include <random>
#include <type_traits>
#include <vector>

using namespace rtm;

// These tests measure the error of a function in ulp (units in the last place).
// The error of a result is the number of representable values between the result and the
// correctly rounded value of the function. A correctly rounded result has an error of 0 ulp.
//
// The tests also compare the code path of the build (SSE2, AVX, NEON, WASM, or scalar) with the
// scalar code path. test_ulp_reference.cpp compiles the scalar code path in the same executable.
//
// The inputs are edge values and random values. The random values are the same on all platforms:
// std::mt19937 gives the same sequence everywhere, and the tests do not use the std distributions.

namespace
{
	// The number of random inputs for each function and each float type
	constexpr uint32_t k_num_random_inputs = 1 << 16;

	enum class ulp_function
	{
		exp,
		log,
	};

	const char* get_function_name(ulp_function function)
	{
		return function == ulp_function::exp ? "exp" : "log";
	}

	template<typename FloatType>
	struct ulp_traits;

	template<>
	struct ulp_traits<float>
	{
		using int_type = int32_t;

		// A float64 value has 29 more bits than a float32 value: it is a sufficient reference
		using reference_type = double;

		static uint64_t get_max_exp_error() { return 1; }
		static uint64_t get_max_log_error() { return 1; }

		static uint64_t get_max_path_difference()
		{
#if defined(RTM_NEON_INTRINSICS)
			// The NEON code path uses FMA and the scalar code path does not: the last bit can differ
			return 1;
#else
			return 0;
#endif
		}

		static uint64_t get_excess_precision_path_difference() { return 1; }

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

		static uint64_t get_reference_error()
		{
#if LDBL_MANT_DIG == 64
			return 0;
#else
			// The float64 reference can have an error of 1 ulp
			return 1;
#endif
		}

		static uint64_t get_max_exp_error() { return 2 + get_reference_error(); }
		static uint64_t get_max_log_error() { return 2 + get_reference_error(); }

		static uint64_t get_max_path_difference()
		{
#if defined(RTM_NEON_INTRINSICS)
			// The NEON code path uses FMA and the scalar code path does not.
			// For a few inputs of exp, the results differ by 2 ulp.
			return 2;
#else
			return 0;
#endif
		}

		static uint64_t get_excess_precision_path_difference() { return 2; }

		static bool flushes_denormals() { return false; }
	};

	// Returns the largest difference in ulp that we permit between the code path of the build and the scalar code path.
	// With excess precision (x87), the scalar code path rounds some results twice:
	// first to the precision of the register, then to float32 or float64. Thus the last bits can differ.
	template<typename FloatType>
	uint64_t get_max_path_difference(uint64_t max_simd_path_difference)
	{
		if (!rtm_ulp_ref::has_excess_precision())
			return max_simd_path_difference;

		return (std::max<uint64_t>)(max_simd_path_difference, ulp_traits<FloatType>::get_excess_precision_path_difference());
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

	template<typename FloatType>
	void add_edge_inputs(ulp_function function, std::vector<FloatType>& inputs)
	{
		using limits = std::numeric_limits<FloatType>;
		using reference_type = typename ulp_traits<FloatType>::reference_type;

		const FloatType infinity = limits::infinity();
		const FloatType max_value = (limits::max)();
		const FloatType min_value = (limits::min)();
		const FloatType denormal_min_value = limits::denorm_min();
		const FloatType mantissa_shift = FloatType(std::ldexp(FloatType(1.0), limits::digits - 1));	// 2^23 or 2^52

		const FloatType common_inputs[] =
		{
			FloatType(0.0), FloatType(-0.0), FloatType(1.0), FloatType(-1.0), FloatType(0.5), FloatType(2.0),
			infinity, -infinity, limits::quiet_NaN(),
			max_value, -max_value, min_value, -min_value, denormal_min_value, -denormal_min_value,
			limits::epsilon(), FloatType(1.0) + limits::epsilon(), FloatType(1.0) - (limits::epsilon() * FloatType(0.5)),
			FloatType(0.70710678118654752440), FloatType(1.41421356237309504880), FloatType(0.69314718055994530942),
			mantissa_shift, mantissa_shift * FloatType(1.5),
		};
		inputs.insert(inputs.end(), std::begin(common_inputs), std::end(common_inputs));

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
			{
				inputs.push_back(threshold);
				inputs.push_back(std::nextafter(threshold, -infinity));
				inputs.push_back(std::nextafter(threshold, infinity));
			}
		}
	}

	template<typename FloatType>
	std::vector<FloatType> get_inputs(ulp_function function)
	{
		std::vector<FloatType> inputs;
		add_edge_inputs(function, inputs);

		std::mt19937 generator(function == ulp_function::exp ? 1 : 2);
		const bool is_float32 = std::is_same<FloatType, float>::value;

		for (uint32_t input_index = 0; input_index < k_num_random_inputs; ++input_index)
		{
			if (function == ulp_function::exp)
			{
				// The range includes the inputs where the result overflows and where the result is zero
				inputs.push_back(is_float32 ? get_random_value<FloatType>(generator, -110.0, 95.0) : get_random_value<FloatType>(generator, -750.0, 715.0));
			}
			else if ((input_index % 2) == 0)
			{
				// All the exponents
				inputs.push_back(get_random_positive_bits(generator, FloatType()));
			}
			else
			{
				// The results near zero are the most difficult to compute
				inputs.push_back(get_random_value<FloatType>(generator, 0.5, 2.0));
			}
		}

		// The functions process 4 inputs at a time
		while ((inputs.size() % 4) != 0)
			inputs.push_back(FloatType(1.0));

		return inputs;
	}

	template<typename FloatType>
	FloatType compute_scalar(ulp_function function, FloatType input)
	{
		return function == ulp_function::exp ? scalar_cast(scalar_exp(scalar_set(input))) : scalar_cast(scalar_log(scalar_set(input)));
	}

	template<typename FloatType>
	void compute_vector(ulp_function function, const FloatType* input, FloatType* output)
	{
		if (function == ulp_function::exp)
			vector_store(vector_exp(vector_load(input)), output);
		else
			vector_store(vector_log(vector_load(input)), output);
	}

	template<typename FloatType>
	void compute_reference_path(ulp_function function, const FloatType* input, FloatType* output)
	{
		if (function == ulp_function::exp)
			rtm_ulp_ref::vector_exp(input, output);
		else
			rtm_ulp_ref::vector_log(input, output);
	}

	template<typename FloatType>
	void test_exp_log_ulp_impl(ulp_function function)
	{
		using traits = ulp_traits<FloatType>;
		using reference_type = typename traits::reference_type;

		const uint64_t max_error = function == ulp_function::exp ? traits::get_max_exp_error() : traits::get_max_log_error();
		const uint64_t max_path_difference = get_max_path_difference<FloatType>(traits::get_max_path_difference());

		const std::vector<FloatType> inputs = get_inputs<FloatType>(function);

		uint64_t largest_error = 0;
		FloatType largest_error_input = FloatType(0.0);
		uint64_t largest_path_difference = 0;
		FloatType largest_path_difference_input = FloatType(0.0);
		size_t num_tested_inputs = 0;

		for (size_t input_index = 0; input_index < inputs.size(); input_index += 4)
		{
			FloatType vector_results[4];
			FloatType reference_path_results[4];
			compute_vector(function, &inputs[input_index], &vector_results[0]);
			compute_reference_path(function, &inputs[input_index], &reference_path_results[0]);

			for (size_t lane_index = 0; lane_index < 4; ++lane_index)
			{
				const FloatType input = inputs[input_index + lane_index];
				const reference_type exact_result = function == ulp_function::exp ? std::exp(reference_type(input)) : std::log(reference_type(input));
				const FloatType rounded_result = round_reference<FloatType>(exact_result);

				if (traits::flushes_denormals() && (is_denormal(input) || is_denormal(rounded_result) || (rounded_result == FloatType(0.0) && exact_result != reference_type(0.0))))
					continue;	// The SIMD code path flushes the input or the result to zero

				const FloatType scalar_result = compute_scalar(function, input);
				const FloatType vector_result = vector_results[lane_index];
				const FloatType reference_path_result = reference_path_results[lane_index];

				uint64_t error = get_ulp_distance(vector_result, rounded_result);
				error = (std::max<uint64_t>)(error, get_ulp_distance(scalar_result, rounded_result));
				error = (std::max<uint64_t>)(error, get_ulp_distance(reference_path_result, rounded_result));

				const uint64_t path_difference = (std::max<uint64_t>)(get_ulp_distance(vector_result, reference_path_result), get_ulp_distance(scalar_result, reference_path_result));

				if (error > largest_error)
				{
					largest_error = error;
					largest_error_input = input;
				}

				if (path_difference > largest_path_difference)
				{
					largest_path_difference = path_difference;
					largest_path_difference_input = input;
				}

				num_tested_inputs++;
			}
		}

		CHECK(num_tested_inputs > k_num_random_inputs / 2);

		{
			INFO(get_function_name(function) << ": the largest error is " << largest_error << " ulp for the input " << std::setprecision(std::numeric_limits<FloatType>::max_digits10) << largest_error_input);
			CHECK(largest_error <= max_error);
		}

		{
			INFO(get_function_name(function) << ": the largest difference with the scalar code path is " << largest_path_difference << " ulp for the input " << std::setprecision(std::numeric_limits<FloatType>::max_digits10) << largest_path_difference_input);
			CHECK(largest_path_difference <= max_path_difference);
		}
	}

	template<typename FloatType>
	void test_quat_normalize_deterministic_impl()
	{
		std::mt19937 generator(3);

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
		const uint64_t max_path_difference = get_max_path_difference<FloatType>(0);
		CHECK(largest_path_difference <= max_path_difference);
	}
}

TEST_CASE("vector4f math exp/log ulp", "[math][vector4][ulp]")
{
	test_exp_log_ulp_impl<float>(ulp_function::exp);
	test_exp_log_ulp_impl<float>(ulp_function::log);
}

TEST_CASE("vector4d math exp/log ulp", "[math][vector4][ulp]")
{
	test_exp_log_ulp_impl<double>(ulp_function::exp);
	test_exp_log_ulp_impl<double>(ulp_function::log);
}

TEST_CASE("quatf math normalize_deterministic code paths", "[math][quat][ulp]")
{
	test_quat_normalize_deterministic_impl<float>();
}

TEST_CASE("quatd math normalize_deterministic code paths", "[math][quat][ulp]")
{
	test_quat_normalize_deterministic_impl<double>();
}
