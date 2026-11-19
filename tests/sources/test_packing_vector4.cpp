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

#include <rtm/vector4f.h>
#include <rtm/packing/vector4f.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

using namespace rtm;

namespace
{
	uint32_t test_float_bits(float value)
	{
		uint32_t result;
		std::memcpy(&result, &value, sizeof(float));
		return result;
	}

	float test_bits_float(uint32_t value)
	{
		float result;
		std::memcpy(&result, &value, sizeof(float));
		return result;
	}

	bool is_half_nan(uint16_t value)
	{
		return (value & 0x7C00) == 0x7C00 && (value & 0x03FF) != 0;
	}

	// The absolute value of a finite half, computed with double arithmetic. The function returns 65536.0 for
	// the infinity: it is the next value after the largest half when the conversion rounds a float.
	double half_abs_to_double(uint16_t value)
	{
		const int exponent = (value >> 10) & 0x1F;
		const int mantissa = value & 0x03FF;
		if (exponent == 0)
			return std::ldexp(double(mantissa), -24);
		return std::ldexp(double(1024 + mantissa), exponent - 25);
	}

	// The reference conversion from half to float, independent of the code in RTM
	float reference_half_to_float(uint16_t value)
	{
		const bool is_negative = (value & 0x8000) != 0;
		const uint16_t abs_value = static_cast<uint16_t>(value & 0x7FFF);

		float result;
		if (abs_value == 0x7C00)
			result = std::numeric_limits<float>::infinity();
		else if (abs_value > 0x7C00)
			result = std::numeric_limits<float>::quiet_NaN();
		else
			result = float(half_abs_to_double(abs_value));	// The conversion is exact

		return is_negative ? -result : result;
	}

	uint16_t pack_one(float value)
	{
		uint16_t result[4];
		vector_pack_half(vector_set(value), &result[0]);
		return result[0];
	}
}

TEST_CASE("vector4f packing half unpack", "[math][vector4][packing]")
{
	// Test all the half values, four at a time
	for (uint32_t value = 0; value < 0x10000; value += 4)
	{
		const uint16_t input[4] = { uint16_t(value), uint16_t(value + 1), uint16_t(value + 2), uint16_t(value + 3) };
		const vector4f result = vector_unpack_half(&input[0]);

		float lanes[4];
		vector_store(result, &lanes[0]);

		for (uint32_t lane_index = 0; lane_index < 4; ++lane_index)
		{
			const uint16_t half_value = input[lane_index];
			const float expected = reference_half_to_float(half_value);
			const float actual = lanes[lane_index];

			if (is_half_nan(half_value))
			{
				if (!std::isnan(actual))
				{
					INFO("half: " << half_value);
					CHECK(std::isnan(actual));
				}
			}
			else if (test_float_bits(actual) != test_float_bits(expected))
			{
				INFO("half: " << half_value << " expected: " << expected << " actual: " << actual);
				CHECK(test_float_bits(actual) == test_float_bits(expected));
			}
		}
	}
}

TEST_CASE("vector4f packing half round trip", "[math][vector4][packing]")
{
	// Each half value must give the same half value after an unpack and a pack
	for (uint32_t value = 0; value < 0x10000; value += 4)
	{
		const uint16_t input[4] = { uint16_t(value), uint16_t(value + 1), uint16_t(value + 2), uint16_t(value + 3) };

		uint16_t output[4];
		vector_pack_half(vector_unpack_half(&input[0]), &output[0]);

		for (uint32_t lane_index = 0; lane_index < 4; ++lane_index)
		{
			const uint16_t half_value = input[lane_index];

			if (is_half_nan(half_value))
			{
				if (!is_half_nan(output[lane_index]))
				{
					INFO("half: " << half_value << " output: " << output[lane_index]);
					CHECK(is_half_nan(output[lane_index]));
				}
			}
			else if (output[lane_index] != half_value)
			{
				INFO("half: " << half_value << " output: " << output[lane_index]);
				CHECK(output[lane_index] == half_value);
			}
		}
	}
}

TEST_CASE("vector4f packing half rounding", "[math][vector4][packing]")
{
	// For each pair of consecutive positive finite halves, test the midpoint and the floats next to it.
	// The midpoint has 12 significant bits or fewer, thus a float holds it exactly.
	// The last pair is the largest half and the infinity: the midpoint 65520.0 rounds to the infinity.
	for (uint32_t value = 0; value < 0x7C00; ++value)
	{
		const uint16_t lower = uint16_t(value);
		const uint16_t upper = uint16_t(value + 1);
		const uint16_t even = (lower & 1) == 0 ? lower : upper;

		const float midpoint = float((half_abs_to_double(lower) + half_abs_to_double(upper)) * 0.5);
		const float below_midpoint = std::nextafter(midpoint, 0.0F);
		const float above_midpoint = std::nextafter(midpoint, std::numeric_limits<float>::infinity());

		uint16_t output[4];
		vector_pack_half(vector_set(midpoint, below_midpoint, above_midpoint, -midpoint), &output[0]);

		if (output[0] != even || output[1] != lower || output[2] != upper || output[3] != uint16_t(even | 0x8000))
		{
			INFO("lower: " << lower << " midpoint: " << midpoint);
			CHECK(output[0] == even);
			CHECK(output[1] == lower);
			CHECK(output[2] == upper);
			CHECK(output[3] == uint16_t(even | 0x8000));
		}
	}
}

TEST_CASE("vector4f packing half special values", "[math][vector4][packing]")
{
	const float infinity = std::numeric_limits<float>::infinity();

	// Zero keeps its sign
	CHECK(pack_one(0.0F) == 0x0000);
	CHECK(pack_one(-0.0F) == 0x8000);

	// Infinity and the values that are too large
	CHECK(pack_one(infinity) == 0x7C00);
	CHECK(pack_one(-infinity) == 0xFC00);
	CHECK(pack_one(65504.0F) == 0x7BFF);
	CHECK(pack_one(65519.996F) == 0x7BFF);
	CHECK(pack_one(65520.0F) == 0x7C00);
	CHECK(pack_one(65536.0F) == 0x7C00);
	CHECK(pack_one(1.0E10F) == 0x7C00);
	CHECK(pack_one(-1.0E10F) == 0xFC00);
	CHECK(pack_one((std::numeric_limits<float>::max)()) == 0x7C00);
	CHECK(pack_one(-(std::numeric_limits<float>::max)()) == 0xFC00);

	// NaN
	CHECK(is_half_nan(pack_one(std::numeric_limits<float>::quiet_NaN())));
	CHECK(is_half_nan(pack_one(-std::numeric_limits<float>::quiet_NaN())));
	CHECK(is_half_nan(pack_one(test_bits_float(0x7F800001U))));	// The payload is only in the low bits
	CHECK(is_half_nan(pack_one(test_bits_float(0x7FC00000U))));

	// The small values
	const float smallest_subnormal_half = std::ldexp(1.0F, -24);
	CHECK(pack_one(smallest_subnormal_half) == 0x0001);
	CHECK(pack_one(-smallest_subnormal_half) == 0x8001);
	CHECK(pack_one(std::ldexp(1.0F, -14)) == 0x0400);	// The smallest normal half
	CHECK(pack_one(std::ldexp(1.0F, -25)) == 0x0000);	// Half of the smallest subnormal half rounds to even
	CHECK(pack_one(-std::ldexp(1.0F, -25)) == 0x8000);
	CHECK(pack_one(std::nextafter(std::ldexp(1.0F, -25), 1.0F)) == 0x0001);
	CHECK(pack_one(std::ldexp(1.0F, -26)) == 0x0000);
	CHECK(pack_one((std::numeric_limits<float>::min)()) == 0x0000);
	CHECK(pack_one(std::numeric_limits<float>::denorm_min()) == 0x0000);
	CHECK(pack_one(-std::numeric_limits<float>::denorm_min()) == 0x8000);

	// The other values
	CHECK(pack_one(1.0F) == 0x3C00);
	CHECK(pack_one(-2.0F) == 0xC000);
	CHECK(pack_one(0.333333343F) == 0x3555);
	CHECK(pack_one(3.14159274F) == 0x4248);

	// The unpack of the special values
	const uint16_t specials[4] = { 0x7C00, 0xFC00, 0x8000, 0x7E00 };
	const vector4f unpacked = vector_unpack_half(&specials[0]);
	CHECK(vector_get_x(unpacked) == infinity);
	CHECK(vector_get_y(unpacked) == -infinity);
	CHECK(test_float_bits(vector_get_z(unpacked)) == 0x80000000U);
	CHECK(std::isnan(vector_get_w(unpacked)));
}

TEST_CASE("vector4f packing half lanes", "[math][vector4][packing]")
{
	// Use an offset of one value to test memory that is not 8 byte aligned
	uint16_t buffer[6] = { 0xDEAD, 0xDEAD, 0xDEAD, 0xDEAD, 0xDEAD, 0xDEAD };
	vector_pack_half(vector_set(1.0F, -2.0F, 0.5F, 65504.0F), &buffer[1]);

	CHECK(buffer[0] == 0xDEAD);
	CHECK(buffer[1] == 0x3C00);
	CHECK(buffer[2] == 0xC000);
	CHECK(buffer[3] == 0x3800);
	CHECK(buffer[4] == 0x7BFF);
	CHECK(buffer[5] == 0xDEAD);

	const vector4f unpacked = vector_unpack_half(&buffer[1]);
	CHECK(vector_get_x(unpacked) == 1.0F);
	CHECK(vector_get_y(unpacked) == -2.0F);
	CHECK(vector_get_z(unpacked) == 0.5F);
	CHECK(vector_get_w(unpacked) == 65504.0F);
}
