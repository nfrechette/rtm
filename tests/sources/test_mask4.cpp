////////////////////////////////////////////////////////////////////////////////
// The MIT License (MIT)
//
// Copyright (c) 2019 Nicholas Frechette & Realtime Math contributors
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

#include <rtm/type_traits.h>
#include <rtm/mask4d.h>
#include <rtm/mask4f.h>
#include <rtm/mask4i.h>
#include <rtm/mask4q.h>

#include <cstring>

using namespace rtm;

template<typename IntType, typename Mask4Type>
inline Mask4Type reference_mask_and(const Mask4Type& input0, const Mask4Type& input1)
{
	IntType input0_[4];
	IntType input1_[4];

	static_assert(sizeof(Mask4Type) == sizeof(input0_), "Unexpected size");
	std::memcpy(&input0_[0], &input0, sizeof(Mask4Type));
	std::memcpy(&input1_[0], &input1, sizeof(Mask4Type));

	IntType result_[4];
	result_[0] = input0_[0] & input1_[0];
	result_[1] = input0_[1] & input1_[1];
	result_[2] = input0_[2] & input1_[2];
	result_[3] = input0_[3] & input1_[3];

	Mask4Type result;
	std::memcpy(&result, &result_[0], sizeof(Mask4Type));

	return result;
}

template<typename IntType, typename Mask4Type>
inline Mask4Type reference_mask_or(const Mask4Type& input0, const Mask4Type& input1)
{
	IntType input0_[4];
	IntType input1_[4];

	static_assert(sizeof(Mask4Type) == sizeof(input0_), "Unexpected size");
	std::memcpy(&input0_[0], &input0, sizeof(Mask4Type));
	std::memcpy(&input1_[0], &input1, sizeof(Mask4Type));

	IntType result_[4];
	result_[0] = input0_[0] | input1_[0];
	result_[1] = input0_[1] | input1_[1];
	result_[2] = input0_[2] | input1_[2];
	result_[3] = input0_[3] | input1_[3];

	Mask4Type result;
	std::memcpy(&result, &result_[0], sizeof(Mask4Type));

	return result;
}

template<typename IntType, typename Mask4Type>
inline Mask4Type reference_mask_xor(const Mask4Type& input0, const Mask4Type& input1)
{
	IntType input0_[4];
	IntType input1_[4];

	static_assert(sizeof(Mask4Type) == sizeof(input0_), "Unexpected size");
	std::memcpy(&input0_[0], &input0, sizeof(Mask4Type));
	std::memcpy(&input1_[0], &input1, sizeof(Mask4Type));

	IntType result_[4];
	result_[0] = input0_[0] ^ input1_[0];
	result_[1] = input0_[1] ^ input1_[1];
	result_[2] = input0_[2] ^ input1_[2];
	result_[3] = input0_[3] ^ input1_[3];

	Mask4Type result;
	std::memcpy(&result, &result_[0], sizeof(Mask4Type));

	return result;
}

template<typename MaskType, typename IntType>
static void test_mask_impl()
{
	{
		const MaskType mask = mask_set(IntType(0), ~IntType(0), IntType(0), ~IntType(0));
		CHECK(mask_get_x(mask) == IntType(0));
		CHECK(mask_get_y(mask) == ~IntType(0));
		CHECK(mask_get_z(mask) == IntType(0));
		CHECK(mask_get_w(mask) == ~IntType(0));
	}

	{
		const MaskType mask = mask_set(false, true, false, true);
		CHECK(mask_get_x(mask) == IntType(0));
		CHECK(mask_get_y(mask) == ~IntType(0));
		CHECK(mask_get_z(mask) == IntType(0));
		CHECK(mask_get_w(mask) == ~IntType(0));
	}

	{
		const MaskType mask0 = mask_set(IntType(0), ~IntType(0), IntType(0), ~IntType(0));
		const MaskType mask1 = mask_set(IntType(0), IntType(0), IntType(0), IntType(0));
		const MaskType mask2 = mask_set(IntType(0), IntType(0), IntType(0), ~IntType(0));
		const MaskType mask3 = mask_set(IntType(0), IntType(0), ~IntType(0), ~IntType(0));
		const MaskType mask4 = mask_set(IntType(0), ~IntType(0), ~IntType(0), ~IntType(0));
		const MaskType mask5 = mask_set(~IntType(0), ~IntType(0), ~IntType(0), ~IntType(0));
		const MaskType mask6 = mask_set(~IntType(0), IntType(0), IntType(0), IntType(0));
		const MaskType mask7 = mask_set(~IntType(0), ~IntType(0), IntType(0), IntType(0));
		const MaskType mask8 = mask_set(~IntType(0), ~IntType(0), ~IntType(0), IntType(0));

		CHECK(mask_all_true(mask0) == false);
		CHECK(mask_all_true(mask1) == false);
		CHECK(mask_all_true(mask2) == false);
		CHECK(mask_all_true(mask3) == false);
		CHECK(mask_all_true(mask4) == false);
		CHECK(mask_all_true(mask5) == true);
		CHECK(mask_all_true(mask6) == false);
		CHECK(mask_all_true(mask7) == false);
		CHECK(mask_all_true(mask8) == false);

		CHECK(mask_all_true2(mask0) == false);
		CHECK(mask_all_true2(mask1) == false);
		CHECK(mask_all_true2(mask2) == false);
		CHECK(mask_all_true2(mask3) == false);
		CHECK(mask_all_true2(mask4) == false);
		CHECK(mask_all_true2(mask5) == true);
		CHECK(mask_all_true2(mask6) == false);
		CHECK(mask_all_true2(mask7) == true);
		CHECK(mask_all_true2(mask8) == true);

		CHECK(mask_all_true3(mask0) == false);
		CHECK(mask_all_true3(mask1) == false);
		CHECK(mask_all_true3(mask2) == false);
		CHECK(mask_all_true3(mask3) == false);
		CHECK(mask_all_true3(mask4) == false);
		CHECK(mask_all_true3(mask5) == true);
		CHECK(mask_all_true3(mask6) == false);
		CHECK(mask_all_true3(mask7) == false);
		CHECK(mask_all_true3(mask8) == true);

		CHECK(mask_any_true(mask0) == true);
		CHECK(mask_any_true(mask1) == false);
		CHECK(mask_any_true(mask2) == true);
		CHECK(mask_any_true(mask3) == true);
		CHECK(mask_any_true(mask4) == true);
		CHECK(mask_any_true(mask5) == true);
		CHECK(mask_any_true(mask6) == true);
		CHECK(mask_any_true(mask7) == true);
		CHECK(mask_any_true(mask8) == true);

		CHECK(mask_any_true2(mask0) == true);
		CHECK(mask_any_true2(mask1) == false);
		CHECK(mask_any_true2(mask2) == false);
		CHECK(mask_any_true2(mask3) == false);
		CHECK(mask_any_true2(mask4) == true);
		CHECK(mask_any_true2(mask5) == true);
		CHECK(mask_any_true2(mask6) == true);
		CHECK(mask_any_true2(mask7) == true);
		CHECK(mask_any_true2(mask8) == true);

		CHECK(mask_any_true3(mask0) == true);
		CHECK(mask_any_true3(mask1) == false);
		CHECK(mask_any_true3(mask2) == false);
		CHECK(mask_any_true3(mask3) == true);
		CHECK(mask_any_true3(mask4) == true);
		CHECK(mask_any_true3(mask5) == true);
		CHECK(mask_any_true3(mask6) == true);
		CHECK(mask_any_true3(mask7) == true);
		CHECK(mask_any_true3(mask8) == true);
	}

	{
		const MaskType all_true = mask_set(true, true, true, true);
		const MaskType all_false = mask_set(false, false, false, false);

		CHECK(!mask_all_equal2(all_true, mask_set(false, false, false, false)));
		CHECK(!mask_all_equal2(all_true, mask_set(true, false, false, false)));
		CHECK(mask_all_equal2(all_true, mask_set(true, true, false, false)));
		CHECK(mask_all_equal2(all_true, mask_set(true, true, true, false)));
		CHECK(mask_all_equal2(all_true, mask_set(true, true, true, true)));

		CHECK(mask_all_equal2(all_false, mask_set(false, false, false, false)));
		CHECK(mask_all_equal2(all_false, mask_set(false, false, false, true)));
		CHECK(mask_all_equal2(all_false, mask_set(false, false, true, true)));
		CHECK(!mask_all_equal2(all_false, mask_set(false, true, true, true)));
		CHECK(!mask_all_equal2(all_false, mask_set(true, true, true, true)));

		CHECK(!mask_all_equal3(all_true, mask_set(false, false, false, false)));
		CHECK(!mask_all_equal3(all_true, mask_set(true, false, false, false)));
		CHECK(!mask_all_equal3(all_true, mask_set(true, true, false, false)));
		CHECK(mask_all_equal3(all_true, mask_set(true, true, true, false)));
		CHECK(mask_all_equal3(all_true, mask_set(true, true, true, true)));

		CHECK(mask_all_equal3(all_false, mask_set(false, false, false, false)));
		CHECK(mask_all_equal3(all_false, mask_set(false, false, false, true)));
		CHECK(!mask_all_equal3(all_false, mask_set(false, false, true, true)));
		CHECK(!mask_all_equal3(all_false, mask_set(false, true, true, true)));
		CHECK(!mask_all_equal3(all_false, mask_set(true, true, true, true)));

		CHECK(!mask_all_equal(all_true, mask_set(false, false, false, false)));
		CHECK(!mask_all_equal(all_true, mask_set(true, false, false, false)));
		CHECK(!mask_all_equal(all_true, mask_set(true, true, false, false)));
		CHECK(!mask_all_equal(all_true, mask_set(true, true, true, false)));
		CHECK(mask_all_equal(all_true, mask_set(true, true, true, true)));

		CHECK(mask_all_equal(all_false, mask_set(false, false, false, false)));
		CHECK(!mask_all_equal(all_false, mask_set(false, false, false, true)));
		CHECK(!mask_all_equal(all_false, mask_set(false, false, true, true)));
		CHECK(!mask_all_equal(all_false, mask_set(false, true, true, true)));
		CHECK(!mask_all_equal(all_false, mask_set(true, true, true, true)));

		CHECK(!mask_any_equal2(all_true, mask_set(false, false, false, false)));
		CHECK(mask_any_equal2(all_true, mask_set(true, false, false, false)));
		CHECK(mask_any_equal2(all_true, mask_set(false, true, false, false)));
		CHECK(mask_any_equal2(all_true, mask_set(true, true, true, false)));
		CHECK(mask_any_equal2(all_true, mask_set(true, true, true, true)));

		CHECK(mask_any_equal2(all_false, mask_set(false, false, false, false)));
		CHECK(mask_any_equal2(all_false, mask_set(false, false, false, true)));
		CHECK(mask_any_equal2(all_false, mask_set(true, false, true, true)));
		CHECK(mask_any_equal2(all_false, mask_set(false, true, true, true)));
		CHECK(!mask_any_equal2(all_false, mask_set(true, true, true, true)));

		CHECK(!mask_any_equal3(all_true, mask_set(false, false, false, false)));
		CHECK(mask_any_equal3(all_true, mask_set(true, false, false, false)));
		CHECK(mask_any_equal3(all_true, mask_set(true, true, false, false)));
		CHECK(mask_any_equal3(all_true, mask_set(true, true, true, false)));
		CHECK(mask_any_equal3(all_true, mask_set(true, true, true, true)));

		CHECK(mask_any_equal3(all_false, mask_set(false, false, false, false)));
		CHECK(mask_any_equal3(all_false, mask_set(false, false, false, true)));
		CHECK(mask_any_equal3(all_false, mask_set(false, false, true, true)));
		CHECK(mask_any_equal3(all_false, mask_set(false, true, true, true)));
		CHECK(!mask_any_equal3(all_false, mask_set(true, true, true, true)));

		CHECK(!mask_any_equal(all_true, mask_set(false, false, false, false)));
		CHECK(mask_any_equal(all_true, mask_set(true, false, false, false)));
		CHECK(mask_any_equal(all_true, mask_set(true, true, false, false)));
		CHECK(mask_any_equal(all_true, mask_set(true, true, true, false)));
		CHECK(mask_any_equal(all_true, mask_set(true, true, true, true)));

		CHECK(mask_any_equal(all_false, mask_set(false, false, false, false)));
		CHECK(mask_any_equal(all_false, mask_set(false, false, false, true)));
		CHECK(mask_any_equal(all_false, mask_set(false, false, true, true)));
		CHECK(mask_any_equal(all_false, mask_set(false, true, true, true)));
		CHECK(!mask_any_equal(all_false, mask_set(true, true, true, true)));
	}

	{
		const MaskType mask0 = mask_set(true, false, true, false);
		const MaskType mask1 = mask_set(true, true, false, false);
		const MaskType mask2 = mask_set(false, true, false, true);

		CHECK(mask_all_equal(mask_and(mask0, mask1), reference_mask_and<IntType>(mask0, mask1)));
		CHECK(mask_all_equal(mask_and(mask0, mask2), reference_mask_and<IntType>(mask0, mask2)));
		CHECK(mask_all_equal(mask_and(mask1, mask2), reference_mask_and<IntType>(mask1, mask2)));

		CHECK(mask_all_equal(mask_or(mask0, mask1), reference_mask_or<IntType>(mask0, mask1)));
		CHECK(mask_all_equal(mask_or(mask0, mask2), reference_mask_or<IntType>(mask0, mask2)));
		CHECK(mask_all_equal(mask_or(mask1, mask2), reference_mask_or<IntType>(mask1, mask2)));

		CHECK(mask_all_equal(mask_xor(mask0, mask1), reference_mask_xor<IntType>(mask0, mask1)));
		CHECK(mask_all_equal(mask_xor(mask0, mask2), reference_mask_xor<IntType>(mask0, mask2)));
		CHECK(mask_all_equal(mask_xor(mask1, mask2), reference_mask_xor<IntType>(mask1, mask2)));
	}
}

TEST_CASE("mask4f math", "[math][mask]")
{
	test_mask_impl<mask4f, uint32_t>();
}

TEST_CASE("mask4d math", "[math][mask]")
{
	test_mask_impl<mask4d, uint64_t>();
}

TEST_CASE("mask4i math", "[math][mask]")
{
	test_mask_impl<mask4i, uint32_t>();
}

TEST_CASE("mask4q math", "[math][mask]")
{
	test_mask_impl<mask4q, uint64_t>();
}
