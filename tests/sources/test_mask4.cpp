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

#include <catch2/catch.hpp>

#include <rtm/type_traits.h>
#include <rtm/mask4d.h>
#include <rtm/mask4f.h>
#include <rtm/mask4i.h>
#include <rtm/mask4q.h>

using namespace rtm;

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
