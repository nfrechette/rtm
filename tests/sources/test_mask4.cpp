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

#include <catch.hpp>

#include <rtm/type_traits.h>
#include <rtm/mask4i.h>
#include <rtm/mask4q.h>

using namespace rtm;

template<typename IntType>
static void test_mask_impl()
{
	{
		REQUIRE(mask_get_x(mask_set(IntType(0), ~IntType(0), IntType(0), ~IntType(0))) == IntType(0));
		REQUIRE(mask_get_y(mask_set(IntType(0), ~IntType(0), IntType(0), ~IntType(0))) == ~IntType(0));
		REQUIRE(mask_get_z(mask_set(IntType(0), ~IntType(0), IntType(0), ~IntType(0))) == IntType(0));
		REQUIRE(mask_get_w(mask_set(IntType(0), ~IntType(0), IntType(0), ~IntType(0))) == ~IntType(0));
	}
}

TEST_CASE("mask4i math", "[math][mask]")
{
	test_mask_impl<uint32_t>();
}

TEST_CASE("mask4d math", "[math][mask]")
{
	test_mask_impl<uint64_t>();
}
