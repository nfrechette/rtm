////////////////////////////////////////////////////////////////////////////////
// The MIT License (MIT)
//
// Copyright (c) 2018 Nicholas Frechette & Animation Compression Library contributors
// Copyright (c) 2018 Nicholas Frechette & Realtime Math contributors
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

#include <rtm/scalard.h>
#include <rtm/vector4d.h>
#include <rtm/quatd.h>
#include <rtm/mask4d.h>

#include "test_vector4_impl.h"

TEST_CASE("vector4d math get/set", "[math][vector4]")
{
	test_vector4_getset_impl<double>();
}

TEST_CASE("vector4d math arithmetic", "[math][vector4]")
{
	test_vector4_arithmetic_impl<double>(1.0E-9);
}

TEST_CASE("vector4d math relational", "[math][vector4]")
{
	test_vector4_relational_impl<double>(1.0E-9);
}

TEST_CASE("vector4d math logical", "[math][vector4]")
{
	test_vector4_logical_impl<double>();
}

TEST_CASE("vector4d math misc", "[math][vector4]")
{
	test_vector4_impl<double>(1.0E-9);
}

TEST_CASE("vector4d math mix zero", "[math][vector4]")
{
	test_vector4_mix_zero_impl<double>();
}

TEST_CASE("vector4d math exp/log", "[math][vector4]")
{
	test_vector4_exp_log_impl<double>(1.0E-12);
}
