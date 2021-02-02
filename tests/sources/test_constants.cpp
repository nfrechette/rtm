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

#include <rtm/constants.h>
#include <rtm/scalarf.h>
#include <rtm/scalard.h>

using namespace rtm;

TEST_CASE("constants", "[math][constants]")
{
	// Float
	CHECK(scalar_near_equal(float(constants::pi()), 3.141592653589793238462643383279502884F, 1.0e-6F));
	CHECK(scalar_near_equal(float(-constants::pi()), -3.141592653589793238462643383279502884F, 1.0e-6F));
	CHECK(scalar_near_equal(float(+constants::pi()), +3.141592653589793238462643383279502884F, 1.0e-6F));
	CHECK(scalar_near_equal(float(constants::pi()) * 2.0F, 3.141592653589793238462643383279502884F * 2.0F, 1.0e-6F));
	CHECK(scalar_near_equal(float(2.0F * constants::pi()), 2.0F * 3.141592653589793238462643383279502884F, 1.0e-6F));
	CHECK(scalar_near_equal(float(constants::pi()) / 2.0F, 3.141592653589793238462643383279502884F / 2.0F, 1.0e-6F));
	CHECK(scalar_near_equal(float(2.0F / constants::pi()), 2.0F / 3.141592653589793238462643383279502884F, 1.0e-6F));
	CHECK(scalar_near_equal(float(constants::pi()) + 1.0F, 3.141592653589793238462643383279502884F + 1.0F, 1.0e-6F));
	CHECK(scalar_near_equal(float(1.0F + constants::pi()), 1.0F + 3.141592653589793238462643383279502884F, 1.0e-6F));
	CHECK(scalar_near_equal(float(constants::pi()) - 1.0F, 3.141592653589793238462643383279502884F - 1.0F, 1.0e-6F));
	CHECK(scalar_near_equal(float(1.0F - constants::pi()), 1.0F - 3.141592653589793238462643383279502884F, 1.0e-6F));

	// Double
	CHECK(scalar_near_equal(double(constants::pi()), 3.141592653589793238462643383279502884, 1.0e-6));
	CHECK(scalar_near_equal(double(-constants::pi()), -3.141592653589793238462643383279502884, 1.0e-6));
	CHECK(scalar_near_equal(double(+constants::pi()), +3.141592653589793238462643383279502884, 1.0e-6));
	CHECK(scalar_near_equal(double(constants::pi() * 2.0), 3.141592653589793238462643383279502884 * 2.0, 1.0e-6));
	CHECK(scalar_near_equal(double(2.0 * constants::pi()), 2.0 * 3.141592653589793238462643383279502884, 1.0e-6));
	CHECK(scalar_near_equal(double(constants::pi() / 2.0), 3.141592653589793238462643383279502884 / 2.0, 1.0e-6));
	CHECK(scalar_near_equal(double(2.0 / constants::pi()), 2.0 / 3.141592653589793238462643383279502884, 1.0e-6));
	CHECK(scalar_near_equal(double(constants::pi() + 1.0), 3.141592653589793238462643383279502884 + 1.0, 1.0e-6));
	CHECK(scalar_near_equal(double(1.0 + constants::pi()), 1.0 + 3.141592653589793238462643383279502884, 1.0e-6));
	CHECK(scalar_near_equal(double(constants::pi() - 1.0), 3.141592653589793238462643383279502884 - 1.0, 1.0e-6));
	CHECK(scalar_near_equal(double(1.0 - constants::pi()), 1.0 - 3.141592653589793238462643383279502884, 1.0e-6));
}
