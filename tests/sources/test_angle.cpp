////////////////////////////////////////////////////////////////////////////////
// The MIT License (MIT)
//
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

#include <catch.hpp>

#include <rtm/anglef.h>
#include <rtm/angled.h>
#include <rtm/scalarf.h>
#include <rtm/scalard.h>

using namespace rtm;

template<typename FloatType>
static void test_angle_impl(const FloatType threshold)
{
	REQUIRE(scalar_deg_to_rad(FloatType(0.0)) == FloatType(0.0));
	REQUIRE(scalar_near_equal(scalar_deg_to_rad(FloatType(90.0)), k_pi_2, threshold));
	REQUIRE(scalar_near_equal(scalar_deg_to_rad(FloatType(-90.0)), -k_pi_2, threshold));
	REQUIRE(scalar_near_equal(scalar_deg_to_rad(FloatType(180.0)), k_pi, threshold));
	REQUIRE(scalar_near_equal(scalar_deg_to_rad(FloatType(-180.0)), -k_pi, threshold));
	REQUIRE(scalar_near_equal(scalar_deg_to_rad(FloatType(360.0)), k_2_pi, threshold));
	REQUIRE(scalar_near_equal(scalar_deg_to_rad(FloatType(-360.0)), -k_2_pi, threshold));

	REQUIRE(scalar_rad_to_deg(FloatType(0.0)) == FloatType(0.0));
	REQUIRE(scalar_near_equal(scalar_rad_to_deg(FloatType(k_pi_2)), FloatType(90.0), threshold));
	REQUIRE(scalar_near_equal(scalar_rad_to_deg(FloatType(-k_pi_2)), FloatType(-90.0), threshold));
	REQUIRE(scalar_near_equal(scalar_rad_to_deg(FloatType(k_pi)), FloatType(180.0), threshold));
	REQUIRE(scalar_near_equal(scalar_rad_to_deg(FloatType(-k_pi)), FloatType(-180.0), threshold));
	REQUIRE(scalar_near_equal(scalar_rad_to_deg(FloatType(k_2_pi)), FloatType(360.0), threshold));
	REQUIRE(scalar_near_equal(scalar_rad_to_deg(FloatType(-k_2_pi)), FloatType(-360.0), threshold));

	REQUIRE(scalar_near_equal(degrees(FloatType(180.0)).as_radians(), radians(FloatType(k_pi)).as_radians(), threshold));
	REQUIRE(scalar_near_equal(degrees(FloatType(90.0)).as_radians(), radians(FloatType(k_pi_2)).as_radians(), threshold));
	REQUIRE(scalar_near_equal(degrees(FloatType(180.0)).as_degrees(), radians(FloatType(k_pi)).as_degrees(), threshold));
	REQUIRE(scalar_near_equal(degrees(FloatType(90.0)).as_degrees(), radians(FloatType(k_pi_2)).as_degrees(), threshold));
}

TEST_CASE("anglef math", "[math][angle]")
{
	test_angle_impl<float>(1.0E-6F);
}

TEST_CASE("angled math", "[math][angle]")
{
	test_angle_impl<double>(1.0E-9);
}
