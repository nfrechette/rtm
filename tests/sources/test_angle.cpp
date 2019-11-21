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
#include <rtm/type_traits.h>

using namespace rtm;

template<typename FloatType>
static void test_angle_impl(const FloatType threshold)
{
	using AngleType = typename float_traits<FloatType>::angle;

	const AngleType half_pi = constants::half_pi();
	const AngleType pi = constants::pi();
	const AngleType two_pi = constants::two_pi();
	const AngleType one_eighty_div_pi = constants::one_eighty_div_pi();

	CHECK(scalar_deg_to_rad(FloatType(0.0)) == FloatType(0.0));
	CHECK(scalar_near_equal(scalar_deg_to_rad(FloatType(90.0)), half_pi.as_radians(), threshold));
	CHECK(scalar_near_equal(scalar_deg_to_rad(FloatType(-90.0)), -half_pi.as_radians(), threshold));
	CHECK(scalar_near_equal(scalar_deg_to_rad(FloatType(180.0)), pi.as_radians(), threshold));
	CHECK(scalar_near_equal(scalar_deg_to_rad(FloatType(-180.0)), -pi.as_radians(), threshold));
	CHECK(scalar_near_equal(scalar_deg_to_rad(FloatType(360.0)), two_pi.as_radians(), threshold));
	CHECK(scalar_near_equal(scalar_deg_to_rad(FloatType(-360.0)), -two_pi.as_radians(), threshold));

	CHECK(scalar_rad_to_deg(FloatType(0.0)) == FloatType(0.0));
	CHECK(scalar_near_equal(scalar_rad_to_deg(half_pi.as_radians()), FloatType(90.0), threshold));
	CHECK(scalar_near_equal(scalar_rad_to_deg(-half_pi.as_radians()), FloatType(-90.0), threshold));
	CHECK(scalar_near_equal(scalar_rad_to_deg(pi.as_radians()), FloatType(180.0), threshold));
	CHECK(scalar_near_equal(scalar_rad_to_deg(-pi.as_radians()), FloatType(-180.0), threshold));
	CHECK(scalar_near_equal(scalar_rad_to_deg(two_pi.as_radians()), FloatType(360.0), threshold));
	CHECK(scalar_near_equal(scalar_rad_to_deg(-two_pi.as_radians()), FloatType(-360.0), threshold));

	CHECK(scalar_near_equal(degrees(FloatType(180.0)).as_radians(), radians(pi.as_radians()).as_radians(), threshold));
	CHECK(scalar_near_equal(degrees(FloatType(90.0)).as_radians(), radians(half_pi.as_radians()).as_radians(), threshold));
	CHECK(scalar_near_equal(degrees(FloatType(180.0)).as_degrees(), radians(pi.as_radians()).as_degrees(), threshold));
	CHECK(scalar_near_equal(degrees(FloatType(90.0)).as_degrees(), radians(half_pi.as_radians()).as_degrees(), threshold));

	CHECK(scalar_near_equal((constants::pi() / FloatType(2.0)).as_radians(), half_pi.as_radians(), threshold));
	CHECK(scalar_near_equal((FloatType(180.0) / constants::pi()).as_radians(), one_eighty_div_pi.as_radians(), threshold));
	CHECK(scalar_near_equal((constants::pi() * FloatType(2.0)).as_radians(), two_pi.as_radians(), threshold));
	CHECK(scalar_near_equal((FloatType(2.0) * constants::pi()).as_radians(), two_pi.as_radians(), threshold));
	CHECK(scalar_near_equal(AngleType(constants::pi() + constants::pi()).as_radians(), two_pi.as_radians(), threshold));

	CHECK(scalar_near_equal((pi / FloatType(2.0)).as_radians(), half_pi.as_radians(), threshold));
	CHECK(scalar_near_equal((FloatType(180.0) / pi).as_radians(), one_eighty_div_pi.as_radians(), threshold));
	CHECK(scalar_near_equal((pi * FloatType(2.0)).as_radians(), two_pi.as_radians(), threshold));
	CHECK(scalar_near_equal((FloatType(2.0) * pi).as_radians(), two_pi.as_radians(), threshold));
	CHECK(scalar_near_equal((pi + pi).as_radians(), two_pi.as_radians(), threshold));
}

TEST_CASE("anglef math", "[math][angle]")
{
	test_angle_impl<float>(1.0E-5F);
}

TEST_CASE("angled math", "[math][angle]")
{
	test_angle_impl<double>(1.0E-9);
}
