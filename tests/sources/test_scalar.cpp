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

#include <catch.hpp>

#include <rtm/anglef.h>
#include <rtm/angled.h>
#include <rtm/scalarf.h>
#include <rtm/scalard.h>
#include <rtm/type_traits.h>

#include <limits>

using namespace rtm;

template<typename FloatType>
static void test_scalar_impl(const FloatType threshold)
{
	CHECK(scalar_cast(scalar_set(FloatType(0.2345))) == FloatType(0.2345));

	const FloatType value = FloatType(0.2345);
	CHECK(scalar_is_equal(scalar_load(&value), scalar_set(value)));
	CHECK(scalar_load(&value) == FloatType(0.2345));

	FloatType tmp = FloatType(0.0);
	scalar_store(FloatType(16.5), &tmp);
	CHECK(tmp == FloatType(16.5));
	scalar_store(scalar_set(FloatType(4.5)), &tmp);
	CHECK(tmp == FloatType(4.5));

	CHECK(scalar_floor(FloatType(0.0)) == FloatType(0.0));
	CHECK(scalar_floor(FloatType(0.5)) == FloatType(0.0));
	CHECK(scalar_floor(FloatType(2.5)) == FloatType(2.0));
	CHECK(scalar_floor(FloatType(3.0)) == FloatType(3.0));
	CHECK(scalar_floor(FloatType(-0.5)) == FloatType(-1.0));
	CHECK(scalar_floor(FloatType(-2.5)) == FloatType(-3.0));
	CHECK(scalar_floor(FloatType(-3.0)) == FloatType(-3.0));

	CHECK(scalar_ceil(FloatType(0.0)) == FloatType(0.0));
	CHECK(scalar_ceil(FloatType(0.5)) == FloatType(1.0));
	CHECK(scalar_ceil(FloatType(2.5)) == FloatType(3.0));
	CHECK(scalar_ceil(FloatType(3.0)) == FloatType(3.0));
	CHECK(scalar_ceil(FloatType(-0.5)) == FloatType(0.0));
	CHECK(scalar_ceil(FloatType(-2.5)) == FloatType(-2.0));
	CHECK(scalar_ceil(FloatType(-3.0)) == FloatType(-3.0));

	CHECK(scalar_clamp(FloatType(0.5), FloatType(0.0), FloatType(1.0)) == FloatType(0.5));
	CHECK(scalar_clamp(FloatType(-0.5), FloatType(0.0), FloatType(1.0)) == FloatType(0.0));
	CHECK(scalar_clamp(FloatType(1.5), FloatType(0.0), FloatType(1.0)) == FloatType(1.0));

	CHECK(scalar_abs(FloatType(0.0)) == FloatType(0.0));
	CHECK(scalar_abs(FloatType(2.0)) == FloatType(2.0));
	CHECK(scalar_abs(FloatType(-2.0)) == FloatType(2.0));

	CHECK(scalar_is_equal(scalar_abs(scalar_set(FloatType(0.0))), scalar_set(FloatType(0.0))));
	CHECK(scalar_is_equal(scalar_abs(scalar_set(FloatType(2.0))), scalar_set(FloatType(2.0))));
	CHECK(scalar_is_equal(scalar_abs(scalar_set(FloatType(-2.0))), scalar_set(FloatType(2.0))));

	CHECK(scalar_is_equal(FloatType(1.123), FloatType(1.123)) == true);
	CHECK(scalar_is_equal(FloatType(1.123), FloatType(1.124)) == false);
	CHECK(scalar_is_equal(scalar_set(FloatType(1.123)), scalar_set(FloatType(1.123))) == true);
	CHECK(scalar_is_equal(scalar_set(FloatType(1.123)), scalar_set(FloatType(1.124))) == false);

	CHECK(scalar_near_equal(FloatType(1.0), FloatType(1.0), FloatType(0.00001)) == true);
	CHECK(scalar_near_equal(FloatType(1.0), FloatType(1.000001), FloatType(0.00001)) == true);
	CHECK(scalar_near_equal(FloatType(1.0), FloatType(0.999999), FloatType(0.00001)) == true);
	CHECK(scalar_near_equal(FloatType(1.0), FloatType(1.001), FloatType(0.00001)) == false);
	CHECK(scalar_near_equal(FloatType(1.0), FloatType(0.999), FloatType(0.00001)) == false);

	CHECK(scalar_sqrt(FloatType(0.0)) == FloatType(0.0));
	CHECK(scalar_near_equal(scalar_sqrt(FloatType(0.5)), std::sqrt(FloatType(0.5)), threshold));
	CHECK(scalar_near_equal(scalar_sqrt(FloatType(32.5)), std::sqrt(FloatType(32.5)), threshold));
	CHECK(scalar_is_equal(scalar_sqrt(FloatType(32.5)), scalar_cast(scalar_sqrt(scalar_set(FloatType(32.5))))));

	CHECK(scalar_is_lower(FloatType(1.123), FloatType(1.123)) == false);
	CHECK(scalar_is_lower(FloatType(1.123), FloatType(1.124)) == true);
	CHECK(scalar_is_lower(FloatType(1.124), FloatType(1.123)) == false);
	CHECK(scalar_is_lower(scalar_set(FloatType(1.123)), scalar_set(FloatType(1.123))) == false);
	CHECK(scalar_is_lower(scalar_set(FloatType(1.123)), scalar_set(FloatType(1.124))) == true);
	CHECK(scalar_is_lower(scalar_set(FloatType(1.124)), scalar_set(FloatType(1.123))) == false);

	CHECK(scalar_is_lower_equal(FloatType(1.123), FloatType(1.123)) == true);
	CHECK(scalar_is_lower_equal(FloatType(1.123), FloatType(1.124)) == true);
	CHECK(scalar_is_lower_equal(FloatType(1.124), FloatType(1.123)) == false);
	CHECK(scalar_is_lower_equal(scalar_set(FloatType(1.123)), scalar_set(FloatType(1.123))) == true);
	CHECK(scalar_is_lower_equal(scalar_set(FloatType(1.123)), scalar_set(FloatType(1.124))) == true);
	CHECK(scalar_is_lower_equal(scalar_set(FloatType(1.124)), scalar_set(FloatType(1.123))) == false);

	CHECK(scalar_is_greater(FloatType(1.123), FloatType(1.123)) == false);
	CHECK(scalar_is_greater(FloatType(1.123), FloatType(1.124)) == false);
	CHECK(scalar_is_greater(FloatType(1.124), FloatType(1.123)) == true);
	CHECK(scalar_is_greater(scalar_set(FloatType(1.123)), scalar_set(FloatType(1.123))) == false);
	CHECK(scalar_is_greater(scalar_set(FloatType(1.123)), scalar_set(FloatType(1.124))) == false);
	CHECK(scalar_is_greater(scalar_set(FloatType(1.124)), scalar_set(FloatType(1.123))) == true);

	CHECK(scalar_is_greater_equal(FloatType(1.123), FloatType(1.123)) == true);
	CHECK(scalar_is_greater_equal(FloatType(1.123), FloatType(1.124)) == false);
	CHECK(scalar_is_greater_equal(FloatType(1.124), FloatType(1.123)) == true);
	CHECK(scalar_is_greater_equal(scalar_set(FloatType(1.123)), scalar_set(FloatType(1.123))) == true);
	CHECK(scalar_is_greater_equal(scalar_set(FloatType(1.123)), scalar_set(FloatType(1.124))) == false);
	CHECK(scalar_is_greater_equal(scalar_set(FloatType(1.124)), scalar_set(FloatType(1.123))) == true);

	CHECK(scalar_near_equal(scalar_sqrt_reciprocal(FloatType(0.5)), FloatType(1.0) / std::sqrt(FloatType(0.5)), threshold));
	CHECK(scalar_near_equal(scalar_sqrt_reciprocal(FloatType(32.5)), FloatType(1.0) / std::sqrt(FloatType(32.5)), threshold));

	CHECK(scalar_near_equal(scalar_reciprocal(FloatType(0.5)), FloatType(1.0 / 0.5), threshold));
	CHECK(scalar_near_equal(scalar_reciprocal(FloatType(32.5)), FloatType(1.0 / 32.5), threshold));
	CHECK(scalar_near_equal(scalar_reciprocal(FloatType(-0.5)), FloatType(1.0 / -0.5), threshold));
	CHECK(scalar_near_equal(scalar_reciprocal(FloatType(-32.5)), FloatType(1.0 / -32.5), threshold));

	CHECK(scalar_near_equal(scalar_cast(scalar_reciprocal(scalar_set(FloatType(0.5)))), FloatType(1.0 / 0.5), threshold));
	CHECK(scalar_near_equal(scalar_cast(scalar_reciprocal(scalar_set(FloatType(32.5)))), FloatType(1.0 / 32.5), threshold));
	CHECK(scalar_near_equal(scalar_cast(scalar_reciprocal(scalar_set(FloatType(-0.5)))), FloatType(1.0 / -0.5), threshold));
	CHECK(scalar_near_equal(scalar_cast(scalar_reciprocal(scalar_set(FloatType(-32.5)))), FloatType(1.0 / -32.5), threshold));

	CHECK(scalar_near_equal(scalar_add(FloatType(-0.5), FloatType(1.0)), FloatType(-0.5) + FloatType(1.0), threshold));
	CHECK(scalar_near_equal(scalar_add(FloatType(1.0), FloatType(-0.5)), FloatType(1.0) + FloatType(-0.5), threshold));
	CHECK(scalar_near_equal(scalar_add(FloatType(1.0), FloatType(1.0)), FloatType(1.0) + FloatType(1.0), threshold));

	CHECK(scalar_near_equal(scalar_cast(scalar_add(scalar_set(FloatType(-0.5)), scalar_set(FloatType(1.0)))), FloatType(-0.5) + FloatType(1.0), threshold));
	CHECK(scalar_near_equal(scalar_cast(scalar_add(scalar_set(FloatType(1.0)), scalar_set(FloatType(-0.5)))), FloatType(1.0) + FloatType(-0.5), threshold));
	CHECK(scalar_near_equal(scalar_cast(scalar_add(scalar_set(FloatType(1.0)), scalar_set(FloatType(1.0)))), FloatType(1.0) + FloatType(1.0), threshold));

	CHECK(scalar_near_equal(scalar_sub(FloatType(-0.5), FloatType(1.0)), FloatType(-0.5) - FloatType(1.0), threshold));
	CHECK(scalar_near_equal(scalar_sub(FloatType(1.0), FloatType(-0.5)), FloatType(1.0) - FloatType(-0.5), threshold));
	CHECK(scalar_near_equal(scalar_sub(FloatType(1.0), FloatType(1.0)), FloatType(1.0) - FloatType(1.0), threshold));

	CHECK(scalar_near_equal(scalar_cast(scalar_sub(scalar_set(FloatType(-0.5)), scalar_set(FloatType(1.0)))), FloatType(-0.5) - FloatType(1.0), threshold));
	CHECK(scalar_near_equal(scalar_cast(scalar_sub(scalar_set(FloatType(1.0)), scalar_set(FloatType(-0.5)))), FloatType(1.0) - FloatType(-0.5), threshold));
	CHECK(scalar_near_equal(scalar_cast(scalar_sub(scalar_set(FloatType(1.0)), scalar_set(FloatType(1.0)))), FloatType(1.0) - FloatType(1.0), threshold));

	CHECK(scalar_near_equal(scalar_mul(FloatType(-0.5), FloatType(1.0)), FloatType(-0.5) * FloatType(1.0), threshold));
	CHECK(scalar_near_equal(scalar_mul(FloatType(1.0), FloatType(-0.5)), FloatType(1.0) * FloatType(-0.5), threshold));
	CHECK(scalar_near_equal(scalar_mul(FloatType(1.0), FloatType(1.0)), FloatType(1.0) * FloatType(1.0), threshold));

	CHECK(scalar_near_equal(scalar_cast(scalar_mul(scalar_set(FloatType(-0.5)), scalar_set(FloatType(1.0)))), FloatType(-0.5) * FloatType(1.0), threshold));
	CHECK(scalar_near_equal(scalar_cast(scalar_mul(scalar_set(FloatType(1.0)), scalar_set(FloatType(-0.5)))), FloatType(1.0) * FloatType(-0.5), threshold));
	CHECK(scalar_near_equal(scalar_cast(scalar_mul(scalar_set(FloatType(1.0)), scalar_set(FloatType(1.0)))), FloatType(1.0) * FloatType(1.0), threshold));

	CHECK(scalar_near_equal(scalar_div(FloatType(-0.5), FloatType(1.0)), FloatType(-0.5) / FloatType(1.0), threshold));
	CHECK(scalar_near_equal(scalar_div(FloatType(1.0), FloatType(-0.5)), FloatType(1.0) / FloatType(-0.5), threshold));
	CHECK(scalar_near_equal(scalar_div(FloatType(1.0), FloatType(1.0)), FloatType(1.0) / FloatType(1.0), threshold));

	CHECK(scalar_near_equal(scalar_cast(scalar_div(scalar_set(FloatType(-0.5)), scalar_set(FloatType(1.0)))), FloatType(-0.5) / FloatType(1.0), threshold));
	CHECK(scalar_near_equal(scalar_cast(scalar_div(scalar_set(FloatType(1.0)), scalar_set(FloatType(-0.5)))), FloatType(1.0) / FloatType(-0.5), threshold));
	CHECK(scalar_near_equal(scalar_cast(scalar_div(scalar_set(FloatType(1.0)), scalar_set(FloatType(1.0)))), FloatType(1.0) / FloatType(1.0), threshold));

	const FloatType values[] = { FloatType(-0.5123), FloatType(1.0341), FloatType(-0.54132) };
	CHECK(scalar_near_equal(scalar_mul_add(values[0], values[1], values[2]), (values[0] * values[1]) + values[2], threshold));
	CHECK(scalar_near_equal(scalar_cast(scalar_mul_add(scalar_set(values[0]), scalar_set(values[1]), scalar_set(values[2]))), (values[0] * values[1]) + values[2], threshold));

	CHECK(scalar_near_equal(scalar_lerp(values[0], values[1], values[2]), ((values[1] - values[0]) * values[2]) + values[0], threshold));
	CHECK(scalar_near_equal(scalar_cast(scalar_lerp(scalar_set(values[0]), scalar_set(values[1]), values[2])), ((values[1] - values[0]) * values[2]) + values[0], threshold));

	using AngleType = typename float_traits<FloatType>::angle;

	const AngleType half_pi = constants::half_pi();
	const AngleType pi = constants::pi();

	const FloatType angles[] = { FloatType(0.0), pi.as_radians(), -pi.as_radians(), half_pi.as_radians(), -half_pi.as_radians(), FloatType(0.5), FloatType(32.5), FloatType(-0.5), FloatType(-32.5) };

	for (const FloatType angle : angles)
	{
		CHECK(scalar_near_equal(scalar_sin(angle), std::sin(angle), threshold));
		CHECK(scalar_near_equal(scalar_cos(angle), std::cos(angle), threshold));

		FloatType sin_result;
		FloatType cos_result;
		scalar_sincos(angle, sin_result, cos_result);
		CHECK(scalar_near_equal(sin_result, std::sin(angle), threshold));
		CHECK(scalar_near_equal(cos_result, std::cos(angle), threshold));
	}

	CHECK(scalar_near_equal(scalar_acos(FloatType(-1.0)), std::acos(FloatType(-1.0)), threshold));
	CHECK(scalar_near_equal(scalar_acos(FloatType(-0.75)), std::acos(FloatType(-0.75)), threshold));
	CHECK(scalar_near_equal(scalar_acos(FloatType(-0.5)), std::acos(FloatType(-0.5)), threshold));
	CHECK(scalar_near_equal(scalar_acos(FloatType(-0.25)), std::acos(-FloatType(0.25)), threshold));
	CHECK(scalar_near_equal(scalar_acos(FloatType(0.0)), std::acos(FloatType(0.0)), threshold));
	CHECK(scalar_near_equal(scalar_acos(FloatType(0.25)), std::acos(FloatType(0.25)), threshold));
	CHECK(scalar_near_equal(scalar_acos(FloatType(0.5)), std::acos(FloatType(0.5)), threshold));
	CHECK(scalar_near_equal(scalar_acos(FloatType(0.75)), std::acos(FloatType(0.75)), threshold));
	CHECK(scalar_near_equal(scalar_acos(FloatType(1.0)), std::acos(FloatType(1.0)), threshold));

	CHECK(scalar_near_equal(scalar_atan2(FloatType(-2.0), FloatType(-2.0)), std::atan2(FloatType(-2.0), FloatType(-2.0)), threshold));
	CHECK(scalar_near_equal(scalar_atan2(FloatType(-1.0), FloatType(-2.0)), std::atan2(FloatType(-1.0), FloatType(-2.0)), threshold));
	CHECK(scalar_near_equal(scalar_atan2(FloatType(-2.0), FloatType(-1.0)), std::atan2(FloatType(-2.0), FloatType(-1.0)), threshold));
	CHECK(scalar_near_equal(scalar_atan2(FloatType(2.0), FloatType(2.0)), std::atan2(FloatType(2.0), FloatType(2.0)), threshold));
	CHECK(scalar_near_equal(scalar_atan2(FloatType(1.0), FloatType(2.0)), std::atan2(FloatType(1.0), FloatType(2.0)), threshold));
	CHECK(scalar_near_equal(scalar_atan2(FloatType(2.0), FloatType(1.0)), std::atan2(FloatType(2.0), FloatType(1.0)), threshold));

	CHECK(scalar_min(FloatType(-0.5), FloatType(1.0)) == FloatType(-0.5));
	CHECK(scalar_min(FloatType(1.0), FloatType(-0.5)) == FloatType(-0.5));
	CHECK(scalar_min(FloatType(1.0), FloatType(1.0)) == FloatType(1.0));

	CHECK(scalar_is_equal(scalar_min(scalar_set(FloatType(-0.5)), scalar_set(FloatType(1.0))), scalar_set(FloatType(-0.5))));
	CHECK(scalar_is_equal(scalar_min(scalar_set(FloatType(1.0)), scalar_set(FloatType(-0.5))), scalar_set(FloatType(-0.5))));
	CHECK(scalar_is_equal(scalar_min(scalar_set(FloatType(1.0)), scalar_set(FloatType(1.0))), scalar_set(FloatType(1.0))));

	CHECK(scalar_max(FloatType(-0.5), FloatType(1.0)) == FloatType(1.0));
	CHECK(scalar_max(FloatType(1.0), FloatType(-0.5)) == FloatType(1.0));
	CHECK(scalar_max(FloatType(1.0), FloatType(1.0)) == FloatType(1.0));

	CHECK(scalar_is_equal(scalar_max(scalar_set(FloatType(-0.5)), scalar_set(FloatType(1.0))), scalar_set(FloatType(1.0))));
	CHECK(scalar_is_equal(scalar_max(scalar_set(FloatType(1.0)), scalar_set(FloatType(-0.5))), scalar_set(FloatType(1.0))));
	CHECK(scalar_is_equal(scalar_max(scalar_set(FloatType(1.0)), scalar_set(FloatType(1.0))), scalar_set(FloatType(1.0))));

	CHECK(scalar_is_finite(FloatType(0.0)) == true);
	CHECK(scalar_is_finite(FloatType(32.0)) == true);
	CHECK(scalar_is_finite(FloatType(-32.0)) == true);
	CHECK(scalar_is_finite(std::numeric_limits<FloatType>::infinity()) == false);
	CHECK(scalar_is_finite(std::numeric_limits<FloatType>::quiet_NaN()) == false);
	CHECK(scalar_is_finite(std::numeric_limits<FloatType>::signaling_NaN()) == false);

	CHECK(scalar_symmetric_round(FloatType(-1.75)) == FloatType(-2.0));
	CHECK(scalar_symmetric_round(FloatType(-1.5)) == FloatType(-2.0));
	CHECK(scalar_symmetric_round(FloatType(-1.4999)) == FloatType(-1.0));
	CHECK(scalar_symmetric_round(FloatType(-0.5)) == FloatType(-1.0));
	CHECK(scalar_symmetric_round(FloatType(-0.4999)) == FloatType(0.0));
	CHECK(scalar_symmetric_round(FloatType(0.0)) == FloatType(0.0));
	CHECK(scalar_symmetric_round(FloatType(0.4999)) == FloatType(0.0));
	CHECK(scalar_symmetric_round(FloatType(0.5)) == FloatType(1.0));
	CHECK(scalar_symmetric_round(FloatType(1.4999)) == FloatType(1.0));
	CHECK(scalar_symmetric_round(FloatType(1.5)) == FloatType(2.0));
	CHECK(scalar_symmetric_round(FloatType(1.75)) == FloatType(2.0));

	CHECK(scalar_fraction(FloatType(0.0)) == FloatType(0.0));
	CHECK(scalar_fraction(FloatType(1.0)) == FloatType(0.0));
	CHECK(scalar_fraction(FloatType(-1.0)) == FloatType(0.0));
	CHECK(scalar_near_equal(scalar_fraction(FloatType(0.25)), FloatType(0.25), threshold));
	CHECK(scalar_near_equal(scalar_fraction(FloatType(0.5)), FloatType(0.5), threshold));
	CHECK(scalar_near_equal(scalar_fraction(FloatType(0.75)), FloatType(0.75), threshold));
}

TEST_CASE("scalarf math", "[math][scalar]")
{
	test_scalar_impl<float>(1.0E-6F);
}

TEST_CASE("scalard math", "[math][scalar]")
{
	test_scalar_impl<double>(1.0E-9);
}
