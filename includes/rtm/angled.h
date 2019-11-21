#pragma once

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

#include "rtm/math.h"
#include <rtm/impl/angle_constants.h>
#include "rtm/impl/compiler_utils.h"

RTM_IMPL_FILE_PRAGMA_PUSH

namespace rtm
{
	//////////////////////////////////////////////////////////////////////////
	// Converts degrees into radians.
	//////////////////////////////////////////////////////////////////////////
	constexpr double scalar_deg_to_rad(double deg) RTM_NO_EXCEPT
	{
		return deg * constants::pi_div_one_eighty().dbl;
	}

	//////////////////////////////////////////////////////////////////////////
	// Converts radians into degrees.
	//////////////////////////////////////////////////////////////////////////
	constexpr double scalar_rad_to_deg(double rad) RTM_NO_EXCEPT
	{
		return rad * constants::one_eighty_div_pi().dbl;
	}

	//////////////////////////////////////////////////////////////////////////
	// An angle class for added type safety.
	//////////////////////////////////////////////////////////////////////////
	class angled
	{
	public:
		constexpr angled() RTM_NO_EXCEPT : m_radians(0.0) {}
		constexpr angled(rtm_impl::angle_constant value) RTM_NO_EXCEPT : m_radians(value.dbl) {}

		constexpr angled operator-() const RTM_NO_EXCEPT { return angled(-m_radians); }

		constexpr double as_radians() const RTM_NO_EXCEPT { return m_radians; }
		constexpr double as_degrees() const RTM_NO_EXCEPT { return scalar_rad_to_deg(m_radians); }

	private:
		explicit constexpr angled(double rad) RTM_NO_EXCEPT : m_radians(rad) {}

		double m_radians;

		friend constexpr angled radians(double rad) RTM_NO_EXCEPT;
		friend constexpr angled degrees(double rad) RTM_NO_EXCEPT;
	};

	//////////////////////////////////////////////////////////////////////////
	// Constructs an angle from a radians value.
	//////////////////////////////////////////////////////////////////////////
	constexpr angled radians(double rad) RTM_NO_EXCEPT
	{
		return angled(rad);
	}

	//////////////////////////////////////////////////////////////////////////
	// Constructs an angle from a degrees value.
	//////////////////////////////////////////////////////////////////////////
	constexpr angled degrees(double deg) RTM_NO_EXCEPT
	{
		return angled(scalar_deg_to_rad(deg));
	}
}

//////////////////////////////////////////////////////////////////////////
// Divide a constant by an angle.
//////////////////////////////////////////////////////////////////////////
constexpr rtm::angled operator/(double lhs, rtm::angled rhs) RTM_NO_EXCEPT
{
	return rtm::radians(lhs / rhs.as_radians());
}

//////////////////////////////////////////////////////////////////////////
// Divide an angle by a constant.
//////////////////////////////////////////////////////////////////////////
constexpr rtm::angled operator/(rtm::angled lhs, double rhs) RTM_NO_EXCEPT
{
	return rtm::radians(lhs.as_radians() / rhs);
}

//////////////////////////////////////////////////////////////////////////
// Divide a constant by an angle constant.
//////////////////////////////////////////////////////////////////////////
constexpr rtm::angled operator/(double lhs, rtm::rtm_impl::angle_constant rhs) RTM_NO_EXCEPT
{
	return rtm::radians(lhs / rhs.dbl);
}

//////////////////////////////////////////////////////////////////////////
// Divide an angle constant by a constant.
//////////////////////////////////////////////////////////////////////////
constexpr rtm::angled operator/(rtm::rtm_impl::angle_constant lhs, double rhs) RTM_NO_EXCEPT
{
	return rtm::radians(lhs.dbl / rhs);
}

//////////////////////////////////////////////////////////////////////////
// Multiply a constant by an angle.
//////////////////////////////////////////////////////////////////////////
constexpr rtm::angled operator*(double lhs, rtm::angled rhs) RTM_NO_EXCEPT
{
	return rtm::radians(lhs * rhs.as_radians());
}

//////////////////////////////////////////////////////////////////////////
// Multiply an angle by a constant.
//////////////////////////////////////////////////////////////////////////
constexpr rtm::angled operator*(rtm::angled lhs, double rhs) RTM_NO_EXCEPT
{
	return rtm::radians(lhs.as_radians() * rhs);
}

//////////////////////////////////////////////////////////////////////////
// Multiply a constant by an angle constant.
//////////////////////////////////////////////////////////////////////////
constexpr rtm::angled operator*(double lhs, rtm::rtm_impl::angle_constant rhs) RTM_NO_EXCEPT
{
	return rtm::radians(lhs * rhs.dbl);
}

//////////////////////////////////////////////////////////////////////////
// Multiply an angle constant by a constant.
//////////////////////////////////////////////////////////////////////////
constexpr rtm::angled operator*(rtm::rtm_impl::angle_constant lhs, double rhs) RTM_NO_EXCEPT
{
	return rtm::radians(lhs.dbl * rhs);
}

//////////////////////////////////////////////////////////////////////////
// Add two angles.
//////////////////////////////////////////////////////////////////////////
constexpr rtm::angled operator+(rtm::angled lhs, rtm::angled rhs) RTM_NO_EXCEPT
{
	return rtm::radians(lhs.as_radians() + rhs.as_radians());
}

RTM_IMPL_FILE_PRAGMA_POP
