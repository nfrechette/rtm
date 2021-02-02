#pragma once

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

#include <rtm/vector4f.h>
#include <rtm/vector4d.h>

using namespace rtm;

template<typename Vector4Type, typename FloatType, mix4 XArg, mix4 YArg>
void test_vector_mix_impl(const FloatType threshold)
{
	(void)threshold;

#if defined(RTM_IMPL_WITH_VECTOR_MIX_TESTS)
	const FloatType test_value0_flt[4] = { FloatType(2.0), FloatType(9.34), FloatType(-54.12), FloatType(6000.0) };
	const FloatType test_value1_flt[4] = { FloatType(0.75), FloatType(-4.52), FloatType(44.68), FloatType(-54225.0) };

	const Vector4Type test_value0 = vector_set(test_value0_flt[0], test_value0_flt[1], test_value0_flt[2], test_value0_flt[3]);
	const Vector4Type test_value1 = vector_set(test_value1_flt[0], test_value1_flt[1], test_value1_flt[2], test_value1_flt[3]);

	Vector4Type results[8 * 8];
	uint32_t index = 0;

#define RTM_TEST_MIX_XYZ(comp0, comp1, comp2) \
	results[index++] = vector_mix<comp0, comp1, comp2, mix4::x>(test_value0, test_value1); \
	results[index++] = vector_mix<comp0, comp1, comp2, mix4::y>(test_value0, test_value1); \
	results[index++] = vector_mix<comp0, comp1, comp2, mix4::z>(test_value0, test_value1); \
	results[index++] = vector_mix<comp0, comp1, comp2, mix4::w>(test_value0, test_value1); \
	results[index++] = vector_mix<comp0, comp1, comp2, mix4::a>(test_value0, test_value1); \
	results[index++] = vector_mix<comp0, comp1, comp2, mix4::b>(test_value0, test_value1); \
	results[index++] = vector_mix<comp0, comp1, comp2, mix4::c>(test_value0, test_value1); \
	results[index++] = vector_mix<comp0, comp1, comp2, mix4::d>(test_value0, test_value1)

#define RTM_TEST_MIX_XY(comp0, comp1) \
	RTM_TEST_MIX_XYZ(comp0, comp1, mix4::x); \
	RTM_TEST_MIX_XYZ(comp0, comp1, mix4::y); \
	RTM_TEST_MIX_XYZ(comp0, comp1, mix4::z); \
	RTM_TEST_MIX_XYZ(comp0, comp1, mix4::w); \
	RTM_TEST_MIX_XYZ(comp0, comp1, mix4::a); \
	RTM_TEST_MIX_XYZ(comp0, comp1, mix4::b); \
	RTM_TEST_MIX_XYZ(comp0, comp1, mix4::c); \
	RTM_TEST_MIX_XYZ(comp0, comp1, mix4::d)

	// This generates 8*8 = 64 unit tests... it takes a while to compile and uses a lot of stack space
	RTM_TEST_MIX_XY(XArg, YArg);

	index = 0;

	const int comp0 = (int)XArg;
	const int comp1 = (int)YArg;
	for (int comp2 = 0; comp2 < 8; ++comp2)
	{
		for (int comp3 = 0; comp3 < 8; ++comp3)
		{
			INFO("vector_mix<" << comp0 << ", " << comp1 << ", " << comp2 << ", " << comp3 << ">");

			const Vector4Type expected = vector_set(
				rtm_impl::is_mix_xyzw((mix4)comp0) ? test_value0_flt[comp0 - (int)mix4::x] : test_value1_flt[comp0 - (int)mix4::a],
				rtm_impl::is_mix_xyzw((mix4)comp1) ? test_value0_flt[comp1 - (int)mix4::x] : test_value1_flt[comp1 - (int)mix4::a],
				rtm_impl::is_mix_xyzw((mix4)comp2) ? test_value0_flt[comp2 - (int)mix4::x] : test_value1_flt[comp2 - (int)mix4::a],
				rtm_impl::is_mix_xyzw((mix4)comp3) ? test_value0_flt[comp3 - (int)mix4::x] : test_value1_flt[comp3 - (int)mix4::a]);

			CHECK(vector_all_near_equal(expected, results[index], threshold));

			++index;
		}
	}

#undef RTM_TEST_MIX_XY
#undef RTM_TEST_MIX_XYZ
#endif	// defined(RTM_IMPL_WITH_VECTOR_MIX_TESTS)
}
