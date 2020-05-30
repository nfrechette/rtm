#pragma once

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

#include <rtm/type_traits.h>
#include <rtm/scalarf.h>
#include <rtm/scalard.h>
#include <rtm/vector4f.h>
#include <rtm/vector4d.h>
#include <rtm/quatf.h>
#include <rtm/quatd.h>
#include <rtm/mask4i.h>
#include <rtm/mask4q.h>

#include <cstring>
#include <limits>

using namespace rtm;

template<typename Vector4Type, typename FloatType>
inline const FloatType* vector_as_float_ptr_raw(const Vector4Type& input);

template<>
inline const float* vector_as_float_ptr_raw<vector4f, float>(const vector4f& input)
{
	return vector_to_pointer(input);
}

template<>
inline const double* vector_as_float_ptr_raw<vector4d, double>(const vector4d& input)
{
	return vector_to_pointer(input);
}

template<typename Vector4Type, typename FloatType>
inline Vector4Type scalar_cross3(const Vector4Type& lhs, const Vector4Type& rhs)
{
	const FloatType lhs_x = vector_get_x(lhs);
	const FloatType lhs_y = vector_get_y(lhs);
	const FloatType lhs_z = vector_get_z(lhs);
	const FloatType rhs_x = vector_get_x(rhs);
	const FloatType rhs_y = vector_get_y(rhs);
	const FloatType rhs_z = vector_get_z(rhs);
	return vector_set((lhs_y * rhs_z) - (lhs_z * rhs_y), (lhs_z * rhs_x) - (lhs_x * rhs_z), (lhs_x * rhs_y) - (lhs_y * rhs_x));
}

template<typename Vector4Type, typename FloatType>
inline FloatType scalar_dot(const Vector4Type& lhs, const Vector4Type& rhs)
{
	const FloatType lhs_x = vector_get_x(lhs);
	const FloatType lhs_y = vector_get_y(lhs);
	const FloatType lhs_z = vector_get_z(lhs);
	const FloatType lhs_w = vector_get_w(lhs);
	const FloatType rhs_x = vector_get_x(rhs);
	const FloatType rhs_y = vector_get_y(rhs);
	const FloatType rhs_z = vector_get_z(rhs);
	const FloatType rhs_w = vector_get_w(rhs);
	return (lhs_x * rhs_x) + (lhs_y * rhs_y) + (lhs_z * rhs_z) + (lhs_w * rhs_w);
}

template<typename Vector4Type, typename FloatType>
inline FloatType scalar_dot3(const Vector4Type& lhs, const Vector4Type& rhs)
{
	const FloatType lhs_x = vector_get_x(lhs);
	const FloatType lhs_y = vector_get_y(lhs);
	const FloatType lhs_z = vector_get_z(lhs);
	const FloatType rhs_x = vector_get_x(rhs);
	const FloatType rhs_y = vector_get_y(rhs);
	const FloatType rhs_z = vector_get_z(rhs);
	return (lhs_x * rhs_x) + (lhs_y * rhs_y) + (lhs_z * rhs_z);
}

template<typename Vector4Type, typename FloatType>
inline Vector4Type scalar_normalize3(const Vector4Type& input, const Vector4Type& fallback, FloatType threshold)
{
	FloatType len_sq = scalar_dot3<Vector4Type, FloatType>(input, input);
	if (len_sq >= threshold)
	{
		FloatType inv_len = rtm::scalar_sqrt_reciprocal(len_sq);
		return vector_set(vector_get_x(input) * inv_len, vector_get_y(input) * inv_len, vector_get_z(input) * inv_len);
	}
	else
		return fallback;
}

template<typename FloatType>
void test_vector4_getset_impl()
{
	using QuatType = typename float_traits<FloatType>::quat;
	using Vector4Type = typename float_traits<FloatType>::vector4;
	using ScalarType = typename float_traits<FloatType>::scalar;
	using Float2Type = typename float_traits<FloatType>::float2;
	using Float3Type = typename float_traits<FloatType>::float3;
	using Float4Type = typename float_traits<FloatType>::float4;

	const Vector4Type zero = vector_zero();
	const QuatType identity = quat_identity();

	struct alignas(16) Tmp
	{
		uint8_t padding0[8];	//  8 |  8
		FloatType values[4];	// 24 | 40
		uint8_t padding1[8];	// 32 | 48
	};

	Tmp tmp = { { 0 }, { FloatType(0.0), FloatType(2.34), FloatType(-3.12), FloatType(10000.0) }, {} };
	alignas(16) uint8_t buffer[64];

	const FloatType test_value0_flt[4] = { FloatType(2.0), FloatType(9.34), FloatType(-54.12), FloatType(6000.0) };
	const FloatType test_value1_flt[4] = { FloatType(0.75), FloatType(-4.52), FloatType(44.68), FloatType(-54225.0) };
	const FloatType test_value3_flt[4] = { FloatType(2.0), FloatType(-9.34), FloatType(54.12), FloatType(6000.1) };
	const Vector4Type test_value0 = vector_set(test_value0_flt[0], test_value0_flt[1], test_value0_flt[2], test_value0_flt[3]);
	const Vector4Type test_value1 = vector_set(test_value1_flt[0], test_value1_flt[1], test_value1_flt[2], test_value1_flt[3]);
	const Vector4Type test_value3 = vector_set(test_value3_flt[0], test_value3_flt[1], test_value3_flt[2], test_value3_flt[3]);

	//////////////////////////////////////////////////////////////////////////
	// Setters, getters, and casts

	CHECK(scalar_cast(vector_get_x(vector_set(FloatType(0.0), FloatType(2.34), FloatType(-3.12), FloatType(10000.0)))) == FloatType(0.0));
	CHECK(scalar_cast(vector_get_y(vector_set(FloatType(0.0), FloatType(2.34), FloatType(-3.12), FloatType(10000.0)))) == FloatType(2.34));
	CHECK(scalar_cast(vector_get_z(vector_set(FloatType(0.0), FloatType(2.34), FloatType(-3.12), FloatType(10000.0)))) == FloatType(-3.12));
	CHECK(scalar_cast(vector_get_w(vector_set(FloatType(0.0), FloatType(2.34), FloatType(-3.12), FloatType(10000.0)))) == FloatType(10000.0));

	CHECK(scalar_cast(vector_get_x(vector_set(scalar_set(FloatType(0.0)), scalar_set(FloatType(2.34)), scalar_set(FloatType(-3.12)), scalar_set(FloatType(10000.0))))) == FloatType(0.0));
	CHECK(scalar_cast(vector_get_y(vector_set(scalar_set(FloatType(0.0)), scalar_set(FloatType(2.34)), scalar_set(FloatType(-3.12)), scalar_set(FloatType(10000.0))))) == FloatType(2.34));
	CHECK(scalar_cast(vector_get_z(vector_set(scalar_set(FloatType(0.0)), scalar_set(FloatType(2.34)), scalar_set(FloatType(-3.12)), scalar_set(FloatType(10000.0))))) == FloatType(-3.12));
	CHECK(scalar_cast(vector_get_w(vector_set(scalar_set(FloatType(0.0)), scalar_set(FloatType(2.34)), scalar_set(FloatType(-3.12)), scalar_set(FloatType(10000.0))))) == FloatType(10000.0));

	CHECK(scalar_cast(vector_get_x(vector_set(FloatType(0.0), FloatType(2.34), FloatType(-3.12)))) == FloatType(0.0));
	CHECK(scalar_cast(vector_get_y(vector_set(FloatType(0.0), FloatType(2.34), FloatType(-3.12)))) == FloatType(2.34));
	CHECK(scalar_cast(vector_get_z(vector_set(FloatType(0.0), FloatType(2.34), FloatType(-3.12)))) == FloatType(-3.12));
	CHECK(scalar_cast(vector_get_w(vector_set(FloatType(0.0), FloatType(2.34), FloatType(-3.12)))) == FloatType(0.0));

	CHECK(scalar_cast(vector_get_x(vector_set(scalar_set(FloatType(0.0)), scalar_set(FloatType(2.34)), scalar_set(FloatType(-3.12))))) == FloatType(0.0));
	CHECK(scalar_cast(vector_get_y(vector_set(scalar_set(FloatType(0.0)), scalar_set(FloatType(2.34)), scalar_set(FloatType(-3.12))))) == FloatType(2.34));
	CHECK(scalar_cast(vector_get_z(vector_set(scalar_set(FloatType(0.0)), scalar_set(FloatType(2.34)), scalar_set(FloatType(-3.12))))) == FloatType(-3.12));
	CHECK(scalar_cast(vector_get_w(vector_set(scalar_set(FloatType(0.0)), scalar_set(FloatType(2.34)), scalar_set(FloatType(-3.12))))) == FloatType(0.0));

	CHECK(scalar_cast(vector_get_x(vector_set(FloatType(-3.12)))) == FloatType(-3.12));
	CHECK(scalar_cast(vector_get_y(vector_set(FloatType(-3.12)))) == FloatType(-3.12));
	CHECK(scalar_cast(vector_get_z(vector_set(FloatType(-3.12)))) == FloatType(-3.12));
	CHECK(scalar_cast(vector_get_w(vector_set(FloatType(-3.12)))) == FloatType(-3.12));

	CHECK(scalar_cast(vector_get_x(vector_set(scalar_set(FloatType(-3.12))))) == FloatType(-3.12));
	CHECK(scalar_cast(vector_get_y(vector_set(scalar_set(FloatType(-3.12))))) == FloatType(-3.12));
	CHECK(scalar_cast(vector_get_z(vector_set(scalar_set(FloatType(-3.12))))) == FloatType(-3.12));
	CHECK(scalar_cast(vector_get_w(vector_set(scalar_set(FloatType(-3.12))))) == FloatType(-3.12));

	CHECK(scalar_cast(vector_get_x(zero)) == FloatType(0.0));
	CHECK(scalar_cast(vector_get_y(zero)) == FloatType(0.0));
	CHECK(scalar_cast(vector_get_z(zero)) == FloatType(0.0));
	CHECK(scalar_cast(vector_get_w(zero)) == FloatType(0.0));

	CHECK(scalar_cast(vector_get_x((Vector4Type)vector_load(&tmp.values[0]))) == tmp.values[0]);
	CHECK(scalar_cast(vector_get_y((Vector4Type)vector_load(&tmp.values[0]))) == tmp.values[1]);
	CHECK(scalar_cast(vector_get_z((Vector4Type)vector_load(&tmp.values[0]))) == tmp.values[2]);
	CHECK(scalar_cast(vector_get_w((Vector4Type)vector_load(&tmp.values[0]))) == tmp.values[3]);

	CHECK(scalar_cast(vector_get_x((Vector4Type)vector_load1(&tmp.values[1]))) == tmp.values[1]);
	CHECK(scalar_cast(vector_get_y((Vector4Type)vector_load1(&tmp.values[1]))) == FloatType(0.0));
	CHECK(scalar_cast(vector_get_z((Vector4Type)vector_load1(&tmp.values[1]))) == FloatType(0.0));
	CHECK(scalar_cast(vector_get_w((Vector4Type)vector_load1(&tmp.values[1]))) == FloatType(0.0));

	CHECK(scalar_cast(vector_get_x((Vector4Type)vector_load2(&tmp.values[1]))) == tmp.values[1]);
	CHECK(scalar_cast(vector_get_y((Vector4Type)vector_load2(&tmp.values[1]))) == tmp.values[2]);
	CHECK(scalar_cast(vector_get_z((Vector4Type)vector_load2(&tmp.values[1]))) == FloatType(0.0));
	CHECK(scalar_cast(vector_get_w((Vector4Type)vector_load2(&tmp.values[1]))) == FloatType(0.0));

	CHECK(scalar_cast(vector_get_x((Vector4Type)vector_load3(&tmp.values[1]))) == tmp.values[1]);
	CHECK(scalar_cast(vector_get_y((Vector4Type)vector_load3(&tmp.values[1]))) == tmp.values[2]);
	CHECK(scalar_cast(vector_get_z((Vector4Type)vector_load3(&tmp.values[1]))) == tmp.values[3]);
	CHECK(scalar_cast(vector_get_w((Vector4Type)vector_load3(&tmp.values[1]))) == FloatType(0.0));

	Float2Type tmpf2 = { tmp.values[0], tmp.values[1] };
	Float3Type tmpf3 = { tmp.values[0], tmp.values[1], tmp.values[2] };
	Float4Type tmpf4 = { tmp.values[0], tmp.values[1], tmp.values[2], tmp.values[3] };

	CHECK(scalar_cast(vector_get_x((Vector4Type)vector_load2(&tmpf2))) == tmpf2.x);
	CHECK(scalar_cast(vector_get_y((Vector4Type)vector_load2(&tmpf2))) == tmpf2.y);
	CHECK(scalar_cast(vector_get_z((Vector4Type)vector_load2(&tmpf2))) == FloatType(0.0));
	CHECK(scalar_cast(vector_get_w((Vector4Type)vector_load2(&tmpf2))) == FloatType(0.0));

	CHECK(scalar_cast(vector_get_x((Vector4Type)vector_load3(&tmpf3))) == tmpf3.x);
	CHECK(scalar_cast(vector_get_y((Vector4Type)vector_load3(&tmpf3))) == tmpf3.y);
	CHECK(scalar_cast(vector_get_z((Vector4Type)vector_load3(&tmpf3))) == tmpf3.z);
	CHECK(scalar_cast(vector_get_w((Vector4Type)vector_load3(&tmpf3))) == FloatType(0.0));

	CHECK(scalar_cast(vector_get_x((Vector4Type)vector_load(&tmpf4))) == tmpf4.x);
	CHECK(scalar_cast(vector_get_y((Vector4Type)vector_load(&tmpf4))) == tmpf4.y);
	CHECK(scalar_cast(vector_get_z((Vector4Type)vector_load(&tmpf4))) == tmpf4.z);
	CHECK(scalar_cast(vector_get_w((Vector4Type)vector_load(&tmpf4))) == tmpf4.w);

	std::memcpy(&buffer[1], &tmp.values[0], sizeof(tmp.values));
	CHECK(scalar_cast(vector_get_x((Vector4Type)vector_load(&buffer[1]))) == tmp.values[0]);
	CHECK(scalar_cast(vector_get_y((Vector4Type)vector_load(&buffer[1]))) == tmp.values[1]);
	CHECK(scalar_cast(vector_get_z((Vector4Type)vector_load(&buffer[1]))) == tmp.values[2]);
	CHECK(scalar_cast(vector_get_w((Vector4Type)vector_load(&buffer[1]))) == tmp.values[3]);

	CHECK(scalar_cast(vector_get_x((Vector4Type)vector_load1(&buffer[1 + sizeof(FloatType)]))) == tmp.values[1]);
	CHECK(scalar_cast(vector_get_y((Vector4Type)vector_load1(&buffer[1 + sizeof(FloatType)]))) == FloatType(0.0));
	CHECK(scalar_cast(vector_get_z((Vector4Type)vector_load1(&buffer[1 + sizeof(FloatType)]))) == FloatType(0.0));
	CHECK(scalar_cast(vector_get_w((Vector4Type)vector_load1(&buffer[1 + sizeof(FloatType)]))) == FloatType(0.0));

	CHECK(scalar_cast(vector_get_x((Vector4Type)vector_load2(&buffer[1 + sizeof(FloatType)]))) == tmp.values[1]);
	CHECK(scalar_cast(vector_get_y((Vector4Type)vector_load2(&buffer[1 + sizeof(FloatType)]))) == tmp.values[2]);
	CHECK(scalar_cast(vector_get_z((Vector4Type)vector_load2(&buffer[1 + sizeof(FloatType)]))) == FloatType(0.0));
	CHECK(scalar_cast(vector_get_w((Vector4Type)vector_load2(&buffer[1 + sizeof(FloatType)]))) == FloatType(0.0));

	CHECK(scalar_cast(vector_get_x((Vector4Type)vector_load3(&buffer[1 + sizeof(FloatType)]))) == tmp.values[1]);
	CHECK(scalar_cast(vector_get_y((Vector4Type)vector_load3(&buffer[1 + sizeof(FloatType)]))) == tmp.values[2]);
	CHECK(scalar_cast(vector_get_z((Vector4Type)vector_load3(&buffer[1 + sizeof(FloatType)]))) == tmp.values[3]);
	CHECK(scalar_cast(vector_get_w((Vector4Type)vector_load3(&buffer[1 + sizeof(FloatType)]))) == FloatType(0.0));

	CHECK(scalar_cast(vector_get_x((Vector4Type)vector_broadcast(&tmp.values[0]))) == tmp.values[0]);
	CHECK(scalar_cast(vector_get_y((Vector4Type)vector_broadcast(&tmp.values[0]))) == tmp.values[0]);
	CHECK(scalar_cast(vector_get_z((Vector4Type)vector_broadcast(&tmp.values[0]))) == tmp.values[0]);
	CHECK(scalar_cast(vector_get_w((Vector4Type)vector_broadcast(&tmp.values[0]))) == tmp.values[0]);

	CHECK(scalar_cast(vector_get_x(quat_to_vector(identity))) == quat_get_x(identity));
	CHECK(scalar_cast(vector_get_y(quat_to_vector(identity))) == quat_get_y(identity));
	CHECK(scalar_cast(vector_get_z(quat_to_vector(identity))) == quat_get_z(identity));
	CHECK(scalar_cast(vector_get_w(quat_to_vector(identity))) == quat_get_w(identity));

	CHECK(vector_get_component<mix4::x>(vector_set(FloatType(0.0), FloatType(2.34), FloatType(-3.12), FloatType(10000.0))) == FloatType(0.0));
	CHECK(vector_get_component<mix4::y>(vector_set(FloatType(0.0), FloatType(2.34), FloatType(-3.12), FloatType(10000.0))) == FloatType(2.34));
	CHECK(vector_get_component<mix4::z>(vector_set(FloatType(0.0), FloatType(2.34), FloatType(-3.12), FloatType(10000.0))) == FloatType(-3.12));
	CHECK(vector_get_component<mix4::w>(vector_set(FloatType(0.0), FloatType(2.34), FloatType(-3.12), FloatType(10000.0))) == FloatType(10000.0));

	CHECK(vector_get_component<mix4::a>(vector_set(FloatType(0.0), FloatType(2.34), FloatType(-3.12), FloatType(10000.0))) == FloatType(0.0));
	CHECK(vector_get_component<mix4::b>(vector_set(FloatType(0.0), FloatType(2.34), FloatType(-3.12), FloatType(10000.0))) == FloatType(2.34));
	CHECK(vector_get_component<mix4::c>(vector_set(FloatType(0.0), FloatType(2.34), FloatType(-3.12), FloatType(10000.0))) == FloatType(-3.12));
	CHECK(vector_get_component<mix4::d>(vector_set(FloatType(0.0), FloatType(2.34), FloatType(-3.12), FloatType(10000.0))) == FloatType(10000.0));

	CHECK(scalar_cast(vector_get_component<mix4::x>(vector_set(FloatType(0.0), FloatType(2.34), FloatType(-3.12), FloatType(10000.0)))) == FloatType(0.0));
	CHECK(scalar_cast(vector_get_component<mix4::y>(vector_set(FloatType(0.0), FloatType(2.34), FloatType(-3.12), FloatType(10000.0)))) == FloatType(2.34));
	CHECK(scalar_cast(vector_get_component<mix4::z>(vector_set(FloatType(0.0), FloatType(2.34), FloatType(-3.12), FloatType(10000.0)))) == FloatType(-3.12));
	CHECK(scalar_cast(vector_get_component<mix4::w>(vector_set(FloatType(0.0), FloatType(2.34), FloatType(-3.12), FloatType(10000.0)))) == FloatType(10000.0));

	CHECK(scalar_cast(vector_get_component<mix4::a>(vector_set(FloatType(0.0), FloatType(2.34), FloatType(-3.12), FloatType(10000.0)))) == FloatType(0.0));
	CHECK(scalar_cast(vector_get_component<mix4::b>(vector_set(FloatType(0.0), FloatType(2.34), FloatType(-3.12), FloatType(10000.0)))) == FloatType(2.34));
	CHECK(scalar_cast(vector_get_component<mix4::c>(vector_set(FloatType(0.0), FloatType(2.34), FloatType(-3.12), FloatType(10000.0)))) == FloatType(-3.12));
	CHECK(scalar_cast(vector_get_component<mix4::d>(vector_set(FloatType(0.0), FloatType(2.34), FloatType(-3.12), FloatType(10000.0)))) == FloatType(10000.0));

	CHECK(vector_get_component(vector_set(FloatType(0.0), FloatType(2.34), FloatType(-3.12), FloatType(10000.0)), mix4::x) == FloatType(0.0));
	CHECK(vector_get_component(vector_set(FloatType(0.0), FloatType(2.34), FloatType(-3.12), FloatType(10000.0)), mix4::y) == FloatType(2.34));
	CHECK(vector_get_component(vector_set(FloatType(0.0), FloatType(2.34), FloatType(-3.12), FloatType(10000.0)), mix4::z) == FloatType(-3.12));
	CHECK(vector_get_component(vector_set(FloatType(0.0), FloatType(2.34), FloatType(-3.12), FloatType(10000.0)), mix4::w) == FloatType(10000.0));

	CHECK(vector_get_component(vector_set(FloatType(0.0), FloatType(2.34), FloatType(-3.12), FloatType(10000.0)), mix4::a) == FloatType(0.0));
	CHECK(vector_get_component(vector_set(FloatType(0.0), FloatType(2.34), FloatType(-3.12), FloatType(10000.0)), mix4::b) == FloatType(2.34));
	CHECK(vector_get_component(vector_set(FloatType(0.0), FloatType(2.34), FloatType(-3.12), FloatType(10000.0)), mix4::c) == FloatType(-3.12));
	CHECK(vector_get_component(vector_set(FloatType(0.0), FloatType(2.34), FloatType(-3.12), FloatType(10000.0)), mix4::d) == FloatType(10000.0));

	CHECK((vector_as_float_ptr_raw<Vector4Type, FloatType>(vector_load(&tmp.values[0]))[0] == tmp.values[0]));
	CHECK((vector_as_float_ptr_raw<Vector4Type, FloatType>(vector_load(&tmp.values[0]))[1] == tmp.values[1]));
	CHECK((vector_as_float_ptr_raw<Vector4Type, FloatType>(vector_load(&tmp.values[0]))[2] == tmp.values[2]));
	CHECK((vector_as_float_ptr_raw<Vector4Type, FloatType>(vector_load(&tmp.values[0]))[3] == tmp.values[3]));

	vector_store(test_value0, &tmp.values[0]);
	CHECK(scalar_cast(vector_get_x(test_value0)) == tmp.values[0]);
	CHECK(scalar_cast(vector_get_y(test_value0)) == tmp.values[1]);
	CHECK(scalar_cast(vector_get_z(test_value0)) == tmp.values[2]);
	CHECK(scalar_cast(vector_get_w(test_value0)) == tmp.values[3]);

	vector_store1(test_value0, &tmp.values[0]);
	CHECK(scalar_cast(vector_get_x(test_value0)) == tmp.values[0]);

	vector_store2(test_value0, &tmp.values[0]);
	CHECK(scalar_cast(vector_get_x(test_value0)) == tmp.values[0]);
	CHECK(scalar_cast(vector_get_y(test_value0)) == tmp.values[1]);

	vector_store3(test_value1, &tmp.values[0]);
	CHECK(scalar_cast(vector_get_x(test_value1)) == tmp.values[0]);
	CHECK(scalar_cast(vector_get_y(test_value1)) == tmp.values[1]);
	CHECK(scalar_cast(vector_get_z(test_value1)) == tmp.values[2]);
	CHECK(scalar_cast(vector_get_w(test_value0)) == tmp.values[3]);

	vector_store(test_value1, &buffer[1]);
	CHECK(scalar_cast(vector_get_x(test_value1)) == scalar_cast(vector_get_x((Vector4Type)vector_load(&buffer[1]))));
	CHECK(scalar_cast(vector_get_y(test_value1)) == scalar_cast(vector_get_y((Vector4Type)vector_load(&buffer[1]))));
	CHECK(scalar_cast(vector_get_z(test_value1)) == scalar_cast(vector_get_z((Vector4Type)vector_load(&buffer[1]))));
	CHECK(scalar_cast(vector_get_w(test_value1)) == scalar_cast(vector_get_w((Vector4Type)vector_load(&buffer[1]))));

	vector_store1(test_value1, &buffer[1]);
	CHECK(scalar_cast(vector_get_x(test_value1)) == scalar_cast(vector_get_x((Vector4Type)vector_load1(&buffer[1]))));

	vector_store2(test_value1, &buffer[1]);
	CHECK(scalar_cast(vector_get_x(test_value1)) == scalar_cast(vector_get_x((Vector4Type)vector_load2(&buffer[1]))));

	vector_store3(test_value1, &buffer[1]);
	CHECK(scalar_cast(vector_get_x(test_value1)) == scalar_cast(vector_get_x((Vector4Type)vector_load3(&buffer[1]))));
	CHECK(scalar_cast(vector_get_y(test_value1)) == scalar_cast(vector_get_y((Vector4Type)vector_load3(&buffer[1]))));
	CHECK(scalar_cast(vector_get_z(test_value1)) == scalar_cast(vector_get_z((Vector4Type)vector_load3(&buffer[1]))));

	vector_store(test_value1, &tmpf4);
	CHECK(scalar_cast(vector_get_x(test_value1)) == tmpf4.x);
	CHECK(scalar_cast(vector_get_y(test_value1)) == tmpf4.y);
	CHECK(scalar_cast(vector_get_z(test_value1)) == tmpf4.z);
	CHECK(scalar_cast(vector_get_w(test_value1)) == tmpf4.w);

	vector_store2(test_value1, &tmpf2);
	CHECK(scalar_cast(vector_get_x(test_value1)) == tmpf2.x);
	CHECK(scalar_cast(vector_get_y(test_value1)) == tmpf2.y);

	vector_store3(test_value1, &tmpf3);
	CHECK(scalar_cast(vector_get_x(test_value1)) == tmpf3.x);
	CHECK(scalar_cast(vector_get_y(test_value1)) == tmpf3.y);
	CHECK(scalar_cast(vector_get_z(test_value1)) == tmpf3.z);

	CHECK((FloatType)vector_get_min_component(test_value0) == scalar_cast(vector_get_z(test_value0)));
	CHECK((FloatType)vector_get_min_component(test_value3) == scalar_cast(vector_get_y(test_value3)));
	CHECK((FloatType)vector_get_max_component(test_value0) == scalar_cast(vector_get_w(test_value0)));
	CHECK(scalar_is_equal(vector_get_min_component(test_value0), (ScalarType)vector_as_scalar(vector_dup_z(test_value0))));
	CHECK(scalar_is_equal(vector_get_min_component(test_value3), (ScalarType)vector_as_scalar(vector_dup_y(test_value3))));
	CHECK(scalar_is_equal(vector_get_max_component(test_value0), (ScalarType)vector_as_scalar(vector_dup_w(test_value0))));

	CHECK(vector_all_near_equal(vector_set_x(zero, FloatType(4.0)), vector_set(FloatType(4.0), FloatType(0.0), FloatType(0.0), FloatType(0.0)), FloatType(0.0)));
	CHECK(vector_all_near_equal(vector_set_y(zero, FloatType(4.0)), vector_set(FloatType(0.0), FloatType(4.0), FloatType(0.0), FloatType(0.0)), FloatType(0.0)));
	CHECK(vector_all_near_equal(vector_set_z(zero, FloatType(4.0)), vector_set(FloatType(0.0), FloatType(0.0), FloatType(4.0), FloatType(0.0)), FloatType(0.0)));
	CHECK(vector_all_near_equal(vector_set_w(zero, FloatType(4.0)), vector_set(FloatType(0.0), FloatType(0.0), FloatType(0.0), FloatType(4.0)), FloatType(0.0)));

	CHECK(vector_all_near_equal(vector_set_x(zero, scalar_set(FloatType(4.0))), vector_set(FloatType(4.0), FloatType(0.0), FloatType(0.0), FloatType(0.0)), FloatType(0.0)));
	CHECK(vector_all_near_equal(vector_set_y(zero, scalar_set(FloatType(4.0))), vector_set(FloatType(0.0), FloatType(4.0), FloatType(0.0), FloatType(0.0)), FloatType(0.0)));
	CHECK(vector_all_near_equal(vector_set_z(zero, scalar_set(FloatType(4.0))), vector_set(FloatType(0.0), FloatType(0.0), FloatType(4.0), FloatType(0.0)), FloatType(0.0)));
	CHECK(vector_all_near_equal(vector_set_w(zero, scalar_set(FloatType(4.0))), vector_set(FloatType(0.0), FloatType(0.0), FloatType(0.0), FloatType(4.0)), FloatType(0.0)));

	CHECK((FloatType)vector_as_scalar(test_value1) == scalar_cast(vector_get_x(test_value1)));
	CHECK(scalar_is_equal(vector_as_scalar(test_value1), scalar_set(vector_get_x(test_value1))));
}

template<typename FloatType>
void test_vector4_arithmetic_impl(const FloatType threshold)
{
	using Vector4Type = typename float_traits<FloatType>::vector4;
	using ScalarType = typename float_traits<FloatType>::scalar;

	const Vector4Type zero = vector_zero();

	const FloatType test_value0_flt[4] = { FloatType(2.0), FloatType(9.34), FloatType(-54.12), FloatType(6000.0) };
	const FloatType test_value1_flt[4] = { FloatType(0.75), FloatType(-4.52), FloatType(44.68), FloatType(-54225.0) };
	const FloatType test_value2_flt[4] = { FloatType(-2.65), FloatType(2.996113), FloatType(0.68123521), FloatType(-5.9182) };
	const Vector4Type test_value0 = vector_set(test_value0_flt[0], test_value0_flt[1], test_value0_flt[2], test_value0_flt[3]);
	const Vector4Type test_value1 = vector_set(test_value1_flt[0], test_value1_flt[1], test_value1_flt[2], test_value1_flt[3]);
	const Vector4Type test_value2 = vector_set(test_value2_flt[0], test_value2_flt[1], test_value2_flt[2], test_value2_flt[3]);

	//////////////////////////////////////////////////////////////////////////
	// Arithmetic

	CHECK(scalar_near_equal(vector_get_x(vector_add(test_value0, test_value1)), test_value0_flt[0] + test_value1_flt[0], threshold));
	CHECK(scalar_near_equal(vector_get_y(vector_add(test_value0, test_value1)), test_value0_flt[1] + test_value1_flt[1], threshold));
	CHECK(scalar_near_equal(vector_get_z(vector_add(test_value0, test_value1)), test_value0_flt[2] + test_value1_flt[2], threshold));
	CHECK(scalar_near_equal(vector_get_w(vector_add(test_value0, test_value1)), test_value0_flt[3] + test_value1_flt[3], threshold));

	CHECK(scalar_near_equal(vector_get_x(vector_sub(test_value0, test_value1)), test_value0_flt[0] - test_value1_flt[0], threshold));
	CHECK(scalar_near_equal(vector_get_y(vector_sub(test_value0, test_value1)), test_value0_flt[1] - test_value1_flt[1], threshold));
	CHECK(scalar_near_equal(vector_get_z(vector_sub(test_value0, test_value1)), test_value0_flt[2] - test_value1_flt[2], threshold));
	CHECK(scalar_near_equal(vector_get_w(vector_sub(test_value0, test_value1)), test_value0_flt[3] - test_value1_flt[3], threshold));

	CHECK(scalar_near_equal(vector_get_x(vector_mul(test_value0, test_value1)), test_value0_flt[0] * test_value1_flt[0], threshold));
	CHECK(scalar_near_equal(vector_get_y(vector_mul(test_value0, test_value1)), test_value0_flt[1] * test_value1_flt[1], threshold));
	CHECK(scalar_near_equal(vector_get_z(vector_mul(test_value0, test_value1)), test_value0_flt[2] * test_value1_flt[2], threshold));
	// We have a strange codegen bug with gcc5, use the Catch near equal impl instead
	CHECK(scalar_cast(vector_get_w(vector_mul(test_value0, test_value1))) == Approx(test_value0_flt[3] * test_value1_flt[3]).margin(threshold));

	CHECK(scalar_near_equal(vector_get_x(vector_mul(test_value0, FloatType(2.34))), test_value0_flt[0] * FloatType(2.34), threshold));
	CHECK(scalar_near_equal(vector_get_y(vector_mul(test_value0, FloatType(2.34))), test_value0_flt[1] * FloatType(2.34), threshold));
	CHECK(scalar_near_equal(vector_get_z(vector_mul(test_value0, FloatType(2.34))), test_value0_flt[2] * FloatType(2.34), threshold));
	CHECK(scalar_near_equal(vector_get_w(vector_mul(test_value0, FloatType(2.34))), test_value0_flt[3] * FloatType(2.34), threshold));

	CHECK(scalar_near_equal(vector_get_x(vector_mul(test_value0, scalar_set(FloatType(2.34)))), test_value0_flt[0] * FloatType(2.34), threshold));
	CHECK(scalar_near_equal(vector_get_y(vector_mul(test_value0, scalar_set(FloatType(2.34)))), test_value0_flt[1] * FloatType(2.34), threshold));
	CHECK(scalar_near_equal(vector_get_z(vector_mul(test_value0, scalar_set(FloatType(2.34)))), test_value0_flt[2] * FloatType(2.34), threshold));
	CHECK(scalar_near_equal(vector_get_w(vector_mul(test_value0, scalar_set(FloatType(2.34)))), test_value0_flt[3] * FloatType(2.34), threshold));

	CHECK(scalar_near_equal(vector_get_x(vector_div(test_value0, test_value1)), test_value0_flt[0] / test_value1_flt[0], threshold));
	CHECK(scalar_near_equal(vector_get_y(vector_div(test_value0, test_value1)), test_value0_flt[1] / test_value1_flt[1], threshold));
	CHECK(scalar_near_equal(vector_get_z(vector_div(test_value0, test_value1)), test_value0_flt[2] / test_value1_flt[2], threshold));
	CHECK(scalar_near_equal(vector_get_w(vector_div(test_value0, test_value1)), test_value0_flt[3] / test_value1_flt[3], threshold));

	CHECK(scalar_near_equal(vector_get_x(vector_max(test_value0, test_value1)), scalar_max(test_value0_flt[0], test_value1_flt[0]), threshold));
	CHECK(scalar_near_equal(vector_get_y(vector_max(test_value0, test_value1)), scalar_max(test_value0_flt[1], test_value1_flt[1]), threshold));
	CHECK(scalar_near_equal(vector_get_z(vector_max(test_value0, test_value1)), scalar_max(test_value0_flt[2], test_value1_flt[2]), threshold));
	CHECK(scalar_near_equal(vector_get_w(vector_max(test_value0, test_value1)), scalar_max(test_value0_flt[3], test_value1_flt[3]), threshold));

	CHECK(scalar_near_equal(vector_get_x(vector_min(test_value0, test_value1)), scalar_min(test_value0_flt[0], test_value1_flt[0]), threshold));
	CHECK(scalar_near_equal(vector_get_y(vector_min(test_value0, test_value1)), scalar_min(test_value0_flt[1], test_value1_flt[1]), threshold));
	CHECK(scalar_near_equal(vector_get_z(vector_min(test_value0, test_value1)), scalar_min(test_value0_flt[2], test_value1_flt[2]), threshold));
	CHECK(scalar_near_equal(vector_get_w(vector_min(test_value0, test_value1)), scalar_min(test_value0_flt[3], test_value1_flt[3]), threshold));

	CHECK(scalar_near_equal(vector_get_x(vector_clamp(test_value0, test_value1, test_value2)), scalar_clamp(test_value0_flt[0], test_value1_flt[0], test_value2_flt[0]), threshold));
	CHECK(scalar_near_equal(vector_get_y(vector_clamp(test_value0, test_value1, test_value2)), scalar_clamp(test_value0_flt[1], test_value1_flt[1], test_value2_flt[1]), threshold));
	CHECK(scalar_near_equal(vector_get_z(vector_clamp(test_value0, test_value1, test_value2)), scalar_clamp(test_value0_flt[2], test_value1_flt[2], test_value2_flt[2]), threshold));
	CHECK(scalar_near_equal(vector_get_w(vector_clamp(test_value0, test_value1, test_value2)), scalar_clamp(test_value0_flt[3], test_value1_flt[3], test_value2_flt[3]), threshold));

	CHECK(scalar_near_equal(vector_get_x(vector_abs(test_value0)), scalar_abs(test_value0_flt[0]), threshold));
	CHECK(scalar_near_equal(vector_get_y(vector_abs(test_value0)), scalar_abs(test_value0_flt[1]), threshold));
	CHECK(scalar_near_equal(vector_get_z(vector_abs(test_value0)), scalar_abs(test_value0_flt[2]), threshold));
	CHECK(scalar_near_equal(vector_get_w(vector_abs(test_value0)), scalar_abs(test_value0_flt[3]), threshold));

	CHECK(scalar_near_equal(vector_get_x(vector_neg(test_value0)), -test_value0_flt[0], threshold));
	CHECK(scalar_near_equal(vector_get_y(vector_neg(test_value0)), -test_value0_flt[1], threshold));
	CHECK(scalar_near_equal(vector_get_z(vector_neg(test_value0)), -test_value0_flt[2], threshold));
	CHECK(scalar_near_equal(vector_get_w(vector_neg(test_value0)), -test_value0_flt[3], threshold));

	CHECK(scalar_near_equal(vector_get_x(vector_reciprocal(test_value0)), scalar_reciprocal(test_value0_flt[0]), threshold));
	CHECK(scalar_near_equal(vector_get_y(vector_reciprocal(test_value0)), scalar_reciprocal(test_value0_flt[1]), threshold));
	CHECK(scalar_near_equal(vector_get_z(vector_reciprocal(test_value0)), scalar_reciprocal(test_value0_flt[2]), threshold));
	CHECK(scalar_near_equal(vector_get_w(vector_reciprocal(test_value0)), scalar_reciprocal(test_value0_flt[3]), threshold));

	CHECK(scalar_near_equal(vector_get_x(vector_floor(test_value0)), scalar_floor(test_value0_flt[0]), threshold));
	CHECK(scalar_near_equal(vector_get_y(vector_floor(test_value0)), scalar_floor(test_value0_flt[1]), threshold));
	CHECK(scalar_near_equal(vector_get_z(vector_floor(test_value0)), scalar_floor(test_value0_flt[2]), threshold));
	CHECK(scalar_near_equal(vector_get_w(vector_floor(test_value0)), scalar_floor(test_value0_flt[3]), threshold));

	CHECK(scalar_near_equal(vector_get_x(vector_ceil(test_value0)), scalar_ceil(test_value0_flt[0]), threshold));
	CHECK(scalar_near_equal(vector_get_y(vector_ceil(test_value0)), scalar_ceil(test_value0_flt[1]), threshold));
	CHECK(scalar_near_equal(vector_get_z(vector_ceil(test_value0)), scalar_ceil(test_value0_flt[2]), threshold));
	CHECK(scalar_near_equal(vector_get_w(vector_ceil(test_value0)), scalar_ceil(test_value0_flt[3]), threshold));

	const Vector4Type scalar_cross3_result = scalar_cross3<Vector4Type, FloatType>(test_value0, test_value1);
	const Vector4Type vector_cross3_result = vector_cross3(test_value0, test_value1);
	CHECK(scalar_near_equal(vector_get_x(vector_cross3_result), vector_get_x(scalar_cross3_result), threshold));
	CHECK(scalar_near_equal(vector_get_y(vector_cross3_result), vector_get_y(scalar_cross3_result), threshold));
	CHECK(scalar_near_equal(vector_get_z(vector_cross3_result), vector_get_z(scalar_cross3_result), threshold));

	const FloatType test_value10_flt[4] = { FloatType(-0.001138), FloatType(0.91623), FloatType(-1.624598), FloatType(0.715671) };
	const FloatType test_value11_flt[4] = { FloatType(0.1138), FloatType(-0.623), FloatType(1.4598), FloatType(-0.5671) };
	const Vector4Type test_value10 = vector_set(test_value10_flt[0], test_value10_flt[1], test_value10_flt[2], test_value10_flt[3]);
	const Vector4Type test_value11 = vector_set(test_value11_flt[0], test_value11_flt[1], test_value11_flt[2], test_value11_flt[3]);
	const FloatType scalar_dot_result = scalar_dot<Vector4Type, FloatType>(test_value10, test_value11);
	const FloatType vector_dot_result = vector_dot(test_value10, test_value11);
	CHECK(scalar_near_equal(vector_dot_result, scalar_dot_result, threshold));

	const FloatType scalar_dot3_result = scalar_dot3<Vector4Type, FloatType>(test_value10, test_value11);
	const FloatType vector_dot3_result = vector_dot3(test_value10, test_value11);
	CHECK(scalar_near_equal(vector_dot3_result, scalar_dot3_result, threshold));
	const ScalarType vector_dot3_result_scalar = vector_dot3(test_value10, test_value11);
	CHECK(scalar_is_equal(vector_dot3_result, scalar_cast(vector_dot3_result_scalar)));

	const ScalarType vector_sdot_result = vector_dot(test_value10, test_value11);
	CHECK(scalar_near_equal(scalar_cast(vector_sdot_result), scalar_dot_result, threshold));

	const Vector4Type vector_vdot_result = vector_dot_as_vector(test_value10, test_value11);
	CHECK(scalar_near_equal(vector_get_x(vector_vdot_result), scalar_dot_result, threshold));
	CHECK(scalar_near_equal(vector_get_y(vector_vdot_result), scalar_dot_result, threshold));
	CHECK(scalar_near_equal(vector_get_z(vector_vdot_result), scalar_dot_result, threshold));
	CHECK(scalar_near_equal(vector_get_w(vector_vdot_result), scalar_dot_result, threshold));

	CHECK(scalar_near_equal(scalar_dot<Vector4Type, FloatType>(test_value0, test_value0), vector_length_squared(test_value0), threshold));
	CHECK(scalar_near_equal(scalar_dot<Vector4Type, FloatType>(test_value0, test_value0), scalar_cast(vector_length_squared(test_value0)), threshold));
	const FloatType vector_length_squared3_ref = scalar_dot3<Vector4Type, FloatType>(test_value0, test_value0);
	const FloatType vector_length_squared3_result = vector_length_squared3(test_value0);
	CHECK(scalar_near_equal(vector_length_squared3_ref, vector_length_squared3_result, threshold));
	const ScalarType vector_length_squared3_result_scalar = vector_length_squared3(test_value0);
	CHECK(scalar_is_equal(vector_length_squared3_result, scalar_cast(vector_length_squared3_result_scalar)));

	CHECK(scalar_near_equal(rtm::scalar_sqrt(scalar_dot<Vector4Type, FloatType>(test_value0, test_value0)), vector_length(test_value0), threshold));
	CHECK(scalar_near_equal(rtm::scalar_sqrt(scalar_dot<Vector4Type, FloatType>(test_value0, test_value0)), scalar_cast(vector_length(test_value0)), threshold));
	const FloatType vector_length3_result = vector_length3(test_value0);
	CHECK(scalar_near_equal(rtm::scalar_sqrt(scalar_dot3<Vector4Type, FloatType>(test_value0, test_value0)), vector_length3_result, threshold));
	const ScalarType vector_length3_result_scalar = vector_length3(test_value0);
	CHECK(scalar_is_equal(vector_length3_result, scalar_cast(vector_length3_result_scalar)));

	CHECK(scalar_near_equal(rtm::scalar_sqrt_reciprocal(scalar_dot<Vector4Type, FloatType>(test_value0, test_value0)), vector_length_reciprocal(test_value0), threshold));
	CHECK(scalar_near_equal(rtm::scalar_sqrt_reciprocal(scalar_dot<Vector4Type, FloatType>(test_value0, test_value0)), scalar_cast(vector_length_reciprocal(test_value0)), threshold));
	CHECK(scalar_near_equal(rtm::scalar_sqrt_reciprocal(scalar_dot3<Vector4Type, FloatType>(test_value0, test_value0)), vector_length_reciprocal3(test_value0), threshold));
	CHECK(scalar_near_equal(rtm::scalar_sqrt_reciprocal(scalar_dot3<Vector4Type, FloatType>(test_value0, test_value0)), scalar_cast(vector_length_reciprocal3(test_value0)), threshold));

	const Vector4Type test_value_diff = vector_sub(test_value0, test_value1);
	const FloatType vector_distance3_result = vector_distance3(test_value0, test_value1);
	CHECK(scalar_near_equal(rtm::scalar_sqrt(scalar_dot3<Vector4Type, FloatType>(test_value_diff, test_value_diff)), vector_distance3_result, threshold));
	const ScalarType vector_distance3_result_scalar = vector_distance3(test_value0, test_value1);
	CHECK(scalar_is_equal(vector_distance3_result, scalar_cast(vector_distance3_result_scalar)));

	const Vector4Type scalar_normalize3_result = scalar_normalize3<Vector4Type, FloatType>(test_value0, zero, threshold);
	const Vector4Type vector_normalize3_result = vector_normalize3(test_value0);
	CHECK(scalar_near_equal(vector_get_x(vector_normalize3_result), vector_get_x(scalar_normalize3_result), threshold));
	CHECK(scalar_near_equal(vector_get_y(vector_normalize3_result), vector_get_y(scalar_normalize3_result), threshold));
	CHECK(scalar_near_equal(vector_get_z(vector_normalize3_result), vector_get_z(scalar_normalize3_result), threshold));

	const Vector4Type vector_normalize3_result_safe = vector_normalize3(test_value0, zero, threshold);
	CHECK(scalar_near_equal(vector_get_x(vector_normalize3_result_safe), vector_get_x(scalar_normalize3_result), threshold));
	CHECK(scalar_near_equal(vector_get_y(vector_normalize3_result_safe), vector_get_y(scalar_normalize3_result), threshold));
	CHECK(scalar_near_equal(vector_get_z(vector_normalize3_result_safe), vector_get_z(scalar_normalize3_result), threshold));

	const Vector4Type scalar_normalize3_result0 = scalar_normalize3<Vector4Type, FloatType>(zero, zero, threshold);
	const Vector4Type vector_normalize3_result0 = vector_normalize3(zero, zero, threshold);
	CHECK(scalar_near_equal(vector_get_x(vector_normalize3_result0), vector_get_x(scalar_normalize3_result0), threshold));
	CHECK(scalar_near_equal(vector_get_y(vector_normalize3_result0), vector_get_y(scalar_normalize3_result0), threshold));
	CHECK(scalar_near_equal(vector_get_z(vector_normalize3_result0), vector_get_z(scalar_normalize3_result0), threshold));

	CHECK(scalar_near_equal(vector_get_x(vector_lerp(test_value10, test_value11, FloatType(0.33))), ((test_value11_flt[0] - test_value10_flt[0]) * FloatType(0.33)) + test_value10_flt[0], threshold));
	CHECK(scalar_near_equal(vector_get_y(vector_lerp(test_value10, test_value11, FloatType(0.33))), ((test_value11_flt[1] - test_value10_flt[1]) * FloatType(0.33)) + test_value10_flt[1], threshold));
	CHECK(scalar_near_equal(vector_get_z(vector_lerp(test_value10, test_value11, FloatType(0.33))), ((test_value11_flt[2] - test_value10_flt[2]) * FloatType(0.33)) + test_value10_flt[2], threshold));
	CHECK(scalar_near_equal(vector_get_w(vector_lerp(test_value10, test_value11, FloatType(0.33))), ((test_value11_flt[3] - test_value10_flt[3]) * FloatType(0.33)) + test_value10_flt[3], threshold));

	// Lerp must be stable and return exactly the start when the interpolation alpha is 0.0 and exactly the end when 1.0
	CHECK(vector_all_near_equal(vector_lerp(test_value10, test_value11, FloatType(0.0)), test_value10, FloatType(0.0)));
	CHECK(vector_all_near_equal(vector_lerp(test_value10, test_value11, FloatType(1.0)), test_value11, FloatType(0.0)));

	CHECK(scalar_near_equal(vector_get_x(vector_fraction(test_value0)), scalar_fraction(test_value0_flt[0]), threshold));
	CHECK(scalar_near_equal(vector_get_y(vector_fraction(test_value0)), scalar_fraction(test_value0_flt[1]), threshold));
	CHECK(scalar_near_equal(vector_get_z(vector_fraction(test_value0)), scalar_fraction(test_value0_flt[2]), threshold));
	CHECK(scalar_near_equal(vector_get_w(vector_fraction(test_value0)), scalar_fraction(test_value0_flt[3]), threshold));

	CHECK(scalar_near_equal(vector_get_x(vector_mul_add(test_value10, test_value11, test_value2)), (test_value10_flt[0] * test_value11_flt[0]) + test_value2_flt[0], threshold));
	CHECK(scalar_near_equal(vector_get_y(vector_mul_add(test_value10, test_value11, test_value2)), (test_value10_flt[1] * test_value11_flt[1]) + test_value2_flt[1], threshold));
	CHECK(scalar_near_equal(vector_get_z(vector_mul_add(test_value10, test_value11, test_value2)), (test_value10_flt[2] * test_value11_flt[2]) + test_value2_flt[2], threshold));
	CHECK(scalar_near_equal(vector_get_w(vector_mul_add(test_value10, test_value11, test_value2)), (test_value10_flt[3] * test_value11_flt[3]) + test_value2_flt[3], threshold));

	CHECK(scalar_near_equal(vector_get_x(vector_mul_add(test_value10, test_value11_flt[0], test_value2)), (test_value10_flt[0] * test_value11_flt[0]) + test_value2_flt[0], threshold));
	CHECK(scalar_near_equal(vector_get_y(vector_mul_add(test_value10, test_value11_flt[0], test_value2)), (test_value10_flt[1] * test_value11_flt[0]) + test_value2_flt[1], threshold));
	CHECK(scalar_near_equal(vector_get_z(vector_mul_add(test_value10, test_value11_flt[0], test_value2)), (test_value10_flt[2] * test_value11_flt[0]) + test_value2_flt[2], threshold));
	CHECK(scalar_near_equal(vector_get_w(vector_mul_add(test_value10, test_value11_flt[0], test_value2)), (test_value10_flt[3] * test_value11_flt[0]) + test_value2_flt[3], threshold));

	CHECK(scalar_near_equal(vector_get_x(vector_neg_mul_sub(test_value10, test_value11, test_value2)), (test_value10_flt[0] * -test_value11_flt[0]) + test_value2_flt[0], threshold));
	CHECK(scalar_near_equal(vector_get_y(vector_neg_mul_sub(test_value10, test_value11, test_value2)), (test_value10_flt[1] * -test_value11_flt[1]) + test_value2_flt[1], threshold));
	CHECK(scalar_near_equal(vector_get_z(vector_neg_mul_sub(test_value10, test_value11, test_value2)), (test_value10_flt[2] * -test_value11_flt[2]) + test_value2_flt[2], threshold));
	CHECK(scalar_near_equal(vector_get_w(vector_neg_mul_sub(test_value10, test_value11, test_value2)), (test_value10_flt[3] * -test_value11_flt[3]) + test_value2_flt[3], threshold));

	CHECK(scalar_near_equal(vector_get_x(vector_neg_mul_sub(test_value10, test_value11_flt[0], test_value2)), (test_value10_flt[0] * -test_value11_flt[0]) + test_value2_flt[0], threshold));
	CHECK(scalar_near_equal(vector_get_y(vector_neg_mul_sub(test_value10, test_value11_flt[0], test_value2)), (test_value10_flt[1] * -test_value11_flt[0]) + test_value2_flt[1], threshold));
	CHECK(scalar_near_equal(vector_get_z(vector_neg_mul_sub(test_value10, test_value11_flt[0], test_value2)), (test_value10_flt[2] * -test_value11_flt[0]) + test_value2_flt[2], threshold));
	CHECK(scalar_near_equal(vector_get_w(vector_neg_mul_sub(test_value10, test_value11_flt[0], test_value2)), (test_value10_flt[3] * -test_value11_flt[0]) + test_value2_flt[3], threshold));
}

template<typename FloatType>
void test_vector4_relational_impl(const FloatType threshold)
{
	using Vector4Type = typename float_traits<FloatType>::vector4;

	const Vector4Type zero = vector_zero();

	const FloatType test_value0_flt[4] = { FloatType(2.0), FloatType(9.34), FloatType(-54.12), FloatType(6000.0) };
	const FloatType test_value1_flt[4] = { FloatType(0.75), FloatType(-4.52), FloatType(44.68), FloatType(-54225.0) };
	const FloatType test_value3_flt[4] = { FloatType(2.0), FloatType(-9.34), FloatType(54.12), FloatType(6000.1) };
	const Vector4Type test_value0 = vector_set(test_value0_flt[0], test_value0_flt[1], test_value0_flt[2], test_value0_flt[3]);
	const Vector4Type test_value1 = vector_set(test_value1_flt[0], test_value1_flt[1], test_value1_flt[2], test_value1_flt[3]);
	const Vector4Type test_value3 = vector_set(test_value3_flt[0], test_value3_flt[1], test_value3_flt[2], test_value3_flt[3]);

	//////////////////////////////////////////////////////////////////////////
	// Comparisons and masking

	CHECK((mask_get_x(vector_less_than(test_value0, test_value1)) != 0) == (test_value0_flt[0] < test_value1_flt[0]));
	CHECK((mask_get_y(vector_less_than(test_value0, test_value1)) != 0) == (test_value0_flt[1] < test_value1_flt[1]));
	CHECK((mask_get_z(vector_less_than(test_value0, test_value1)) != 0) == (test_value0_flt[2] < test_value1_flt[2]));
	CHECK((mask_get_w(vector_less_than(test_value0, test_value1)) != 0) == (test_value0_flt[3] < test_value1_flt[3]));

	CHECK((mask_get_x(vector_less_equal(test_value0, test_value3)) != 0) == (test_value0_flt[0] <= test_value3_flt[0]));
	CHECK((mask_get_y(vector_less_equal(test_value0, test_value3)) != 0) == (test_value0_flt[1] <= test_value3_flt[1]));
	CHECK((mask_get_z(vector_less_equal(test_value0, test_value3)) != 0) == (test_value0_flt[2] <= test_value3_flt[2]));
	CHECK((mask_get_w(vector_less_equal(test_value0, test_value3)) != 0) == (test_value0_flt[3] <= test_value3_flt[3]));

	CHECK((mask_get_x(vector_greater_equal(test_value0, test_value1)) != 0) == (test_value0_flt[0] >= test_value1_flt[0]));
	CHECK((mask_get_y(vector_greater_equal(test_value0, test_value1)) != 0) == (test_value0_flt[1] >= test_value1_flt[1]));
	CHECK((mask_get_z(vector_greater_equal(test_value0, test_value1)) != 0) == (test_value0_flt[2] >= test_value1_flt[2]));
	CHECK((mask_get_w(vector_greater_equal(test_value0, test_value1)) != 0) == (test_value0_flt[3] >= test_value1_flt[3]));

	CHECK(vector_all_less_than(zero, vector_set(FloatType(1.0), FloatType(1.0), FloatType(1.0), FloatType(1.0))) == true);
	CHECK(vector_all_less_than(zero, vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0), FloatType(0.0))) == false);
	CHECK(vector_all_less_than(zero, vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0), FloatType(0.0))) == false);
	CHECK(vector_all_less_than(zero, vector_set(FloatType(0.0), FloatType(0.0), FloatType(1.0), FloatType(0.0))) == false);
	CHECK(vector_all_less_than(zero, vector_set(FloatType(0.0), FloatType(0.0), FloatType(0.0), FloatType(1.0))) == false);
	CHECK(vector_all_less_than(zero, zero) == false);

	CHECK(vector_all_less_than2(zero, vector_set(FloatType(1.0), FloatType(1.0), FloatType(1.0), FloatType(0.0))) == true);
	CHECK(vector_all_less_than2(zero, vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0), FloatType(0.0))) == false);
	CHECK(vector_all_less_than2(zero, vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0), FloatType(0.0))) == false);
	CHECK(vector_all_less_than2(zero, vector_set(FloatType(0.0), FloatType(0.0), FloatType(1.0), FloatType(0.0))) == false);
	CHECK(vector_all_less_than2(zero, zero) == false);

	CHECK(vector_all_less_than3(zero, vector_set(FloatType(1.0), FloatType(1.0), FloatType(1.0), FloatType(0.0))) == true);
	CHECK(vector_all_less_than3(zero, vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0), FloatType(0.0))) == false);
	CHECK(vector_all_less_than3(zero, vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0), FloatType(0.0))) == false);
	CHECK(vector_all_less_than3(zero, vector_set(FloatType(0.0), FloatType(0.0), FloatType(1.0), FloatType(0.0))) == false);
	CHECK(vector_all_less_than3(zero, zero) == false);

	CHECK(vector_any_less_than(zero, vector_set(FloatType(1.0), FloatType(1.0), FloatType(1.0), FloatType(1.0))) == true);
	CHECK(vector_any_less_than(zero, vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0), FloatType(0.0))) == true);
	CHECK(vector_any_less_than(zero, vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0), FloatType(0.0))) == true);
	CHECK(vector_any_less_than(zero, vector_set(FloatType(0.0), FloatType(0.0), FloatType(1.0), FloatType(0.0))) == true);
	CHECK(vector_any_less_than(zero, vector_set(FloatType(0.0), FloatType(0.0), FloatType(0.0), FloatType(1.0))) == true);
	CHECK(vector_any_less_than(zero, zero) == false);

	CHECK(vector_any_less_than2(zero, vector_set(FloatType(1.0), FloatType(1.0), FloatType(1.0), FloatType(0.0))) == true);
	CHECK(vector_any_less_than2(zero, vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0), FloatType(0.0))) == true);
	CHECK(vector_any_less_than2(zero, vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0), FloatType(0.0))) == true);
	CHECK(vector_any_less_than2(zero, vector_set(FloatType(0.0), FloatType(0.0), FloatType(1.0), FloatType(0.0))) == false);
	CHECK(vector_any_less_than2(zero, zero) == false);

	CHECK(vector_any_less_than3(zero, vector_set(FloatType(1.0), FloatType(1.0), FloatType(1.0), FloatType(0.0))) == true);
	CHECK(vector_any_less_than3(zero, vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0), FloatType(0.0))) == true);
	CHECK(vector_any_less_than3(zero, vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0), FloatType(0.0))) == true);
	CHECK(vector_any_less_than3(zero, vector_set(FloatType(0.0), FloatType(0.0), FloatType(1.0), FloatType(0.0))) == true);
	CHECK(vector_any_less_than3(zero, zero) == false);

	CHECK(vector_all_less_equal(zero, vector_set(FloatType(1.0), FloatType(1.0), FloatType(1.0), FloatType(1.0))) == true);
	CHECK(vector_all_less_equal(zero, vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0), FloatType(0.0))) == true);
	CHECK(vector_all_less_equal(zero, vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0), FloatType(0.0))) == true);
	CHECK(vector_all_less_equal(zero, vector_set(FloatType(0.0), FloatType(0.0), FloatType(1.0), FloatType(0.0))) == true);
	CHECK(vector_all_less_equal(zero, vector_set(FloatType(0.0), FloatType(0.0), FloatType(0.0), FloatType(1.0))) == true);
	CHECK(vector_all_less_equal(zero, vector_set(FloatType(-1.0), FloatType(0.0), FloatType(0.0), FloatType(0.0))) == false);
	CHECK(vector_all_less_equal(zero, vector_set(FloatType(0.0), FloatType(-1.0), FloatType(0.0), FloatType(0.0))) == false);
	CHECK(vector_all_less_equal(zero, vector_set(FloatType(0.0), FloatType(0.0), FloatType(-1.0), FloatType(0.0))) == false);
	CHECK(vector_all_less_equal(zero, vector_set(FloatType(0.0), FloatType(0.0), FloatType(0.0), FloatType(-1.0))) == false);
	CHECK(vector_all_less_equal(zero, zero) == true);

	CHECK(vector_all_less_equal2(zero, vector_set(FloatType(1.0), FloatType(1.0), FloatType(1.0), FloatType(0.0))) == true);
	CHECK(vector_all_less_equal2(zero, vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0), FloatType(0.0))) == true);
	CHECK(vector_all_less_equal2(zero, vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0), FloatType(0.0))) == true);
	CHECK(vector_all_less_equal2(zero, vector_set(FloatType(0.0), FloatType(0.0), FloatType(1.0), FloatType(0.0))) == true);
	CHECK(vector_all_less_equal2(zero, vector_set(FloatType(-1.0), FloatType(0.0), FloatType(0.0), FloatType(0.0))) == false);
	CHECK(vector_all_less_equal2(zero, vector_set(FloatType(0.0), FloatType(-1.0), FloatType(0.0), FloatType(0.0))) == false);
	CHECK(vector_all_less_equal2(zero, vector_set(FloatType(0.0), FloatType(0.0), FloatType(-1.0), FloatType(0.0))) == true);
	CHECK(vector_all_less_equal2(zero, zero) == true);

	CHECK(vector_all_less_equal3(zero, vector_set(FloatType(1.0), FloatType(1.0), FloatType(1.0), FloatType(0.0))) == true);
	CHECK(vector_all_less_equal3(zero, vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0), FloatType(0.0))) == true);
	CHECK(vector_all_less_equal3(zero, vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0), FloatType(0.0))) == true);
	CHECK(vector_all_less_equal3(zero, vector_set(FloatType(0.0), FloatType(0.0), FloatType(1.0), FloatType(0.0))) == true);
	CHECK(vector_all_less_equal3(zero, vector_set(FloatType(-1.0), FloatType(0.0), FloatType(0.0), FloatType(0.0))) == false);
	CHECK(vector_all_less_equal3(zero, vector_set(FloatType(0.0), FloatType(-1.0), FloatType(0.0), FloatType(0.0))) == false);
	CHECK(vector_all_less_equal3(zero, vector_set(FloatType(0.0), FloatType(0.0), FloatType(-1.0), FloatType(0.0))) == false);
	CHECK(vector_all_less_equal3(zero, zero) == true);

	CHECK(vector_any_less_equal(zero, vector_set(FloatType(1.0), FloatType(1.0), FloatType(1.0), FloatType(1.0))) == true);
	CHECK(vector_any_less_equal(zero, vector_set(FloatType(1.0), FloatType(-1.0), FloatType(-1.0), FloatType(-1.0))) == true);
	CHECK(vector_any_less_equal(zero, vector_set(FloatType(-1.0), FloatType(1.0), FloatType(-1.0), FloatType(-1.0))) == true);
	CHECK(vector_any_less_equal(zero, vector_set(FloatType(-1.0), FloatType(-1.0), FloatType(1.0), FloatType(-1.0))) == true);
	CHECK(vector_any_less_equal(zero, vector_set(FloatType(-1.0), FloatType(-1.0), FloatType(-1.0), FloatType(1.0))) == true);
	CHECK(vector_any_less_equal(zero, vector_set(FloatType(-1.0), FloatType(-1.0), FloatType(-1.0), FloatType(-1.0))) == false);
	CHECK(vector_any_less_equal(zero, zero) == true);

	CHECK(vector_any_less_equal2(zero, vector_set(FloatType(1.0), FloatType(1.0), FloatType(1.0), FloatType(0.0))) == true);
	CHECK(vector_any_less_equal2(zero, vector_set(FloatType(1.0), FloatType(-1.0), FloatType(-1.0), FloatType(0.0))) == true);
	CHECK(vector_any_less_equal2(zero, vector_set(FloatType(-1.0), FloatType(1.0), FloatType(-1.0), FloatType(0.0))) == true);
	CHECK(vector_any_less_equal2(zero, vector_set(FloatType(-1.0), FloatType(-1.0), FloatType(1.0), FloatType(0.0))) == false);
	CHECK(vector_any_less_equal2(zero, vector_set(FloatType(-1.0), FloatType(-1.0), FloatType(-1.0), FloatType(0.0))) == false);
	CHECK(vector_any_less_equal2(zero, zero) == true);

	CHECK(vector_any_less_equal3(zero, vector_set(FloatType(1.0), FloatType(1.0), FloatType(1.0), FloatType(0.0))) == true);
	CHECK(vector_any_less_equal3(zero, vector_set(FloatType(1.0), FloatType(-1.0), FloatType(-1.0), FloatType(0.0))) == true);
	CHECK(vector_any_less_equal3(zero, vector_set(FloatType(-1.0), FloatType(1.0), FloatType(-1.0), FloatType(0.0))) == true);
	CHECK(vector_any_less_equal3(zero, vector_set(FloatType(-1.0), FloatType(-1.0), FloatType(1.0), FloatType(0.0))) == true);
	CHECK(vector_any_less_equal3(zero, vector_set(FloatType(-1.0), FloatType(-1.0), FloatType(-1.0), FloatType(0.0))) == false);
	CHECK(vector_any_less_equal3(zero, zero) == true);

	CHECK(vector_all_greater_equal(vector_set(FloatType(1.0), FloatType(1.0), FloatType(1.0), FloatType(1.0)), zero) == true);
	CHECK(vector_all_greater_equal(vector_set(FloatType(1.0), FloatType(-1.0), FloatType(-1.0), FloatType(-1.0)), zero) == false);
	CHECK(vector_all_greater_equal(vector_set(FloatType(-1.0), FloatType(1.0), FloatType(-1.0), FloatType(-1.0)), zero) == false);
	CHECK(vector_all_greater_equal(vector_set(FloatType(-1.0), FloatType(-1.0), FloatType(1.0), FloatType(-1.0)), zero) == false);
	CHECK(vector_all_greater_equal(vector_set(FloatType(-1.0), FloatType(-1.0), FloatType(-1.0), FloatType(1.0)), zero) == false);
	CHECK(vector_all_greater_equal(vector_set(FloatType(0.0), FloatType(-1.0), FloatType(-1.0), FloatType(-1.0)), zero) == false);
	CHECK(vector_all_greater_equal(vector_set(FloatType(-1.0), FloatType(0.0), FloatType(-1.0), FloatType(-1.0)), zero) == false);
	CHECK(vector_all_greater_equal(vector_set(FloatType(-1.0), FloatType(-1.0), FloatType(0.0), FloatType(-1.0)), zero) == false);
	CHECK(vector_all_greater_equal(vector_set(FloatType(-1.0), FloatType(-1.0), FloatType(-1.0), FloatType(0.0)), zero) == false);
	CHECK(vector_all_greater_equal(vector_set(FloatType(-1.0), FloatType(-1.0), FloatType(-1.0), FloatType(-1.0)), zero) == false);
	CHECK(vector_all_greater_equal(zero, zero) == true);

	CHECK(vector_all_greater_equal2(vector_set(FloatType(1.0), FloatType(1.0), FloatType(1.0), FloatType(0.0)), zero) == true);
	CHECK(vector_all_greater_equal2(vector_set(FloatType(1.0), FloatType(-1.0), FloatType(-1.0), FloatType(0.0)), zero) == false);
	CHECK(vector_all_greater_equal2(vector_set(FloatType(-1.0), FloatType(1.0), FloatType(-1.0), FloatType(0.0)), zero) == false);
	CHECK(vector_all_greater_equal2(vector_set(FloatType(-1.0), FloatType(-1.0), FloatType(1.0), FloatType(0.0)), zero) == false);
	CHECK(vector_all_greater_equal2(vector_set(FloatType(0.0), FloatType(-1.0), FloatType(-1.0), FloatType(0.0)), zero) == false);
	CHECK(vector_all_greater_equal2(vector_set(FloatType(-1.0), FloatType(0.0), FloatType(-1.0), FloatType(0.0)), zero) == false);
	CHECK(vector_all_greater_equal2(vector_set(FloatType(-1.0), FloatType(-1.0), FloatType(0.0), FloatType(0.0)), zero) == false);
	CHECK(vector_all_greater_equal2(vector_set(FloatType(-1.0), FloatType(-1.0), FloatType(-1.0), FloatType(0.0)), zero) == false);
	CHECK(vector_all_greater_equal2(zero, zero) == true);

	CHECK(vector_all_greater_equal3(vector_set(FloatType(1.0), FloatType(1.0), FloatType(1.0), FloatType(0.0)), zero) == true);
	CHECK(vector_all_greater_equal3(vector_set(FloatType(1.0), FloatType(-1.0), FloatType(-1.0), FloatType(0.0)), zero) == false);
	CHECK(vector_all_greater_equal3(vector_set(FloatType(-1.0), FloatType(1.0), FloatType(-1.0), FloatType(0.0)), zero) == false);
	CHECK(vector_all_greater_equal3(vector_set(FloatType(-1.0), FloatType(-1.0), FloatType(1.0), FloatType(0.0)), zero) == false);
	CHECK(vector_all_greater_equal3(vector_set(FloatType(0.0), FloatType(-1.0), FloatType(-1.0), FloatType(0.0)), zero) == false);
	CHECK(vector_all_greater_equal3(vector_set(FloatType(-1.0), FloatType(0.0), FloatType(-1.0), FloatType(0.0)), zero) == false);
	CHECK(vector_all_greater_equal3(vector_set(FloatType(-1.0), FloatType(-1.0), FloatType(0.0), FloatType(0.0)), zero) == false);
	CHECK(vector_all_greater_equal3(vector_set(FloatType(-1.0), FloatType(-1.0), FloatType(-1.0), FloatType(0.0)), zero) == false);
	CHECK(vector_all_greater_equal3(zero, zero) == true);

	CHECK(vector_any_greater_equal(vector_set(FloatType(1.0), FloatType(1.0), FloatType(1.0), FloatType(1.0)), zero) == true);
	CHECK(vector_any_greater_equal(vector_set(FloatType(1.0), FloatType(-1.0), FloatType(-1.0), FloatType(-1.0)), zero) == true);
	CHECK(vector_any_greater_equal(vector_set(FloatType(-1.0), FloatType(1.0), FloatType(-1.0), FloatType(-1.0)), zero) == true);
	CHECK(vector_any_greater_equal(vector_set(FloatType(-1.0), FloatType(-1.0), FloatType(1.0), FloatType(-1.0)), zero) == true);
	CHECK(vector_any_greater_equal(vector_set(FloatType(-1.0), FloatType(-1.0), FloatType(-1.0), FloatType(1.0)), zero) == true);
	CHECK(vector_any_greater_equal(vector_set(FloatType(0.0), FloatType(-1.0), FloatType(-1.0), FloatType(-1.0)), zero) == true);
	CHECK(vector_any_greater_equal(vector_set(FloatType(-1.0), FloatType(0.0), FloatType(-1.0), FloatType(-1.0)), zero) == true);
	CHECK(vector_any_greater_equal(vector_set(FloatType(-1.0), FloatType(-1.0), FloatType(0.0), FloatType(-1.0)), zero) == true);
	CHECK(vector_any_greater_equal(vector_set(FloatType(-1.0), FloatType(-1.0), FloatType(-1.0), FloatType(0.0)), zero) == true);
	CHECK(vector_any_greater_equal(vector_set(FloatType(-1.0), FloatType(-1.0), FloatType(-1.0), FloatType(-1.0)), zero) == false);
	CHECK(vector_any_greater_equal(zero, zero) == true);

	CHECK(vector_any_greater_equal2(vector_set(FloatType(1.0), FloatType(1.0), FloatType(1.0), FloatType(0.0)), zero) == true);
	CHECK(vector_any_greater_equal2(vector_set(FloatType(1.0), FloatType(-1.0), FloatType(-1.0), FloatType(0.0)), zero) == true);
	CHECK(vector_any_greater_equal2(vector_set(FloatType(-1.0), FloatType(1.0), FloatType(-1.0), FloatType(0.0)), zero) == true);
	CHECK(vector_any_greater_equal2(vector_set(FloatType(-1.0), FloatType(-1.0), FloatType(1.0), FloatType(0.0)), zero) == false);
	CHECK(vector_any_greater_equal2(vector_set(FloatType(0.0), FloatType(-1.0), FloatType(-1.0), FloatType(0.0)), zero) == true);
	CHECK(vector_any_greater_equal2(vector_set(FloatType(-1.0), FloatType(0.0), FloatType(-1.0), FloatType(0.0)), zero) == true);
	CHECK(vector_any_greater_equal2(vector_set(FloatType(-1.0), FloatType(-1.0), FloatType(0.0), FloatType(0.0)), zero) == false);
	CHECK(vector_any_greater_equal2(vector_set(FloatType(-1.0), FloatType(-1.0), FloatType(-1.0), FloatType(0.0)), zero) == false);
	CHECK(vector_any_greater_equal2(zero, zero) == true);

	CHECK(vector_any_greater_equal3(vector_set(FloatType(1.0), FloatType(1.0), FloatType(1.0), FloatType(0.0)), zero) == true);
	CHECK(vector_any_greater_equal3(vector_set(FloatType(1.0), FloatType(-1.0), FloatType(-1.0), FloatType(0.0)), zero) == true);
	CHECK(vector_any_greater_equal3(vector_set(FloatType(-1.0), FloatType(1.0), FloatType(-1.0), FloatType(0.0)), zero) == true);
	CHECK(vector_any_greater_equal3(vector_set(FloatType(-1.0), FloatType(-1.0), FloatType(1.0), FloatType(0.0)), zero) == true);
	CHECK(vector_any_greater_equal3(vector_set(FloatType(0.0), FloatType(-1.0), FloatType(-1.0), FloatType(0.0)), zero) == true);
	CHECK(vector_any_greater_equal3(vector_set(FloatType(-1.0), FloatType(0.0), FloatType(-1.0), FloatType(0.0)), zero) == true);
	CHECK(vector_any_greater_equal3(vector_set(FloatType(-1.0), FloatType(-1.0), FloatType(0.0), FloatType(0.0)), zero) == true);
	CHECK(vector_any_greater_equal3(vector_set(FloatType(-1.0), FloatType(-1.0), FloatType(-1.0), FloatType(0.0)), zero) == false);
	CHECK(vector_any_greater_equal3(zero, zero) == true);

	CHECK(vector_all_near_equal(zero, zero, threshold) == true);
	CHECK(vector_all_near_equal(zero, vector_set(FloatType(1.0), FloatType(1.0), FloatType(1.0), FloatType(1.0)), FloatType(1.0001)) == true);
	CHECK(vector_all_near_equal(zero, vector_set(FloatType(1.0), FloatType(1.0), FloatType(1.0), FloatType(1.0)), FloatType(1.0)) == true);
	CHECK(vector_all_near_equal(zero, vector_set(FloatType(1.0), FloatType(1.0), FloatType(1.0), FloatType(1.0)), FloatType(0.9999)) == false);

	CHECK(vector_all_near_equal2(zero, zero, threshold) == true);
	CHECK(vector_all_near_equal2(zero, vector_set(FloatType(1.0), FloatType(1.0), FloatType(1.0), FloatType(2.0)), FloatType(1.0001)) == true);
	CHECK(vector_all_near_equal2(zero, vector_set(FloatType(1.0), FloatType(1.0), FloatType(1.0), FloatType(2.0)), FloatType(1.0)) == true);
	CHECK(vector_all_near_equal2(zero, vector_set(FloatType(1.0), FloatType(1.0), FloatType(1.0), FloatType(2.0)), FloatType(0.9999)) == false);

	CHECK(vector_all_near_equal3(zero, zero, threshold) == true);
	CHECK(vector_all_near_equal3(zero, vector_set(FloatType(1.0), FloatType(1.0), FloatType(1.0), FloatType(2.0)), FloatType(1.0001)) == true);
	CHECK(vector_all_near_equal3(zero, vector_set(FloatType(1.0), FloatType(1.0), FloatType(1.0), FloatType(2.0)), FloatType(1.0)) == true);
	CHECK(vector_all_near_equal3(zero, vector_set(FloatType(1.0), FloatType(1.0), FloatType(1.0), FloatType(2.0)), FloatType(0.9999)) == false);

	CHECK(vector_any_near_equal(zero, zero, threshold) == true);
	CHECK(vector_any_near_equal(zero, vector_set(FloatType(1.0), FloatType(2.0), FloatType(2.0), FloatType(2.0)), FloatType(1.0001)) == true);
	CHECK(vector_any_near_equal(zero, vector_set(FloatType(2.0), FloatType(1.0), FloatType(2.0), FloatType(2.0)), FloatType(1.0001)) == true);
	CHECK(vector_any_near_equal(zero, vector_set(FloatType(2.0), FloatType(2.0), FloatType(1.0), FloatType(2.0)), FloatType(1.0001)) == true);
	CHECK(vector_any_near_equal(zero, vector_set(FloatType(2.0), FloatType(2.0), FloatType(2.0), FloatType(1.0)), FloatType(1.0001)) == true);
	CHECK(vector_any_near_equal(zero, vector_set(FloatType(1.0), FloatType(2.0), FloatType(2.0), FloatType(2.0)), FloatType(1.0)) == true);
	CHECK(vector_any_near_equal(zero, vector_set(FloatType(2.0), FloatType(1.0), FloatType(2.0), FloatType(2.0)), FloatType(1.0)) == true);
	CHECK(vector_any_near_equal(zero, vector_set(FloatType(2.0), FloatType(2.0), FloatType(1.0), FloatType(2.0)), FloatType(1.0)) == true);
	CHECK(vector_any_near_equal(zero, vector_set(FloatType(2.0), FloatType(2.0), FloatType(2.0), FloatType(1.0)), FloatType(1.0)) == true);
	CHECK(vector_any_near_equal(zero, vector_set(FloatType(1.0), FloatType(1.0), FloatType(1.0), FloatType(1.0)), FloatType(0.9999)) == false);

	CHECK(vector_any_near_equal2(zero, zero, threshold) == true);
	CHECK(vector_any_near_equal2(zero, vector_set(FloatType(1.0), FloatType(2.0), FloatType(2.0), FloatType(2.0)), FloatType(1.0001)) == true);
	CHECK(vector_any_near_equal2(zero, vector_set(FloatType(2.0), FloatType(1.0), FloatType(2.0), FloatType(2.0)), FloatType(1.0001)) == true);
	CHECK(vector_any_near_equal2(zero, vector_set(FloatType(2.0), FloatType(2.0), FloatType(1.0), FloatType(2.0)), FloatType(1.0001)) == false);
	CHECK(vector_any_near_equal2(zero, vector_set(FloatType(1.0), FloatType(2.0), FloatType(2.0), FloatType(2.0)), FloatType(1.0)) == true);
	CHECK(vector_any_near_equal2(zero, vector_set(FloatType(2.0), FloatType(1.0), FloatType(2.0), FloatType(2.0)), FloatType(1.0)) == true);
	CHECK(vector_any_near_equal2(zero, vector_set(FloatType(2.0), FloatType(2.0), FloatType(1.0), FloatType(2.0)), FloatType(1.0)) == false);
	CHECK(vector_any_near_equal2(zero, vector_set(FloatType(1.0), FloatType(1.0), FloatType(1.0), FloatType(2.0)), FloatType(0.9999)) == false);

	CHECK(vector_any_near_equal3(zero, zero, threshold) == true);
	CHECK(vector_any_near_equal3(zero, vector_set(FloatType(1.0), FloatType(2.0), FloatType(2.0), FloatType(2.0)), FloatType(1.0001)) == true);
	CHECK(vector_any_near_equal3(zero, vector_set(FloatType(2.0), FloatType(1.0), FloatType(2.0), FloatType(2.0)), FloatType(1.0001)) == true);
	CHECK(vector_any_near_equal3(zero, vector_set(FloatType(2.0), FloatType(2.0), FloatType(1.0), FloatType(2.0)), FloatType(1.0001)) == true);
	CHECK(vector_any_near_equal3(zero, vector_set(FloatType(1.0), FloatType(2.0), FloatType(2.0), FloatType(2.0)), FloatType(1.0)) == true);
	CHECK(vector_any_near_equal3(zero, vector_set(FloatType(2.0), FloatType(1.0), FloatType(2.0), FloatType(2.0)), FloatType(1.0)) == true);
	CHECK(vector_any_near_equal3(zero, vector_set(FloatType(2.0), FloatType(2.0), FloatType(1.0), FloatType(2.0)), FloatType(1.0)) == true);
	CHECK(vector_any_near_equal3(zero, vector_set(FloatType(1.0), FloatType(1.0), FloatType(1.0), FloatType(2.0)), FloatType(0.9999)) == false);
}

template<typename FloatType>
void test_vector4_impl(const FloatType threshold)
{
	using Vector4Type = typename float_traits<FloatType>::vector4;

	const Vector4Type zero = vector_zero();

	const FloatType test_value0_flt[4] = { FloatType(2.0), FloatType(9.34), FloatType(-54.12), FloatType(6000.0) };
	const FloatType test_value1_flt[4] = { FloatType(0.75), FloatType(-4.52), FloatType(44.68), FloatType(-54225.0) };
	const Vector4Type test_value0 = vector_set(test_value0_flt[0], test_value0_flt[1], test_value0_flt[2], test_value0_flt[3]);
	const Vector4Type test_value1 = vector_set(test_value1_flt[0], test_value1_flt[1], test_value1_flt[2], test_value1_flt[3]);

	const FloatType inf = std::numeric_limits<FloatType>::infinity();
	const FloatType nan = std::numeric_limits<FloatType>::quiet_NaN();
	CHECK(vector_is_finite(zero) == true);
	CHECK(vector_is_finite(vector_set(inf, inf, inf, inf)) == false);
	CHECK(vector_is_finite(vector_set(inf, FloatType(1.0), FloatType(1.0), FloatType(1.0))) == false);
	CHECK(vector_is_finite(vector_set(FloatType(1.0), FloatType(inf), FloatType(1.0), FloatType(1.0))) == false);
	CHECK(vector_is_finite(vector_set(FloatType(1.0), FloatType(1.0), FloatType(inf), FloatType(1.0))) == false);
	CHECK(vector_is_finite(vector_set(FloatType(1.0), FloatType(1.0), FloatType(1.0), FloatType(inf))) == false);
	CHECK(vector_is_finite(vector_set(nan, nan, nan, nan)) == false);
	CHECK(vector_is_finite(vector_set(nan, FloatType(1.0), FloatType(1.0), FloatType(1.0))) == false);
	CHECK(vector_is_finite(vector_set(FloatType(1.0), FloatType(nan), FloatType(1.0), FloatType(1.0))) == false);
	CHECK(vector_is_finite(vector_set(FloatType(1.0), FloatType(1.0), FloatType(nan), FloatType(1.0))) == false);
	CHECK(vector_is_finite(vector_set(FloatType(1.0), FloatType(1.0), FloatType(1.0), FloatType(nan))) == false);

	CHECK(vector_is_finite2(zero) == true);
	CHECK(vector_is_finite2(vector_set(inf, inf, inf, inf)) == false);
	CHECK(vector_is_finite2(vector_set(inf, FloatType(1.0), FloatType(1.0), FloatType(1.0))) == false);
	CHECK(vector_is_finite2(vector_set(FloatType(1.0), FloatType(inf), FloatType(1.0), FloatType(1.0))) == false);
	CHECK(vector_is_finite2(vector_set(FloatType(1.0), FloatType(1.0), FloatType(inf), FloatType(1.0))) == true);
	CHECK(vector_is_finite2(vector_set(FloatType(1.0), FloatType(1.0), FloatType(1.0), FloatType(inf))) == true);
	CHECK(vector_is_finite2(vector_set(nan, nan, nan, nan)) == false);
	CHECK(vector_is_finite2(vector_set(nan, FloatType(1.0), FloatType(1.0), FloatType(1.0))) == false);
	CHECK(vector_is_finite2(vector_set(FloatType(1.0), FloatType(nan), FloatType(1.0), FloatType(1.0))) == false);
	CHECK(vector_is_finite2(vector_set(FloatType(1.0), FloatType(1.0), FloatType(nan), FloatType(1.0))) == true);
	CHECK(vector_is_finite2(vector_set(FloatType(1.0), FloatType(1.0), FloatType(1.0), FloatType(nan))) == true);

	CHECK(vector_is_finite3(zero) == true);
	CHECK(vector_is_finite3(vector_set(inf, inf, inf, inf)) == false);
	CHECK(vector_is_finite3(vector_set(inf, FloatType(1.0), FloatType(1.0), FloatType(1.0))) == false);
	CHECK(vector_is_finite3(vector_set(FloatType(1.0), FloatType(inf), FloatType(1.0), FloatType(1.0))) == false);
	CHECK(vector_is_finite3(vector_set(FloatType(1.0), FloatType(1.0), FloatType(inf), FloatType(1.0))) == false);
	CHECK(vector_is_finite3(vector_set(FloatType(1.0), FloatType(1.0), FloatType(1.0), FloatType(inf))) == true);
	CHECK(vector_is_finite3(vector_set(nan, nan, nan, nan)) == false);
	CHECK(vector_is_finite3(vector_set(nan, FloatType(1.0), FloatType(1.0), FloatType(1.0))) == false);
	CHECK(vector_is_finite3(vector_set(FloatType(1.0), FloatType(nan), FloatType(1.0), FloatType(1.0))) == false);
	CHECK(vector_is_finite3(vector_set(FloatType(1.0), FloatType(1.0), FloatType(nan), FloatType(1.0))) == false);
	CHECK(vector_is_finite3(vector_set(FloatType(1.0), FloatType(1.0), FloatType(1.0), FloatType(nan))) == true);

	//////////////////////////////////////////////////////////////////////////
	// Swizzling, permutations, and mixing

	CHECK(scalar_near_equal(vector_get_x(vector_select(vector_less_than(zero, vector_set(FloatType(1.0))), test_value0, test_value1)), test_value0_flt[0], threshold));
	CHECK(scalar_near_equal(vector_get_y(vector_select(vector_less_than(zero, vector_set(FloatType(1.0))), test_value0, test_value1)), test_value0_flt[1], threshold));
	CHECK(scalar_near_equal(vector_get_z(vector_select(vector_less_than(zero, vector_set(FloatType(1.0))), test_value0, test_value1)), test_value0_flt[2], threshold));
	CHECK(scalar_near_equal(vector_get_w(vector_select(vector_less_than(zero, vector_set(FloatType(1.0))), test_value0, test_value1)), test_value0_flt[3], threshold));
	CHECK(scalar_near_equal(vector_get_x(vector_select(vector_less_than(vector_set(FloatType(1.0)), zero), test_value0, test_value1)), test_value1_flt[0], threshold));
	CHECK(scalar_near_equal(vector_get_y(vector_select(vector_less_than(vector_set(FloatType(1.0)), zero), test_value0, test_value1)), test_value1_flt[1], threshold));
	CHECK(scalar_near_equal(vector_get_z(vector_select(vector_less_than(vector_set(FloatType(1.0)), zero), test_value0, test_value1)), test_value1_flt[2], threshold));
	CHECK(scalar_near_equal(vector_get_w(vector_select(vector_less_than(vector_set(FloatType(1.0)), zero), test_value0, test_value1)), test_value1_flt[3], threshold));

	//////////////////////////////////////////////////////////////////////////
	// Misc

	auto scalar_sign = [](FloatType value) { return value >= FloatType(0.0) ? FloatType(1.0) : FloatType(-1.0); };
	CHECK(scalar_cast(vector_get_x(vector_sign(test_value0))) == scalar_sign(test_value0_flt[0]));
	CHECK(scalar_cast(vector_get_y(vector_sign(test_value0))) == scalar_sign(test_value0_flt[1]));
	CHECK(scalar_cast(vector_get_z(vector_sign(test_value0))) == scalar_sign(test_value0_flt[2]));
	CHECK(scalar_cast(vector_get_w(vector_sign(test_value0))) == scalar_sign(test_value0_flt[3]));

	{
		const Vector4Type input0 = vector_set(FloatType(-1.75), FloatType(-1.5), FloatType(-1.4999), FloatType(-0.5));
		const Vector4Type input1 = vector_set(FloatType(-0.4999), FloatType(0.0), FloatType(0.4999), FloatType(0.5));
		const Vector4Type input2 = vector_set(FloatType(1.4999), FloatType(1.5), FloatType(1.75), FloatType(0.0));

		const Vector4Type result0 = vector_symmetric_round(input0);
		const Vector4Type result1 = vector_symmetric_round(input1);
		const Vector4Type result2 = vector_symmetric_round(input2);

		CHECK(scalar_cast(vector_get_x(result0)) == scalar_symmetric_round(vector_get_x(input0)));
		CHECK(scalar_cast(vector_get_y(result0)) == scalar_symmetric_round(vector_get_y(input0)));
		CHECK(scalar_cast(vector_get_z(result0)) == scalar_symmetric_round(vector_get_z(input0)));
		CHECK(scalar_cast(vector_get_w(result0)) == scalar_symmetric_round(vector_get_w(input0)));
		CHECK(scalar_cast(vector_get_x(result1)) == scalar_symmetric_round(vector_get_x(input1)));
		CHECK(scalar_cast(vector_get_y(result1)) == scalar_symmetric_round(vector_get_y(input1)));
		CHECK(scalar_cast(vector_get_z(result1)) == scalar_symmetric_round(vector_get_z(input1)));
		CHECK(scalar_cast(vector_get_w(result1)) == scalar_symmetric_round(vector_get_w(input1)));
		CHECK(scalar_cast(vector_get_x(result2)) == scalar_symmetric_round(vector_get_x(input2)));
		CHECK(scalar_cast(vector_get_y(result2)) == scalar_symmetric_round(vector_get_y(input2)));
		CHECK(scalar_cast(vector_get_z(result2)) == scalar_symmetric_round(vector_get_z(input2)));
		CHECK(scalar_cast(vector_get_w(result2)) == scalar_symmetric_round(vector_get_w(input2)));
	}
}
