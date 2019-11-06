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

#include <rtm/qvvf.h>
#include <rtm/qvvd.h>

using namespace rtm;

template<typename TransformType, typename FloatType>
static void test_qvv_impl(const TransformType& identity, const FloatType threshold)
{
	using QuatType = decltype(TransformType::rotation);
	using Vector4Type = decltype(TransformType::translation);

	{
		Vector4Type zero = vector_set(FloatType(0.0));
		Vector4Type one = vector_set(FloatType(1.0));
		QuatType q_identity = quat_set(FloatType(0.0), FloatType(0.0), FloatType(0.0), FloatType(1.0));
		TransformType tmp = qvv_set(q_identity, zero, one);
		REQUIRE(quat_near_equal(identity.rotation, tmp.rotation, threshold));
		REQUIRE(vector_all_near_equal3(identity.translation, tmp.translation, threshold));
		REQUIRE(vector_all_near_equal3(identity.scale, tmp.scale, threshold));
		REQUIRE(quat_near_equal(q_identity, tmp.rotation, threshold));
		REQUIRE(vector_all_near_equal3(zero, tmp.translation, threshold));
		REQUIRE(vector_all_near_equal3(one, tmp.scale, threshold));
	}

	{
		Vector4Type x_axis = vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0));
		Vector4Type y_axis = vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0));
		Vector4Type test_scale = vector_set(FloatType(1.2));

		QuatType rotation_around_z = quat_from_euler(degrees(FloatType(0.0)), degrees(FloatType(90.0)), degrees(FloatType(0.0)));
		TransformType transform_a = qvv_set(rotation_around_z, x_axis, test_scale);
		Vector4Type result = qvv_mul_point3(x_axis, transform_a);
		REQUIRE(vector_all_near_equal3(result, vector_set(FloatType(1.0), FloatType(1.2), FloatType(0.0)), threshold));
		result = qvv_mul_point3(y_axis, transform_a);
		REQUIRE(vector_all_near_equal3(result, vector_set(FloatType(-0.2), FloatType(0.0), FloatType(0.0)), threshold));

		QuatType rotation_around_x = quat_from_euler(degrees(FloatType(0.0)), degrees(FloatType(0.0)), degrees(FloatType(90.0)));
		TransformType transform_b = qvv_set(rotation_around_x, y_axis, test_scale);
		result = qvv_mul_point3(x_axis, transform_b);
		REQUIRE(vector_all_near_equal3(result, vector_set(FloatType(1.2), FloatType(1.0), FloatType(0.0)), threshold));
		result = qvv_mul_point3(y_axis, transform_b);
		REQUIRE(vector_all_near_equal3(result, vector_set(FloatType(0.0), FloatType(1.0), FloatType(-1.2)), threshold));

		TransformType transform_ab = qvv_mul(transform_a, transform_b);
		TransformType transform_ba = qvv_mul(transform_b, transform_a);
		result = qvv_mul_point3(x_axis, transform_ab);
		REQUIRE(vector_all_near_equal3(result, vector_set(FloatType(1.2), FloatType(1.0), FloatType(-1.44)), threshold));
		REQUIRE(vector_all_near_equal3(result, qvv_mul_point3(qvv_mul_point3(x_axis, transform_a), transform_b), threshold));
		result = qvv_mul_point3(y_axis, transform_ab);
		REQUIRE(vector_all_near_equal3(result, vector_set(FloatType(-0.24), FloatType(1.0), FloatType(0.0)), threshold));
		REQUIRE(vector_all_near_equal3(result, qvv_mul_point3(qvv_mul_point3(y_axis, transform_a), transform_b), threshold));
		result = qvv_mul_point3(x_axis, transform_ba);
		REQUIRE(vector_all_near_equal3(result, vector_set(FloatType(-0.2), FloatType(1.44), FloatType(0.0)), threshold));
		REQUIRE(vector_all_near_equal3(result, qvv_mul_point3(qvv_mul_point3(x_axis, transform_b), transform_a), threshold));
		result = qvv_mul_point3(y_axis, transform_ba);
		REQUIRE(vector_all_near_equal3(result, vector_set(FloatType(-0.2), FloatType(0.0), FloatType(-1.44)), threshold));
		REQUIRE(vector_all_near_equal3(result, qvv_mul_point3(qvv_mul_point3(y_axis, transform_b), transform_a), threshold));
	}

	{
		Vector4Type x_axis = vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0));
		Vector4Type y_axis = vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0));

		QuatType rotation_around_z = quat_from_euler(degrees(FloatType(0.0)), degrees(FloatType(90.0)), degrees(FloatType(0.0)));
		TransformType transform_a = qvv_set(rotation_around_z, x_axis, vector_set(FloatType(1.0)));
		Vector4Type result = qvv_mul_point3_no_scale(x_axis, transform_a);
		REQUIRE(vector_all_near_equal3(result, vector_set(FloatType(1.0), FloatType(1.0), FloatType(0.0)), threshold));
		result = qvv_mul_point3_no_scale(y_axis, transform_a);
		REQUIRE(vector_all_near_equal3(result, vector_set(FloatType(0.0), FloatType(0.0), FloatType(0.0)), threshold));

		QuatType rotation_around_x = quat_from_euler(degrees(FloatType(0.0)), degrees(FloatType(0.0)), degrees(FloatType(90.0)));
		TransformType transform_b = qvv_set(rotation_around_x, y_axis, vector_set(FloatType(1.0)));
		result = qvv_mul_point3_no_scale(x_axis, transform_b);
		REQUIRE(vector_all_near_equal3(result, vector_set(FloatType(1.0), FloatType(1.0), FloatType(0.0)), threshold));
		result = qvv_mul_point3_no_scale(y_axis, transform_b);
		REQUIRE(vector_all_near_equal3(result, vector_set(FloatType(0.0), FloatType(1.0), FloatType(-1.0)), threshold));

		TransformType transform_ab = qvv_mul_no_scale(transform_a, transform_b);
		TransformType transform_ba = qvv_mul_no_scale(transform_b, transform_a);
		result = qvv_mul_point3_no_scale(x_axis, transform_ab);
		REQUIRE(vector_all_near_equal3(result, vector_set(FloatType(1.0), FloatType(1.0), FloatType(-1.0)), threshold));
		REQUIRE(vector_all_near_equal3(result, qvv_mul_point3_no_scale(qvv_mul_point3_no_scale(x_axis, transform_a), transform_b), threshold));
		result = qvv_mul_point3_no_scale(y_axis, transform_ab);
		REQUIRE(vector_all_near_equal3(result, vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0)), threshold));
		REQUIRE(vector_all_near_equal3(result, qvv_mul_point3_no_scale(qvv_mul_point3_no_scale(y_axis, transform_a), transform_b), threshold));
		result = qvv_mul_point3_no_scale(x_axis, transform_ba);
		REQUIRE(vector_all_near_equal3(result, vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0)), threshold));
		REQUIRE(vector_all_near_equal3(result, qvv_mul_point3_no_scale(qvv_mul_point3_no_scale(x_axis, transform_b), transform_a), threshold));
		result = qvv_mul_point3_no_scale(y_axis, transform_ba);
		REQUIRE(vector_all_near_equal3(result, vector_set(FloatType(0.0), FloatType(0.0), FloatType(-1.0)), threshold));
		REQUIRE(vector_all_near_equal3(result, qvv_mul_point3_no_scale(qvv_mul_point3_no_scale(y_axis, transform_b), transform_a), threshold));
	}

	{
		Vector4Type x_axis = vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0));
		Vector4Type test_scale1 = vector_set(FloatType(1.2));
		Vector4Type test_scale2 = vector_set(FloatType(-1.2));

		QuatType rotation_around_z = quat_from_euler(degrees(FloatType(0.0)), degrees(FloatType(90.0)), degrees(FloatType(0.0)));
		TransformType transform_a = qvv_set(rotation_around_z, x_axis, test_scale1);
		TransformType transform_b = qvv_inverse(transform_a);
		TransformType transform_ab = qvv_mul(transform_a, transform_b);
		REQUIRE(quat_near_equal(identity.rotation, transform_ab.rotation, threshold));
		REQUIRE(vector_all_near_equal3(identity.translation, transform_ab.translation, threshold));
		REQUIRE(vector_all_near_equal3(identity.scale, transform_ab.scale, threshold));

		transform_a = qvv_set(rotation_around_z, x_axis, test_scale2);
		transform_b = qvv_inverse(transform_a);
		transform_ab = qvv_mul(transform_a, transform_b);
		REQUIRE(quat_near_equal(identity.rotation, transform_ab.rotation, threshold));
		REQUIRE(vector_all_near_equal3(identity.translation, transform_ab.translation, threshold));
		REQUIRE(vector_all_near_equal3(identity.scale, transform_ab.scale, threshold));
	}

	{
		Vector4Type x_axis = vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0));

		QuatType rotation_around_z = quat_from_euler(degrees(FloatType(0.0)), degrees(FloatType(90.0)), degrees(FloatType(0.0)));
		TransformType transform_a = qvv_set(rotation_around_z, x_axis, vector_set(FloatType(1.0)));
		TransformType transform_b = qvv_inverse_no_scale(transform_a);
		TransformType transform_ab = qvv_mul_no_scale(transform_a, transform_b);
		REQUIRE(quat_near_equal(identity.rotation, transform_ab.rotation, threshold));
		REQUIRE(vector_all_near_equal3(identity.translation, transform_ab.translation, threshold));
		REQUIRE(vector_all_near_equal3(identity.scale, transform_ab.scale, threshold));
	}

	{
		Vector4Type x_axis = vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0));
		QuatType rotation_around_z = quat_from_euler(degrees(FloatType(0.0)), degrees(FloatType(90.0)), degrees(FloatType(0.0)));
		TransformType transform_a = qvv_set(rotation_around_z, x_axis, vector_set(FloatType(1.0)));
		REQUIRE(quat_is_normalized(qvv_normalize(transform_a).rotation, threshold));

		QuatType quat = quat_set(FloatType(-0.001138), FloatType(0.91623), FloatType(-1.624598), FloatType(0.715671));
		TransformType transform_b = qvv_set(quat, x_axis, vector_set(FloatType(1.0)));
		REQUIRE(!quat_is_normalized(transform_b.rotation, threshold));
		REQUIRE(quat_is_normalized(qvv_normalize(transform_b).rotation, threshold));
	}
}

TEST_CASE("qvvf math", "[math][qvv]")
{
	test_qvv_impl<qvvf, float>(qvv_identity(), 1.0E-4F);

	const quatf src_rotation = quat_set(0.39564531008956383F, 0.044254239301713752F, 0.22768840967675355F, 0.88863059760894492F);
	const vector4f src_translation = vector_set(-2.65F, 2.996113F, 0.68123521F);
	const vector4f src_scale = vector_set(1.2F, 0.8F, 2.1F);
	const qvvf src = qvv_set(src_rotation, src_translation, src_scale);
	const qvvd dst = qvv_cast(src);
	REQUIRE(quat_near_equal(src.rotation, quat_cast(dst.rotation), 1.0E-6F));
	REQUIRE(vector_all_near_equal3(src.translation, vector_cast(dst.translation), 1.0E-6F));
	REQUIRE(vector_all_near_equal3(src.scale, vector_cast(dst.scale), 1.0E-6F));
}

TEST_CASE("qvvd math", "[math][qvv]")
{
	test_qvv_impl<qvvd, double>(qvv_identity(), 1.0E-6);

	const quatd src_rotation = quat_set(0.39564531008956383, 0.044254239301713752, 0.22768840967675355, 0.88863059760894492);
	const vector4d src_translation = vector_set(-2.65, 2.996113, 0.68123521);
	const vector4d src_scale = vector_set(1.2, 0.8, 2.1);
	const qvvd src = qvv_set(src_rotation, src_translation, src_scale);
	const qvvf dst = qvv_cast(src);
	REQUIRE(quat_near_equal(src.rotation, quat_cast(dst.rotation), 1.0E-6));
	REQUIRE(vector_all_near_equal3(src.translation, vector_cast(dst.translation), 1.0E-6));
	REQUIRE(vector_all_near_equal3(src.scale, vector_cast(dst.scale), 1.0E-6));
}
