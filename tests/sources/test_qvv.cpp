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

#include <catch2/catch.hpp>

#include <rtm/qvvf.h>
#include <rtm/qvvd.h>

using namespace rtm;

template<typename TransformType, typename FloatType>
static void test_qvv_impl(const TransformType& identity, const FloatType threshold)
{
	using QuatType = decltype(TransformType::rotation);
	using Vector4Type = decltype(TransformType::translation);
	using ScalarType = typename float_traits<FloatType>::scalar;

	{
		Vector4Type zero = vector_set(FloatType(0.0));
		Vector4Type one = vector_set(FloatType(1.0));
		QuatType q_identity = quat_set(FloatType(0.0), FloatType(0.0), FloatType(0.0), FloatType(1.0));
		TransformType tmp = qvv_set(q_identity, zero, one);
		CHECK(quat_near_equal(identity.rotation, tmp.rotation, threshold));
		CHECK(vector_all_near_equal3(identity.translation, tmp.translation, threshold));
		CHECK(vector_all_near_equal3(identity.scale, tmp.scale, threshold));
		CHECK(quat_near_equal(q_identity, tmp.rotation, threshold));
		CHECK(vector_all_near_equal3(zero, tmp.translation, threshold));
		CHECK(vector_all_near_equal3(one, tmp.scale, threshold));
	}

	{
		Vector4Type x_axis = vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0));
		Vector4Type y_axis = vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0));
		Vector4Type test_scale = vector_set(FloatType(1.2));

		QuatType rotation_around_z = quat_from_euler(scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(90.0)), scalar_deg_to_rad(FloatType(0.0)));
		TransformType transform_a = qvv_set(rotation_around_z, x_axis, test_scale);
		Vector4Type result = qvv_mul_point3(x_axis, transform_a);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(1.0), FloatType(1.2), FloatType(0.0)), threshold));
		result = qvv_mul_point3(y_axis, transform_a);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(-0.2), FloatType(0.0), FloatType(0.0)), threshold));

		QuatType rotation_around_x = quat_from_euler(scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(90.0)));
		TransformType transform_b = qvv_set(rotation_around_x, y_axis, test_scale);
		result = qvv_mul_point3(x_axis, transform_b);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(1.2), FloatType(1.0), FloatType(0.0)), threshold));
		result = qvv_mul_point3(y_axis, transform_b);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(0.0), FloatType(1.0), FloatType(-1.2)), threshold));

		TransformType transform_ab = qvv_mul(transform_a, transform_b);
		TransformType transform_ba = qvv_mul(transform_b, transform_a);
		result = qvv_mul_point3(x_axis, transform_ab);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(1.2), FloatType(1.0), FloatType(-1.44)), threshold));
		CHECK(vector_all_near_equal3(result, qvv_mul_point3(qvv_mul_point3(x_axis, transform_a), transform_b), threshold));
		result = qvv_mul_point3(y_axis, transform_ab);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(-0.24), FloatType(1.0), FloatType(0.0)), threshold));
		CHECK(vector_all_near_equal3(result, qvv_mul_point3(qvv_mul_point3(y_axis, transform_a), transform_b), threshold));
		result = qvv_mul_point3(x_axis, transform_ba);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(-0.2), FloatType(1.44), FloatType(0.0)), threshold));
		CHECK(vector_all_near_equal3(result, qvv_mul_point3(qvv_mul_point3(x_axis, transform_b), transform_a), threshold));
		result = qvv_mul_point3(y_axis, transform_ba);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(-0.2), FloatType(0.0), FloatType(-1.44)), threshold));
		CHECK(vector_all_near_equal3(result, qvv_mul_point3(qvv_mul_point3(y_axis, transform_b), transform_a), threshold));
	}

	{
		Vector4Type x_axis = vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0));
		Vector4Type y_axis = vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0));

		QuatType rotation_around_z = quat_from_euler(scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(90.0)), scalar_deg_to_rad(FloatType(0.0)));
		TransformType transform_a = qvv_set(rotation_around_z, x_axis, vector_set(FloatType(1.0)));
		Vector4Type result = qvv_mul_point3_no_scale(x_axis, transform_a);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(1.0), FloatType(1.0), FloatType(0.0)), threshold));
		result = qvv_mul_point3_no_scale(y_axis, transform_a);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(0.0), FloatType(0.0), FloatType(0.0)), threshold));

		QuatType rotation_around_x = quat_from_euler(scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(90.0)));
		TransformType transform_b = qvv_set(rotation_around_x, y_axis, vector_set(FloatType(1.0)));
		result = qvv_mul_point3_no_scale(x_axis, transform_b);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(1.0), FloatType(1.0), FloatType(0.0)), threshold));
		result = qvv_mul_point3_no_scale(y_axis, transform_b);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(0.0), FloatType(1.0), FloatType(-1.0)), threshold));

		TransformType transform_ab = qvv_mul_no_scale(transform_a, transform_b);
		TransformType transform_ba = qvv_mul_no_scale(transform_b, transform_a);
		result = qvv_mul_point3_no_scale(x_axis, transform_ab);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(1.0), FloatType(1.0), FloatType(-1.0)), threshold));
		CHECK(vector_all_near_equal3(result, qvv_mul_point3_no_scale(qvv_mul_point3_no_scale(x_axis, transform_a), transform_b), threshold));
		result = qvv_mul_point3_no_scale(y_axis, transform_ab);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0)), threshold));
		CHECK(vector_all_near_equal3(result, qvv_mul_point3_no_scale(qvv_mul_point3_no_scale(y_axis, transform_a), transform_b), threshold));
		result = qvv_mul_point3_no_scale(x_axis, transform_ba);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0)), threshold));
		CHECK(vector_all_near_equal3(result, qvv_mul_point3_no_scale(qvv_mul_point3_no_scale(x_axis, transform_b), transform_a), threshold));
		result = qvv_mul_point3_no_scale(y_axis, transform_ba);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(0.0), FloatType(0.0), FloatType(-1.0)), threshold));
		CHECK(vector_all_near_equal3(result, qvv_mul_point3_no_scale(qvv_mul_point3_no_scale(y_axis, transform_b), transform_a), threshold));
	}

	{
		Vector4Type x_axis = vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0));
		Vector4Type test_scale1 = vector_set(FloatType(1.2));
		Vector4Type test_scale2 = vector_set(FloatType(-1.2));

		QuatType rotation_around_z = quat_from_euler(scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(90.0)), scalar_deg_to_rad(FloatType(0.0)));
		TransformType transform_a = qvv_set(rotation_around_z, x_axis, test_scale1);
		TransformType transform_b = qvv_inverse(transform_a);
		TransformType transform_ab = qvv_mul(transform_a, transform_b);
		CHECK(quat_near_equal(identity.rotation, transform_ab.rotation, threshold));
		CHECK(vector_all_near_equal3(identity.translation, transform_ab.translation, threshold));
		CHECK(vector_all_near_equal3(identity.scale, transform_ab.scale, threshold));

		transform_a = qvv_set(rotation_around_z, x_axis, test_scale2);
		transform_b = qvv_inverse(transform_a);
		transform_ab = qvv_mul(transform_a, transform_b);
		CHECK(quat_near_equal(identity.rotation, transform_ab.rotation, threshold));
		CHECK(vector_all_near_equal3(identity.translation, transform_ab.translation, threshold));
		CHECK(vector_all_near_equal3(identity.scale, transform_ab.scale, threshold));
	}

	{
		Vector4Type x_axis = vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0));

		QuatType rotation_around_z = quat_from_euler(scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(90.0)), scalar_deg_to_rad(FloatType(0.0)));
		TransformType transform_a = qvv_set(rotation_around_z, x_axis, vector_set(FloatType(1.0)));
		TransformType transform_b = qvv_inverse_no_scale(transform_a);
		TransformType transform_ab = qvv_mul_no_scale(transform_a, transform_b);
		CHECK(quat_near_equal(identity.rotation, transform_ab.rotation, threshold));
		CHECK(vector_all_near_equal3(identity.translation, transform_ab.translation, threshold));
		CHECK(vector_all_near_equal3(identity.scale, transform_ab.scale, threshold));
	}

	{
		Vector4Type x_axis = vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0));
		QuatType rotation_around_z = quat_from_euler(scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(90.0)), scalar_deg_to_rad(FloatType(0.0)));
		TransformType transform_a = qvv_set(rotation_around_z, x_axis, vector_set(FloatType(1.0)));
		CHECK(quat_is_normalized(qvv_normalize(transform_a).rotation, threshold));

		QuatType quat = quat_set(FloatType(-0.001138), FloatType(0.91623), FloatType(-1.624598), FloatType(0.715671));
		TransformType transform_b = qvv_set(quat, x_axis, vector_set(FloatType(1.0)));
		CHECK(!quat_is_normalized(transform_b.rotation, threshold));
		CHECK(quat_is_normalized(qvv_normalize(transform_b).rotation, threshold));
	}

	{
		FloatType alpha = FloatType(0.33);
		ScalarType alpha_s = scalar_set(alpha);
		QuatType quat0 = quat_normalize(quat_from_euler(scalar_deg_to_rad(FloatType(30.0)), scalar_deg_to_rad(FloatType(-45.0)), scalar_deg_to_rad(FloatType(90.0))));
		QuatType quat1 = quat_normalize(quat_from_euler(scalar_deg_to_rad(FloatType(45.0)), scalar_deg_to_rad(FloatType(60.0)), scalar_deg_to_rad(FloatType(120.0))));

		QuatType quat_ref_lerp = quat_lerp(quat0, quat1, alpha);
		QuatType quat_ref_lerp_s = quat_lerp(quat0, quat1, alpha_s);
		QuatType quat_ref_slerp = quat_slerp(quat0, quat1, alpha);
		QuatType quat_ref_slerp_s = quat_slerp(quat0, quat1, alpha_s);

		Vector4Type trans0 = vector_set(FloatType(-0.001138), FloatType(0.91623), FloatType(-1.624598));
		Vector4Type trans1 = vector_set(FloatType(-0.001138), FloatType(0.91623), FloatType(-1.624598));

		Vector4Type trans_ref = vector_lerp(trans0, trans1, alpha);
		Vector4Type trans_ref_s = vector_lerp(trans0, trans1, alpha_s);

		Vector4Type scale0 = vector_set(FloatType(-1.915), FloatType(0.23656), FloatType(-3.7811));
		Vector4Type scale1 = vector_set(FloatType(-0.2113), FloatType(12.22335), FloatType(-1.7261));

		Vector4Type scale_ref = vector_lerp(scale0, scale1, alpha);
		Vector4Type scale_ref_s = vector_lerp(scale0, scale1, alpha_s);

		TransformType transform0 = qvv_set(quat0, trans0, scale0);
		TransformType transform1 = qvv_set(quat1, trans1, scale1);

		TransformType transform_ref_lerp = qvv_set(quat_ref_lerp, trans_ref, scale_ref);
		TransformType transform_ref_slerp = qvv_set(quat_ref_slerp, trans_ref, scale_ref);

		TransformType transform_ref_lerp_s = qvv_set(quat_ref_lerp_s, trans_ref_s, scale_ref_s);
		TransformType transform_ref_slerp_s = qvv_set(quat_ref_slerp_s, trans_ref_s, scale_ref_s);

		TransformType transform_lerp = qvv_lerp(transform0, transform1, alpha);
		TransformType transform_lerp_s = qvv_lerp(transform0, transform1, alpha_s);

		TransformType transform_slerp = qvv_slerp(transform0, transform1, alpha);
		TransformType transform_slerp_s = qvv_slerp(transform0, transform1, alpha_s);

		TransformType transform_lerp_no_scale = qvv_lerp_no_scale(transform0, transform1, alpha);
		TransformType transform_lerp_no_scale_s = qvv_lerp_no_scale(transform0, transform1, alpha_s);

		TransformType transform_slerp_no_scale = qvv_slerp_no_scale(transform0, transform1, alpha);
		TransformType transform_slerp_no_scale_s = qvv_slerp_no_scale(transform0, transform1, alpha_s);

		CHECK(quat_near_equal(transform_lerp.rotation, transform_ref_lerp.rotation, threshold));
		CHECK(quat_near_equal(transform_lerp_s.rotation, transform_ref_lerp_s.rotation, threshold));
		CHECK(vector_all_near_equal3(transform_lerp.translation, transform_ref_lerp.translation, threshold));
		CHECK(vector_all_near_equal3(transform_lerp_s.translation, transform_ref_lerp_s.translation, threshold));
		CHECK(vector_all_near_equal3(transform_lerp.scale, transform_ref_lerp.scale, threshold));
		CHECK(vector_all_near_equal3(transform_lerp_s.scale, transform_ref_lerp_s.scale, threshold));

		CHECK(quat_near_equal(transform_slerp.rotation, transform_ref_slerp.rotation, threshold));
		CHECK(quat_near_equal(transform_slerp_s.rotation, transform_ref_slerp_s.rotation, threshold));
		CHECK(vector_all_near_equal3(transform_slerp.translation, transform_ref_slerp.translation, threshold));
		CHECK(vector_all_near_equal3(transform_slerp_s.translation, transform_ref_slerp_s.translation, threshold));
		CHECK(vector_all_near_equal3(transform_slerp.scale, transform_ref_slerp.scale, threshold));
		CHECK(vector_all_near_equal3(transform_slerp_s.scale, transform_ref_slerp_s.scale, threshold));

		CHECK(quat_near_equal(transform_lerp_no_scale.rotation, transform_ref_lerp.rotation, threshold));
		CHECK(quat_near_equal(transform_lerp_no_scale_s.rotation, transform_ref_lerp_s.rotation, threshold));
		CHECK(vector_all_near_equal3(transform_lerp_no_scale.translation, transform_ref_lerp.translation, threshold));
		CHECK(vector_all_near_equal3(transform_lerp_no_scale_s.translation, transform_ref_lerp_s.translation, threshold));
		CHECK(vector_all_near_equal3(transform_lerp_no_scale.scale, transform0.scale, threshold));
		CHECK(vector_all_near_equal3(transform_lerp_no_scale_s.scale, transform0.scale, threshold));

		CHECK(quat_near_equal(transform_slerp_no_scale.rotation, transform_ref_slerp.rotation, threshold));
		CHECK(quat_near_equal(transform_slerp_no_scale_s.rotation, transform_ref_slerp_s.rotation, threshold));
		CHECK(vector_all_near_equal3(transform_slerp_no_scale.translation, transform_ref_slerp.translation, threshold));
		CHECK(vector_all_near_equal3(transform_slerp_no_scale_s.translation, transform_ref_slerp_s.translation, threshold));
		CHECK(vector_all_near_equal3(transform_slerp_no_scale.scale, transform0.scale, threshold));
		CHECK(vector_all_near_equal3(transform_slerp_no_scale_s.scale, transform0.scale, threshold));
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
	CHECK(quat_near_equal(src.rotation, quat_cast(dst.rotation), 1.0E-6F));
	CHECK(vector_all_near_equal3(src.translation, vector_cast(dst.translation), 1.0E-6F));
	CHECK(vector_all_near_equal3(src.scale, vector_cast(dst.scale), 1.0E-6F));
}

TEST_CASE("qvvd math", "[math][qvv]")
{
	test_qvv_impl<qvvd, double>(qvv_identity(), 1.0E-6);

	const quatd src_rotation = quat_set(0.39564531008956383, 0.044254239301713752, 0.22768840967675355, 0.88863059760894492);
	const vector4d src_translation = vector_set(-2.65, 2.996113, 0.68123521);
	const vector4d src_scale = vector_set(1.2, 0.8, 2.1);
	const qvvd src = qvv_set(src_rotation, src_translation, src_scale);
	const qvvf dst = qvv_cast(src);
	CHECK(quat_near_equal(src.rotation, quat_cast(dst.rotation), 1.0E-6));
	CHECK(vector_all_near_equal3(src.translation, vector_cast(dst.translation), 1.0E-6));
	CHECK(vector_all_near_equal3(src.scale, vector_cast(dst.scale), 1.0E-6));
}
