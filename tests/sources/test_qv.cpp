////////////////////////////////////////////////////////////////////////////////
// The MIT License (MIT)
//
// Copyright (c) 2023 Nicholas Frechette & Realtime Math contributors
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

#include <rtm/qvf.h>
#include <rtm/qvd.h>

using namespace rtm;

template<typename TransformType, typename FloatType>
static void test_qv_impl(const TransformType& identity, const FloatType threshold)
{
	using QuatType = decltype(TransformType::rotation);
	using Vector4Type = decltype(TransformType::translation);
	using ScalarType = typename float_traits<FloatType>::scalar;

	{
		Vector4Type zero = vector_set(FloatType(0.0));
		QuatType q_identity = quat_set(FloatType(0.0), FloatType(0.0), FloatType(0.0), FloatType(1.0));
		TransformType tmp = qv_set(q_identity, zero);
		CHECK(quat_near_equal(identity.rotation, tmp.rotation, threshold));
		CHECK(vector_all_near_equal3(identity.translation, tmp.translation, threshold));
		CHECK(quat_near_equal(q_identity, tmp.rotation, threshold));
		CHECK(vector_all_near_equal3(zero, tmp.translation, threshold));
	}

	{
		Vector4Type x_axis = vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0));
		Vector4Type y_axis = vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0));

		QuatType rotation_around_z = quat_from_euler(scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(90.0)), scalar_deg_to_rad(FloatType(0.0)));
		TransformType transform_a = qv_set(rotation_around_z, x_axis);
		Vector4Type result = qv_mul_point3(x_axis, transform_a);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(1.0), FloatType(1.0), FloatType(0.0)), threshold));
		result = qv_mul_point3(y_axis, transform_a);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(0.0), FloatType(0.0), FloatType(0.0)), threshold));

		QuatType rotation_around_x = quat_from_euler(scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(90.0)));
		TransformType transform_b = qv_set(rotation_around_x, y_axis);
		result = qv_mul_point3(x_axis, transform_b);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(1.0), FloatType(1.0), FloatType(0.0)), threshold));
		result = qv_mul_point3(y_axis, transform_b);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(0.0), FloatType(1.0), FloatType(-1.0)), threshold));

		TransformType transform_ab = qv_mul(transform_a, transform_b);
		TransformType transform_ba = qv_mul(transform_b, transform_a);
		result = qv_mul_point3(x_axis, transform_ab);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(1.0), FloatType(1.0), FloatType(-1.0)), threshold));
		CHECK(vector_all_near_equal3(result, qv_mul_point3(qv_mul_point3(x_axis, transform_a), transform_b), threshold));
		result = qv_mul_point3(y_axis, transform_ab);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0)), threshold));
		CHECK(vector_all_near_equal3(result, qv_mul_point3(qv_mul_point3(y_axis, transform_a), transform_b), threshold));
		result = qv_mul_point3(x_axis, transform_ba);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0)), threshold));
		CHECK(vector_all_near_equal3(result, qv_mul_point3(qv_mul_point3(x_axis, transform_b), transform_a), threshold));
		result = qv_mul_point3(y_axis, transform_ba);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(0.0), FloatType(0.0), FloatType(-1.0)), threshold));
		CHECK(vector_all_near_equal3(result, qv_mul_point3(qv_mul_point3(y_axis, transform_b), transform_a), threshold));
	}

	{
		Vector4Type x_axis = vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0));
		Vector4Type y_axis = vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0));

		QuatType rotation_around_z = quat_from_euler(scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(90.0)), scalar_deg_to_rad(FloatType(0.0)));
		TransformType transform_a = qv_set(rotation_around_z, x_axis);
		Vector4Type result = qv_mul_point3(x_axis, transform_a);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(1.0), FloatType(1.0), FloatType(0.0)), threshold));
		result = qv_mul_point3(y_axis, transform_a);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(0.0), FloatType(0.0), FloatType(0.0)), threshold));

		QuatType rotation_around_x = quat_from_euler(scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(90.0)));
		TransformType transform_b = qv_set(rotation_around_x, y_axis);
		result = qv_mul_point3(x_axis, transform_b);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(1.0), FloatType(1.0), FloatType(0.0)), threshold));
		result = qv_mul_point3(y_axis, transform_b);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(0.0), FloatType(1.0), FloatType(-1.0)), threshold));

		TransformType transform_ab = qv_mul(transform_a, transform_b);
		TransformType transform_ba = qv_mul(transform_b, transform_a);
		result = qv_mul_point3(x_axis, transform_ab);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(1.0), FloatType(1.0), FloatType(-1.0)), threshold));
		CHECK(vector_all_near_equal3(result, qv_mul_point3(qv_mul_point3(x_axis, transform_a), transform_b), threshold));
		result = qv_mul_point3(y_axis, transform_ab);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0)), threshold));
		CHECK(vector_all_near_equal3(result, qv_mul_point3(qv_mul_point3(y_axis, transform_a), transform_b), threshold));
		result = qv_mul_point3(x_axis, transform_ba);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0)), threshold));
		CHECK(vector_all_near_equal3(result, qv_mul_point3(qv_mul_point3(x_axis, transform_b), transform_a), threshold));
		result = qv_mul_point3(y_axis, transform_ba);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(0.0), FloatType(0.0), FloatType(-1.0)), threshold));
		CHECK(vector_all_near_equal3(result, qv_mul_point3(qv_mul_point3(y_axis, transform_b), transform_a), threshold));
	}

	{
		Vector4Type x_axis = vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0));

		QuatType rotation_around_z = quat_from_euler(scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(90.0)), scalar_deg_to_rad(FloatType(0.0)));
		TransformType transform_a = qv_set(rotation_around_z, x_axis);
		TransformType transform_b = qv_inverse(transform_a);
		TransformType transform_ab = qv_mul(transform_a, transform_b);
		CHECK(quat_near_equal(identity.rotation, transform_ab.rotation, threshold));
		CHECK(vector_all_near_equal3(identity.translation, transform_ab.translation, threshold));

		transform_a = qv_set(rotation_around_z, x_axis);
		transform_b = qv_inverse(transform_a);
		transform_ab = qv_mul(transform_a, transform_b);
		CHECK(quat_near_equal(identity.rotation, transform_ab.rotation, threshold));
		CHECK(vector_all_near_equal3(identity.translation, transform_ab.translation, threshold));
	}

	{
		Vector4Type x_axis = vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0));

		QuatType rotation_around_z = quat_from_euler(scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(90.0)), scalar_deg_to_rad(FloatType(0.0)));
		TransformType transform_a = qv_set(rotation_around_z, x_axis);
		TransformType transform_b = qv_inverse(transform_a);
		TransformType transform_ab = qv_mul(transform_a, transform_b);
		CHECK(quat_near_equal(identity.rotation, transform_ab.rotation, threshold));
		CHECK(vector_all_near_equal3(identity.translation, transform_ab.translation, threshold));
	}

	{
		Vector4Type x_axis = vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0));
		QuatType rotation_around_z = quat_from_euler(scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(90.0)), scalar_deg_to_rad(FloatType(0.0)));
		TransformType transform_a = qv_set(rotation_around_z, x_axis);
		CHECK(quat_is_normalized(qv_normalize(transform_a).rotation, threshold));

		QuatType quat = quat_set(FloatType(-0.001138), FloatType(0.91623), FloatType(-1.624598), FloatType(0.715671));
		TransformType transform_b = qv_set(quat, x_axis);
		CHECK(!quat_is_normalized(transform_b.rotation, threshold));
		CHECK(quat_is_normalized(qv_normalize(transform_b).rotation, threshold));
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

		TransformType transform0 = qv_set(quat0, trans0);
		TransformType transform1 = qv_set(quat1, trans1);

		TransformType transform_ref_lerp = qv_set(quat_ref_lerp, trans_ref);
		TransformType transform_ref_slerp = qv_set(quat_ref_slerp, trans_ref);

		TransformType transform_ref_lerp_s = qv_set(quat_ref_lerp_s, trans_ref_s);
		TransformType transform_ref_slerp_s = qv_set(quat_ref_slerp_s, trans_ref_s);

		TransformType transform_lerp = qv_lerp(transform0, transform1, alpha);
		TransformType transform_lerp_s = qv_lerp(transform0, transform1, alpha_s);

		TransformType transform_slerp = qv_slerp(transform0, transform1, alpha);
		TransformType transform_slerp_s = qv_slerp(transform0, transform1, alpha_s);

		CHECK(quat_near_equal(transform_lerp.rotation, transform_ref_lerp.rotation, threshold));
		CHECK(quat_near_equal(transform_lerp_s.rotation, transform_ref_lerp_s.rotation, threshold));
		CHECK(vector_all_near_equal3(transform_lerp.translation, transform_ref_lerp.translation, threshold));
		CHECK(vector_all_near_equal3(transform_lerp_s.translation, transform_ref_lerp_s.translation, threshold));

		CHECK(quat_near_equal(transform_slerp.rotation, transform_ref_slerp.rotation, threshold));
		CHECK(quat_near_equal(transform_slerp_s.rotation, transform_ref_slerp_s.rotation, threshold));
		CHECK(vector_all_near_equal3(transform_slerp.translation, transform_ref_slerp.translation, threshold));
		CHECK(vector_all_near_equal3(transform_slerp_s.translation, transform_ref_slerp_s.translation, threshold));
	}

	{
		const FloatType inf = std::numeric_limits<FloatType>::infinity();
		const FloatType nan = std::numeric_limits<FloatType>::quiet_NaN();

		const QuatType quat0 = quat_normalize(quat_from_euler(scalar_deg_to_rad(FloatType(30.0)), scalar_deg_to_rad(FloatType(-45.0)), scalar_deg_to_rad(FloatType(90.0))));
		const QuatType quat_inf = quat_set(inf, inf, inf, inf);
		const QuatType quat_nan = quat_set(nan, nan, nan, nan);

		const Vector4Type trans0 = vector_set(FloatType(-0.001138), FloatType(0.91623), FloatType(-1.624598));
		const Vector4Type vec_inf = vector_set(inf, inf, inf, inf);
		const Vector4Type vec_nan = vector_set(nan, nan, nan, nan);

		CHECK(qv_is_finite(identity) == true);
		CHECK(qv_is_finite(qv_set(quat0, trans0)) == true);
		CHECK(qv_is_finite(qv_set(quat_inf, trans0)) == false);
		CHECK(qv_is_finite(qv_set(quat_nan, trans0)) == false);
		CHECK(qv_is_finite(qv_set(quat0, vec_inf)) == false);
		CHECK(qv_is_finite(qv_set(quat0, vec_nan)) == false);
	}
}

TEST_CASE("qvf math", "[math][qv]")
{
	test_qv_impl<qvf, float>(qv_identity(), 1.0E-4F);

	const quatf src_rotation = quat_set(0.39564531008956383F, 0.044254239301713752F, 0.22768840967675355F, 0.88863059760894492F);
	const vector4f src_translation = vector_set(-2.65F, 2.996113F, 0.68123521F);
	const qvf src = qv_set(src_rotation, src_translation);
	const qvd dst = qv_cast(src);
	CHECK(quat_near_equal(src.rotation, quat_cast(dst.rotation), 1.0E-6F));
	CHECK(vector_all_near_equal3(src.translation, vector_cast(dst.translation), 1.0E-6F));
}

TEST_CASE("qvd math", "[math][qv]")
{
	test_qv_impl<qvd, double>(qv_identity(), 1.0E-6);

	const quatd src_rotation = quat_set(0.39564531008956383, 0.044254239301713752, 0.22768840967675355, 0.88863059760894492);
	const vector4d src_translation = vector_set(-2.65, 2.996113, 0.68123521);
	const qvd src = qv_set(src_rotation, src_translation);
	const qvf dst = qv_cast(src);
	CHECK(quat_near_equal(src.rotation, quat_cast(dst.rotation), 1.0E-6));
	CHECK(vector_all_near_equal3(src.translation, vector_cast(dst.translation), 1.0E-6));
}
