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

#include <rtm/qvsf.h>
#include <rtm/qvsd.h>
#include <rtm/type_traits.h>

using namespace rtm;

template<typename TransformType, typename FloatType>
static void test_qvs_impl(const TransformType& identity, const FloatType threshold)
{
	using QuatType = typename float_traits<FloatType>::quat;
	using Vector4Type = typename float_traits<FloatType>::vector4;
	using ScalarType = typename float_traits<FloatType>::scalar;

	{
		Vector4Type zero = vector_set(FloatType(0.0));
		FloatType one = FloatType(1.0);
		QuatType q_identity = quat_set(FloatType(0.0), FloatType(0.0), FloatType(0.0), FloatType(1.0));
		TransformType tmp = qvs_set(q_identity, zero, one);
		CHECK(quat_near_equal(identity.rotation, tmp.rotation, threshold));
		CHECK(vector_all_near_equal(identity.translation_scale, tmp.translation_scale, threshold));
		CHECK(quat_near_equal(q_identity, qvs_get_rotation(tmp), threshold));
		CHECK(vector_all_near_equal3(zero, qvs_get_translation(tmp), threshold));
		CHECK(scalar_near_equal(one, qvs_get_scale(tmp), threshold));
	}

	{
		Vector4Type x_axis = vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0));
		Vector4Type y_axis = vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0));
		FloatType test_scale = FloatType(1.2);

		QuatType rotation_around_z = quat_from_euler(scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(90.0)), scalar_deg_to_rad(FloatType(0.0)));
		TransformType transform_a = qvs_set(rotation_around_z, x_axis, test_scale);
		Vector4Type result = qvs_mul_point3(x_axis, transform_a);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(1.0), FloatType(1.2), FloatType(0.0)), threshold));
		result = qvs_mul_point3(y_axis, transform_a);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(-0.2), FloatType(0.0), FloatType(0.0)), threshold));

		QuatType rotation_around_x = quat_from_euler(scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(90.0)));
		TransformType transform_b = qvs_set(rotation_around_x, y_axis, test_scale);
		result = qvs_mul_point3(x_axis, transform_b);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(1.2), FloatType(1.0), FloatType(0.0)), threshold));
		result = qvs_mul_point3(y_axis, transform_b);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(0.0), FloatType(1.0), FloatType(-1.2)), threshold));

		TransformType transform_ab = qvs_mul(transform_a, transform_b);
		TransformType transform_ba = qvs_mul(transform_b, transform_a);
		result = qvs_mul_point3(x_axis, transform_ab);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(1.2), FloatType(1.0), FloatType(-1.44)), threshold));
		CHECK(vector_all_near_equal3(result, qvs_mul_point3(qvs_mul_point3(x_axis, transform_a), transform_b), threshold));
		result = qvs_mul_point3(y_axis, transform_ab);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(-0.24), FloatType(1.0), FloatType(0.0)), threshold));
		CHECK(vector_all_near_equal3(result, qvs_mul_point3(qvs_mul_point3(y_axis, transform_a), transform_b), threshold));
		result = qvs_mul_point3(x_axis, transform_ba);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(-0.2), FloatType(1.44), FloatType(0.0)), threshold));
		CHECK(vector_all_near_equal3(result, qvs_mul_point3(qvs_mul_point3(x_axis, transform_b), transform_a), threshold));
		result = qvs_mul_point3(y_axis, transform_ba);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(-0.2), FloatType(0.0), FloatType(-1.44)), threshold));
		CHECK(vector_all_near_equal3(result, qvs_mul_point3(qvs_mul_point3(y_axis, transform_b), transform_a), threshold));
	}

	{
		Vector4Type x_axis = vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0));
		Vector4Type y_axis = vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0));

		QuatType rotation_around_z = quat_from_euler(scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(90.0)), scalar_deg_to_rad(FloatType(0.0)));
		TransformType transform_a = qvs_set(rotation_around_z, x_axis, FloatType(1.0));
		Vector4Type result = qvs_mul_point3_no_scale(x_axis, transform_a);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(1.0), FloatType(1.0), FloatType(0.0)), threshold));
		result = qvs_mul_point3_no_scale(y_axis, transform_a);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(0.0), FloatType(0.0), FloatType(0.0)), threshold));

		QuatType rotation_around_x = quat_from_euler(scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(90.0)));
		TransformType transform_b = qvs_set(rotation_around_x, y_axis, FloatType(1.0));
		result = qvs_mul_point3_no_scale(x_axis, transform_b);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(1.0), FloatType(1.0), FloatType(0.0)), threshold));
		result = qvs_mul_point3_no_scale(y_axis, transform_b);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(0.0), FloatType(1.0), FloatType(-1.0)), threshold));

		TransformType transform_ab = qvs_mul_no_scale(transform_a, transform_b);
		TransformType transform_ba = qvs_mul_no_scale(transform_b, transform_a);
		result = qvs_mul_point3_no_scale(x_axis, transform_ab);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(1.0), FloatType(1.0), FloatType(-1.0)), threshold));
		CHECK(vector_all_near_equal3(result, qvs_mul_point3_no_scale(qvs_mul_point3_no_scale(x_axis, transform_a), transform_b), threshold));
		result = qvs_mul_point3_no_scale(y_axis, transform_ab);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0)), threshold));
		CHECK(vector_all_near_equal3(result, qvs_mul_point3_no_scale(qvs_mul_point3_no_scale(y_axis, transform_a), transform_b), threshold));
		result = qvs_mul_point3_no_scale(x_axis, transform_ba);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0)), threshold));
		CHECK(vector_all_near_equal3(result, qvs_mul_point3_no_scale(qvs_mul_point3_no_scale(x_axis, transform_b), transform_a), threshold));
		result = qvs_mul_point3_no_scale(y_axis, transform_ba);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(0.0), FloatType(0.0), FloatType(-1.0)), threshold));
		CHECK(vector_all_near_equal3(result, qvs_mul_point3_no_scale(qvs_mul_point3_no_scale(y_axis, transform_b), transform_a), threshold));
	}

	{
		Vector4Type x_axis = vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0));
		FloatType test_scale1 = FloatType(1.2);
		FloatType test_scale2 = FloatType(-1.2);
		FloatType test_scale3 = FloatType(0.0);

		QuatType rotation_around_z = quat_from_euler(scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(90.0)), scalar_deg_to_rad(FloatType(0.0)));
		TransformType transform_a = qvs_set(rotation_around_z, x_axis, test_scale1);
		TransformType transform_b = qvs_inverse(transform_a);
		TransformType transform_ab = qvs_mul(transform_a, transform_b);
		CHECK(quat_near_equal(identity.rotation, transform_ab.rotation, threshold));
		CHECK(vector_all_near_equal3(qvs_get_translation(identity), qvs_get_translation(transform_ab), threshold));
		CHECK(scalar_near_equal(qvs_get_scale(identity), qvs_get_scale(transform_ab), threshold));

		transform_a = qvs_set(rotation_around_z, x_axis, test_scale2);
		transform_b = qvs_inverse(transform_a);
		transform_ab = qvs_mul(transform_a, transform_b);
		CHECK(quat_near_equal(identity.rotation, transform_ab.rotation, threshold));
		CHECK(vector_all_near_equal3(qvs_get_translation(identity), qvs_get_translation(transform_ab), threshold));
		CHECK(scalar_near_equal(qvs_get_scale(identity), qvs_get_scale(transform_ab), threshold));

		transform_a = qvs_set(rotation_around_z, x_axis, test_scale3);
		transform_b = qvs_inverse(transform_a, FloatType(1.0));
		CHECK(qvs_is_finite(transform_b));
	}

	{
		Vector4Type x_axis = vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0));

		QuatType rotation_around_z = quat_from_euler(scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(90.0)), scalar_deg_to_rad(FloatType(0.0)));
		TransformType transform_a = qvs_set(rotation_around_z, x_axis, FloatType(1.0));
		TransformType transform_b = qvs_inverse_no_scale(transform_a);
		TransformType transform_ab = qvs_mul_no_scale(transform_a, transform_b);
		CHECK(quat_near_equal(identity.rotation, transform_ab.rotation, threshold));
		CHECK(vector_all_near_equal3(qvs_get_translation(identity), qvs_get_translation(transform_ab), threshold));
		CHECK(scalar_near_equal(qvs_get_scale(identity), qvs_get_scale(transform_ab), threshold));
	}

	{
		Vector4Type x_axis = vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0));
		QuatType rotation_around_z = quat_from_euler(scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(90.0)), scalar_deg_to_rad(FloatType(0.0)));
		TransformType transform_a = qvs_set(rotation_around_z, x_axis, FloatType(1.0));
		transform_a = qvs_normalize(transform_a);
		CHECK(quat_is_normalized(transform_a.rotation, threshold));

		QuatType quat = quat_set(FloatType(-0.001138), FloatType(0.91623), FloatType(-1.624598), FloatType(0.715671));
		TransformType transform_b = qvs_set(quat, x_axis, FloatType(1.0));
		CHECK(!quat_is_normalized(transform_b.rotation, threshold));
		transform_b = qvs_normalize(transform_b);
		CHECK(quat_is_normalized(transform_b.rotation, threshold));
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

		FloatType scale0 = FloatType(-1.915);
		FloatType scale1 = FloatType(-0.2113);

		FloatType scale_ref = scalar_lerp(scale0, scale1, alpha);
		ScalarType scale_ref_s = scalar_lerp(scalar_set(scale0), scalar_set(scale1), alpha_s);

		TransformType transform0 = qvs_set(quat0, trans0, scale0);
		TransformType transform1 = qvs_set(quat1, trans1, scale1);

		TransformType transform_ref_lerp = qvs_set(quat_ref_lerp, trans_ref, scale_ref);
		TransformType transform_ref_slerp = qvs_set(quat_ref_slerp, trans_ref, scale_ref);

		TransformType transform_ref_lerp_s = qvs_set(quat_ref_lerp_s, trans_ref_s, scale_ref_s);
		TransformType transform_ref_slerp_s = qvs_set(quat_ref_slerp_s, trans_ref_s, scale_ref_s);

		TransformType transform_lerp = qvs_lerp(transform0, transform1, alpha);
		TransformType transform_lerp_s = qvs_lerp(transform0, transform1, alpha_s);

		TransformType transform_slerp = qvs_slerp(transform0, transform1, alpha);
		TransformType transform_slerp_s = qvs_slerp(transform0, transform1, alpha_s);

		TransformType transform_lerp_no_scale = qvs_lerp_no_scale(transform0, transform1, alpha);
		TransformType transform_lerp_no_scale_s = qvs_lerp_no_scale(transform0, transform1, alpha_s);

		TransformType transform_slerp_no_scale = qvs_slerp_no_scale(transform0, transform1, alpha);
		TransformType transform_slerp_no_scale_s = qvs_slerp_no_scale(transform0, transform1, alpha_s);

		CHECK(quat_near_equal(transform_lerp.rotation, transform_ref_lerp.rotation, threshold));
		CHECK(quat_near_equal(transform_lerp_s.rotation, transform_ref_lerp_s.rotation, threshold));
		CHECK(vector_all_near_equal3(qvs_get_translation(transform_lerp), qvs_get_translation(transform_ref_lerp), threshold));
		CHECK(vector_all_near_equal3(qvs_get_translation(transform_lerp_s), qvs_get_translation(transform_ref_lerp_s), threshold));
		CHECK(scalar_near_equal(qvs_get_scale(transform_lerp), qvs_get_scale(transform_ref_lerp), threshold));
		CHECK(scalar_near_equal(qvs_get_scale(transform_lerp_s), qvs_get_scale(transform_ref_lerp_s), threshold));

		CHECK(quat_near_equal(transform_slerp.rotation, transform_ref_slerp.rotation, threshold));
		CHECK(quat_near_equal(transform_slerp_s.rotation, transform_ref_slerp_s.rotation, threshold));
		CHECK(vector_all_near_equal3(qvs_get_translation(transform_slerp), qvs_get_translation(transform_ref_slerp), threshold));
		CHECK(vector_all_near_equal3(qvs_get_translation(transform_slerp_s), qvs_get_translation(transform_ref_slerp_s), threshold));
		CHECK(scalar_near_equal(qvs_get_scale(transform_slerp), qvs_get_scale(transform_ref_slerp), threshold));
		CHECK(scalar_near_equal(qvs_get_scale(transform_slerp_s), qvs_get_scale(transform_ref_slerp_s), threshold));

		CHECK(quat_near_equal(transform_lerp_no_scale.rotation, transform_ref_lerp.rotation, threshold));
		CHECK(quat_near_equal(transform_lerp_no_scale_s.rotation, transform_ref_lerp_s.rotation, threshold));
		CHECK(vector_all_near_equal3(qvs_get_translation(transform_lerp_no_scale), qvs_get_translation(transform_ref_lerp), threshold));
		CHECK(vector_all_near_equal3(qvs_get_translation(transform_lerp_no_scale_s), qvs_get_translation(transform_ref_lerp_s), threshold));
		CHECK(scalar_near_equal(qvs_get_scale(transform_lerp_no_scale), qvs_get_scale(transform0), threshold));
		CHECK(scalar_near_equal(qvs_get_scale(transform_lerp_no_scale_s), qvs_get_scale(transform0), threshold));

		CHECK(quat_near_equal(transform_slerp_no_scale.rotation, transform_ref_slerp.rotation, threshold));
		CHECK(quat_near_equal(transform_slerp_no_scale_s.rotation, transform_ref_slerp_s.rotation, threshold));
		CHECK(vector_all_near_equal3(qvs_get_translation(transform_slerp_no_scale), qvs_get_translation(transform_ref_slerp), threshold));
		CHECK(vector_all_near_equal3(qvs_get_translation(transform_slerp_no_scale_s), qvs_get_translation(transform_ref_slerp_s), threshold));
		CHECK(scalar_near_equal(qvs_get_scale(transform_slerp_no_scale), qvs_get_scale(transform0), threshold));
		CHECK(scalar_near_equal(qvs_get_scale(transform_slerp_no_scale_s), qvs_get_scale(transform0), threshold));
	}

	{
		const FloatType inf = std::numeric_limits<FloatType>::infinity();
		const FloatType nan = std::numeric_limits<FloatType>::quiet_NaN();

		const QuatType quat0 = quat_normalize(quat_from_euler(scalar_deg_to_rad(FloatType(30.0)), scalar_deg_to_rad(FloatType(-45.0)), scalar_deg_to_rad(FloatType(90.0))));
		const QuatType quat_inf = quat_set(inf, inf, inf, inf);
		const QuatType quat_nan = quat_set(nan, nan, nan, nan);

		const Vector4Type trans0 = vector_set(FloatType(-0.001138), FloatType(0.91623), FloatType(-1.624598));
		const FloatType scale0 = FloatType(-1.915);
		const Vector4Type vec_inf = vector_set(inf, inf, inf, inf);
		const Vector4Type vec_nan = vector_set(nan, nan, nan, nan);

		CHECK(qvs_is_finite(identity) == true);
		CHECK(qvs_is_finite(qvs_set(quat0, trans0, scale0)) == true);
		CHECK(qvs_is_finite(qvs_set(quat_inf, trans0, scale0)) == false);
		CHECK(qvs_is_finite(qvs_set(quat_nan, trans0, scale0)) == false);
		CHECK(qvs_is_finite(qvs_set(quat0, vec_inf, scale0)) == false);
		CHECK(qvs_is_finite(qvs_set(quat0, vec_nan, scale0)) == false);
		CHECK(qvs_is_finite(qvs_set(quat0, trans0, inf)) == false);
		CHECK(qvs_is_finite(qvs_set(quat0, trans0, nan)) == false);
		CHECK(qvs_is_finite(qvs_set(quat_inf, vec_inf, inf)) == false);
		CHECK(qvs_is_finite(qvs_set(quat_nan, vec_nan, nan)) == false);
	}
}

TEST_CASE("qvsf math", "[math][qvs]")
{
	test_qvs_impl<qvsf, float>(qvs_identity(), 1.0E-4F);

	const quatf src_rotation = quat_set(0.39564531008956383F, 0.044254239301713752F, 0.22768840967675355F, 0.88863059760894492F);
	const vector4f src_translation = vector_set(-2.65F, 2.996113F, 0.68123521F);
	const float src_scale = 1.2F;
	const qvsf src = qvs_set(src_rotation, src_translation, src_scale);
	const qvsd dst = qvs_cast(src);
	CHECK(quat_near_equal(src.rotation, quat_cast(dst.rotation), 1.0E-6F));
	CHECK(vector_all_near_equal(src.translation_scale, vector_cast(dst.translation_scale), 1.0E-6F));
}

TEST_CASE("qvsd math", "[math][qvs]")
{
	test_qvs_impl<qvsd, double>(qvs_identity(), 1.0E-6);

	const quatd src_rotation = quat_set(0.39564531008956383, 0.044254239301713752, 0.22768840967675355, 0.88863059760894492);
	const vector4d src_translation = vector_set(-2.65, 2.996113, 0.68123521);
	const double src_scale = 1.2;
	const qvsd src = qvs_set(src_rotation, src_translation, src_scale);
	const qvsf dst = qvs_cast(src);
	CHECK(quat_near_equal(src.rotation, quat_cast(dst.rotation), 1.0E-6));
	CHECK(vector_all_near_equal(src.translation_scale, vector_cast(dst.translation_scale), 1.0E-6));
}
