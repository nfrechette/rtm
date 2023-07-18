#pragma once

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

#include "rtm/math.h"
#include "rtm/quatf.h"
#include "rtm/vector4f.h"
#include "rtm/matrix3x4f.h"
#include "rtm/version.h"
#include "rtm/impl/compiler_utils.h"
#include "rtm/impl/qvvs_common.h"

RTM_IMPL_FILE_PRAGMA_PUSH

namespace rtm
{
	RTM_IMPL_VERSION_NAMESPACE_BEGIN

	//////////////////////////////////////////////////////////////////////////
	// Casts a QVVS transform float64 variant to a float32 variant.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE qvvsf RTM_SIMD_CALL qvvs_cast(const qvvsd& input) RTM_NO_EXCEPT
	{
		return qvvsf{ quat_cast(input.rotation), vector_cast(input.translation_uniform_scale), vector_cast(input.non_uniform_scale) };
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the rotation part of a QVVS transform.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE quatf RTM_SIMD_CALL qvvs_get_rotation(qvvsf_arg0 input) RTM_NO_EXCEPT
	{
		return input.rotation;
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the translation part of a QVVS transform.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4f RTM_SIMD_CALL qvvs_get_translation(qvvsf_arg0 input) RTM_NO_EXCEPT
	{
		return input.translation_uniform_scale;
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the uniform scale part of a QVVS transform.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE constexpr rtm_impl::vector4f_vector_get_w RTM_SIMD_CALL qvvs_get_uniform_scale(qvvsf_arg0 input) RTM_NO_EXCEPT
	{
		return rtm_impl::vector4f_vector_get_w{ input.translation_uniform_scale };
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the non-uniform scale part of a QVVS transform.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4f RTM_SIMD_CALL qvvs_get_non_uniform_scale(qvvsf_arg0 input) RTM_NO_EXCEPT
	{
		return input.non_uniform_scale;
	}

	//////////////////////////////////////////////////////////////////////////
	// Multiplies two QVVS transforms.
	// Multiplication order is as follow: local_to_world = qvvs_mul(local_to_object, object_to_world)
	// NOTE: The uniform scale portion will combine through multiplication but the non-uniform part does not.
	// The resulting QVVS transform will obtain its non-uniform scale part from the left hand side value, as is.
	// By design, QVVS transforms do not propagate the parent transform non-uniform scale. This means that
	// qvvs_mul_point3(point, local_to_world) != qvvs_mul_point3(qvvs_mul_point3(point, local_to_object), object_to_world)
	// However, the multiplication operation can be reversed by multiplying with an inverse transform as expected:
	// local_to_object == qvvs_mul(local_to_world, qvvs_inverse(object_to_world))
	// local_to_object == qvvs_mul(local_to_world, world_to_object)
	// Care must be taken as the inverse can behave unexpectedly:
	// point == qvvs_mul_point3(qvvs_mul_point3(point, transform), qvvs_inverse(transform))
	// But also:
	// identity != qvvs_mul(transform, qvvs_inverse(transform))
	// Essentially, QVVS transform multiplication is non-inversable.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline qvvsf RTM_SIMD_CALL qvvs_mul(qvvsf_arg0 lhs, qvvsf_arg1 rhs) RTM_NO_EXCEPT
	{
		const vector4f rhs_uniform_scale = vector_dup_w(rhs.translation_uniform_scale);

		const quatf rotation = quat_mul(lhs.rotation, rhs.rotation);
		const vector4f translation = vector_add(quat_mul_vector3(vector_mul(lhs.translation_uniform_scale, rhs_uniform_scale), rhs.rotation), rhs.translation_uniform_scale);
		const vector4f uniform_scale_w = vector_mul(lhs.translation_uniform_scale, rhs.translation_uniform_scale);
		const vector4f translation_uniform_scale = vector_mix<mix4::x, mix4::y, mix4::z, mix4::d>(translation, uniform_scale_w);

		return qvvsf{ rotation, translation_uniform_scale, lhs.non_uniform_scale };
	}

	//////////////////////////////////////////////////////////////////////////
	// Multiplies two QVVS transforms ignoring 3D scale.
	// The resulting QVVS transform will have the LHS scale.
	// Multiplication order is as follow: local_to_world = qvvs_mul(local_to_object, object_to_world)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline qvvsf RTM_SIMD_CALL qvvs_mul_no_scale(qvvsf_arg0 lhs, qvvsf_arg1 rhs) RTM_NO_EXCEPT
	{
		const quatf rotation = quat_mul(lhs.rotation, rhs.rotation);
		const vector4f translation = vector_add(quat_mul_vector3(lhs.translation_uniform_scale, rhs.rotation), rhs.translation_uniform_scale);
		const vector4f translation_uniform_scale = vector_mix<mix4::x, mix4::y, mix4::z, mix4::d>(translation, lhs.translation_uniform_scale);

		return qvvsf{ rotation, translation_uniform_scale, lhs.non_uniform_scale };
	}

	//////////////////////////////////////////////////////////////////////////
	// Multiplies a QVVS transform and a 3D point.
	// Multiplication order is as follow: world_position = qvvs_mul_point3(local_position, local_to_world)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vector4f RTM_SIMD_CALL qvvs_mul_point3(vector4f_arg0 point, qvvsf_arg1 qvvs) RTM_NO_EXCEPT
	{
		const vector4f uniform_scale = vector_dup_w(qvvs.translation_uniform_scale);
		const vector4f scale = vector_mul(uniform_scale, qvvs.non_uniform_scale);
		return vector_add(quat_mul_vector3(vector_mul(scale, point), qvvs.rotation), qvvs.translation_uniform_scale);
	}

	//////////////////////////////////////////////////////////////////////////
	// Multiplies a QVVS transform and a 3D point ignoring 3D scale.
	// Multiplication order is as follow: world_position = qvvs_mul_point3_no_scale(local_position, local_to_world)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vector4f RTM_SIMD_CALL qvvs_mul_point3_no_scale(vector4f_arg0 point, qvvsf_arg1 qvvs) RTM_NO_EXCEPT
	{
		return vector_add(quat_mul_vector3(point, qvvs.rotation), qvvs.translation_uniform_scale);
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the inverse of the input QVVS transform.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline qvvsf RTM_SIMD_CALL qvvs_inverse(qvvsf_arg0 input) RTM_NO_EXCEPT
	{
		const quatf inv_rotation = quat_conjugate(input.rotation);
		const vector4f inv_uniform_scale_w = vector_reciprocal(input.translation_uniform_scale);
		const vector4f inv_uniform_scale = vector_dup_w(inv_uniform_scale_w);
		const vector4f inv_non_uniform_scale = vector_reciprocal(input.non_uniform_scale);
		const vector4f inv_scale = vector_mul(inv_uniform_scale, inv_non_uniform_scale);
		const vector4f inv_translation = vector_neg(quat_mul_vector3(vector_mul(input.translation_uniform_scale, inv_scale), inv_rotation));
		const vector4f inv_translation_uniform_scale = vector_mix<mix4::x, mix4::y, mix4::z, mix4::d>(inv_translation, inv_uniform_scale_w);

		return qvvsf{ inv_rotation, inv_translation_uniform_scale, inv_non_uniform_scale };
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the inverse of the input QVVS transform ignoring 3D scale.
	// The resulting QVVS transform will have the input scale.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline qvvsf RTM_SIMD_CALL qvvs_inverse_no_scale(qvvsf_arg0 input) RTM_NO_EXCEPT
	{
		const quatf inv_rotation = quat_conjugate(input.rotation);
		const vector4f inv_translation = vector_neg(quat_mul_vector3(input.translation_uniform_scale, inv_rotation));
		const vector4f inv_translation_uniform_scale = vector_mix<mix4::x, mix4::y, mix4::z, mix4::d>(inv_translation, input.translation_uniform_scale);

		return qvvsf{ inv_rotation, inv_translation_uniform_scale, input.non_uniform_scale };
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns a QVVS transforms with the rotation part normalized.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE qvvsf RTM_SIMD_CALL qvvs_normalize(qvvsf_arg0 input) RTM_NO_EXCEPT
	{
		const quatf rotation = quat_normalize(input.rotation);
		return qvvsf{ rotation, input.translation_uniform_scale, input.non_uniform_scale };
	}

	//////////////////////////////////////////////////////////////////////////
	// Per component linear interpolation of the two inputs at the specified alpha.
	// The formula used is: ((1.0 - alpha) * start) + (alpha * end).
	// Interpolation is stable and will return 'start' when alpha is 0.0 and 'end' when it is 1.0.
	// This is the same instruction count when FMA is present but it might be slightly slower
	// due to the extra multiplication compared to: start + (alpha * (end - start)).
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE qvvsf RTM_SIMD_CALL qvvs_lerp(qvvsf_arg0 start, qvvsf_arg1 end, float alpha) RTM_NO_EXCEPT
	{
		const quatf rotation = quat_lerp(start.rotation, end.rotation, alpha);
		const vector4f translation_uniform_scale = vector_lerp(start.translation_uniform_scale, end.translation_uniform_scale, alpha);
		const vector4f non_uniform_scale = vector_lerp(start.non_uniform_scale, end.non_uniform_scale, alpha);
		return qvvsf{ rotation, translation_uniform_scale, non_uniform_scale };
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Per component linear interpolation of the two inputs at the specified alpha.
	// The formula used is: ((1.0 - alpha) * start) + (alpha * end).
	// Interpolation is stable and will return 'start' when alpha is 0.0 and 'end' when it is 1.0.
	// This is the same instruction count when FMA is present but it might be slightly slower
	// due to the extra multiplication compared to: start + (alpha * (end - start)).
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE qvvsf RTM_SIMD_CALL qvvs_lerp(qvvsf_arg0 start, qvvsf_arg1 end, scalarf_arg2 alpha) RTM_NO_EXCEPT
	{
		const quatf rotation = quat_lerp(start.rotation, end.rotation, alpha);
		const vector4f translation_uniform_scale = vector_lerp(start.translation_uniform_scale, end.translation_uniform_scale, alpha);
		const vector4f non_uniform_scale = vector_lerp(start.non_uniform_scale, end.non_uniform_scale, alpha);
		return qvvsf{ rotation, translation_uniform_scale, non_uniform_scale };
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Per component linear interpolation of the two inputs at the specified alpha.
	// The formula used is: ((1.0 - alpha) * start) + (alpha * end).
	// Interpolation is stable and will return 'start' when alpha is 0.0 and 'end' when it is 1.0.
	// This is the same instruction count when FMA is present but it might be slightly slower
	// due to the extra multiplication compared to: start + (alpha * (end - start)).
	// The resulting QVV transform will have the start scale.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE qvvsf RTM_SIMD_CALL qvvs_lerp_no_scale(qvvsf_arg0 start, qvvsf_arg1 end, float alpha) RTM_NO_EXCEPT
	{
		const quatf rotation = quat_lerp(start.rotation, end.rotation, alpha);
		const vector4f translation_uniform_scale_lerp = vector_lerp(start.translation_uniform_scale, end.translation_uniform_scale, alpha);
		const vector4f translation_uniform_scale = vector_mix<mix4::x, mix4::y, mix4::z, mix4::d>(translation_uniform_scale_lerp, start.translation_uniform_scale);
		return qvvsf{ rotation, translation_uniform_scale, start.non_uniform_scale };
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Per component linear interpolation of the two inputs at the specified alpha.
	// The formula used is: ((1.0 - alpha) * start) + (alpha * end).
	// Interpolation is stable and will return 'start' when alpha is 0.0 and 'end' when it is 1.0.
	// This is the same instruction count when FMA is present but it might be slightly slower
	// due to the extra multiplication compared to: start + (alpha * (end - start)).
	// The resulting QVV transform will have the start scale.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE qvvsf RTM_SIMD_CALL qvvs_lerp_no_scale(qvvsf_arg0 start, qvvsf_arg1 end, scalarf_arg2 alpha) RTM_NO_EXCEPT
	{
		const quatf rotation = quat_lerp(start.rotation, end.rotation, alpha);
		const vector4f translation_uniform_scale_lerp = vector_lerp(start.translation_uniform_scale, end.translation_uniform_scale, alpha);
		const vector4f translation_uniform_scale = vector_mix<mix4::x, mix4::y, mix4::z, mix4::d>(translation_uniform_scale_lerp, start.translation_uniform_scale);
		return qvvsf{ rotation, translation_uniform_scale, start.non_uniform_scale };
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Per component spherical interpolation of the two inputs at the specified alpha.
	// See quat_slerp(..)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE qvvsf RTM_SIMD_CALL qvvs_slerp(qvvsf_arg0 start, qvvsf_arg1 end, float alpha) RTM_NO_EXCEPT
	{
		const quatf rotation = quat_slerp(start.rotation, end.rotation, alpha);
		const vector4f translation_uniform_scale = vector_lerp(start.translation_uniform_scale, end.translation_uniform_scale, alpha);
		const vector4f non_uniform_scale = vector_lerp(start.non_uniform_scale, end.non_uniform_scale, alpha);
		return qvvsf{ rotation, translation_uniform_scale, non_uniform_scale };
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Per component spherical interpolation of the two inputs at the specified alpha.
	// See quat_slerp(..)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE qvvsf RTM_SIMD_CALL qvvs_slerp(qvvsf_arg0 start, qvvsf_arg1 end, scalarf_arg2 alpha) RTM_NO_EXCEPT
	{
		const quatf rotation = quat_slerp(start.rotation, end.rotation, alpha);
		const vector4f translation_uniform_scale = vector_lerp(start.translation_uniform_scale, end.translation_uniform_scale, alpha);
		const vector4f non_uniform_scale = vector_lerp(start.non_uniform_scale, end.non_uniform_scale, alpha);
		return qvvsf{ rotation, translation_uniform_scale, non_uniform_scale };
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Per component spherical interpolation of the two inputs at the specified alpha.
	// See quat_slerp(..)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE qvvsf RTM_SIMD_CALL qvvs_slerp_no_scale(qvvsf_arg0 start, qvvsf_arg1 end, float alpha) RTM_NO_EXCEPT
	{
		const quatf rotation = quat_slerp(start.rotation, end.rotation, alpha);
		const vector4f translation_uniform_scale_lerp = vector_lerp(start.translation_uniform_scale, end.translation_uniform_scale, alpha);
		const vector4f translation_uniform_scale = vector_mix<mix4::x, mix4::y, mix4::z, mix4::d>(translation_uniform_scale_lerp, start.translation_uniform_scale);
		return qvvsf{ rotation, translation_uniform_scale, start.non_uniform_scale };
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Per component spherical interpolation of the two inputs at the specified alpha.
	// See quat_slerp(..)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE qvvsf RTM_SIMD_CALL qvvs_slerp_no_scale(qvvsf_arg0 start, qvvsf_arg1 end, scalarf_arg2 alpha) RTM_NO_EXCEPT
	{
		const quatf rotation = quat_slerp(start.rotation, end.rotation, alpha);
		const vector4f translation_uniform_scale_lerp = vector_lerp(start.translation_uniform_scale, end.translation_uniform_scale, alpha);
		const vector4f translation_uniform_scale = vector_mix<mix4::x, mix4::y, mix4::z, mix4::d>(translation_uniform_scale_lerp, start.translation_uniform_scale);
		return qvvsf{ rotation, translation_uniform_scale, start.non_uniform_scale };
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Returns true if the input QVVS does not contain any NaN or Inf, otherwise false.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL qvvs_is_finite(qvvsf_arg0 input) RTM_NO_EXCEPT
	{
		return quat_is_finite(input.rotation) && vector_is_finite(input.translation_uniform_scale) && vector_is_finite3(input.non_uniform_scale);
	}

	RTM_IMPL_VERSION_NAMESPACE_END
}

RTM_IMPL_FILE_PRAGMA_POP
