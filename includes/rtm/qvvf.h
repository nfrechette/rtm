#pragma once

////////////////////////////////////////////////////////////////////////////////
// The MIT License (MIT)
//
// Copyright (c) 2026 Nicholas Frechette & Realtime Math contributors
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
#include "rtm/version.h"
#include "rtm/vqvf.h"
#include "rtm/impl/compiler_utils.h"
#include "rtm/impl/error.h"
#include "rtm/impl/qvv_common.h"

RTM_IMPL_FILE_PRAGMA_PUSH

// Deprecated in 2.4, to be removed in 2.5
// This header keeps the previous qvv names. Use rtm/vqvf.h instead.

namespace rtm
{
	RTM_IMPL_VERSION_NAMESPACE_BEGIN

	/// Casts a VQV transform float64 variant to a float32 variant.
	RTM_DEPRECATED("Use 'vqv_cast' instead. To be removed in 2.5.")
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vqvf RTM_SIMD_CALL qvv_cast(vqvd_arg0 input) RTM_NO_EXCEPT
	{
		return vqv_cast(input);
	}

	/// Converts a pure rotation 3x3 matrix into a VQV transform.
	RTM_DEPRECATED("Use 'vqv_from_matrix' instead. To be removed in 2.5.")
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vqvf RTM_SIMD_CALL qvv_from_matrix(matrix3x3f_arg0 input) RTM_NO_EXCEPT
	{
		return vqv_from_matrix(input);
	}

	/// Converts a 3x4 affine matrix into a VQV transform.
	RTM_DEPRECATED("Use 'vqv_from_matrix' instead. To be removed in 2.5.")
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vqvf RTM_SIMD_CALL qvv_from_matrix(matrix3x4f_arg0 input) RTM_NO_EXCEPT
	{
		return vqv_from_matrix(input);
	}

	/// Multiplies two VQV transforms.
	RTM_DEPRECATED("Use 'vqv_mul' instead. To be removed in 2.5.")
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vqvf RTM_SIMD_CALL qvv_mul(vqvf_arg0 lhs, vqvf_arg1 rhs) RTM_NO_EXCEPT
	{
		return vqv_mul(lhs, rhs);
	}

	/// Multiplies two VQV transforms ignoring 3D scale.
	RTM_DEPRECATED("Use 'vqv_mul_no_scale' instead. To be removed in 2.5.")
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vqvf RTM_SIMD_CALL qvv_mul_no_scale(vqvf_arg0 lhs, vqvf_arg1 rhs) RTM_NO_EXCEPT
	{
		return vqv_mul_no_scale(lhs, rhs);
	}

	/// Multiplies a VQV transform and a 3D point.
	RTM_DEPRECATED("Use 'vqv_mul_point3' instead. To be removed in 2.5.")
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vector4f RTM_SIMD_CALL qvv_mul_point3(vector4f_arg0 point, vqvf_arg1 vqv) RTM_NO_EXCEPT
	{
		return vqv_mul_point3(point, vqv);
	}

	/// Multiplies a VQV transform and a 3D point ignoring 3D scale.
	RTM_DEPRECATED("Use 'vqv_mul_point3_no_scale' instead. To be removed in 2.5.")
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vector4f RTM_SIMD_CALL qvv_mul_point3_no_scale(vector4f_arg0 point, vqvf_arg1 vqv) RTM_NO_EXCEPT
	{
		return vqv_mul_point3_no_scale(point, vqv);
	}

	/// Returns the inverse of the input VQV transform.
	RTM_DEPRECATED("Use 'vqv_inverse' instead. To be removed in 2.5.")
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vqvf RTM_SIMD_CALL qvv_inverse(vqvf_arg0 input) RTM_NO_EXCEPT
	{
		return vqv_inverse(input);
	}

	/// Returns the inverse of the input VQV transform with a fallback scale.
	RTM_DEPRECATED("Use 'vqv_inverse' instead. To be removed in 2.5.")
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vqvf RTM_SIMD_CALL qvv_inverse(vqvf_arg0 input, vector4f_arg3 fallback_scale, float threshold = 1.0E-8F) RTM_NO_EXCEPT
	{
		return vqv_inverse(input, fallback_scale, threshold);
	}

	/// Returns the inverse of the input VQV transform ignoring 3D scale.
	RTM_DEPRECATED("Use 'vqv_inverse_no_scale' instead. To be removed in 2.5.")
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vqvf RTM_SIMD_CALL qvv_inverse_no_scale(vqvf_arg0 input) RTM_NO_EXCEPT
	{
		return vqv_inverse_no_scale(input);
	}

	/// Returns a VQV transform with the rotation part normalized.
	RTM_DEPRECATED("Use 'vqv_normalize' instead. To be removed in 2.5.")
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vqvf RTM_SIMD_CALL qvv_normalize(vqvf_arg0 input) RTM_NO_EXCEPT
	{
		return vqv_normalize(input);
	}

	/// Per component linear interpolation of the two inputs at the specified alpha.
	RTM_DEPRECATED("Use 'vqv_lerp' instead. To be removed in 2.5.")
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vqvf RTM_SIMD_CALL qvv_lerp(vqvf_arg0 start, vqvf_arg1 end, float alpha) RTM_NO_EXCEPT
	{
		return vqv_lerp(start, end, alpha);
	}

#if defined(RTM_SSE2_INTRINSICS)
	/// Per component linear interpolation of the two inputs at the specified alpha.
	RTM_DEPRECATED("Use 'vqv_lerp' instead. To be removed in 2.5.")
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vqvf RTM_SIMD_CALL qvv_lerp(vqvf_arg0 start, vqvf_arg1 end, scalarf_arg2 alpha) RTM_NO_EXCEPT
	{
		return vqv_lerp(start, end, alpha);
	}
#endif

	/// Per component linear interpolation of the two inputs at the specified alpha, without the scale.
	RTM_DEPRECATED("Use 'vqv_lerp_no_scale' instead. To be removed in 2.5.")
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vqvf RTM_SIMD_CALL qvv_lerp_no_scale(vqvf_arg0 start, vqvf_arg1 end, float alpha) RTM_NO_EXCEPT
	{
		return vqv_lerp_no_scale(start, end, alpha);
	}

#if defined(RTM_SSE2_INTRINSICS)
	/// Per component linear interpolation of the two inputs at the specified alpha, without the scale.
	RTM_DEPRECATED("Use 'vqv_lerp_no_scale' instead. To be removed in 2.5.")
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vqvf RTM_SIMD_CALL qvv_lerp_no_scale(vqvf_arg0 start, vqvf_arg1 end, scalarf_arg2 alpha) RTM_NO_EXCEPT
	{
		return vqv_lerp_no_scale(start, end, alpha);
	}
#endif

	/// Per component spherical interpolation of the two inputs at the specified alpha.
	RTM_DEPRECATED("Use 'vqv_slerp' instead. To be removed in 2.5.")
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vqvf RTM_SIMD_CALL qvv_slerp(vqvf_arg0 start, vqvf_arg1 end, float alpha) RTM_NO_EXCEPT
	{
		return vqv_slerp(start, end, alpha);
	}

#if defined(RTM_SSE2_INTRINSICS)
	/// Per component spherical interpolation of the two inputs at the specified alpha.
	RTM_DEPRECATED("Use 'vqv_slerp' instead. To be removed in 2.5.")
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vqvf RTM_SIMD_CALL qvv_slerp(vqvf_arg0 start, vqvf_arg1 end, scalarf_arg2 alpha) RTM_NO_EXCEPT
	{
		return vqv_slerp(start, end, alpha);
	}
#endif

	/// Per component spherical interpolation of the two inputs at the specified alpha, without the scale.
	RTM_DEPRECATED("Use 'vqv_slerp_no_scale' instead. To be removed in 2.5.")
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vqvf RTM_SIMD_CALL qvv_slerp_no_scale(vqvf_arg0 start, vqvf_arg1 end, float alpha) RTM_NO_EXCEPT
	{
		return vqv_slerp_no_scale(start, end, alpha);
	}

#if defined(RTM_SSE2_INTRINSICS)
	/// Per component spherical interpolation of the two inputs at the specified alpha, without the scale.
	RTM_DEPRECATED("Use 'vqv_slerp_no_scale' instead. To be removed in 2.5.")
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vqvf RTM_SIMD_CALL qvv_slerp_no_scale(vqvf_arg0 start, vqvf_arg1 end, scalarf_arg2 alpha) RTM_NO_EXCEPT
	{
		return vqv_slerp_no_scale(start, end, alpha);
	}
#endif

	/// Returns true if the input VQV does not contain any NaN or Inf, otherwise false.
	RTM_DEPRECATED("Use 'vqv_is_finite' instead. To be removed in 2.5.")
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL qvv_is_finite(vqvf_arg0 input) RTM_NO_EXCEPT
	{
		return vqv_is_finite(input);
	}

	RTM_IMPL_VERSION_NAMESPACE_END
}

RTM_IMPL_FILE_PRAGMA_POP
