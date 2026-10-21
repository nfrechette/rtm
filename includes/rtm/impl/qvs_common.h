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
#include "rtm/impl/compiler_utils.h"
#include "rtm/impl/error.h"
#include "rtm/impl/vqs_common.h"

RTM_IMPL_FILE_PRAGMA_PUSH

// Deprecated in 2.4, to be removed in 2.5
// This header keeps the previous qvs names. Use rtm/impl/vqs_common.h instead.

namespace rtm
{
	RTM_IMPL_VERSION_NAMESPACE_BEGIN

	/// Creates a VQS transform from a rotation quaternion, a translation, and scalar scale.
	RTM_DEPRECATED("Use 'vqs_set' instead. To be removed in 2.5.")
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vqsf RTM_SIMD_CALL qvs_set(quatf_arg0 rotation, vector4f_arg1 translation, float scale) RTM_NO_EXCEPT
	{
		return vqs_set(rotation, translation, scale);
	}

#if defined(RTM_SSE2_INTRINSICS)
	/// Creates a VQS transform from a rotation quaternion, a translation, and scalar scale.
	RTM_DEPRECATED("Use 'vqs_set' instead. To be removed in 2.5.")
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vqsf RTM_SIMD_CALL qvs_set(quatf_arg0 rotation, vector4f_arg1 translation, scalarf_arg2 scale) RTM_NO_EXCEPT
	{
		return vqs_set(rotation, translation, scale);
	}
#endif

	/// Creates a VQS transform from a rotation quaternion, a translation, and scalar scale.
	RTM_DEPRECATED("Use 'vqs_set' instead. To be removed in 2.5.")
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vqsd RTM_SIMD_CALL qvs_set(quatd_arg0 rotation, vector4d_arg1 translation, double scale) RTM_NO_EXCEPT
	{
		return vqs_set(rotation, translation, scale);
	}

#if defined(RTM_SSE2_INTRINSICS)
	/// Creates a VQS transform from a rotation quaternion, a translation, and scalar scale.
	RTM_DEPRECATED("Use 'vqs_set' instead. To be removed in 2.5.")
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vqsd RTM_SIMD_CALL qvs_set(quatd_arg0 rotation, vector4d_arg1 translation, scalard_arg5 scale) RTM_NO_EXCEPT
	{
		return vqs_set(rotation, translation, scale);
	}
#endif

	/// Returns the identity VQS transform.
	RTM_DEPRECATED("Use 'vqs_identity' instead. To be removed in 2.5.")
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE constexpr rtm_impl::vqs_identity_impl RTM_SIMD_CALL qvs_identity() RTM_NO_EXCEPT
	{
		return vqs_identity();
	}

	RTM_IMPL_VERSION_NAMESPACE_END
}

RTM_IMPL_FILE_PRAGMA_POP
