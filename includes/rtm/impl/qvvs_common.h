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
#include "rtm/version.h"
#include "rtm/impl/compiler_utils.h"

RTM_IMPL_FILE_PRAGMA_PUSH

namespace rtm
{
	RTM_IMPL_VERSION_NAMESPACE_BEGIN

	//////////////////////////////////////////////////////////////////////////
	// Creates a QVVS transform from a rotation quaternion, a translation, a scalar uniform scale, and a 3D non-uniform scale.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE qvvsf RTM_SIMD_CALL qvvs_set(quatf_arg0 rotation, vector4f_arg1 translation, float uniform_scale, vector4f_arg3 non_uniform_scale) RTM_NO_EXCEPT
	{
		const vector4f translation_uniform_scale = vector_set_w(translation, uniform_scale);
		return qvvsf{ rotation, translation_uniform_scale, non_uniform_scale };
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Creates a QVVS transform from a rotation quaternion, a translation, a scalar uniform scale, and a 3D non-uniform scale.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE qvvsf RTM_SIMD_CALL qvvs_set(quatf_arg0 rotation, vector4f_arg1 translation, scalarf_arg2 uniform_scale, vector4f_arg3 non_uniform_scale) RTM_NO_EXCEPT
	{
		const vector4f translation_uniform_scale = vector_set_w(translation, uniform_scale);
		return qvvsf{ rotation, translation_uniform_scale, non_uniform_scale };
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Creates a QVVS transform from a rotation quaternion, a translation, a scalar uniform scale, and a 3D non-uniform scale.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE qvvsd RTM_SIMD_CALL qvvs_set(const quatd& rotation, const vector4d& translation, double uniform_scale, const vector4d& non_uniform_scale) RTM_NO_EXCEPT
	{
		const vector4d translation_uniform_scale = vector_set_w(translation, uniform_scale);
		return qvvsd{ rotation, translation_uniform_scale, non_uniform_scale };
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Creates a QVVS transform from a rotation quaternion, a translation, a scalar uniform scale, and a 3D non-uniform scale.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE qvvsd RTM_SIMD_CALL qvvs_set(const quatd& rotation, const vector4d& translation, const scalard& uniform_scale, const vector4d& non_uniform_scale) RTM_NO_EXCEPT
	{
		const vector4d translation_uniform_scale = vector_set_w(translation, uniform_scale);
		return qvvsd{ rotation, translation_uniform_scale, non_uniform_scale };
	}
#endif

	namespace rtm_impl
	{
		//////////////////////////////////////////////////////////////////////////
		// This is a helper struct to allow a single consistent API between
		// various QVVS transform types when the semantics are identical but the return
		// type differs. Implicit coercion is used to return the desired value
		// at the call site.
		//////////////////////////////////////////////////////////////////////////
		struct qvvs_identity_impl
		{
			RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE RTM_SIMD_CALL operator qvvsd() const RTM_NO_EXCEPT
			{
				return qvvsd{ quat_identity(), vector_set(0.0, 0.0, 0.0, 1.0), vector_set(1.0) };
			}

			RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE RTM_SIMD_CALL operator qvvsf() const RTM_NO_EXCEPT
			{
				return qvvsf{ quat_identity(), vector_set(0.0F, 0.0F, 0.0F, 1.0F), vector_set(1.0F) };
			}
		};
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the identity QVVS transform.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE constexpr rtm_impl::qvvs_identity_impl RTM_SIMD_CALL qvvs_identity() RTM_NO_EXCEPT
	{
		return rtm_impl::qvvs_identity_impl();
	}

	RTM_IMPL_VERSION_NAMESPACE_END
}

RTM_IMPL_FILE_PRAGMA_POP
