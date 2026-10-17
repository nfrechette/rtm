#pragma once

////////////////////////////////////////////////////////////////////////////////
// The MIT License (MIT)
//
// Copyright (c) 2024 Nicholas Frechette & Realtime Math contributors
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
#include "rtm/types.h"
#include "rtm/vector4d.h"
#include "rtm/vector4f.h"
#include "rtm/version.h"
#include "rtm/impl/compiler_utils.h"

#include <cstdint>
#include <cstring>

RTM_IMPL_FILE_PRAGMA_PUSH

namespace rtm
{
	RTM_IMPL_VERSION_NAMESPACE_BEGIN

	namespace rtm_impl
	{
		//////////////////////////////////////////////////////////////////////////
		// When the scale/shear part of a VQM transform is exactly the identity, the m22 slot can hold a flag.
		// Then the functions skip the scale/shear part. The other entries of the scale/shear part hold their
		// identity values: m00 = m11 = 1.0, and the entries that are not on the diagonal are 0.0.
		// When the m22 slot does not hold the flag, the scale/shear part can have any value, the identity too.
		//
		// The flag is a quiet NaN with a fixed payload. A valid scale/shear part is finite, thus m22 cannot have this value.
		// The hardware does not make this payload: the default NaN is 0x7FC00000 (ARM, WebAssembly) or 0xFFC00000 (x86).
		// The flag is a quiet NaN because x87 changes the bits of a signaling NaN.
		// A float32 to float64 conversion of the float32 flag gives the float64 flag, and the reverse conversion gives
		// the float32 flag. The functions do not use this because some targets do not keep the payload of a NaN.
		// The flag goes only through loads, stores, and shuffles, which keep its bits. The functions remove it before arithmetic.
		//////////////////////////////////////////////////////////////////////////
		constexpr uint32_t k_vqmf_identity_scale_shear_flag = 0x7FC00001U;
		constexpr uint64_t k_vqmd_identity_scale_shear_flag = 0x7FF8000020000000ULL;

		//////////////////////////////////////////////////////////////////////////
		// Returns the value of the flag for the m22 slot.
		//////////////////////////////////////////////////////////////////////////
		template<typename FloatType>
		FloatType vqm_identity_scale_shear_flag() RTM_NO_EXCEPT;

		template<>
		RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE float vqm_identity_scale_shear_flag<float>() RTM_NO_EXCEPT
		{
			const uint32_t bits = k_vqmf_identity_scale_shear_flag;
			float value;
			std::memcpy(&value, &bits, sizeof(float));
			return value;
		}

		template<>
		RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE double vqm_identity_scale_shear_flag<double>() RTM_NO_EXCEPT
		{
			const uint64_t bits = k_vqmd_identity_scale_shear_flag;
			double value;
			std::memcpy(&value, &bits, sizeof(double));
			return value;
		}

		//////////////////////////////////////////////////////////////////////////
		// Returns true if the m22 slot of the input holds the flag: the scale/shear part is exactly the identity.
		// The function compares the bits, not the value. A lane extraction keeps the bits of a NaN.
		//////////////////////////////////////////////////////////////////////////
		RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool vqm_has_identity_scale_shear(const vqmf& input) RTM_NO_EXCEPT
		{
			const float m22 = vector_get_w(input.translation_m22);
			uint32_t m22_bits;
			std::memcpy(&m22_bits, &m22, sizeof(uint32_t));
			return m22_bits == k_vqmf_identity_scale_shear_flag;
		}

		RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool vqm_has_identity_scale_shear(const vqmd& input) RTM_NO_EXCEPT
		{
			const double m22 = vector_get_w(input.translation_m22);
			uint64_t m22_bits;
			std::memcpy(&m22_bits, &m22, sizeof(uint64_t));
			return m22_bits == k_vqmd_identity_scale_shear_flag;
		}

		//////////////////////////////////////////////////////////////////////////
		// This is a helper struct to allow a single consistent API between
		// various VQM transform types when the semantics are identical but the return
		// type differs. Implicit coercion is used to return the desired value
		// at the call site.
		//////////////////////////////////////////////////////////////////////////
		struct vqm_identity_impl
		{
			RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE RTM_SIMD_CALL operator vqmd() const RTM_NO_EXCEPT
			{
				vqmd result;
				result.rotation = quat_identity();
				result.translation_m22 = vector_set(0.0, 0.0, 0.0, vqm_identity_scale_shear_flag<double>());
				result.x_axis_m20 = vector_set(1.0, 0.0, 0.0, 0.0);
				result.y_axis_m21 = vector_set(0.0, 1.0, 0.0, 0.0);

				return result;
			}

			RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE RTM_SIMD_CALL operator vqmf() const RTM_NO_EXCEPT
			{
				vqmf result;
				result.rotation = quat_identity();
				result.translation_m22 = vector_set(0.0F, 0.0F, 0.0F, vqm_identity_scale_shear_flag<float>());
				result.x_axis_m20 = vector_set(1.0F, 0.0F, 0.0F, 0.0F);
				result.y_axis_m21 = vector_set(0.0F, 1.0F, 0.0F, 0.0F);

				return result;
			}
		};
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the identity VQM transform.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE constexpr rtm_impl::vqm_identity_impl RTM_SIMD_CALL vqm_identity() RTM_NO_EXCEPT
	{
		return rtm_impl::vqm_identity_impl();
	}

	RTM_IMPL_VERSION_NAMESPACE_END
}

RTM_IMPL_FILE_PRAGMA_POP
