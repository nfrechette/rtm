#pragma once

////////////////////////////////////////////////////////////////////////////////
// The MIT License (MIT)
//
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

#include "rtm/types.h"
#include "rtm/impl/compiler_utils.h"

#include <cstdint>
#include <cstring>

RTM_IMPL_FILE_PRAGMA_PUSH

namespace rtm
{
	//////////////////////////////////////////////////////////////////////////
	// Creates a vector4 from all 4 components.
	//////////////////////////////////////////////////////////////////////////
	inline vector4f RTM_SIMD_CALL vector_set(float x, float y, float z, float w) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_set_ps(w, z, y, x);
#elif defined(RTM_NEON_INTRINSICS)
#if 1
		float32x2_t V0 = vcreate_f32(((uint64_t)*(const uint32_t*)&x) | ((uint64_t)(*(const uint32_t*)&y) << 32));
		float32x2_t V1 = vcreate_f32(((uint64_t)*(const uint32_t*)&z) | ((uint64_t)(*(const uint32_t*)&w) << 32));
		return vcombine_f32(V0, V1);
#else
		float __attribute__((aligned(16))) data[4] = { x, y, z, w };
		return vld1q_f32(data);
#endif
#else
		return vector4f{ x, y, z, w };
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Creates a vector4 from the [xyz] components and sets [w] to 0.0.
	//////////////////////////////////////////////////////////////////////////
	inline vector4f RTM_SIMD_CALL vector_set(float x, float y, float z) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_set_ps(0.0f, z, y, x);
#elif defined(RTM_NEON_INTRINSICS)
#if 1
		float32x2_t V0 = vcreate_f32(((uint64_t)*(const uint32_t*)&x) | ((uint64_t)(*(const uint32_t*)&y) << 32));
		float32x2_t V1 = vcreate_f32((uint64_t)*(const uint32_t*)&z);
		return vcombine_f32(V0, V1);
#else
		float __attribute__((aligned(16))) data[4] = { x, y, z };
		return vld1q_f32(data);
#endif
#else
		return vector4f{ x, y, z, 0.0f };
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Creates a vector4 from a single value for all 4 components.
	//////////////////////////////////////////////////////////////////////////
	inline vector4f RTM_SIMD_CALL vector_set(float xyzw) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_set_ps1(xyzw);
#elif defined(RTM_NEON_INTRINSICS)
		return vdupq_n_f32(xyzw);
#else
		return vector4f{ xyzw, xyzw, xyzw, xyzw };
#endif
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Creates a vector4 from a single value for all 4 components.
	//////////////////////////////////////////////////////////////////////////
	inline vector4f RTM_SIMD_CALL vector_set(scalarf_arg0 xyzw) RTM_NO_EXCEPT
	{
		return _mm_shuffle_ps(xyzw, xyzw, _MM_SHUFFLE(0, 0, 0, 0));
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Creates a vector4 from all 4 components.
	//////////////////////////////////////////////////////////////////////////
	inline vector4d vector_set(double x, double y, double z, double w) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return vector4d{ _mm_set_pd(y, x), _mm_set_pd(w, z) };
#else
		return vector4d{ x, y, z, w };
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Creates a vector4 from the [xyz] components and sets [w] to 0.0.
	//////////////////////////////////////////////////////////////////////////
	inline vector4d vector_set(double x, double y, double z) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return vector4d{ _mm_set_pd(y, x), _mm_set_pd(0.0, z) };
#else
		return vector4d{ x, y, z, 0.0 };
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Creates a vector4 from a single value for all 4 components.
	//////////////////////////////////////////////////////////////////////////
	inline vector4d vector_set(double xyzw) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		const __m128d xyzw_pd = _mm_set1_pd(xyzw);
		return vector4d{ xyzw_pd, xyzw_pd };
#else
		return vector4d{ xyzw, xyzw, xyzw, xyzw };
#endif
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Creates a vector4 from a single value for all 4 components.
	//////////////////////////////////////////////////////////////////////////
	inline vector4d RTM_SIMD_CALL vector_set(scalard xyzw) RTM_NO_EXCEPT
	{
		const __m128d xyzw_pd = _mm_shuffle_pd(xyzw, xyzw, 0);
		return vector4d{ xyzw_pd, xyzw_pd };
	}
#endif

	namespace rtm_impl
	{
		//////////////////////////////////////////////////////////////////////////
		// Returns true if mix4 component is one of [xyzw]
		//////////////////////////////////////////////////////////////////////////
		constexpr bool is_mix_xyzw(mix4 arg) RTM_NO_EXCEPT { return uint32_t(arg) <= uint32_t(mix4::w); }

		//////////////////////////////////////////////////////////////////////////
		// Returns true if mix4 component is one of [abcd]
		//////////////////////////////////////////////////////////////////////////
		constexpr bool is_mix_abcd(mix4 arg) RTM_NO_EXCEPT { return uint32_t(arg) >= uint32_t(mix4::a); }

		//////////////////////////////////////////////////////////////////////////
		// This is a helper struct to help manipulate SIMD masks.
		//////////////////////////////////////////////////////////////////////////
		union mask_converter
		{
			uint64_t u64;
			uint32_t u32[2];

			constexpr mask_converter(uint64_t value) RTM_NO_EXCEPT : u64(value) {}

			constexpr operator uint32_t() const RTM_NO_EXCEPT { return u32[0]; }
			constexpr operator uint64_t() const RTM_NO_EXCEPT { return u64; }
		};

		//////////////////////////////////////////////////////////////////////////
		// Returns a SIMD mask value from a boolean.
		//////////////////////////////////////////////////////////////////////////
		constexpr mask_converter get_mask_value(bool is_true) RTM_NO_EXCEPT
		{
			return mask_converter(is_true ? uint64_t(0xFFFFFFFFFFFFFFFFull) : uint64_t(0));
		}

		//////////////////////////////////////////////////////////////////////////
		// Selects if_false if the SIMD mask value is 0, otherwise if_true.
		//////////////////////////////////////////////////////////////////////////
		constexpr double select(uint64_t mask, double if_true, double if_false) RTM_NO_EXCEPT
		{
			return mask == 0 ? if_false : if_true;
		}

		//////////////////////////////////////////////////////////////////////////
		// Selects if_false if the SIMD mask value is 0, otherwise if_true.
		//////////////////////////////////////////////////////////////////////////
		constexpr float select(uint32_t mask, float if_true, float if_false) RTM_NO_EXCEPT
		{
			return mask == 0 ? if_false : if_true;
		}

		//////////////////////////////////////////////////////////////////////////
		// Various vector constants
		//////////////////////////////////////////////////////////////////////////
		enum class vector_constants
		{
			zero
		};

		//////////////////////////////////////////////////////////////////////////
		// This is a helper struct to allow a single consistent API between
		// various vector types when the semantics are identical but the return
		// type differs. Implicit coercion is used to return the desired value
		// at the call site.
		//////////////////////////////////////////////////////////////////////////
		template<vector_constants constant>
		struct vector_constant
		{
			inline RTM_SIMD_CALL operator vector4d() const RTM_NO_EXCEPT
			{
				switch (constant)
				{
				case vector_constants::zero:
				default:
#if defined(RTM_SSE2_INTRINSICS)
					const __m128d zero_pd = _mm_setzero_pd();
					return vector4d{ zero_pd, zero_pd };
#else
					return vector_set(0.0);
#endif
				}
			}

			inline RTM_SIMD_CALL operator vector4f() const RTM_NO_EXCEPT
			{
				switch (constant)
				{
				case vector_constants::zero:
				default:
#if defined(RTM_SSE2_INTRINSICS)
					return _mm_setzero_ps();
#else
					return vector_set(0.0f);
#endif
				}
			}
		};

		//////////////////////////////////////////////////////////////////////////
		// Various vector widths we can load
		//////////////////////////////////////////////////////////////////////////
		enum class vector_unaligned_loader_width
		{
			vec1,
			vec2,
			vec3,
			vec4,
		};

		//////////////////////////////////////////////////////////////////////////
		// This is a helper struct to allow a single consistent API between
		// various vector types when the semantics are identical but the return
		// type differs. Implicit coercion is used to return the desired value
		// at the call site.
		//////////////////////////////////////////////////////////////////////////
		template<vector_unaligned_loader_width width>
		struct vector_unaligned_loader
		{
			inline RTM_SIMD_CALL operator vector4d() const RTM_NO_EXCEPT
			{
				switch (width)
				{
				case vector_unaligned_loader_width::vec1:
				{
					double data[1];
					std::memcpy(&data[0], ptr, sizeof(double) * 1);
					return vector_set(data[0]);
				}
				case vector_unaligned_loader_width::vec2:
				{
					double data[2];
					std::memcpy(&data[0], ptr, sizeof(double) * 2);
					return vector_set(data[0], data[1], 0.0, 0.0);
				}
				case vector_unaligned_loader_width::vec3:
				{
					double data[3];
					std::memcpy(&data[0], ptr, sizeof(double) * 3);
					return vector_set(data[0], data[1], data[2], 0.0);
				}
				case vector_unaligned_loader_width::vec4:
				default:
				{
					vector4d result;
					std::memcpy(&result, ptr, sizeof(vector4d));
					return result;
				}
				}
			}

			inline RTM_SIMD_CALL operator vector4f() const RTM_NO_EXCEPT
			{
				switch (width)
				{
				case vector_unaligned_loader_width::vec1:
				{
					float data[1];
					std::memcpy(&data[0], ptr, sizeof(float) * 1);
					return vector_set(data[0]);
				}
				case vector_unaligned_loader_width::vec2:
				{
					float data[2];
					std::memcpy(&data[0], ptr, sizeof(float) * 2);
					return vector_set(data[0], data[1], 0.0f, 0.0f);
				}
				case vector_unaligned_loader_width::vec3:
				{
					float data[3];
					std::memcpy(&data[0], ptr, sizeof(float) * 3);
					return vector_set(data[0], data[1], data[2], 0.0f);
				}
				case vector_unaligned_loader_width::vec4:
				default:
				{
#if defined(RTM_SSE2_INTRINSICS)
					return _mm_loadu_ps((const float*)ptr);
#elif defined(RTM_NEON_INTRINSICS)
					return vreinterpretq_f32_u8(vld1q_u8(ptr));
#else
					vector4f result;
					std::memcpy(&result, ptr, sizeof(vector4f));
					return result;
#endif
				}
				}
			}

			const uint8_t* ptr;
		};
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns a vector consisting of all zeros.
	//////////////////////////////////////////////////////////////////////////
	constexpr rtm_impl::vector_constant<rtm_impl::vector_constants::zero> RTM_SIMD_CALL vector_zero() RTM_NO_EXCEPT
	{
		return rtm_impl::vector_constant<rtm_impl::vector_constants::zero>();
	}

	//////////////////////////////////////////////////////////////////////////
	// Loads an unaligned vector4 from memory.
	//////////////////////////////////////////////////////////////////////////
	constexpr rtm_impl::vector_unaligned_loader<rtm_impl::vector_unaligned_loader_width::vec4> RTM_SIMD_CALL vector_load(const uint8_t* input) RTM_NO_EXCEPT
	{
		return rtm_impl::vector_unaligned_loader<rtm_impl::vector_unaligned_loader_width::vec4>{ input };
	}

	//////////////////////////////////////////////////////////////////////////
	// Loads an unaligned vector1 from memory and leaves the [yzw] components undefined.
	//////////////////////////////////////////////////////////////////////////
	constexpr rtm_impl::vector_unaligned_loader<rtm_impl::vector_unaligned_loader_width::vec1> RTM_SIMD_CALL vector_load1(const uint8_t* input) RTM_NO_EXCEPT
	{
		return rtm_impl::vector_unaligned_loader<rtm_impl::vector_unaligned_loader_width::vec1>{ input };
	}

	//////////////////////////////////////////////////////////////////////////
	// Loads an unaligned vector2 from memory and leaves the [zw] components undefined.
	//////////////////////////////////////////////////////////////////////////
	constexpr rtm_impl::vector_unaligned_loader<rtm_impl::vector_unaligned_loader_width::vec2> RTM_SIMD_CALL vector_load2(const uint8_t* input) RTM_NO_EXCEPT
	{
		return rtm_impl::vector_unaligned_loader<rtm_impl::vector_unaligned_loader_width::vec2>{ input };
	}

	//////////////////////////////////////////////////////////////////////////
	// Loads an unaligned vector3 from memory and leaves the [w] component undefined.
	//////////////////////////////////////////////////////////////////////////
	constexpr rtm_impl::vector_unaligned_loader<rtm_impl::vector_unaligned_loader_width::vec3> RTM_SIMD_CALL vector_load3(const uint8_t* input) RTM_NO_EXCEPT
	{
		return rtm_impl::vector_unaligned_loader<rtm_impl::vector_unaligned_loader_width::vec3>{ input };
	}
}

RTM_IMPL_FILE_PRAGMA_POP
