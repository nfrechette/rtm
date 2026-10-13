#pragma once

////////////////////////////////////////////////////////////////////////////////
// The MIT License (MIT)
//
// Copyright (c) 2017 Nicholas Frechette & Animation Compression Library contributors
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

#include "rtm/math.h"
#include "rtm/scalard.h"
#include "rtm/version.h"
#include "rtm/impl/bit_cast.impl.h"
#include "rtm/impl/compiler_utils.h"
#include "rtm/impl/macros.mask4.impl.h"
#include "rtm/impl/memory_utils.h"
#include "rtm/impl/vector_common.h"

#include <cstring>
#include <limits>

RTM_IMPL_FILE_PRAGMA_PUSH

namespace rtm
{
	RTM_IMPL_VERSION_NAMESPACE_BEGIN

	//////////////////////////////////////////////////////////////////////////
	// Setters, getters, and casts
	//////////////////////////////////////////////////////////////////////////


	//////////////////////////////////////////////////////////////////////////
	// Loads an unaligned vector4 from memory.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_load(const double* input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return vector4d{ _mm_loadu_pd(input), _mm_loadu_pd(input + 2) };
#elif defined(RTM_WASM128_INTRINSICS)
		return vector4d{ wasm_v128_load(input), wasm_v128_load(input + 2) };
#elif defined(RTM_NEON64_INTRINSICS)
		return vector4d{ vld1q_f64(input), vld1q_f64(input + 2) };
#else
		return vector_set(input[0], input[1], input[2], input[3]);
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Loads an input scalar from memory into the [x] component and sets the [yzw] components to zero.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_load1(const double* input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		const __m128d zero = _mm_setzero_pd();
		return vector4d{ _mm_loadl_pd(zero, input), zero };
#elif defined(RTM_WASM128_INTRINSICS)
		const v128_t zero = wasm_f64x2_splat(0.0);
		return vector4d{ wasm_v128_load64_lane(input, zero, 0), zero };
#elif defined(RTM_NEON64_INTRINSICS)
		const float64x2_t zero = vdupq_n_f64(0.0);
		return vector4d{ vld1q_lane_f64(input, zero, 0), zero };
#else
		return vector_set(input[0], 0.0, 0.0, 0.0);
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Loads an unaligned vector2 from memory and sets the [zw] components to zero.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_load2(const double* input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return vector4d{ _mm_loadu_pd(input), _mm_setzero_pd() };
#elif defined(RTM_WASM128_INTRINSICS)
		return vector4d{ wasm_v128_load(input), wasm_f64x2_splat(0.0) };
#elif defined(RTM_NEON64_INTRINSICS)
		return vector4d{ vld1q_f64(input), vdupq_n_f64(0.0) };
#else
		return vector_set(input[0], input[1], 0.0, 0.0);
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Loads an unaligned vector3 from memory and sets the [w] component to zero.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_load3(const double* input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		const __m128d zero = _mm_setzero_pd();
		return vector4d{ _mm_loadu_pd(input), _mm_loadl_pd(zero, input + 2) };
#elif defined(RTM_WASM128_INTRINSICS)
		const v128_t zero = wasm_f64x2_splat(0.0);
		return vector4d{ wasm_v128_load(input), wasm_v128_load64_lane(input + 2, zero, 0) };
#elif defined(RTM_NEON64_INTRINSICS)
		const float64x2_t zero = vdupq_n_f64(0.0);
		return vector4d{ vld1q_f64(input), vld1q_lane_f64(input + 2, zero, 0) };
#else
		return vector_set(input[0], input[1], input[2], 0.0);
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Loads an unaligned vector4 from memory.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_load(const float4d* input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return vector4d{ _mm_loadu_pd(&input->x), _mm_loadu_pd(&input->z) };
#elif defined(RTM_WASM128_INTRINSICS)
		return vector4d{ wasm_v128_load(&input->x), wasm_v128_load(&input->z) };
#elif defined(RTM_NEON64_INTRINSICS)
		return vector4d{ vld1q_f64(&input->x), vld1q_f64(&input->z) };
#else
		return vector_set(input->x, input->y, input->z, input->w);
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Loads an unaligned vector2 from memory and sets the [zw] components to zero.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_load2(const float2d* input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return vector4d{ _mm_loadu_pd(&input->x), _mm_setzero_pd() };
#elif defined(RTM_WASM128_INTRINSICS)
		return vector4d{ wasm_v128_load(&input->x), wasm_f64x2_splat(0.0) };
#elif defined(RTM_NEON64_INTRINSICS)
		return vector4d{ vld1q_f64(&input->x), vdupq_n_f64(0.0) };
#else
		return vector_set(input->x, input->y, 0.0, 0.0);
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Loads an unaligned vector3 from memory and sets the [w] component to zero.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_load3(const float3d* input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		const __m128d zero = _mm_setzero_pd();
		return vector4d{ _mm_loadu_pd(&input->x), _mm_loadl_pd(zero, &input->z) };
#elif defined(RTM_WASM128_INTRINSICS)
		const v128_t zero = wasm_f64x2_splat(0.0);
		return vector4d{ wasm_v128_load(&input->x), wasm_v128_load64_lane(&input->z, zero, 0) };
#elif defined(RTM_NEON64_INTRINSICS)
		const float64x2_t zero = vdupq_n_f64(0.0);
		return vector4d{ vld1q_f64(&input->x), vld1q_lane_f64(&input->z, zero, 0) };
#else
		return vector_set(input->x, input->y, input->z, 0.0);
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Loads an input scalar from memory into the [xyzw] components.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_broadcast(const double* input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		const __m128d value = _mm_load1_pd(input);
		return vector4d{ value, value };
#elif defined(RTM_WASM128_INTRINSICS)
		const v128_t value = wasm_v128_load64_splat(input);
		return vector4d{ value, value };
#elif defined(RTM_NEON64_INTRINSICS)
		const float64x2_t value = vld1q_dup_f64(input);
		return vector4d{ value, value };
#else
		return vector_set(*input);
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Casts a quaternion to a vector4.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL quat_to_vector(quatd_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return vector4d{ input.xy, input.zw };
#elif defined(RTM_WASM128_INTRINSICS)
		return vector4d{ input.xy, input.zw };
#elif defined(RTM_NEON64_INTRINSICS)
		return vector4d{ input.xy, input.zw };
#else
		return vector4d{ input.x, input.y, input.z, input.w };
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Casts a vector4 float32 variant to a float64 variant.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_cast(vector4f_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return vector4d{ _mm_cvtps_pd(input), _mm_cvtps_pd(_mm_shuffle_ps(input, input, _MM_SHUFFLE(3, 2, 3, 2))) };
#elif defined(RTM_WASM128_INTRINSICS)
		return vector4d{ wasm_f64x2_promote_low_f32x4(input), wasm_f64x2_promote_low_f32x4(wasm_i32x4_shuffle(input, input, 2, 3, 2, 3)) };
#elif defined(RTM_NEON64_INTRINSICS)
		return vector4d{ vcvt_f64_f32(vget_low_f32(input)), vcvt_high_f64_f32(input) };
#elif defined(RTM_NEON_INTRINSICS)
		return vector4d{ double(vgetq_lane_f32(input, 0)), double(vgetq_lane_f32(input, 1)), double(vgetq_lane_f32(input, 2)), double(vgetq_lane_f32(input, 3)) };
#else
		return vector4d{ double(input.x), double(input.y), double(input.z), double(input.w) };
#endif
	}

	namespace rtm_impl
	{
		//////////////////////////////////////////////////////////////////////////
		// This is a helper struct to allow a single consistent API between
		// various vector types when the semantics are identical but the return
		// type differs. Implicit coercion is used to return the desired value
		// at the call site.
		//////////////////////////////////////////////////////////////////////////
		struct vector4d_vector_get_x
		{
			RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE RTM_SIMD_CALL operator double() const RTM_NO_EXCEPT
			{
#if defined(RTM_SSE2_INTRINSICS)
				return _mm_cvtsd_f64(input.xy);
#elif defined(RTM_WASM128_INTRINSICS)
				return wasm_f64x2_extract_lane(input.xy, 0);
#elif defined(RTM_NEON64_INTRINSICS)
				return vgetq_lane_f64(input.xy, 0);
#else
				return input.x;
#endif
			}

#if defined(RTM_SSE2_INTRINSICS)
			RTM_DEPRECATED("Use 'as_scalar' suffix instead. To be removed in 2.4.")
			RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE RTM_SIMD_CALL operator scalard() const RTM_NO_EXCEPT
			{
				return scalard{ input.xy };
			}
#endif

			vector4d input;
		};
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the vector4 [x] component.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE constexpr rtm_impl::vector4d_vector_get_x RTM_SIMD_CALL vector_get_x(vector4d_arg0 input) RTM_NO_EXCEPT
	{
		return rtm_impl::vector4d_vector_get_x{ input };
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the vector4 [x] component.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE scalard RTM_SIMD_CALL vector_get_x_as_scalar(vector4d_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return scalard{ input.xy };
#else
		return vector_get_x(input);
#endif
	}

	namespace rtm_impl
	{
		//////////////////////////////////////////////////////////////////////////
		// This is a helper struct to allow a single consistent API between
		// various vector types when the semantics are identical but the return
		// type differs. Implicit coercion is used to return the desired value
		// at the call site.
		//////////////////////////////////////////////////////////////////////////
		struct vector4d_vector_get_y
		{
			RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE RTM_SIMD_CALL operator double() const RTM_NO_EXCEPT
			{
#if defined(RTM_SSE2_INTRINSICS)
				return _mm_cvtsd_f64(_mm_shuffle_pd(input.xy, input.xy, _MM_SHUFFLE2(0, 1)));
#elif defined(RTM_WASM128_INTRINSICS)
				return wasm_f64x2_extract_lane(input.xy, 1);
#elif defined(RTM_NEON64_INTRINSICS)
				return vgetq_lane_f64(input.xy, 1);
#else
				return input.y;
#endif
			}

#if defined(RTM_SSE2_INTRINSICS)
			RTM_DEPRECATED("Use 'as_scalar' suffix instead. To be removed in 2.4.")
			RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE RTM_SIMD_CALL operator scalard() const RTM_NO_EXCEPT
			{
				return scalard{ _mm_shuffle_pd(input.xy, input.xy, _MM_SHUFFLE2(0, 1)) };
			}
#endif

			vector4d input;
		};
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the vector4 [y] component.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE constexpr rtm_impl::vector4d_vector_get_y RTM_SIMD_CALL vector_get_y(vector4d_arg0 input) RTM_NO_EXCEPT
	{
		return rtm_impl::vector4d_vector_get_y{ input };
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the vector4 [y] component.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE scalard RTM_SIMD_CALL vector_get_y_as_scalar(vector4d_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return scalard{ _mm_shuffle_pd(input.xy, input.xy, _MM_SHUFFLE2(0, 1)) };
#else
		return vector_get_y(input);
#endif
	}

	namespace rtm_impl
	{
		//////////////////////////////////////////////////////////////////////////
		// This is a helper struct to allow a single consistent API between
		// various vector types when the semantics are identical but the return
		// type differs. Implicit coercion is used to return the desired value
		// at the call site.
		//////////////////////////////////////////////////////////////////////////
		struct vector4d_vector_get_z
		{
			RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE RTM_SIMD_CALL operator double() const RTM_NO_EXCEPT
			{
#if defined(RTM_SSE2_INTRINSICS)
				return _mm_cvtsd_f64(input.zw);
#elif defined(RTM_WASM128_INTRINSICS)
				return wasm_f64x2_extract_lane(input.zw, 0);
#elif defined(RTM_NEON64_INTRINSICS)
				return vgetq_lane_f64(input.zw, 0);
#else
				return input.z;
#endif
			}

#if defined(RTM_SSE2_INTRINSICS)
			RTM_DEPRECATED("Use 'as_scalar' suffix instead. To be removed in 2.4.")
			RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE RTM_SIMD_CALL operator scalard() const RTM_NO_EXCEPT
			{
				return scalard{ input.zw };
			}
#endif

			vector4d input;
		};
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the vector4 [z] component.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE constexpr rtm_impl::vector4d_vector_get_z RTM_SIMD_CALL vector_get_z(vector4d_arg0 input) RTM_NO_EXCEPT
	{
		return rtm_impl::vector4d_vector_get_z{ input };
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the vector4 [z] component.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE scalard RTM_SIMD_CALL vector_get_z_as_scalar(vector4d_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return scalard{ input.zw };
#else
		return vector_get_z(input);
#endif
	}

	namespace rtm_impl
	{
		//////////////////////////////////////////////////////////////////////////
		// This is a helper struct to allow a single consistent API between
		// various vector types when the semantics are identical but the return
		// type differs. Implicit coercion is used to return the desired value
		// at the call site.
		//////////////////////////////////////////////////////////////////////////
		struct vector4d_vector_get_w
		{
			RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE RTM_SIMD_CALL operator double() const RTM_NO_EXCEPT
			{
#if defined(RTM_SSE2_INTRINSICS)
				return _mm_cvtsd_f64(_mm_shuffle_pd(input.zw, input.zw, _MM_SHUFFLE2(0, 1)));
#elif defined(RTM_WASM128_INTRINSICS)
				return wasm_f64x2_extract_lane(input.zw, 1);
#elif defined(RTM_NEON64_INTRINSICS)
				return vgetq_lane_f64(input.zw, 1);
#else
				return input.w;
#endif
			}

#if defined(RTM_SSE2_INTRINSICS)
			RTM_DEPRECATED("Use 'as_scalar' suffix instead. To be removed in 2.4.")
			RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE RTM_SIMD_CALL operator scalard() const RTM_NO_EXCEPT
			{
				return scalard{ _mm_shuffle_pd(input.zw, input.zw, _MM_SHUFFLE2(0, 1)) };
			}
#endif

			vector4d input;
		};
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the vector4 [w] component.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE constexpr rtm_impl::vector4d_vector_get_w RTM_SIMD_CALL vector_get_w(vector4d_arg0 input) RTM_NO_EXCEPT
	{
		return rtm_impl::vector4d_vector_get_w{ input };
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the vector4 [w] component.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE scalard RTM_SIMD_CALL vector_get_w_as_scalar(vector4d_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return scalard{ _mm_shuffle_pd(input.zw, input.zw, _MM_SHUFFLE2(0, 1)) };
#else
		return vector_get_w(input);
#endif
	}

	namespace rtm_impl
	{
		//////////////////////////////////////////////////////////////////////////
		// This is a helper struct to allow a single consistent API between
		// various vector types when the semantics are identical but the return
		// type differs. Implicit coercion is used to return the desired value
		// at the call site.
		//////////////////////////////////////////////////////////////////////////
		template<component4 component>
		struct vector4d_vector_get_component_static
		{
			RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE RTM_SIMD_CALL operator double() const RTM_NO_EXCEPT
			{
				switch (component)
				{
					default:
					case component4::x:	return vector_get_x(input);
					case component4::y:	return vector_get_y(input);
					case component4::z:	return vector_get_z(input);
					case component4::w:	return vector_get_w(input);
				}
			}

#if defined(RTM_SSE2_INTRINSICS)
			RTM_DEPRECATED("Use 'as_scalar' suffix instead. To be removed in 2.4.")
			RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE RTM_SIMD_CALL operator scalard() const RTM_NO_EXCEPT
			{
				switch (component)
				{
					default:
					case component4::x:	return vector_get_x_as_scalar(input);
					case component4::y:	return vector_get_y_as_scalar(input);
					case component4::z:	return vector_get_z_as_scalar(input);
					case component4::w:	return vector_get_w_as_scalar(input);
				}
			}
#endif

			vector4d input;
		};
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the vector2 desired component.
	//////////////////////////////////////////////////////////////////////////
	template<component2 component, component4 component_ = static_cast<component4>(component)>
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE constexpr rtm_impl::vector4d_vector_get_component_static<component_> RTM_SIMD_CALL vector_get_component2(vector4d_arg0 input) RTM_NO_EXCEPT
	{
		return rtm_impl::vector4d_vector_get_component_static<component_>{ input };
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the vector2 desired component.
	//////////////////////////////////////////////////////////////////////////
	template<component2 component>
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE scalard RTM_SIMD_CALL vector_get_component2_as_scalar(vector4d_arg0 input) RTM_NO_EXCEPT
	{
		switch (component)
		{
			default:
			case component2::x:	return vector_get_x_as_scalar(input);
			case component2::y:	return vector_get_y_as_scalar(input);
		}
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the vector3 desired component.
	//////////////////////////////////////////////////////////////////////////
	template<component3 component, component4 component_ = static_cast<component4>(component)>
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE constexpr rtm_impl::vector4d_vector_get_component_static<component_> RTM_SIMD_CALL vector_get_component3(vector4d_arg0 input) RTM_NO_EXCEPT
	{
		return rtm_impl::vector4d_vector_get_component_static<component_>{ input };
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the vector3 desired component.
	//////////////////////////////////////////////////////////////////////////
	template<component3 component>
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE scalard RTM_SIMD_CALL vector_get_component3_as_scalar(vector4d_arg0 input) RTM_NO_EXCEPT
	{
		switch (component)
		{
			default:
			case component3::x:	return vector_get_x_as_scalar(input);
			case component3::y:	return vector_get_y_as_scalar(input);
			case component3::z:	return vector_get_z_as_scalar(input);
		}
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the vector4 desired component.
	//////////////////////////////////////////////////////////////////////////
	template<component4 component>
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE constexpr rtm_impl::vector4d_vector_get_component_static<component> RTM_SIMD_CALL vector_get_component(vector4d_arg0 input) RTM_NO_EXCEPT
	{
		return rtm_impl::vector4d_vector_get_component_static<component>{ input };
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the vector4 desired component.
	//////////////////////////////////////////////////////////////////////////
	template<component4 component>
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE scalard RTM_SIMD_CALL vector_get_component_as_scalar(vector4d_arg0 input) RTM_NO_EXCEPT
	{
		switch (component)
		{
			default:
			case component4::x:	return vector_get_x_as_scalar(input);
			case component4::y:	return vector_get_y_as_scalar(input);
			case component4::z:	return vector_get_z_as_scalar(input);
			case component4::w:	return vector_get_w_as_scalar(input);
		}
	}

	namespace rtm_impl
	{
		//////////////////////////////////////////////////////////////////////////
		// This is a helper struct to allow a single consistent API between
		// various vector types when the semantics are identical but the return
		// type differs. Implicit coercion is used to return the desired value
		// at the call site.
		//////////////////////////////////////////////////////////////////////////
		struct vector4d_vector_get_component
		{
			RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE RTM_SIMD_CALL operator double() const RTM_NO_EXCEPT
			{
				switch (component)
				{
					default:
					case component4::x:	return vector_get_x(input);
					case component4::y:	return vector_get_y(input);
					case component4::z:	return vector_get_z(input);
					case component4::w:	return vector_get_w(input);
				}
			}

#if defined(RTM_SSE2_INTRINSICS)
			RTM_DEPRECATED("Use 'as_scalar' suffix instead. To be removed in 2.4.")
			RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE RTM_SIMD_CALL operator scalard() const RTM_NO_EXCEPT
			{
				switch (component)
				{
					default:
					case component4::x:	return vector_get_x_as_scalar(input);
					case component4::y:	return vector_get_y_as_scalar(input);
					case component4::z:	return vector_get_z_as_scalar(input);
					case component4::w:	return vector_get_w_as_scalar(input);
				}
			}
#endif

			vector4d input;
			component4 component;
			int32_t padding[3];
		};
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the vector2 desired component.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE constexpr rtm_impl::vector4d_vector_get_component RTM_SIMD_CALL vector_get_component2(vector4d_arg0 input, component2 component) RTM_NO_EXCEPT
	{
		return rtm_impl::vector4d_vector_get_component{ input, static_cast<component4>(component), { 0 } };
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the vector2 desired component.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE scalard RTM_SIMD_CALL vector_get_component2_as_scalar(vector4d_arg0 input, component2 component) RTM_NO_EXCEPT
	{
		switch (component)
		{
			default:
			case component2::x:	return vector_get_x_as_scalar(input);
			case component2::y:	return vector_get_y_as_scalar(input);
		}
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the vector3 desired component.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE constexpr rtm_impl::vector4d_vector_get_component RTM_SIMD_CALL vector_get_component3(vector4d_arg0 input, component3 component) RTM_NO_EXCEPT
	{
		return rtm_impl::vector4d_vector_get_component{ input, static_cast<component4>(component), { 0 } };
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the vector3 desired component.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE scalard RTM_SIMD_CALL vector_get_component3_as_scalar(vector4d_arg0 input, component3 component) RTM_NO_EXCEPT
	{
		switch (component)
		{
			default:
			case component3::x:	return vector_get_x_as_scalar(input);
			case component3::y:	return vector_get_y_as_scalar(input);
			case component3::z:	return vector_get_z_as_scalar(input);
		}
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the vector4 desired component.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE constexpr rtm_impl::vector4d_vector_get_component RTM_SIMD_CALL vector_get_component(vector4d_arg0 input, component4 component) RTM_NO_EXCEPT
	{
		return rtm_impl::vector4d_vector_get_component{ input, component, { 0 } };
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the vector4 desired component.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE scalard RTM_SIMD_CALL vector_get_component_as_scalar(vector4d_arg0 input, component4 component) RTM_NO_EXCEPT
	{
		switch (component)
		{
			default:
			case component4::x:	return vector_get_x_as_scalar(input);
			case component4::y:	return vector_get_y_as_scalar(input);
			case component4::z:	return vector_get_z_as_scalar(input);
			case component4::w:	return vector_get_w_as_scalar(input);
		}
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the smallest component in the input vector as a scalar.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE constexpr rtm_impl::vector4d_get_min_component RTM_SIMD_CALL vector_get_min_component(vector4d_arg0 input) RTM_NO_EXCEPT
	{
		return rtm_impl::vector4d_get_min_component{ input };
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the smallest component in the input vector as a scalar.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE scalard RTM_SIMD_CALL vector_get_min_component_as_scalar(vector4d_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xz_yw = _mm_min_pd(input.xy, input.zw);
		__m128d yw_yw = _mm_shuffle_pd(xz_yw, xz_yw, _MM_SHUFFLE2(0, 1));
		return scalard{ _mm_min_pd(xz_yw, yw_yw) };
#else
		return vector_get_min_component(input);
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the largest component in the input vector as a scalar.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE constexpr rtm_impl::vector4d_get_max_component RTM_SIMD_CALL vector_get_max_component(vector4d_arg0 input) RTM_NO_EXCEPT
	{
		return rtm_impl::vector4d_get_max_component{ input };
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the largest component in the input vector as a scalar.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE scalard RTM_SIMD_CALL vector_get_max_component_as_scalar(vector4d_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xz_yw = _mm_max_pd(input.xy, input.zw);
		__m128d yw_yw = _mm_shuffle_pd(xz_yw, xz_yw, _MM_SHUFFLE2(0, 1));
		return scalard{ _mm_max_pd(xz_yw, yw_yw) };
#else
		return vector_get_max_component(input);
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Sets the vector4 [x] component and returns the new value.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_set_x(vector4d_arg0 input, double lane_value) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return vector4d{ _mm_move_sd(input.xy, _mm_set_sd(lane_value)), input.zw };
#elif defined(RTM_WASM128_INTRINSICS)
		return vector4d{ wasm_f64x2_replace_lane(input.xy, 0, lane_value), input.zw };
#elif defined(RTM_NEON64_INTRINSICS)
		return vector4d{ vsetq_lane_f64(lane_value, input.xy, 0), input.zw };
#else
		return vector4d{ lane_value, input.y, input.z, input.w };
#endif
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Sets the vector4 [x] component and returns the new value.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_set_x(vector4d_arg0 input, scalard_arg2 lane_value) RTM_NO_EXCEPT
	{
		return vector4d{ _mm_move_sd(input.xy, lane_value.value), input.zw };
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Sets the vector4 [y] component and returns the new value.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_set_y(vector4d_arg0 input, double lane_value) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return vector4d{ _mm_shuffle_pd(input.xy, _mm_set_sd(lane_value), _MM_SHUFFLE2(0, 0)), input.zw };
#elif defined(RTM_WASM128_INTRINSICS)
		return vector4d{ wasm_f64x2_replace_lane(input.xy, 1, lane_value), input.zw };
#elif defined(RTM_NEON64_INTRINSICS)
		return vector4d{ vsetq_lane_f64(lane_value, input.xy, 1), input.zw };
#else
		return vector4d{ input.x, lane_value, input.z, input.w };
#endif
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Sets the vector4 [y] component and returns the new value.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_set_y(vector4d_arg0 input, scalard_arg2 lane_value) RTM_NO_EXCEPT
	{
		return vector4d{ _mm_shuffle_pd(input.xy, lane_value.value, _MM_SHUFFLE2(0, 0)), input.zw };
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Sets the vector4 [z] component and returns the new value.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_set_z(vector4d_arg0 input, double lane_value) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return vector4d{ input.xy, _mm_move_sd(input.zw, _mm_set_sd(lane_value)) };
#elif defined(RTM_WASM128_INTRINSICS)
		return vector4d{ input.xy, wasm_f64x2_replace_lane(input.zw, 0, lane_value) };
#elif defined(RTM_NEON64_INTRINSICS)
		return vector4d{ input.xy, vsetq_lane_f64(lane_value, input.zw, 0) };
#else
		return vector4d{ input.x, input.y, lane_value, input.w };
#endif
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Sets the vector4 [z] component and returns the new value.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_set_z(vector4d_arg0 input, scalard_arg2 lane_value) RTM_NO_EXCEPT
	{
		return vector4d{ input.xy, _mm_move_sd(input.zw, lane_value.value) };
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Sets the vector4 [w] component and returns the new value.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_set_w(vector4d_arg0 input, double lane_value) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return vector4d{ input.xy, _mm_shuffle_pd(input.zw, _mm_set_sd(lane_value), _MM_SHUFFLE2(0, 0)) };
#elif defined(RTM_WASM128_INTRINSICS)
		return vector4d{ input.xy, wasm_f64x2_replace_lane(input.zw, 1, lane_value) };
#elif defined(RTM_NEON64_INTRINSICS)
		return vector4d{ input.xy, vsetq_lane_f64(lane_value, input.zw, 1) };
#else
		return vector4d{ input.x, input.y, input.z, lane_value };
#endif
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Sets the vector4 [w] component and returns the new value.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_set_w(vector4d_arg0 input, scalard_arg2 lane_value) RTM_NO_EXCEPT
	{
		return vector4d{ input.xy, _mm_shuffle_pd(input.zw, lane_value.value, _MM_SHUFFLE2(0, 0)) };
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Sets the desired vector2 component and returns the new value.
	//////////////////////////////////////////////////////////////////////////
	template<component2 component>
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_set_component2(vector4d_arg0 input, double lane_value) RTM_NO_EXCEPT
	{
		switch (component)
		{
			default:
			case component2::x:	return vector_set_x(input, lane_value);
			case component2::y:	return vector_set_y(input, lane_value);
		}
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Sets the desired vector2 component and returns the new value.
	//////////////////////////////////////////////////////////////////////////
	template<component2 component>
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_set_component2(vector4d_arg0 input, scalard_arg2 lane_value) RTM_NO_EXCEPT
	{
		switch (component)
		{
			default:
			case component2::x:	return vector_set_x(input, lane_value);
			case component2::y:	return vector_set_y(input, lane_value);
		}
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Sets the desired vector3 component and returns the new value.
	//////////////////////////////////////////////////////////////////////////
	template<component3 component>
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_set_component3(vector4d_arg0 input, double lane_value) RTM_NO_EXCEPT
	{
		switch (component)
		{
			default:
			case component3::x:	return vector_set_x(input, lane_value);
			case component3::y:	return vector_set_y(input, lane_value);
			case component3::z:	return vector_set_z(input, lane_value);
		}
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Sets the desired vector3 component and returns the new value.
	//////////////////////////////////////////////////////////////////////////
	template<component3 component>
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_set_component3(vector4d_arg0 input, scalard_arg2 lane_value) RTM_NO_EXCEPT
	{
		switch (component)
		{
			default:
			case component3::x:	return vector_set_x(input, lane_value);
			case component3::y:	return vector_set_y(input, lane_value);
			case component3::z:	return vector_set_z(input, lane_value);
		}
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Sets the desired vector4 component and returns the new value.
	//////////////////////////////////////////////////////////////////////////
	template<component4 component>
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_set_component(vector4d_arg0 input, double lane_value) RTM_NO_EXCEPT
	{
		switch (component)
		{
			default:
			case component4::x:	return vector_set_x(input, lane_value);
			case component4::y:	return vector_set_y(input, lane_value);
			case component4::z:	return vector_set_z(input, lane_value);
			case component4::w:	return vector_set_w(input, lane_value);
		}
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Sets the desired vector4 component and returns the new value.
	//////////////////////////////////////////////////////////////////////////
	template<component4 component>
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_set_component(vector4d_arg0 input, scalard_arg2 lane_value) RTM_NO_EXCEPT
	{
		switch (component)
		{
			default:
			case component4::x:	return vector_set_x(input, lane_value);
			case component4::y:	return vector_set_y(input, lane_value);
			case component4::z:	return vector_set_z(input, lane_value);
			case component4::w:	return vector_set_w(input, lane_value);
		}
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Sets the desired vector2 component and returns the new value.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_set_component2(vector4d_arg0 input, double lane_value, component2 component) RTM_NO_EXCEPT
	{
		switch (component)
		{
			default:
			case component2::x:	return vector_set_x(input, lane_value);
			case component2::y:	return vector_set_y(input, lane_value);
		}
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Sets the desired vector2 component and returns the new value.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_set_component2(vector4d_arg0 input, scalard_arg2 lane_value, component2 component) RTM_NO_EXCEPT
	{
		switch (component)
		{
			default:
			case component2::x:	return vector_set_x(input, lane_value);
			case component2::y:	return vector_set_y(input, lane_value);
		}
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Sets the desired vector3 component and returns the new value.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_set_component3(vector4d_arg0 input, double lane_value, component3 component) RTM_NO_EXCEPT
	{
		switch (component)
		{
			default:
			case component3::x:	return vector_set_x(input, lane_value);
			case component3::y:	return vector_set_y(input, lane_value);
			case component3::z:	return vector_set_z(input, lane_value);
		}
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Sets the desired vector3 component and returns the new value.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_set_component3(vector4d_arg0 input, scalard_arg2 lane_value, component3 component) RTM_NO_EXCEPT
	{
		switch (component)
		{
			default:
			case component3::x:	return vector_set_x(input, lane_value);
			case component3::y:	return vector_set_y(input, lane_value);
			case component3::z:	return vector_set_z(input, lane_value);
		}
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Sets the desired vector4 component and returns the new value.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_set_component(vector4d_arg0 input, double lane_value, component4 component) RTM_NO_EXCEPT
	{
		switch (component)
		{
			default:
			case component4::x:	return vector_set_x(input, lane_value);
			case component4::y:	return vector_set_y(input, lane_value);
			case component4::z:	return vector_set_z(input, lane_value);
			case component4::w:	return vector_set_w(input, lane_value);
		}
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Sets the desired vector4 component and returns the new value.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_set_component(vector4d_arg0 input, scalard_arg2 lane_value, component4 component) RTM_NO_EXCEPT
	{
		switch (component)
		{
			default:
			case component4::x:	return vector_set_x(input, lane_value);
			case component4::y:	return vector_set_y(input, lane_value);
			case component4::z:	return vector_set_z(input, lane_value);
			case component4::w:	return vector_set_w(input, lane_value);
		}
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Returns a floating point pointer to the vector4 data.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE const double* vector_to_pointer(const vector4d& input) RTM_NO_EXCEPT
	{
		return rtm_impl::bit_cast<const double*>(&input);
	}

	//////////////////////////////////////////////////////////////////////////
	// Writes a vector4 to unaligned memory.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE void RTM_SIMD_CALL vector_store(vector4d_arg0 input, double* output) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		_mm_storeu_pd(output, input.xy);
		_mm_storeu_pd(output + 2, input.zw);
#elif defined(RTM_WASM128_INTRINSICS)
		wasm_v128_store(output, input.xy);
		wasm_v128_store(output + 2, input.zw);
#elif defined(RTM_NEON64_INTRINSICS)
		vst1q_f64(output, input.xy);
		vst1q_f64(output + 2, input.zw);
#else
		output[0] = vector_get_x(input);
		output[1] = vector_get_y(input);
		output[2] = vector_get_z(input);
		output[3] = vector_get_w(input);
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Writes a vector1 to unaligned memory.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE void RTM_SIMD_CALL vector_store1(vector4d_arg0 input, double* output) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		_mm_store_sd(output, input.xy);
#elif defined(RTM_WASM128_INTRINSICS)
		wasm_v128_store64_lane(output, input.xy, 0);
#elif defined(RTM_NEON64_INTRINSICS)
		vst1q_lane_f64(output, input.xy, 0);
#else
		output[0] = vector_get_x(input);
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Writes a vector2 to unaligned memory.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE void RTM_SIMD_CALL vector_store2(vector4d_arg0 input, double* output) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		_mm_storeu_pd(output, input.xy);
#elif defined(RTM_WASM128_INTRINSICS)
		wasm_v128_store(output, input.xy);
#elif defined(RTM_NEON64_INTRINSICS)
		vst1q_f64(output, input.xy);
#else
		output[0] = vector_get_x(input);
		output[1] = vector_get_y(input);
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Writes a vector3 to unaligned memory.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE void RTM_SIMD_CALL vector_store3(vector4d_arg0 input, double* output) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		_mm_storeu_pd(output, input.xy);
		_mm_store_sd(output + 2, input.zw);
#elif defined(RTM_WASM128_INTRINSICS)
		wasm_v128_store(output, input.xy);
		wasm_v128_store64_lane(output + 2, input.zw, 0);
#elif defined(RTM_NEON64_INTRINSICS)
		vst1q_f64(output, input.xy);
		vst1q_lane_f64(output + 2, input.zw, 0);
#else
		output[0] = vector_get_x(input);
		output[1] = vector_get_y(input);
		output[2] = vector_get_z(input);
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Writes a vector4 to unaligned memory.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE void RTM_SIMD_CALL vector_store(vector4d_arg0 input, uint8_t* output) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		_mm_storeu_pd(rtm_impl::bit_cast<double*>(output), input.xy);
		_mm_storeu_pd(rtm_impl::bit_cast<double*>(output) + 2, input.zw);
#elif defined(RTM_WASM128_INTRINSICS)
		wasm_v128_store(output, input.xy);
		wasm_v128_store(output + 16, input.zw);
#elif defined(RTM_NEON64_INTRINSICS)
		vst1q_u8(output, vreinterpretq_u8_f64(input.xy));
		vst1q_u8(output + 16, vreinterpretq_u8_f64(input.zw));
#else
		std::memcpy(output, &input, sizeof(vector4d));
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Writes a vector1 to unaligned memory.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE void RTM_SIMD_CALL vector_store1(vector4d_arg0 input, uint8_t* output)
	{
#if defined(RTM_SSE2_INTRINSICS)
		_mm_store_sd(rtm_impl::bit_cast<double*>(output), input.xy);
#elif defined(RTM_WASM128_INTRINSICS)
		wasm_v128_store64_lane(output, input.xy, 0);
#elif defined(RTM_NEON64_INTRINSICS)
		vst1_u8(output, vreinterpret_u8_f64(vget_low_f64(input.xy)));
#else
		std::memcpy(output, &input, sizeof(double) * 1);
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Writes a vector2 to unaligned memory.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE void RTM_SIMD_CALL vector_store2(vector4d_arg0 input, uint8_t* output)
	{
#if defined(RTM_SSE2_INTRINSICS)
		_mm_storeu_pd(rtm_impl::bit_cast<double*>(output), input.xy);
#elif defined(RTM_WASM128_INTRINSICS)
		wasm_v128_store(output, input.xy);
#elif defined(RTM_NEON64_INTRINSICS)
		vst1q_u8(output, vreinterpretq_u8_f64(input.xy));
#else
		std::memcpy(output, &input, sizeof(double) * 2);
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Writes a vector3 to unaligned memory.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE void RTM_SIMD_CALL vector_store3(vector4d_arg0 input, uint8_t* output)
	{
#if defined(RTM_SSE2_INTRINSICS)
		_mm_storeu_pd(rtm_impl::bit_cast<double*>(output), input.xy);
		_mm_store_sd(rtm_impl::bit_cast<double*>(output) + 2, input.zw);
#elif defined(RTM_WASM128_INTRINSICS)
		wasm_v128_store(output, input.xy);
		wasm_v128_store64_lane(output + 16, input.zw, 0);
#elif defined(RTM_NEON64_INTRINSICS)
		vst1q_u8(output, vreinterpretq_u8_f64(input.xy));
		vst1_u8(output + 16, vreinterpret_u8_f64(vget_low_f64(input.zw)));
#else
		std::memcpy(output, &input, sizeof(double) * 3);
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Writes a vector4 to unaligned memory.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE void RTM_SIMD_CALL vector_store(vector4d_arg0 input, float4d* output) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		_mm_storeu_pd(&output->x, input.xy);
		_mm_storeu_pd(&output->z, input.zw);
#elif defined(RTM_WASM128_INTRINSICS)
		wasm_v128_store(&output->x, input.xy);
		wasm_v128_store(&output->z, input.zw);
#elif defined(RTM_NEON64_INTRINSICS)
		vst1q_f64(&output->x, input.xy);
		vst1q_f64(&output->z, input.zw);
#else
		output->x = vector_get_x(input);
		output->y = vector_get_y(input);
		output->z = vector_get_z(input);
		output->w = vector_get_w(input);
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Writes a vector2 to unaligned memory.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE void RTM_SIMD_CALL vector_store2(vector4d_arg0 input, float2d* output) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		_mm_storeu_pd(&output->x, input.xy);
#elif defined(RTM_WASM128_INTRINSICS)
		wasm_v128_store(&output->x, input.xy);
#elif defined(RTM_NEON64_INTRINSICS)
		vst1q_f64(&output->x, input.xy);
#else
		output->x = vector_get_x(input);
		output->y = vector_get_y(input);
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Writes a vector3 to unaligned memory.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE void RTM_SIMD_CALL vector_store3(vector4d_arg0 input, float3d* output) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		_mm_storeu_pd(&output->x, input.xy);
		_mm_store_sd(&output->z, input.zw);
#elif defined(RTM_WASM128_INTRINSICS)
		wasm_v128_store(&output->x, input.xy);
		wasm_v128_store64_lane(&output->z, input.zw, 0);
#elif defined(RTM_NEON64_INTRINSICS)
		vst1q_f64(&output->x, input.xy);
		vst1q_lane_f64(&output->z, input.zw, 0);
#else
		output->x = vector_get_x(input);
		output->y = vector_get_y(input);
		output->z = vector_get_z(input);
#endif
	}



	//////////////////////////////////////////////////////////////////////////
	// Arithmetic
	//////////////////////////////////////////////////////////////////////////


	//////////////////////////////////////////////////////////////////////////
	// Per component addition of the two inputs: lhs + rhs
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_add(vector4d_arg0 lhs, vector4d_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return vector4d{ _mm_add_pd(lhs.xy, rhs.xy), _mm_add_pd(lhs.zw, rhs.zw) };
#elif defined(RTM_WASM128_INTRINSICS)
		return vector4d{ wasm_f64x2_add(lhs.xy, rhs.xy), wasm_f64x2_add(lhs.zw, rhs.zw) };
#elif defined(RTM_NEON64_INTRINSICS)
		return vector4d{ vaddq_f64(lhs.xy, rhs.xy), vaddq_f64(lhs.zw, rhs.zw) };
#else
		return vector_set(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z, lhs.w + rhs.w);
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Per component subtraction of the two inputs: lhs - rhs
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_sub(vector4d_arg0 lhs, vector4d_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return vector4d{ _mm_sub_pd(lhs.xy, rhs.xy), _mm_sub_pd(lhs.zw, rhs.zw) };
#elif defined(RTM_WASM128_INTRINSICS)
		return vector4d{ wasm_f64x2_sub(lhs.xy, rhs.xy), wasm_f64x2_sub(lhs.zw, rhs.zw) };
#elif defined(RTM_NEON64_INTRINSICS)
		return vector4d{ vsubq_f64(lhs.xy, rhs.xy), vsubq_f64(lhs.zw, rhs.zw) };
#else
		return vector_set(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z, lhs.w - rhs.w);
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Per component multiplication of the two inputs: lhs * rhs
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_mul(vector4d_arg0 lhs, vector4d_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return vector4d{ _mm_mul_pd(lhs.xy, rhs.xy), _mm_mul_pd(lhs.zw, rhs.zw) };
#elif defined(RTM_WASM128_INTRINSICS)
		return vector4d{ wasm_f64x2_mul(lhs.xy, rhs.xy), wasm_f64x2_mul(lhs.zw, rhs.zw) };
#elif defined(RTM_NEON64_INTRINSICS)
		return vector4d{ vmulq_f64(lhs.xy, rhs.xy), vmulq_f64(lhs.zw, rhs.zw) };
#else
		return vector_set(lhs.x * rhs.x, lhs.y * rhs.y, lhs.z * rhs.z, lhs.w * rhs.w);
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Per component multiplication of the vector by a scalar: lhs * rhs
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_mul(vector4d_arg0 lhs, double rhs) RTM_NO_EXCEPT
	{
		return vector_mul(lhs, vector_set(rhs));
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Per component multiplication of the vector by a scalar: lhs * rhs
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_mul(vector4d_arg0 lhs, scalard_arg2 rhs) RTM_NO_EXCEPT
	{
		const __m128d rhs_xx = _mm_shuffle_pd(rhs.value, rhs.value, _MM_SHUFFLE2(0, 0));
		return vector4d{ _mm_mul_pd(lhs.xy, rhs_xx), _mm_mul_pd(lhs.zw, rhs_xx) };
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Per component division of the two inputs: lhs / rhs
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_div(vector4d_arg0 lhs, vector4d_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return vector4d{ _mm_div_pd(lhs.xy, rhs.xy), _mm_div_pd(lhs.zw, rhs.zw) };
#elif defined(RTM_WASM128_INTRINSICS)
		return vector4d{ wasm_f64x2_div(lhs.xy, rhs.xy), wasm_f64x2_div(lhs.zw, rhs.zw) };
#elif defined(RTM_NEON64_INTRINSICS)
		return vector4d{ vdivq_f64(lhs.xy, rhs.xy), vdivq_f64(lhs.zw, rhs.zw) };
#else
		return vector_set(lhs.x / rhs.x, lhs.y / rhs.y, lhs.z / rhs.z, lhs.w / rhs.w);
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Per component maximum of the two inputs: max(lhs, rhs)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_max(vector4d_arg0 lhs, vector4d_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return vector4d{ _mm_max_pd(lhs.xy, rhs.xy), _mm_max_pd(lhs.zw, rhs.zw) };
#elif defined(RTM_WASM128_INTRINSICS)
		return vector4d{ wasm_f64x2_pmax(lhs.xy, rhs.xy), wasm_f64x2_pmax(lhs.zw, rhs.zw) };
#elif defined(RTM_NEON64_INTRINSICS)
		return vector4d{ vmaxq_f64(lhs.xy, rhs.xy), vmaxq_f64(lhs.zw, rhs.zw) };
#else
		return vector_set(scalar_max(lhs.x, rhs.x), scalar_max(lhs.y, rhs.y), scalar_max(lhs.z, rhs.z), scalar_max(lhs.w, rhs.w));
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Per component minimum of the two inputs: min(lhs, rhs)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_min(vector4d_arg0 lhs, vector4d_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return vector4d{ _mm_min_pd(lhs.xy, rhs.xy), _mm_min_pd(lhs.zw, rhs.zw) };
#elif defined(RTM_WASM128_INTRINSICS)
		return vector4d{ wasm_f64x2_pmin(lhs.xy, rhs.xy), wasm_f64x2_pmin(lhs.zw, rhs.zw) };
#elif defined(RTM_NEON64_INTRINSICS)
		return vector4d{ vminq_f64(lhs.xy, rhs.xy), vminq_f64(lhs.zw, rhs.zw) };
#else
		return vector_set(scalar_min(lhs.x, rhs.x), scalar_min(lhs.y, rhs.y), scalar_min(lhs.z, rhs.z), scalar_min(lhs.w, rhs.w));
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Per component clamping of an input between a minimum and a maximum value: min(max_value, max(min_value, input))
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_clamp(vector4d_arg0 input, vector4d_arg1 min_value, vector4d_arg2 max_value) RTM_NO_EXCEPT
	{
		return vector_min(max_value, vector_max(min_value, input));
	}

	//////////////////////////////////////////////////////////////////////////
	// Per component absolute of the input: abs(input)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_abs(vector4d_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		const __m128d abs_mask = _mm_castsi128_pd(_mm_set_epi64x(0x7FFFFFFFFFFFFFFFULL, 0x7FFFFFFFFFFFFFFFULL));
		return vector4d{ _mm_and_pd(input.xy, abs_mask), _mm_and_pd(input.zw, abs_mask) };
#elif defined(RTM_WASM128_INTRINSICS)
		return vector4d{ wasm_f64x2_abs(input.xy), wasm_f64x2_abs(input.zw) };
#elif defined(RTM_NEON64_INTRINSICS)
		return vector4d{ vabsq_f64(input.xy), vabsq_f64(input.zw) };
#else
		return vector_set(scalar_abs(input.x), scalar_abs(input.y), scalar_abs(input.z), scalar_abs(input.w));
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Per component negation of the input: -input
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_neg(vector4d_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		constexpr __m128d signs = RTM_VECTOR2D_MAKE(-0.0, -0.0);
		return vector4d{ _mm_xor_pd(input.xy, signs), _mm_xor_pd(input.zw, signs) };
#elif defined(RTM_WASM128_INTRINSICS)
		return vector4d{ wasm_f64x2_neg(input.xy), wasm_f64x2_neg(input.zw) };
#elif defined(RTM_NEON64_INTRINSICS)
		return vector4d{ vnegq_f64(input.xy), vnegq_f64(input.zw) };
#else
		return vector_mul(input, -1.0);
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Per component negation of the input: -input
	// Each template argument controls whether a SIMD lane should be negated (true) or not (false).
	//////////////////////////////////////////////////////////////////////////
	template<bool x, bool y, bool z, bool w>
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_neg(vector4d_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		constexpr __m128d signs_xy = RTM_VECTOR2D_MAKE(x ? -0.0 : 0.0, y ? -0.0 : 0.0);
		constexpr __m128d signs_zw = RTM_VECTOR2D_MAKE(z ? -0.0 : 0.0, w ? -0.0 : 0.0);
		return vector4d{ _mm_xor_pd(input.xy, signs_xy), _mm_xor_pd(input.zw, signs_zw) };
#elif defined(RTM_WASM128_INTRINSICS)
		const v128_t sign_bit_xy = wasm_f64x2_const(x ? -0.0 : 0.0, y ? -0.0 : 0.0);
		const v128_t sign_bit_zw = wasm_f64x2_const(z ? -0.0 : 0.0, w ? -0.0 : 0.0);
		const v128_t xy = wasm_v128_xor(input.xy, sign_bit_xy);
		const v128_t zw = wasm_v128_xor(input.zw, sign_bit_zw);
		return vector4d{ xy, zw };
#elif defined(RTM_NEON64_INTRINSICS)
		alignas(16) constexpr uint64_t sign_bit_i[4] =
		{
			x ? 0x8000000000000000ULL : 0,
			y ? 0x8000000000000000ULL : 0,
			z ? 0x8000000000000000ULL : 0,
			w ? 0x8000000000000000ULL : 0,
		};
		const uint64x2_t sign_bit_xy = vld1q_u64(&sign_bit_i[0]);
		const uint64x2_t sign_bit_zw = vld1q_u64(&sign_bit_i[2]);
		const float64x2_t xy = vreinterpretq_f64_u64(veorq_u64(vreinterpretq_u64_f64(input.xy), sign_bit_xy));
		const float64x2_t zw = vreinterpretq_f64_u64(veorq_u64(vreinterpretq_u64_f64(input.zw), sign_bit_zw));
		return vector4d{ xy, zw };
#else
		const vector4d signs = vector_set(x ? -1.0 : 1.0, y ? -1.0 : 1.0, z ? -1.0 : 1.0, w ? -1.0 : 1.0);
		return vector_mul(input, signs);
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Per component reciprocal of the input: 1.0 / input
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_reciprocal(vector4d_arg0 input) RTM_NO_EXCEPT
	{
		// Performance note:
		// With modern out-of-order executing processors, it is typically faster to use
		// a full division instead of a reciprocal estimate + Newton-Raphson iterations
		// because the resulting code is more dense and is more likely to inline and
		// as it uses fewer instructions.
		return vector_div(vector_set(1.0), input);
	}

	//////////////////////////////////////////////////////////////////////////
	// Per component square root of the input: sqrt(input)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_sqrt(vector4d_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return vector4d{ _mm_sqrt_pd(input.xy), _mm_sqrt_pd(input.zw) };
#elif defined(RTM_WASM128_INTRINSICS)
		return vector4d{ wasm_f64x2_sqrt(input.xy), wasm_f64x2_sqrt(input.zw) };
#elif defined(RTM_NEON64_INTRINSICS) && defined(RTM_IMPL_VSQRT_SUPPORTED)
		return vector4d{ vsqrtq_f64(input.xy), vsqrtq_f64(input.zw) };
#else
		scalard x = vector_get_x(input);
		scalard y = vector_get_y(input);
		scalard z = vector_get_z(input);
		scalard w = vector_get_w(input);
		return vector_set(scalar_sqrt(x), scalar_sqrt(y), scalar_sqrt(z), scalar_sqrt(w));
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Per component reciprocal square root of the input: 1.0 / sqrt(input)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_sqrt_reciprocal(vector4d_arg0 input) RTM_NO_EXCEPT
	{
		// Performance note:
		// With modern out-of-order executing processors, it is typically faster to use
		// a full division/square root instead of a reciprocal estimate + Newton-Raphson iterations
		// because the resulting code is more dense and is more likely to inline and
		// as it uses fewer instructions.
		return vector_reciprocal(vector_sqrt(input));
	}

	//////////////////////////////////////////////////////////////////////////
	// Per component returns the smallest integer value not less than the input (round towards positive infinity).
	// vector_ceil([1.8, 1.0, -1.8, -1.0]) = [2.0, 1.0, -1.0, -1.0]
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vector4d RTM_SIMD_CALL vector_ceil(vector4d_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE4_INTRINSICS)
		return vector4d{ _mm_ceil_pd(input.xy), _mm_ceil_pd(input.zw) };
#elif defined(RTM_SSE2_INTRINSICS)
		// NaN, +- Infinity, and numbers larger or equal to 2^52 remain unchanged
		// since they have no fractional part.

		const __m128i abs_mask = _mm_set_epi64x(0x7FFFFFFFFFFFFFFFULL, 0x7FFFFFFFFFFFFFFFULL);
		const __m128d fractional_limit = _mm_set1_pd(4503599627370496.0); // 2^52

		// Build our mask, larger values that have no fractional part, and infinities will be true
		// Smaller values and NaN will be false
		__m128d abs_input_xy = _mm_and_pd(input.xy, _mm_castsi128_pd(abs_mask));
		__m128d abs_input_zw = _mm_and_pd(input.zw, _mm_castsi128_pd(abs_mask));
		__m128d is_input_large_xy = _mm_cmpge_pd(abs_input_xy, fractional_limit);
		__m128d is_input_large_zw = _mm_cmpge_pd(abs_input_zw, fractional_limit);

		// Test if our input is NaN with (value != value), it is only true for NaN
		__m128d is_nan_xy = _mm_cmpneq_pd(input.xy, input.xy);
		__m128d is_nan_zw = _mm_cmpneq_pd(input.zw, input.zw);

		// Combine our masks to determine if we should return the original value
		__m128d use_original_input_xy = _mm_or_pd(is_input_large_xy, is_nan_xy);
		__m128d use_original_input_zw = _mm_or_pd(is_input_large_zw, is_nan_zw);

		// Round to the nearest integer: add and subtract 2^52 with the sign of the input.
		// A 64 bit float cannot keep a fractional part at 2^52, thus IEEE 754 rounds the sum.
		// A conversion to a 32 bit integer would overflow for the values between 2^31 and 2^52.
		const __m128d sign_mask = _mm_set_pd(-0.0, -0.0);
		__m128d sign_xy = _mm_and_pd(input.xy, sign_mask);
		__m128d sign_zw = _mm_and_pd(input.zw, sign_mask);
		__m128d truncating_offset_xy = _mm_or_pd(sign_xy, fractional_limit);
		__m128d truncating_offset_zw = _mm_or_pd(sign_zw, fractional_limit);
		__m128d rounded_xy = _mm_sub_pd(_mm_add_pd(input.xy, truncating_offset_xy), truncating_offset_xy);
		__m128d rounded_zw = _mm_sub_pd(_mm_add_pd(input.zw, truncating_offset_zw), truncating_offset_zw);

		// If the rounded value is smaller than the input, add 1.0
		const __m128d one = _mm_set1_pd(1.0);
		__m128d integer_part_xy = _mm_add_pd(rounded_xy, _mm_and_pd(_mm_cmplt_pd(rounded_xy, input.xy), one));
		__m128d integer_part_zw = _mm_add_pd(rounded_zw, _mm_and_pd(_mm_cmplt_pd(rounded_zw, input.zw), one));

		// Keep the sign of the input as the other code paths do, thus floor(-0.0) = -0.0 and ceil(-0.5) = -0.0
		integer_part_xy = _mm_or_pd(integer_part_xy, sign_xy);
		integer_part_zw = _mm_or_pd(integer_part_zw, sign_zw);

		__m128d result_xy = _mm_or_pd(_mm_and_pd(use_original_input_xy, input.xy), _mm_andnot_pd(use_original_input_xy, integer_part_xy));
		__m128d result_zw = _mm_or_pd(_mm_and_pd(use_original_input_zw, input.zw), _mm_andnot_pd(use_original_input_zw, integer_part_zw));
		return vector4d{ result_xy, result_zw };
#elif defined(RTM_WASM128_INTRINSICS)
		return vector4d{ wasm_f64x2_ceil(input.xy), wasm_f64x2_ceil(input.zw) };
#elif defined(RTM_NEON64_INTRINSICS)
		return vector4d{ vrndpq_f64(input.xy), vrndpq_f64(input.zw) };
#else
		return vector_set(scalar_ceil(vector_get_x(input)), scalar_ceil(vector_get_y(input)), scalar_ceil(vector_get_z(input)), scalar_ceil(vector_get_w(input)));
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Per component returns the largest integer value not greater than the input (round towards negative infinity).
	// vector_floor([1.8, 1.0, -1.8, -1.0]) = [1.0, 1.0, -2.0, -1.0]
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vector4d RTM_SIMD_CALL vector_floor(vector4d_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE4_INTRINSICS)
		return vector4d{ _mm_floor_pd(input.xy), _mm_floor_pd(input.zw) };
#elif defined(RTM_SSE2_INTRINSICS)
		// NaN, +- Infinity, and numbers larger or equal to 2^52 remain unchanged
		// since they have no fractional part.

		const __m128i abs_mask = _mm_set_epi64x(0x7FFFFFFFFFFFFFFFULL, 0x7FFFFFFFFFFFFFFFULL);
		const __m128d fractional_limit = _mm_set1_pd(4503599627370496.0); // 2^52

		// Build our mask, larger values that have no fractional part, and infinities will be true
		// Smaller values and NaN will be false
		__m128d abs_input_xy = _mm_and_pd(input.xy, _mm_castsi128_pd(abs_mask));
		__m128d abs_input_zw = _mm_and_pd(input.zw, _mm_castsi128_pd(abs_mask));
		__m128d is_input_large_xy = _mm_cmpge_pd(abs_input_xy, fractional_limit);
		__m128d is_input_large_zw = _mm_cmpge_pd(abs_input_zw, fractional_limit);

		// Test if our input is NaN with (value != value), it is only true for NaN
		__m128d is_nan_xy = _mm_cmpneq_pd(input.xy, input.xy);
		__m128d is_nan_zw = _mm_cmpneq_pd(input.zw, input.zw);

		// Combine our masks to determine if we should return the original value
		__m128d use_original_input_xy = _mm_or_pd(is_input_large_xy, is_nan_xy);
		__m128d use_original_input_zw = _mm_or_pd(is_input_large_zw, is_nan_zw);

		// Round to the nearest integer: add and subtract 2^52 with the sign of the input.
		// A 64 bit float cannot keep a fractional part at 2^52, thus IEEE 754 rounds the sum.
		// A conversion to a 32 bit integer would overflow for the values between 2^31 and 2^52.
		const __m128d sign_mask = _mm_set_pd(-0.0, -0.0);
		__m128d sign_xy = _mm_and_pd(input.xy, sign_mask);
		__m128d sign_zw = _mm_and_pd(input.zw, sign_mask);
		__m128d truncating_offset_xy = _mm_or_pd(sign_xy, fractional_limit);
		__m128d truncating_offset_zw = _mm_or_pd(sign_zw, fractional_limit);
		__m128d rounded_xy = _mm_sub_pd(_mm_add_pd(input.xy, truncating_offset_xy), truncating_offset_xy);
		__m128d rounded_zw = _mm_sub_pd(_mm_add_pd(input.zw, truncating_offset_zw), truncating_offset_zw);

		// If the rounded value is greater than the input, subtract 1.0
		const __m128d one = _mm_set1_pd(1.0);
		__m128d integer_part_xy = _mm_sub_pd(rounded_xy, _mm_and_pd(_mm_cmpgt_pd(rounded_xy, input.xy), one));
		__m128d integer_part_zw = _mm_sub_pd(rounded_zw, _mm_and_pd(_mm_cmpgt_pd(rounded_zw, input.zw), one));

		// Keep the sign of the input as the other code paths do, thus floor(-0.0) = -0.0 and ceil(-0.5) = -0.0
		integer_part_xy = _mm_or_pd(integer_part_xy, sign_xy);
		integer_part_zw = _mm_or_pd(integer_part_zw, sign_zw);

		__m128d result_xy = _mm_or_pd(_mm_and_pd(use_original_input_xy, input.xy), _mm_andnot_pd(use_original_input_xy, integer_part_xy));
		__m128d result_zw = _mm_or_pd(_mm_and_pd(use_original_input_zw, input.zw), _mm_andnot_pd(use_original_input_zw, integer_part_zw));
		return vector4d{ result_xy, result_zw };
#elif defined(RTM_WASM128_INTRINSICS)
		return vector4d{ wasm_f64x2_floor(input.xy), wasm_f64x2_floor(input.zw) };
#elif defined(RTM_NEON64_INTRINSICS)
		return vector4d{ vrndmq_f64(input.xy), vrndmq_f64(input.zw) };
#else
		return vector_set(scalar_floor(vector_get_x(input)), scalar_floor(vector_get_y(input)), scalar_floor(vector_get_z(input)), scalar_floor(vector_get_w(input)));
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// 3D cross product: lhs x rhs
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_cross3(vector4d_arg0 lhs, vector4d_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		// cross(a, b).zxy = (a * b.yzx) - (a.yzx * b)
		__m128d lhs_yz = _mm_shuffle_pd(lhs.xy, lhs.zw, _MM_SHUFFLE2(0, 1));
		__m128d lhs_xy = lhs.xy;
		__m128d rhs_yz = _mm_shuffle_pd(rhs.xy, rhs.zw, _MM_SHUFFLE2(0, 1));
		__m128d rhs_xy = rhs.xy;

		__m128d tmp_zx = _mm_sub_pd(_mm_mul_pd(lhs_xy, rhs_yz), _mm_mul_pd(lhs_yz, rhs_xy));
		__m128d tmp_y = _mm_sub_pd(_mm_mul_pd(lhs.zw, rhs_xy), _mm_mul_pd(lhs_xy, rhs.zw));

		// cross(a, b) = ((a * b.yzx) - (a.yzx * b)).yzx
		__m128d result_xy = _mm_shuffle_pd(tmp_zx, tmp_y, _MM_SHUFFLE2(0, 1));
		__m128d result_z = tmp_zx;
		return vector4d{ result_xy, result_z };
#elif defined(RTM_WASM128_INTRINSICS)
		// cross(a, b).zxy = (a * b.yzx) - (a.yzx * b)
		v128_t lhs_yz = wasm_i64x2_shuffle(lhs.xy, lhs.zw, 1, 2);
		v128_t rhs_yz = wasm_i64x2_shuffle(rhs.xy, rhs.zw, 1, 2);

		v128_t tmp_zx = wasm_f64x2_sub(wasm_f64x2_mul(lhs.xy, rhs_yz), wasm_f64x2_mul(lhs_yz, rhs.xy));
		v128_t tmp_y = wasm_f64x2_sub(wasm_f64x2_mul(lhs.zw, rhs.xy), wasm_f64x2_mul(lhs.xy, rhs.zw));

		// cross(a, b) = ((a * b.yzx) - (a.yzx * b)).yzx
		v128_t result_xy = wasm_i64x2_shuffle(tmp_zx, tmp_y, 1, 2);
		v128_t result_z = tmp_zx;
		return vector4d{ result_xy, result_z };
#elif defined(RTM_NEON64_INTRINSICS)
		// cross(a, b).zxy = (a * b.yzx) - (a.yzx * b)
		float64x2_t lhs_yz = vextq_f64(lhs.xy, lhs.zw, 1);
		float64x2_t rhs_yz = vextq_f64(rhs.xy, rhs.zw, 1);

		float64x2_t tmp_zx = vsubq_f64(vmulq_f64(lhs.xy, rhs_yz), vmulq_f64(lhs_yz, rhs.xy));
		float64x2_t tmp_y = vsubq_f64(vmulq_f64(lhs.zw, rhs.xy), vmulq_f64(lhs.xy, rhs.zw));

		// cross(a, b) = ((a * b.yzx) - (a.yzx * b)).yzx
		float64x2_t result_xy = vextq_f64(tmp_zx, tmp_y, 1);
		float64x2_t result_z = tmp_zx;
		return vector4d{ result_xy, result_z };
#else
		// cross(a, b) = (a.yzx * b.zxy) - (a.zxy * b.yzx)
		const double lhs_x = vector_get_x(lhs);
		const double lhs_y = vector_get_y(lhs);
		const double lhs_z = vector_get_z(lhs);
		const double rhs_x = vector_get_x(rhs);
		const double rhs_y = vector_get_y(rhs);
		const double rhs_z = vector_get_z(rhs);
		return vector_set((lhs_y * rhs_z) - (lhs_z * rhs_y), (lhs_z * rhs_x) - (lhs_x * rhs_z), (lhs_x * rhs_y) - (lhs_y * rhs_x));
#endif
	}

	namespace rtm_impl
	{
		//////////////////////////////////////////////////////////////////////////
		// This is a helper struct to allow a single consistent API between
		// various vector types when the semantics are identical but the return
		// type differs. Implicit coercion is used to return the desired value
		// at the call site.
		//////////////////////////////////////////////////////////////////////////
		struct vector4d_vector_dot
		{
			RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE RTM_SIMD_CALL operator double() const RTM_NO_EXCEPT
			{
#if defined(RTM_SSE2_INTRINSICS)
				const __m128d x2_y2 = _mm_mul_pd(lhs.xy, rhs.xy);
				const __m128d z2_w2 = _mm_mul_pd(lhs.zw, rhs.zw);
				const __m128d x2z2_y2w2 = _mm_add_pd(x2_y2, z2_w2);
				const __m128d y2w2 = _mm_shuffle_pd(x2z2_y2w2, x2z2_y2w2, _MM_SHUFFLE2(1, 1));
				const __m128d x2y2z2w2 = _mm_add_pd(x2z2_y2w2, y2w2);
				return _mm_cvtsd_f64(x2y2z2w2);
#elif defined(RTM_WASM128_INTRINSICS)
				const v128_t x2_y2 = wasm_f64x2_mul(lhs.xy, rhs.xy);
				const v128_t z2_w2 = wasm_f64x2_mul(lhs.zw, rhs.zw);
				const v128_t x2z2_y2w2 = wasm_f64x2_add(x2_y2, z2_w2);
				return wasm_f64x2_extract_lane(x2z2_y2w2, 0) + wasm_f64x2_extract_lane(x2z2_y2w2, 1);
#elif defined(RTM_NEON64_INTRINSICS)
				const float64x2_t x2_y2 = vmulq_f64(lhs.xy, rhs.xy);
				const float64x2_t z2_w2 = vmulq_f64(lhs.zw, rhs.zw);
				const float64x2_t x2z2_y2w2 = vaddq_f64(x2_y2, z2_w2);
				return vgetq_lane_f64(x2z2_y2w2, 0) + vgetq_lane_f64(x2z2_y2w2, 1);
#else
				const scalard lhs_x = vector_get_x_as_scalar(lhs);
				const scalard lhs_y = vector_get_y_as_scalar(lhs);
				const scalard lhs_z = vector_get_z_as_scalar(lhs);
				const scalard lhs_w = vector_get_w_as_scalar(lhs);
				const scalard rhs_x = vector_get_x_as_scalar(rhs);
				const scalard rhs_y = vector_get_y_as_scalar(rhs);
				const scalard rhs_z = vector_get_z_as_scalar(rhs);
				const scalard rhs_w = vector_get_w_as_scalar(rhs);
				const scalard xx = scalar_mul(lhs_x, rhs_x);
				const scalard yy = scalar_mul(lhs_y, rhs_y);
				const scalard zz = scalar_mul(lhs_z, rhs_z);
				const scalard ww = scalar_mul(lhs_w, rhs_w);
				return scalar_cast(scalar_add(scalar_add(xx, yy), scalar_add(zz, ww)));
#endif
			}

#if defined(RTM_SSE2_INTRINSICS)
			RTM_DEPRECATED("Use 'as_scalar' suffix instead. To be removed in 2.4.")
			RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE RTM_SIMD_CALL operator scalard() const RTM_NO_EXCEPT
			{
				const __m128d x2_y2 = _mm_mul_pd(lhs.xy, rhs.xy);
				const __m128d z2_w2 = _mm_mul_pd(lhs.zw, rhs.zw);
				const __m128d x2z2_y2w2 = _mm_add_pd(x2_y2, z2_w2);
				const __m128d y2w2 = _mm_shuffle_pd(x2z2_y2w2, x2z2_y2w2, _MM_SHUFFLE2(1, 1));
				const __m128d x2y2z2w2 = _mm_add_pd(x2z2_y2w2, y2w2);
				return scalard{ x2y2z2w2 };
			}
#endif

			RTM_DEPRECATED("Use 'as_vector' suffix instead. To be removed in 2.4.")
			RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE RTM_SIMD_CALL operator vector4d() const RTM_NO_EXCEPT
			{
				const double dot = *this;
				return vector_set(dot);
			}

			vector4d lhs;
			vector4d rhs;
		};
	}

	//////////////////////////////////////////////////////////////////////////
	// 4D dot product: lhs . rhs
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE constexpr rtm_impl::vector4d_vector_dot RTM_SIMD_CALL vector_dot(vector4d_arg0 lhs, vector4d_arg1 rhs) RTM_NO_EXCEPT
	{
		return rtm_impl::vector4d_vector_dot{ lhs, rhs };
	}

	//////////////////////////////////////////////////////////////////////////
	// 4D dot product: lhs . rhs
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE scalard RTM_SIMD_CALL vector_dot_as_scalar(vector4d_arg0 lhs, vector4d_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		const __m128d x2_y2 = _mm_mul_pd(lhs.xy, rhs.xy);
		const __m128d z2_w2 = _mm_mul_pd(lhs.zw, rhs.zw);
		const __m128d x2z2_y2w2 = _mm_add_pd(x2_y2, z2_w2);
		const __m128d y2w2 = _mm_shuffle_pd(x2z2_y2w2, x2z2_y2w2, _MM_SHUFFLE2(1, 1));
		const __m128d x2y2z2w2 = _mm_add_pd(x2z2_y2w2, y2w2);
		return scalard{ x2y2z2w2 };
#elif defined(RTM_WASM128_INTRINSICS)
		const v128_t x2_y2 = wasm_f64x2_mul(lhs.xy, rhs.xy);
		const v128_t z2_w2 = wasm_f64x2_mul(lhs.zw, rhs.zw);
		const v128_t x2z2_y2w2 = wasm_f64x2_add(x2_y2, z2_w2);
		return wasm_f64x2_extract_lane(x2z2_y2w2, 0) + wasm_f64x2_extract_lane(x2z2_y2w2, 1);
#elif defined(RTM_NEON64_INTRINSICS)
		const float64x2_t x2_y2 = vmulq_f64(lhs.xy, rhs.xy);
		const float64x2_t z2_w2 = vmulq_f64(lhs.zw, rhs.zw);
		const float64x2_t x2z2_y2w2 = vaddq_f64(x2_y2, z2_w2);
		return vgetq_lane_f64(x2z2_y2w2, 0) + vgetq_lane_f64(x2z2_y2w2, 1);
#else
		const scalard lhs_x = vector_get_x_as_scalar(lhs);
		const scalard lhs_y = vector_get_y_as_scalar(lhs);
		const scalard lhs_z = vector_get_z_as_scalar(lhs);
		const scalard lhs_w = vector_get_w_as_scalar(lhs);
		const scalard rhs_x = vector_get_x_as_scalar(rhs);
		const scalard rhs_y = vector_get_y_as_scalar(rhs);
		const scalard rhs_z = vector_get_z_as_scalar(rhs);
		const scalard rhs_w = vector_get_w_as_scalar(rhs);
		const scalard xx = scalar_mul(lhs_x, rhs_x);
		const scalard yy = scalar_mul(lhs_y, rhs_y);
		const scalard zz = scalar_mul(lhs_z, rhs_z);
		const scalard ww = scalar_mul(lhs_w, rhs_w);
		return scalar_add(scalar_add(xx, yy), scalar_add(zz, ww));
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// 4D dot product: lhs . rhs
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_dot_as_vector(vector4d_arg0 lhs, vector4d_arg1 rhs) RTM_NO_EXCEPT
	{
		return vector_set(vector_dot_as_scalar(lhs, rhs));
	}

	namespace rtm_impl
	{
		//////////////////////////////////////////////////////////////////////////
		// This is a helper struct to allow a single consistent API between
		// various vector types when the semantics are identical but the return
		// type differs. Implicit coercion is used to return the desired value
		// at the call site.
		//////////////////////////////////////////////////////////////////////////
		struct vector4d_vector_dot2
		{
			RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE RTM_SIMD_CALL operator double() const RTM_NO_EXCEPT
			{
#if defined(RTM_SSE2_INTRINSICS)
				__m128d x2_y2 = _mm_mul_pd(lhs.xy, rhs.xy);
				__m128d y2 = _mm_shuffle_pd(x2_y2, x2_y2, _MM_SHUFFLE2(0, 1));
				return _mm_cvtsd_f64(_mm_add_sd(x2_y2, y2));
#elif defined(RTM_WASM128_INTRINSICS)
				const v128_t x2_y2 = wasm_f64x2_mul(lhs.xy, rhs.xy);
				return wasm_f64x2_extract_lane(x2_y2, 0) + wasm_f64x2_extract_lane(x2_y2, 1);
#elif defined(RTM_NEON64_INTRINSICS)
				const float64x2_t x2_y2 = vmulq_f64(lhs.xy, rhs.xy);
				return vgetq_lane_f64(x2_y2, 0) + vgetq_lane_f64(x2_y2, 1);
#else
				return (vector_get_x(lhs) * vector_get_x(rhs)) + (vector_get_y(lhs) * vector_get_y(rhs));
#endif
			}

#if defined(RTM_SSE2_INTRINSICS)
			RTM_DEPRECATED("Use 'as_scalar' suffix instead. To be removed in 2.4.")
			RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE RTM_SIMD_CALL operator scalard() const RTM_NO_EXCEPT
			{
				__m128d x2_y2 = _mm_mul_pd(lhs.xy, rhs.xy);
				__m128d y2 = _mm_shuffle_pd(x2_y2, x2_y2, _MM_SHUFFLE2(0, 1));
				return scalard{ _mm_add_sd(x2_y2, y2) };
			}
#endif

			RTM_DEPRECATED("Use 'as_vector' suffix instead. To be removed in 2.4.")
			RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE RTM_SIMD_CALL operator vector4d() const RTM_NO_EXCEPT
			{
				const double dot = *this;
				return vector_set(dot);
			}

			vector4d lhs;
			vector4d rhs;
		};
	}

	//////////////////////////////////////////////////////////////////////////
	// 2D dot product: lhs . rhs
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE constexpr rtm_impl::vector4d_vector_dot2 RTM_SIMD_CALL vector_dot2(vector4d_arg0 lhs, vector4d_arg1 rhs) RTM_NO_EXCEPT
	{
		return rtm_impl::vector4d_vector_dot2{ lhs, rhs };
	}

	//////////////////////////////////////////////////////////////////////////
	// 2D dot product: lhs . rhs
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE scalard RTM_SIMD_CALL vector_dot2_as_scalar(vector4d_arg0 lhs, vector4d_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d x2_y2 = _mm_mul_pd(lhs.xy, rhs.xy);
		__m128d y2 = _mm_shuffle_pd(x2_y2, x2_y2, _MM_SHUFFLE2(0, 1));
		return scalard{ _mm_add_sd(x2_y2, y2) };
#else
		return vector_dot2(lhs, rhs);
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// 2D dot product: lhs . rhs
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_dot2_as_vector(vector4d_arg0 lhs, vector4d_arg1 rhs) RTM_NO_EXCEPT
	{
		return vector_set(vector_dot2_as_scalar(lhs, rhs));
	}

	namespace rtm_impl
	{
		//////////////////////////////////////////////////////////////////////////
		// This is a helper struct to allow a single consistent API between
		// various vector types when the semantics are identical but the return
		// type differs. Implicit coercion is used to return the desired value
		// at the call site.
		//////////////////////////////////////////////////////////////////////////
		struct vector4d_vector_dot3
		{
			RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE RTM_SIMD_CALL operator double() const RTM_NO_EXCEPT
			{
#if defined(RTM_SSE2_INTRINSICS)
				__m128d x2_y2 = _mm_mul_pd(lhs.xy, rhs.xy);
				__m128d z2_w2 = _mm_mul_pd(lhs.zw, rhs.zw);
				__m128d y2 = _mm_shuffle_pd(x2_y2, x2_y2, _MM_SHUFFLE2(0, 1));
				__m128d x2y2 = _mm_add_sd(x2_y2, y2);
				return _mm_cvtsd_f64(_mm_add_sd(x2y2, z2_w2));
#elif defined(RTM_WASM128_INTRINSICS)
				const v128_t x2_y2 = wasm_f64x2_mul(lhs.xy, rhs.xy);
				const v128_t z2_w2 = wasm_f64x2_mul(lhs.zw, rhs.zw);
				return (wasm_f64x2_extract_lane(x2_y2, 0) + wasm_f64x2_extract_lane(x2_y2, 1)) + wasm_f64x2_extract_lane(z2_w2, 0);
#elif defined(RTM_NEON64_INTRINSICS)
				const float64x2_t x2_y2 = vmulq_f64(lhs.xy, rhs.xy);
				const float64x2_t z2_w2 = vmulq_f64(lhs.zw, rhs.zw);
				return (vgetq_lane_f64(x2_y2, 0) + vgetq_lane_f64(x2_y2, 1)) + vgetq_lane_f64(z2_w2, 0);
#else
				return (vector_get_x(lhs) * vector_get_x(rhs)) + (vector_get_y(lhs) * vector_get_y(rhs)) + (vector_get_z(lhs) * vector_get_z(rhs));
#endif
			}

#if defined(RTM_SSE2_INTRINSICS)
			RTM_DEPRECATED("Use 'as_scalar' suffix instead. To be removed in 2.4.")
			RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE RTM_SIMD_CALL operator scalard() const RTM_NO_EXCEPT
			{
				__m128d x2_y2 = _mm_mul_pd(lhs.xy, rhs.xy);
				__m128d z2_w2 = _mm_mul_pd(lhs.zw, rhs.zw);
				__m128d y2 = _mm_shuffle_pd(x2_y2, x2_y2, _MM_SHUFFLE2(0, 1));
				__m128d x2y2 = _mm_add_sd(x2_y2, y2);
				return scalard{ _mm_add_sd(x2y2, z2_w2) };
			}
#endif

			RTM_DEPRECATED("Use 'as_vector' suffix instead. To be removed in 2.4.")
			RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE RTM_SIMD_CALL operator vector4d() const RTM_NO_EXCEPT
			{
				const double dot = *this;
				return vector_set(dot);
			}

			vector4d lhs;
			vector4d rhs;
		};
	}

	//////////////////////////////////////////////////////////////////////////
	// 3D dot product: lhs . rhs
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE constexpr rtm_impl::vector4d_vector_dot3 RTM_SIMD_CALL vector_dot3(vector4d_arg0 lhs, vector4d_arg1 rhs) RTM_NO_EXCEPT
	{
		return rtm_impl::vector4d_vector_dot3{ lhs, rhs };
	}

	//////////////////////////////////////////////////////////////////////////
	// 3D dot product: lhs . rhs
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE scalard RTM_SIMD_CALL vector_dot3_as_scalar(vector4d_arg0 lhs, vector4d_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d x2_y2 = _mm_mul_pd(lhs.xy, rhs.xy);
		__m128d z2_w2 = _mm_mul_pd(lhs.zw, rhs.zw);
		__m128d y2 = _mm_shuffle_pd(x2_y2, x2_y2, _MM_SHUFFLE2(0, 1));
		__m128d x2y2 = _mm_add_sd(x2_y2, y2);
		return scalard{ _mm_add_sd(x2y2, z2_w2) };
#else
		return vector_dot3(lhs, rhs);
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// 3D dot product: lhs . rhs
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_dot3_as_vector(vector4d_arg0 lhs, vector4d_arg1 rhs) RTM_NO_EXCEPT
	{
		return vector_set(vector_dot3_as_scalar(lhs, rhs));
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the squared length/norm of the vector4.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE constexpr rtm_impl::vector4d_vector_dot RTM_SIMD_CALL vector_length_squared(vector4d_arg0 input) RTM_NO_EXCEPT
	{
		return rtm_impl::vector4d_vector_dot{ input, input };
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the squared length/norm of the vector4.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE scalard RTM_SIMD_CALL vector_length_squared_as_scalar(vector4d_arg0 input) RTM_NO_EXCEPT
	{
		return vector_dot_as_scalar(input, input);
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the squared length/norm of the vector4.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_length_squared_as_vector(vector4d_arg0 input) RTM_NO_EXCEPT
	{
		return vector_dot_as_vector(input, input);
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the squared length/norm of the vector2.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE constexpr rtm_impl::vector4d_vector_dot2 RTM_SIMD_CALL vector_length_squared2(vector4d_arg0 input) RTM_NO_EXCEPT
	{
		return rtm_impl::vector4d_vector_dot2{ input, input };
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the squared length/norm of the vector2.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE scalard RTM_SIMD_CALL vector_length_squared2_as_scalar(vector4d_arg0 input) RTM_NO_EXCEPT
	{
		return vector_dot2_as_scalar(input, input);
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the squared length/norm of the vector2.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_length_squared2_as_vector(vector4d_arg0 input) RTM_NO_EXCEPT
	{
		return vector_dot2_as_vector(input, input);
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the squared length/norm of the vector3.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE constexpr rtm_impl::vector4d_vector_dot3 RTM_SIMD_CALL vector_length_squared3(vector4d_arg0 input) RTM_NO_EXCEPT
	{
		return rtm_impl::vector4d_vector_dot3{ input, input };
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the squared length/norm of the vector3.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE scalard RTM_SIMD_CALL vector_length_squared3_as_scalar(vector4d_arg0 input) RTM_NO_EXCEPT
	{
		return vector_dot3_as_scalar(input, input);
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the squared length/norm of the vector3.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_length_squared3_as_vector(vector4d_arg0 input) RTM_NO_EXCEPT
	{
		return vector_dot3_as_vector(input, input);
	}

	namespace rtm_impl
	{
		//////////////////////////////////////////////////////////////////////////
		// This is a helper struct to allow a single consistent API between
		// various vector types when the semantics are identical but the return
		// type differs. Implicit coercion is used to return the desired value
		// at the call site.
		//////////////////////////////////////////////////////////////////////////
		struct vector4d_vector_length
		{
			RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE RTM_SIMD_CALL operator double() const RTM_NO_EXCEPT
			{
				const scalard len_sq = vector_length_squared_as_scalar(input);
				return scalar_cast(scalar_sqrt(len_sq));
			}

#if defined(RTM_SSE2_INTRINSICS)
			RTM_DEPRECATED("Use 'as_scalar' suffix instead. To be removed in 2.4.")
			RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE RTM_SIMD_CALL operator scalard() const RTM_NO_EXCEPT
			{
				const scalard len_sq = vector_length_squared_as_scalar(input);
				return scalar_sqrt(len_sq);
			}
#endif

			vector4d input;
		};
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the length/norm of the vector4.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE constexpr rtm_impl::vector4d_vector_length RTM_SIMD_CALL vector_length(vector4d_arg0 input) RTM_NO_EXCEPT
	{
		return rtm_impl::vector4d_vector_length{ input };
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the length/norm of the vector4.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE scalard RTM_SIMD_CALL vector_length_as_scalar(vector4d_arg0 input) RTM_NO_EXCEPT
	{
		const scalard len_sq = vector_length_squared_as_scalar(input);
		return scalar_sqrt(len_sq);
	}

	namespace rtm_impl
	{
		//////////////////////////////////////////////////////////////////////////
		// This is a helper struct to allow a single consistent API between
		// various vector types when the semantics are identical but the return
		// type differs. Implicit coercion is used to return the desired value
		// at the call site.
		//////////////////////////////////////////////////////////////////////////
		struct vector4d_vector_length3
		{
			RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE RTM_SIMD_CALL operator double() const RTM_NO_EXCEPT
			{
				const scalard len_sq = vector_length_squared3_as_scalar(input);
				return scalar_cast(scalar_sqrt(len_sq));
			}

#if defined(RTM_SSE2_INTRINSICS)
			RTM_DEPRECATED("Use 'as_scalar' suffix instead. To be removed in 2.4.")
			RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE RTM_SIMD_CALL operator scalard() const RTM_NO_EXCEPT
			{
				const scalard len_sq = vector_length_squared3_as_scalar(input);
				return scalar_sqrt(len_sq);
			}
#endif

			vector4d input;
		};
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the length/norm of the vector3.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE constexpr rtm_impl::vector4d_vector_length3 RTM_SIMD_CALL vector_length3(vector4d_arg0 input) RTM_NO_EXCEPT
	{
		return rtm_impl::vector4d_vector_length3{ input };
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the length/norm of the vector3.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE scalard RTM_SIMD_CALL vector_length3_as_scalar(vector4d_arg0 input) RTM_NO_EXCEPT
	{
		const scalard len_sq = vector_length_squared3_as_scalar(input);
		return scalar_sqrt(len_sq);
	}

	namespace rtm_impl
	{
		//////////////////////////////////////////////////////////////////////////
		// This is a helper struct to allow a single consistent API between
		// various vector types when the semantics are identical but the return
		// type differs. Implicit coercion is used to return the desired value
		// at the call site.
		//////////////////////////////////////////////////////////////////////////
		struct vector4d_vector_length_reciprocal
		{
			RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE RTM_SIMD_CALL operator double() const RTM_NO_EXCEPT
			{
				const scalard len_sq = vector_length_squared_as_scalar(input);
				return scalar_cast(scalar_sqrt_reciprocal(len_sq));
			}

#if defined(RTM_SSE2_INTRINSICS)
			RTM_DEPRECATED("Use 'as_scalar' suffix instead. To be removed in 2.4.")
			RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE RTM_SIMD_CALL operator scalard() const RTM_NO_EXCEPT
			{
				const scalard len_sq = vector_length_squared_as_scalar(input);
				return scalar_sqrt_reciprocal(len_sq);
			}
#endif

			vector4d input;
		};
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the reciprocal length/norm of the vector4.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE constexpr rtm_impl::vector4d_vector_length_reciprocal RTM_SIMD_CALL vector_length_reciprocal(vector4d_arg0 input) RTM_NO_EXCEPT
	{
		return rtm_impl::vector4d_vector_length_reciprocal{ input };
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the reciprocal length/norm of the vector4.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE scalard RTM_SIMD_CALL vector_length_reciprocal_as_scalar(vector4d_arg0 input) RTM_NO_EXCEPT
	{
		const scalard len_sq = vector_length_squared_as_scalar(input);
		return scalar_sqrt_reciprocal(len_sq);
	}

	namespace rtm_impl
	{
		//////////////////////////////////////////////////////////////////////////
		// This is a helper struct to allow a single consistent API between
		// various vector types when the semantics are identical but the return
		// type differs. Implicit coercion is used to return the desired value
		// at the call site.
		//////////////////////////////////////////////////////////////////////////
		struct vector4d_vector_length_reciprocal2
		{
			RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE RTM_SIMD_CALL operator double() const RTM_NO_EXCEPT
			{
				const scalard len_sq = vector_length_squared2_as_scalar(input);
				return scalar_cast(scalar_sqrt_reciprocal(len_sq));
			}

#if defined(RTM_SSE2_INTRINSICS)
			RTM_DEPRECATED("Use 'as_scalar' suffix instead. To be removed in 2.4.")
			RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE RTM_SIMD_CALL operator scalard() const RTM_NO_EXCEPT
			{
				const scalard len_sq = vector_length_squared2_as_scalar(input);
				return scalar_sqrt_reciprocal(len_sq);
			}
#endif

			vector4d input;
		};
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the reciprocal length/norm of the vector2.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE constexpr rtm_impl::vector4d_vector_length_reciprocal2 RTM_SIMD_CALL vector_length_reciprocal2(vector4d_arg0 input) RTM_NO_EXCEPT
	{
		return rtm_impl::vector4d_vector_length_reciprocal2{ input };
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the reciprocal length/norm of the vector2.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE scalard RTM_SIMD_CALL vector_length_reciprocal2_as_scalar(vector4d_arg0 input) RTM_NO_EXCEPT
	{
		const scalard len_sq = vector_length_squared2_as_scalar(input);
		return scalar_sqrt_reciprocal(len_sq);
	}

	namespace rtm_impl
	{
		//////////////////////////////////////////////////////////////////////////
		// This is a helper struct to allow a single consistent API between
		// various vector types when the semantics are identical but the return
		// type differs. Implicit coercion is used to return the desired value
		// at the call site.
		//////////////////////////////////////////////////////////////////////////
		struct vector4d_vector_length_reciprocal3
		{
			RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE RTM_SIMD_CALL operator double() const RTM_NO_EXCEPT
			{
				const scalard len_sq = vector_length_squared3_as_scalar(input);
				return scalar_cast(scalar_sqrt_reciprocal(len_sq));
			}

#if defined(RTM_SSE2_INTRINSICS)
			RTM_DEPRECATED("Use 'as_scalar' suffix instead. To be removed in 2.4.")
			RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE RTM_SIMD_CALL operator scalard() const RTM_NO_EXCEPT
			{
				const scalard len_sq = vector_length_squared3_as_scalar(input);
				return scalar_sqrt_reciprocal(len_sq);
			}
#endif

			vector4d input;
		};
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the reciprocal length/norm of the vector3.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE constexpr rtm_impl::vector4d_vector_length_reciprocal3 RTM_SIMD_CALL vector_length_reciprocal3(vector4d_arg0 input) RTM_NO_EXCEPT
	{
		return rtm_impl::vector4d_vector_length_reciprocal3{ input };
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the reciprocal length/norm of the vector3.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE scalard RTM_SIMD_CALL vector_length_reciprocal3_as_scalar(vector4d_arg0 input) RTM_NO_EXCEPT
	{
		const scalard len_sq = vector_length_squared3_as_scalar(input);
		return scalar_sqrt_reciprocal(len_sq);
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the distance between two 3D points.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE rtm_impl::vector4d_vector_length3 RTM_SIMD_CALL vector_distance3(vector4d_arg0 lhs, vector4d_arg1 rhs) RTM_NO_EXCEPT
	{
		const vector4d difference = vector_sub(lhs, rhs);
		return rtm_impl::vector4d_vector_length3{ difference };
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the distance between two 3D points.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE scalard RTM_SIMD_CALL vector_distance3_as_scalar(vector4d_arg0 lhs, vector4d_arg1 rhs) RTM_NO_EXCEPT
	{
		const vector4d difference = vector_sub(lhs, rhs);
		return vector_length3_as_scalar(difference);
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the squared distance between two 3D points.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE rtm_impl::vector4d_vector_dot3 RTM_SIMD_CALL vector_distance_squared3(vector4d_arg0 lhs, vector4d_arg1 rhs) RTM_NO_EXCEPT
	{
		const vector4d difference = vector_sub(lhs, rhs);
		return rtm_impl::vector4d_vector_dot3{ difference, difference };
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the squared distance between two 3D points.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE scalard RTM_SIMD_CALL vector_distance_squared3_as_scalar(vector4d_arg0 lhs, vector4d_arg1 rhs) RTM_NO_EXCEPT
	{
		const vector4d difference = vector_sub(lhs, rhs);
		return vector_length_squared3_as_scalar(difference);
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns a normalized vector2.
	// If the length of the input is not finite or zero, the result is undefined.
	// For a safe alternative, supply a fallback value and a threshold.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_normalize2(vector4d_arg0 input) RTM_NO_EXCEPT
	{
		// Reciprocal is more accurate to normalize with
		const scalard len_sq = vector_length_squared2_as_scalar(input);
		return vector_mul(input, scalar_sqrt_reciprocal(len_sq));
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns a normalized vector2.
	// If the length of the input is below the supplied threshold, the
	// fall back value is returned instead.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_normalize2(vector4d_arg0 input, vector4d_arg1 fallback, double threshold = 1.0E-8) RTM_NO_EXCEPT
	{
		// Reciprocal is more accurate to normalize with
		const scalard len_sq = vector_length_squared2_as_scalar(input);
		if (scalar_cast(len_sq) >= threshold)
			return vector_mul(input, scalar_sqrt_reciprocal(len_sq));
		else
			return fallback;
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns a normalized vector3.
	// If the length of the input is not finite or zero, the result is undefined.
	// For a safe alternative, supply a fallback value and a threshold.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_normalize3(vector4d_arg0 input) RTM_NO_EXCEPT
	{
		// Reciprocal is more accurate to normalize with
		const scalard len_sq = vector_length_squared3_as_scalar(input);
		return vector_mul(input, scalar_sqrt_reciprocal(len_sq));
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns a normalized vector3.
	// If the length of the input is below the supplied threshold, the
	// fall back value is returned instead.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_normalize3(vector4d_arg0 input, vector4d_arg1 fallback, double threshold = 1.0E-8) RTM_NO_EXCEPT
	{
		// Reciprocal is more accurate to normalize with
		const scalard len_sq = vector_length_squared3_as_scalar(input);
		if (scalar_cast(len_sq) >= threshold)
			return vector_mul(input, scalar_sqrt_reciprocal(len_sq));
		else
			return fallback;
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns a normalized vector4.
	// If the length of the input is not finite or zero, the result is undefined.
	// For a safe alternative, supply a fallback value and a threshold.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_normalize(vector4d_arg0 input) RTM_NO_EXCEPT
	{
		// Reciprocal is more accurate to normalize with
		const scalard len_sq = vector_length_squared_as_scalar(input);
		return vector_mul(input, scalar_sqrt_reciprocal(len_sq));
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns a normalized vector4.
	// If the length of the input is below the supplied threshold, the
	// fall back value is returned instead.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_normalize(vector4d_arg0 input, vector4d_arg1 fallback, double threshold = 1.0E-8) RTM_NO_EXCEPT
	{
		// Reciprocal is more accurate to normalize with
		const scalard len_sq = vector_length_squared_as_scalar(input);
		if (scalar_cast(len_sq) >= threshold)
			return vector_mul(input, scalar_sqrt_reciprocal(len_sq));
		else
			return fallback;
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns per component the fractional part of the input.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vector4d RTM_SIMD_CALL vector_fraction(vector4d_arg0 input) RTM_NO_EXCEPT
	{
		return vector_set(scalar_fraction(vector_get_x(input)), scalar_fraction(vector_get_y(input)), scalar_fraction(vector_get_z(input)), scalar_fraction(vector_get_w(input)));
	}

	//////////////////////////////////////////////////////////////////////////
	// Per component multiplication/addition of the three inputs: v2 + (v0 * v1)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_mul_add(vector4d_arg0 v0, vector4d_arg1 v1, vector4d_arg2 v2) RTM_NO_EXCEPT
	{
#if defined(RTM_NEON64_INTRINSICS)
		return vector4d{ vfmaq_f64(v2.xy, v0.xy, v1.xy), vfmaq_f64(v2.zw, v0.zw, v1.zw) };
#else
		return vector_add(vector_mul(v0, v1), v2);
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Per component multiplication/addition of the three inputs: v2 + (v0 * s1)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_mul_add(vector4d_arg0 v0, double s1, vector4d_arg2 v2) RTM_NO_EXCEPT
	{
#if defined(RTM_NEON64_INTRINSICS)
		const float64x2_t s1_xx = vdupq_n_f64(s1);
		return vector4d{ vfmaq_f64(v2.xy, v0.xy, s1_xx), vfmaq_f64(v2.zw, v0.zw, s1_xx) };
#else
		return vector_add(vector_mul(v0, s1), v2);
#endif
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Per component multiplication/addition of the three inputs: v2 + (v0 * s1)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_mul_add(vector4d_arg0 v0, scalard_arg2 s1, vector4d_arg2 v2) RTM_NO_EXCEPT
	{
		return vector_add(vector_mul(v0, s1), v2);
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Per component negative multiplication/subtraction of the three inputs: -((v0 * v1) - v2)
	// This is mathematically equivalent to: v2 - (v0 * v1)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_neg_mul_sub(vector4d_arg0 v0, vector4d_arg1 v1, vector4d_arg2 v2) RTM_NO_EXCEPT
	{
#if defined(RTM_NEON64_INTRINSICS)
		return vector4d{ vfmsq_f64(v2.xy, v0.xy, v1.xy), vfmsq_f64(v2.zw, v0.zw, v1.zw) };
#else
		return vector_sub(v2, vector_mul(v0, v1));
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Per component negative multiplication/subtraction of the three inputs: -((v0 * s1) - v2)
	// This is mathematically equivalent to: v2 - (v0 * s1)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_neg_mul_sub(vector4d_arg0 v0, double s1, vector4d_arg2 v2) RTM_NO_EXCEPT
	{
#if defined(RTM_NEON64_INTRINSICS)
		const float64x2_t s1_xx = vdupq_n_f64(s1);
		return vector4d{ vfmsq_f64(v2.xy, v0.xy, s1_xx), vfmsq_f64(v2.zw, v0.zw, s1_xx) };
#else
		return vector_sub(v2, vector_mul(v0, s1));
#endif
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Per component negative multiplication/subtraction of the three inputs: -((v0 * s1) - v2)
	// This is mathematically equivalent to: v2 - (v0 * s1)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_neg_mul_sub(vector4d_arg0 v0, scalard_arg2 s1, vector4d_arg0 v2) RTM_NO_EXCEPT
	{
		return vector_sub(v2, vector_mul(v0, s1));
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Per component linear interpolation of the two inputs at the specified alpha.
	// The formula used is: ((1.0 - alpha) * start) + (alpha * end).
	// Interpolation is stable and will return 'start' when alpha is 0.0 and 'end' when it is 1.0.
	// This is the same instruction count when FMA is present but it might be slightly slower
	// due to the extra multiplication compared to: start + (alpha * (end - start)).
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_lerp(vector4d_arg0 start, vector4d_arg1 end, double alpha) RTM_NO_EXCEPT
	{
		// ((1.0 - alpha) * start) + (alpha * end) == (start - alpha * start) + (alpha * end)
		return vector_mul_add(end, alpha, vector_neg_mul_sub(start, alpha, start));
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Per component linear interpolation of the two inputs at the specified alpha.
	// The formula used is: ((1.0 - alpha) * start) + (alpha * end).
	// Interpolation is stable and will return 'start' when alpha is 0.0 and 'end' when it is 1.0.
	// This is the same instruction count when FMA is present but it might be slightly slower
	// due to the extra multiplication compared to: start + (alpha * (end - start)).
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_lerp(vector4d_arg0 start, vector4d_arg1 end, scalard_arg4 alpha) RTM_NO_EXCEPT
	{
		// ((1.0 - alpha) * start) + (alpha * end) == (start - alpha * start) + (alpha * end)
		const vector4d alpha_v = vector_set(alpha);
		return vector_mul_add(end, alpha_v, vector_neg_mul_sub(start, alpha_v, start));
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Per component linear interpolation of the two inputs at the specified alpha.
	// The formula used is: ((1.0 - alpha) * start) + (alpha * end).
	// Interpolation is stable and will return 'start' when alpha is 0.0 and 'end' when it is 1.0.
	// This is the same instruction count when FMA is present but it might be slightly slower
	// due to the extra multiplication compared to: start + (alpha * (end - start)).
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_lerp(vector4d_arg0 start, vector4d_arg1 end, vector4d_arg2 alpha) RTM_NO_EXCEPT
	{
		// ((1.0 - alpha) * start) + (alpha * end) == (start - alpha * start) + (alpha * end)
		return vector_mul_add(end, alpha, vector_neg_mul_sub(start, alpha, start));
	}



	//////////////////////////////////////////////////////////////////////////
	// Comparisons and masking
	//////////////////////////////////////////////////////////////////////////


	//////////////////////////////////////////////////////////////////////////
	// Returns per component ~0 if equal, otherwise 0: lhs == rhs ? ~0 : 0
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE mask4d RTM_SIMD_CALL vector_equal(vector4d_arg0 lhs, vector4d_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_lt_pd = _mm_cmpeq_pd(lhs.xy, rhs.xy);
		__m128d zw_lt_pd = _mm_cmpeq_pd(lhs.zw, rhs.zw);
		return mask4d{ xy_lt_pd, zw_lt_pd };
#elif defined(RTM_WASM128_INTRINSICS)
		return mask4d{ wasm_f64x2_eq(lhs.xy, rhs.xy), wasm_f64x2_eq(lhs.zw, rhs.zw) };
#elif defined(RTM_NEON64_INTRINSICS)
		return mask4d{ vceqq_f64(lhs.xy, rhs.xy), vceqq_f64(lhs.zw, rhs.zw) };
#else
		return mask4d{ rtm_impl::get_mask_value(lhs.x == rhs.x), rtm_impl::get_mask_value(lhs.y == rhs.y), rtm_impl::get_mask_value(lhs.z == rhs.z), rtm_impl::get_mask_value(lhs.w == rhs.w) };
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns per component ~0 if not equal, otherwise 0: lhs != rhs ? ~0 : 0
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE mask4d RTM_SIMD_CALL vector_not_equal(vector4d_arg0 lhs, vector4d_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_lt_pd = _mm_cmpneq_pd(lhs.xy, rhs.xy);
		__m128d zw_lt_pd = _mm_cmpneq_pd(lhs.zw, rhs.zw);
		return mask4d{ xy_lt_pd, zw_lt_pd };
#elif defined(RTM_WASM128_INTRINSICS)
		const v128_t xy_eq = wasm_f64x2_eq(lhs.xy, rhs.xy);
		const v128_t zw_eq = wasm_f64x2_eq(lhs.zw, rhs.zw);
		const v128_t xy_neq = wasm_v128_not(xy_eq);
		const v128_t zw_neq = wasm_v128_not(zw_eq);
		return mask4d{ xy_neq, zw_neq };
#elif defined(RTM_NEON64_INTRINSICS)
		const uint64x2_t xy_eq = vceqq_f64(lhs.xy, rhs.xy);
		const uint64x2_t zw_eq = vceqq_f64(lhs.zw, rhs.zw);
		const uint64x2_t xy_neq = vreinterpretq_u64_u32(vmvnq_u32(vreinterpretq_u32_u64(xy_eq)));
		const uint64x2_t zw_neq = vreinterpretq_u64_u32(vmvnq_u32(vreinterpretq_u32_u64(zw_eq)));
		return mask4d{ xy_neq, zw_neq };
#else
		return mask4d{ rtm_impl::get_mask_value(lhs.x != rhs.x), rtm_impl::get_mask_value(lhs.y != rhs.y), rtm_impl::get_mask_value(lhs.z != rhs.z), rtm_impl::get_mask_value(lhs.w != rhs.w) };
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns per component ~0 if less than, otherwise 0: lhs < rhs ? ~0 : 0
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE mask4d RTM_SIMD_CALL vector_less_than(vector4d_arg0 lhs, vector4d_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_lt_pd = _mm_cmplt_pd(lhs.xy, rhs.xy);
		__m128d zw_lt_pd = _mm_cmplt_pd(lhs.zw, rhs.zw);
		return mask4d{xy_lt_pd, zw_lt_pd};
#elif defined(RTM_WASM128_INTRINSICS)
		return mask4d{ wasm_f64x2_lt(lhs.xy, rhs.xy), wasm_f64x2_lt(lhs.zw, rhs.zw) };
#elif defined(RTM_NEON64_INTRINSICS)
		return mask4d{ vcltq_f64(lhs.xy, rhs.xy), vcltq_f64(lhs.zw, rhs.zw) };
#else
		return mask4d{rtm_impl::get_mask_value(lhs.x < rhs.x), rtm_impl::get_mask_value(lhs.y < rhs.y), rtm_impl::get_mask_value(lhs.z < rhs.z), rtm_impl::get_mask_value(lhs.w < rhs.w)};
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns per component ~0 if less equal, otherwise 0: lhs <= rhs ? ~0 : 0
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE mask4d RTM_SIMD_CALL vector_less_equal(vector4d_arg0 lhs, vector4d_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_lt_pd = _mm_cmple_pd(lhs.xy, rhs.xy);
		__m128d zw_lt_pd = _mm_cmple_pd(lhs.zw, rhs.zw);
		return mask4d{ xy_lt_pd, zw_lt_pd };
#elif defined(RTM_WASM128_INTRINSICS)
		return mask4d{ wasm_f64x2_le(lhs.xy, rhs.xy), wasm_f64x2_le(lhs.zw, rhs.zw) };
#elif defined(RTM_NEON64_INTRINSICS)
		return mask4d{ vcleq_f64(lhs.xy, rhs.xy), vcleq_f64(lhs.zw, rhs.zw) };
#else
		return mask4d{ rtm_impl::get_mask_value(lhs.x <= rhs.x), rtm_impl::get_mask_value(lhs.y <= rhs.y), rtm_impl::get_mask_value(lhs.z <= rhs.z), rtm_impl::get_mask_value(lhs.w <= rhs.w) };
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns per component ~0 if greater than, otherwise 0: lhs > rhs ? ~0 : 0
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE mask4d RTM_SIMD_CALL vector_greater_than(vector4d_arg0 lhs, vector4d_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_ge_pd = _mm_cmpgt_pd(lhs.xy, rhs.xy);
		__m128d zw_ge_pd = _mm_cmpgt_pd(lhs.zw, rhs.zw);
		return mask4d{ xy_ge_pd, zw_ge_pd };
#elif defined(RTM_WASM128_INTRINSICS)
		return mask4d{ wasm_f64x2_gt(lhs.xy, rhs.xy), wasm_f64x2_gt(lhs.zw, rhs.zw) };
#elif defined(RTM_NEON64_INTRINSICS)
		return mask4d{ vcgtq_f64(lhs.xy, rhs.xy), vcgtq_f64(lhs.zw, rhs.zw) };
#else
		return mask4d{ rtm_impl::get_mask_value(lhs.x > rhs.x), rtm_impl::get_mask_value(lhs.y > rhs.y), rtm_impl::get_mask_value(lhs.z > rhs.z), rtm_impl::get_mask_value(lhs.w > rhs.w) };
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns per component ~0 if greater equal, otherwise 0: lhs >= rhs ? ~0 : 0
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE mask4d RTM_SIMD_CALL vector_greater_equal(vector4d_arg0 lhs, vector4d_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_ge_pd = _mm_cmpge_pd(lhs.xy, rhs.xy);
		__m128d zw_ge_pd = _mm_cmpge_pd(lhs.zw, rhs.zw);
		return mask4d{ xy_ge_pd, zw_ge_pd };
#elif defined(RTM_WASM128_INTRINSICS)
		return mask4d{ wasm_f64x2_ge(lhs.xy, rhs.xy), wasm_f64x2_ge(lhs.zw, rhs.zw) };
#elif defined(RTM_NEON64_INTRINSICS)
		return mask4d{ vcgeq_f64(lhs.xy, rhs.xy), vcgeq_f64(lhs.zw, rhs.zw) };
#else
		return mask4d{ rtm_impl::get_mask_value(lhs.x >= rhs.x), rtm_impl::get_mask_value(lhs.y >= rhs.y), rtm_impl::get_mask_value(lhs.z >= rhs.z), rtm_impl::get_mask_value(lhs.w >= rhs.w) };
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all 4 components are less than, otherwise false: all(lhs.xyzw < rhs.xyzw)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL vector_all_less_than(vector4d_arg0 lhs, vector4d_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_lt_pd = _mm_cmplt_pd(lhs.xy, rhs.xy);
		__m128d zw_lt_pd = _mm_cmplt_pd(lhs.zw, rhs.zw);
		return (_mm_movemask_pd(xy_lt_pd) & _mm_movemask_pd(zw_lt_pd)) == 3;
#elif defined(RTM_WASM128_INTRINSICS)
		return rtm_impl::mask4q_all_true(wasm_f64x2_lt(lhs.xy, rhs.xy), wasm_f64x2_lt(lhs.zw, rhs.zw));
#elif defined(RTM_NEON64_INTRINSICS)
		return rtm_impl::mask4q_all_true(vcltq_f64(lhs.xy, rhs.xy), vcltq_f64(lhs.zw, rhs.zw));
#else
		return lhs.x < rhs.x && lhs.y < rhs.y && lhs.z < rhs.z && lhs.w < rhs.w;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all [xy] components are less than, otherwise false: all(lhs.xy < rhs.xy)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL vector_all_less_than2(vector4d_arg0 lhs, vector4d_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_lt_pd = _mm_cmplt_pd(lhs.xy, rhs.xy);
		return _mm_movemask_pd(xy_lt_pd) == 3;
#elif defined(RTM_WASM128_INTRINSICS)
		return rtm_impl::mask2q_all_true(wasm_f64x2_lt(lhs.xy, rhs.xy));
#elif defined(RTM_NEON64_INTRINSICS)
		return rtm_impl::mask2q_all_true(vcltq_f64(lhs.xy, rhs.xy));
#else
		return lhs.x < rhs.x && lhs.y < rhs.y;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all [xyz] components are less than, otherwise false: all(lhs.xyz < rhs.xyz)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL vector_all_less_than3(vector4d_arg0 lhs, vector4d_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_lt_pd = _mm_cmplt_pd(lhs.xy, rhs.xy);
		__m128d zw_lt_pd = _mm_cmplt_pd(lhs.zw, rhs.zw);
		return _mm_movemask_pd(xy_lt_pd) == 3 && (_mm_movemask_pd(zw_lt_pd) & 1) == 1;
#elif defined(RTM_WASM128_INTRINSICS)
		return rtm_impl::mask4q_all_true3(wasm_f64x2_lt(lhs.xy, rhs.xy), wasm_f64x2_lt(lhs.zw, rhs.zw));
#elif defined(RTM_NEON64_INTRINSICS)
		return rtm_impl::mask4q_all_true3(vcltq_f64(lhs.xy, rhs.xy), vcltq_f64(lhs.zw, rhs.zw));
#else
		return lhs.x < rhs.x && lhs.y < rhs.y && lhs.z < rhs.z;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if any 4 components are less than, otherwise false: any(lhs.xyzw < rhs.xyzw)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL vector_any_less_than(vector4d_arg0 lhs, vector4d_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_lt_pd = _mm_cmplt_pd(lhs.xy, rhs.xy);
		__m128d zw_lt_pd = _mm_cmplt_pd(lhs.zw, rhs.zw);
		return (_mm_movemask_pd(xy_lt_pd) | _mm_movemask_pd(zw_lt_pd)) != 0;
#elif defined(RTM_WASM128_INTRINSICS)
		return rtm_impl::mask4q_any_true(wasm_f64x2_lt(lhs.xy, rhs.xy), wasm_f64x2_lt(lhs.zw, rhs.zw));
#elif defined(RTM_NEON64_INTRINSICS)
		return rtm_impl::mask4q_any_true(vcltq_f64(lhs.xy, rhs.xy), vcltq_f64(lhs.zw, rhs.zw));
#else
		return lhs.x < rhs.x || lhs.y < rhs.y || lhs.z < rhs.z || lhs.w < rhs.w;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if any [xy] components are less than, otherwise false: any(lhs.xy < rhs.xy)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL vector_any_less_than2(vector4d_arg0 lhs, vector4d_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_lt_pd = _mm_cmplt_pd(lhs.xy, rhs.xy);
		return _mm_movemask_pd(xy_lt_pd) != 0;
#elif defined(RTM_WASM128_INTRINSICS)
		return rtm_impl::mask2q_any_true(wasm_f64x2_lt(lhs.xy, rhs.xy));
#elif defined(RTM_NEON64_INTRINSICS)
		return rtm_impl::mask2q_any_true(vcltq_f64(lhs.xy, rhs.xy));
#else
		return lhs.x < rhs.x || lhs.y < rhs.y;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if any [xyz] components are less than, otherwise false: any(lhs.xyz < rhs.xyz)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL vector_any_less_than3(vector4d_arg0 lhs, vector4d_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_lt_pd = _mm_cmplt_pd(lhs.xy, rhs.xy);
		__m128d zw_lt_pd = _mm_cmplt_pd(lhs.zw, rhs.zw);
		return _mm_movemask_pd(xy_lt_pd) != 0 || (_mm_movemask_pd(zw_lt_pd) & 0x1) != 0;
#elif defined(RTM_WASM128_INTRINSICS)
		return rtm_impl::mask4q_any_true3(wasm_f64x2_lt(lhs.xy, rhs.xy), wasm_f64x2_lt(lhs.zw, rhs.zw));
#elif defined(RTM_NEON64_INTRINSICS)
		return rtm_impl::mask4q_any_true3(vcltq_f64(lhs.xy, rhs.xy), vcltq_f64(lhs.zw, rhs.zw));
#else
		return lhs.x < rhs.x || lhs.y < rhs.y || lhs.z < rhs.z;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all 4 components are less equal, otherwise false: all(lhs.xyzw <= rhs.xyzw)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL vector_all_less_equal(vector4d_arg0 lhs, vector4d_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_le_pd = _mm_cmple_pd(lhs.xy, rhs.xy);
		__m128d zw_le_pd = _mm_cmple_pd(lhs.zw, rhs.zw);
		return (_mm_movemask_pd(xy_le_pd) & _mm_movemask_pd(zw_le_pd)) == 3;
#elif defined(RTM_WASM128_INTRINSICS)
		return rtm_impl::mask4q_all_true(wasm_f64x2_le(lhs.xy, rhs.xy), wasm_f64x2_le(lhs.zw, rhs.zw));
#elif defined(RTM_NEON64_INTRINSICS)
		return rtm_impl::mask4q_all_true(vcleq_f64(lhs.xy, rhs.xy), vcleq_f64(lhs.zw, rhs.zw));
#else
		return lhs.x <= rhs.x && lhs.y <= rhs.y && lhs.z <= rhs.z && lhs.w <= rhs.w;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all [xy] components are less equal, otherwise false: all(lhs.xy <= rhs.xy)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL vector_all_less_equal2(vector4d_arg0 lhs, vector4d_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_le_pd = _mm_cmple_pd(lhs.xy, rhs.xy);
		return _mm_movemask_pd(xy_le_pd) == 3;
#elif defined(RTM_WASM128_INTRINSICS)
		return rtm_impl::mask2q_all_true(wasm_f64x2_le(lhs.xy, rhs.xy));
#elif defined(RTM_NEON64_INTRINSICS)
		return rtm_impl::mask2q_all_true(vcleq_f64(lhs.xy, rhs.xy));
#else
		return lhs.x <= rhs.x && lhs.y <= rhs.y;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all [xyz] components are less equal, otherwise false: all(lhs.xyz <= rhs.xyz)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL vector_all_less_equal3(vector4d_arg0 lhs, vector4d_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_le_pd = _mm_cmple_pd(lhs.xy, rhs.xy);
		__m128d zw_le_pd = _mm_cmple_pd(lhs.zw, rhs.zw);
		return _mm_movemask_pd(xy_le_pd) == 3 && (_mm_movemask_pd(zw_le_pd) & 1) != 0;
#elif defined(RTM_WASM128_INTRINSICS)
		return rtm_impl::mask4q_all_true3(wasm_f64x2_le(lhs.xy, rhs.xy), wasm_f64x2_le(lhs.zw, rhs.zw));
#elif defined(RTM_NEON64_INTRINSICS)
		return rtm_impl::mask4q_all_true3(vcleq_f64(lhs.xy, rhs.xy), vcleq_f64(lhs.zw, rhs.zw));
#else
		return lhs.x <= rhs.x && lhs.y <= rhs.y && lhs.z <= rhs.z;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if any 4 components are less equal, otherwise false: any(lhs.xyzw <= rhs.xyzw)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL vector_any_less_equal(vector4d_arg0 lhs, vector4d_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_le_pd = _mm_cmple_pd(lhs.xy, rhs.xy);
		__m128d zw_le_pd = _mm_cmple_pd(lhs.zw, rhs.zw);
		return (_mm_movemask_pd(xy_le_pd) | _mm_movemask_pd(zw_le_pd)) != 0;
#elif defined(RTM_WASM128_INTRINSICS)
		return rtm_impl::mask4q_any_true(wasm_f64x2_le(lhs.xy, rhs.xy), wasm_f64x2_le(lhs.zw, rhs.zw));
#elif defined(RTM_NEON64_INTRINSICS)
		return rtm_impl::mask4q_any_true(vcleq_f64(lhs.xy, rhs.xy), vcleq_f64(lhs.zw, rhs.zw));
#else
		return lhs.x <= rhs.x || lhs.y <= rhs.y || lhs.z <= rhs.z || lhs.w <= rhs.w;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if any [xy] components are less equal, otherwise false: any(lhs.xy <= rhs.xy)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL vector_any_less_equal2(vector4d_arg0 lhs, vector4d_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_le_pd = _mm_cmple_pd(lhs.xy, rhs.xy);
		return _mm_movemask_pd(xy_le_pd) != 0;
#elif defined(RTM_WASM128_INTRINSICS)
		return rtm_impl::mask2q_any_true(wasm_f64x2_le(lhs.xy, rhs.xy));
#elif defined(RTM_NEON64_INTRINSICS)
		return rtm_impl::mask2q_any_true(vcleq_f64(lhs.xy, rhs.xy));
#else
		return lhs.x <= rhs.x || lhs.y <= rhs.y;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if any [xyz] components are less equal, otherwise false: any(lhs.xyz <= rhs.xyz)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL vector_any_less_equal3(vector4d_arg0 lhs, vector4d_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_le_pd = _mm_cmple_pd(lhs.xy, rhs.xy);
		__m128d zw_le_pd = _mm_cmple_pd(lhs.zw, rhs.zw);
		return _mm_movemask_pd(xy_le_pd) != 0 || (_mm_movemask_pd(zw_le_pd) & 1) != 0;
#elif defined(RTM_WASM128_INTRINSICS)
		return rtm_impl::mask4q_any_true3(wasm_f64x2_le(lhs.xy, rhs.xy), wasm_f64x2_le(lhs.zw, rhs.zw));
#elif defined(RTM_NEON64_INTRINSICS)
		return rtm_impl::mask4q_any_true3(vcleq_f64(lhs.xy, rhs.xy), vcleq_f64(lhs.zw, rhs.zw));
#else
		return lhs.x <= rhs.x || lhs.y <= rhs.y || lhs.z <= rhs.z;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all 4 components are greater than, otherwise false: all(lhs.xyzw > rhs.xyzw)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL vector_all_greater_than(vector4d_arg0 lhs, vector4d_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_ge_pd = _mm_cmpgt_pd(lhs.xy, rhs.xy);
		__m128d zw_ge_pd = _mm_cmpgt_pd(lhs.zw, rhs.zw);
		return (_mm_movemask_pd(xy_ge_pd) & _mm_movemask_pd(zw_ge_pd)) == 3;
#elif defined(RTM_WASM128_INTRINSICS)
		return rtm_impl::mask4q_all_true(wasm_f64x2_gt(lhs.xy, rhs.xy), wasm_f64x2_gt(lhs.zw, rhs.zw));
#elif defined(RTM_NEON64_INTRINSICS)
		return rtm_impl::mask4q_all_true(vcgtq_f64(lhs.xy, rhs.xy), vcgtq_f64(lhs.zw, rhs.zw));
#else
		return lhs.x > rhs.x && lhs.y > rhs.y && lhs.z > rhs.z && lhs.w > rhs.w;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all [xy] components are greater than, otherwise false: all(lhs.xy > rhs.xy)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL vector_all_greater_than2(vector4d_arg0 lhs, vector4d_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_ge_pd = _mm_cmpgt_pd(lhs.xy, rhs.xy);
		return _mm_movemask_pd(xy_ge_pd) == 3;
#elif defined(RTM_WASM128_INTRINSICS)
		return rtm_impl::mask2q_all_true(wasm_f64x2_gt(lhs.xy, rhs.xy));
#elif defined(RTM_NEON64_INTRINSICS)
		return rtm_impl::mask2q_all_true(vcgtq_f64(lhs.xy, rhs.xy));
#else
		return lhs.x > rhs.x && lhs.y > rhs.y;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all [xyz] components are greater than, otherwise false: all(lhs.xyz > rhs.xyz)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL vector_all_greater_than3(vector4d_arg0 lhs, vector4d_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_ge_pd = _mm_cmpgt_pd(lhs.xy, rhs.xy);
		__m128d zw_ge_pd = _mm_cmpgt_pd(lhs.zw, rhs.zw);
		return _mm_movemask_pd(xy_ge_pd) == 3 && (_mm_movemask_pd(zw_ge_pd) & 1) != 0;
#elif defined(RTM_WASM128_INTRINSICS)
		return rtm_impl::mask4q_all_true3(wasm_f64x2_gt(lhs.xy, rhs.xy), wasm_f64x2_gt(lhs.zw, rhs.zw));
#elif defined(RTM_NEON64_INTRINSICS)
		return rtm_impl::mask4q_all_true3(vcgtq_f64(lhs.xy, rhs.xy), vcgtq_f64(lhs.zw, rhs.zw));
#else
		return lhs.x > rhs.x && lhs.y > rhs.y && lhs.z > rhs.z;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if any 4 components are greater than, otherwise false: any(lhs.xyzw > rhs.xyzw)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL vector_any_greater_than(vector4d_arg0 lhs, vector4d_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_ge_pd = _mm_cmpgt_pd(lhs.xy, rhs.xy);
		__m128d zw_ge_pd = _mm_cmpgt_pd(lhs.zw, rhs.zw);
		return (_mm_movemask_pd(xy_ge_pd) | _mm_movemask_pd(zw_ge_pd)) != 0;
#elif defined(RTM_WASM128_INTRINSICS)
		return rtm_impl::mask4q_any_true(wasm_f64x2_gt(lhs.xy, rhs.xy), wasm_f64x2_gt(lhs.zw, rhs.zw));
#elif defined(RTM_NEON64_INTRINSICS)
		return rtm_impl::mask4q_any_true(vcgtq_f64(lhs.xy, rhs.xy), vcgtq_f64(lhs.zw, rhs.zw));
#else
		return lhs.x > rhs.x || lhs.y > rhs.y || lhs.z > rhs.z || lhs.w > rhs.w;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if any [xy] components are greater than, otherwise false: any(lhs.xy > rhs.xy)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL vector_any_greater_than2(vector4d_arg0 lhs, vector4d_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_ge_pd = _mm_cmpgt_pd(lhs.xy, rhs.xy);
		return _mm_movemask_pd(xy_ge_pd) != 0;
#elif defined(RTM_WASM128_INTRINSICS)
		return rtm_impl::mask2q_any_true(wasm_f64x2_gt(lhs.xy, rhs.xy));
#elif defined(RTM_NEON64_INTRINSICS)
		return rtm_impl::mask2q_any_true(vcgtq_f64(lhs.xy, rhs.xy));
#else
		return lhs.x > rhs.x || lhs.y > rhs.y;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if any [xyz] components are greater than, otherwise false: any(lhs.xyz > rhs.xyz)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL vector_any_greater_than3(vector4d_arg0 lhs, vector4d_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_ge_pd = _mm_cmpgt_pd(lhs.xy, rhs.xy);
		__m128d zw_ge_pd = _mm_cmpgt_pd(lhs.zw, rhs.zw);
		return _mm_movemask_pd(xy_ge_pd) != 0 || (_mm_movemask_pd(zw_ge_pd) & 1) != 0;
#elif defined(RTM_WASM128_INTRINSICS)
		return rtm_impl::mask4q_any_true3(wasm_f64x2_gt(lhs.xy, rhs.xy), wasm_f64x2_gt(lhs.zw, rhs.zw));
#elif defined(RTM_NEON64_INTRINSICS)
		return rtm_impl::mask4q_any_true3(vcgtq_f64(lhs.xy, rhs.xy), vcgtq_f64(lhs.zw, rhs.zw));
#else
		return lhs.x > rhs.x || lhs.y > rhs.y || lhs.z > rhs.z;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all 4 components are greater equal, otherwise false: all(lhs.xyzw >= rhs.xyzw)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL vector_all_greater_equal(vector4d_arg0 lhs, vector4d_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_ge_pd = _mm_cmpge_pd(lhs.xy, rhs.xy);
		__m128d zw_ge_pd = _mm_cmpge_pd(lhs.zw, rhs.zw);
		return (_mm_movemask_pd(xy_ge_pd) & _mm_movemask_pd(zw_ge_pd)) == 3;
#elif defined(RTM_WASM128_INTRINSICS)
		return rtm_impl::mask4q_all_true(wasm_f64x2_ge(lhs.xy, rhs.xy), wasm_f64x2_ge(lhs.zw, rhs.zw));
#elif defined(RTM_NEON64_INTRINSICS)
		return rtm_impl::mask4q_all_true(vcgeq_f64(lhs.xy, rhs.xy), vcgeq_f64(lhs.zw, rhs.zw));
#else
		return lhs.x >= rhs.x && lhs.y >= rhs.y && lhs.z >= rhs.z && lhs.w >= rhs.w;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all [xy] components are greater equal, otherwise false: all(lhs.xy >= rhs.xy)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL vector_all_greater_equal2(vector4d_arg0 lhs, vector4d_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_ge_pd = _mm_cmpge_pd(lhs.xy, rhs.xy);
		return _mm_movemask_pd(xy_ge_pd) == 3;
#elif defined(RTM_WASM128_INTRINSICS)
		return rtm_impl::mask2q_all_true(wasm_f64x2_ge(lhs.xy, rhs.xy));
#elif defined(RTM_NEON64_INTRINSICS)
		return rtm_impl::mask2q_all_true(vcgeq_f64(lhs.xy, rhs.xy));
#else
		return lhs.x >= rhs.x && lhs.y >= rhs.y;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all [xyz] components are greater equal, otherwise false: all(lhs.xyz >= rhs.xyz)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL vector_all_greater_equal3(vector4d_arg0 lhs, vector4d_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_ge_pd = _mm_cmpge_pd(lhs.xy, rhs.xy);
		__m128d zw_ge_pd = _mm_cmpge_pd(lhs.zw, rhs.zw);
		return _mm_movemask_pd(xy_ge_pd) == 3 && (_mm_movemask_pd(zw_ge_pd) & 1) != 0;
#elif defined(RTM_WASM128_INTRINSICS)
		return rtm_impl::mask4q_all_true3(wasm_f64x2_ge(lhs.xy, rhs.xy), wasm_f64x2_ge(lhs.zw, rhs.zw));
#elif defined(RTM_NEON64_INTRINSICS)
		return rtm_impl::mask4q_all_true3(vcgeq_f64(lhs.xy, rhs.xy), vcgeq_f64(lhs.zw, rhs.zw));
#else
		return lhs.x >= rhs.x && lhs.y >= rhs.y && lhs.z >= rhs.z;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if any 4 components are greater equal, otherwise false: any(lhs.xyzw >= rhs.xyzw)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL vector_any_greater_equal(vector4d_arg0 lhs, vector4d_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_ge_pd = _mm_cmpge_pd(lhs.xy, rhs.xy);
		__m128d zw_ge_pd = _mm_cmpge_pd(lhs.zw, rhs.zw);
		return (_mm_movemask_pd(xy_ge_pd) | _mm_movemask_pd(zw_ge_pd)) != 0;
#elif defined(RTM_WASM128_INTRINSICS)
		return rtm_impl::mask4q_any_true(wasm_f64x2_ge(lhs.xy, rhs.xy), wasm_f64x2_ge(lhs.zw, rhs.zw));
#elif defined(RTM_NEON64_INTRINSICS)
		return rtm_impl::mask4q_any_true(vcgeq_f64(lhs.xy, rhs.xy), vcgeq_f64(lhs.zw, rhs.zw));
#else
		return lhs.x >= rhs.x || lhs.y >= rhs.y || lhs.z >= rhs.z || lhs.w >= rhs.w;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if any [xy] components are greater equal, otherwise false: any(lhs.xy >= rhs.xy)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL vector_any_greater_equal2(vector4d_arg0 lhs, vector4d_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_ge_pd = _mm_cmpge_pd(lhs.xy, rhs.xy);
		return _mm_movemask_pd(xy_ge_pd) != 0;
#elif defined(RTM_WASM128_INTRINSICS)
		return rtm_impl::mask2q_any_true(wasm_f64x2_ge(lhs.xy, rhs.xy));
#elif defined(RTM_NEON64_INTRINSICS)
		return rtm_impl::mask2q_any_true(vcgeq_f64(lhs.xy, rhs.xy));
#else
		return lhs.x >= rhs.x || lhs.y >= rhs.y;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if any [xyz] components are greater equal, otherwise false: any(lhs.xyz >= rhs.xyz)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL vector_any_greater_equal3(vector4d_arg0 lhs, vector4d_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_ge_pd = _mm_cmpge_pd(lhs.xy, rhs.xy);
		__m128d zw_ge_pd = _mm_cmpge_pd(lhs.zw, rhs.zw);
		return _mm_movemask_pd(xy_ge_pd) != 0 || (_mm_movemask_pd(zw_ge_pd) & 1) != 0;
#elif defined(RTM_WASM128_INTRINSICS)
		return rtm_impl::mask4q_any_true3(wasm_f64x2_ge(lhs.xy, rhs.xy), wasm_f64x2_ge(lhs.zw, rhs.zw));
#elif defined(RTM_NEON64_INTRINSICS)
		return rtm_impl::mask4q_any_true3(vcgeq_f64(lhs.xy, rhs.xy), vcgeq_f64(lhs.zw, rhs.zw));
#else
		return lhs.x >= rhs.x || lhs.y >= rhs.y || lhs.z >= rhs.z;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all [xyzw] components are equal, otherwise false: all(lhs.xyzw == rhs.xyzw)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL vector_all_equal(vector4d_arg0 lhs, vector4d_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_eq_pd = _mm_cmpeq_pd(lhs.xy, rhs.xy);
		__m128d zw_eq_pd = _mm_cmpeq_pd(lhs.zw, rhs.zw);
		return (_mm_movemask_pd(xy_eq_pd) & _mm_movemask_pd(zw_eq_pd)) == 3;
#elif defined(RTM_WASM128_INTRINSICS)
		return rtm_impl::mask4q_all_true(wasm_f64x2_eq(lhs.xy, rhs.xy), wasm_f64x2_eq(lhs.zw, rhs.zw));
#elif defined(RTM_NEON64_INTRINSICS)
		return rtm_impl::mask4q_all_true(vceqq_f64(lhs.xy, rhs.xy), vceqq_f64(lhs.zw, rhs.zw));
#else
		return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z && lhs.w == rhs.w;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all [xy] components are equal, otherwise false: all(lhs.xy == rhs.xy)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL vector_all_equal2(vector4d_arg0 lhs, vector4d_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_eq_pd = _mm_cmpeq_pd(lhs.xy, rhs.xy);
		return _mm_movemask_pd(xy_eq_pd) == 3;
#elif defined(RTM_WASM128_INTRINSICS)
		return rtm_impl::mask2q_all_true(wasm_f64x2_eq(lhs.xy, rhs.xy));
#elif defined(RTM_NEON64_INTRINSICS)
		return rtm_impl::mask2q_all_true(vceqq_f64(lhs.xy, rhs.xy));
#else
		return lhs.x == rhs.x && lhs.y == rhs.y;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all [xyz] components are equal, otherwise false: all(lhs.xyz == rhs.xyz)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL vector_all_equal3(vector4d_arg0 lhs, vector4d_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_eq_pd = _mm_cmpeq_pd(lhs.xy, rhs.xy);
		__m128d zw_eq_pd = _mm_cmpeq_pd(lhs.zw, rhs.zw);
		return _mm_movemask_pd(xy_eq_pd) == 3 && (_mm_movemask_pd(zw_eq_pd) & 1) != 0;
#elif defined(RTM_WASM128_INTRINSICS)
		return rtm_impl::mask4q_all_true3(wasm_f64x2_eq(lhs.xy, rhs.xy), wasm_f64x2_eq(lhs.zw, rhs.zw));
#elif defined(RTM_NEON64_INTRINSICS)
		return rtm_impl::mask4q_all_true3(vceqq_f64(lhs.xy, rhs.xy), vceqq_f64(lhs.zw, rhs.zw));
#else
		return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if any [xyzw] components are equal, otherwise false: any(lhs.xyzw == rhs.xyzw)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL vector_any_equal(vector4d_arg0 lhs, vector4d_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_eq_pd = _mm_cmpeq_pd(lhs.xy, rhs.xy);
		__m128d zw_eq_pd = _mm_cmpeq_pd(lhs.zw, rhs.zw);
		return (_mm_movemask_pd(xy_eq_pd) | _mm_movemask_pd(zw_eq_pd)) != 0;
#elif defined(RTM_WASM128_INTRINSICS)
		return rtm_impl::mask4q_any_true(wasm_f64x2_eq(lhs.xy, rhs.xy), wasm_f64x2_eq(lhs.zw, rhs.zw));
#elif defined(RTM_NEON64_INTRINSICS)
		return rtm_impl::mask4q_any_true(vceqq_f64(lhs.xy, rhs.xy), vceqq_f64(lhs.zw, rhs.zw));
#else
		return lhs.x == rhs.x || lhs.y == rhs.y || lhs.z == rhs.z || lhs.w == rhs.w;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if any [xy] components are equal, otherwise false: any(lhs.xy == rhs.xy)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL vector_any_equal2(vector4d_arg0 lhs, vector4d_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_eq_pd = _mm_cmpeq_pd(lhs.xy, rhs.xy);
		return _mm_movemask_pd(xy_eq_pd) != 0;
#elif defined(RTM_WASM128_INTRINSICS)
		return rtm_impl::mask2q_any_true(wasm_f64x2_eq(lhs.xy, rhs.xy));
#elif defined(RTM_NEON64_INTRINSICS)
		return rtm_impl::mask2q_any_true(vceqq_f64(lhs.xy, rhs.xy));
#else
		return lhs.x == rhs.x || lhs.y == rhs.y;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if any [xyz] components are equal, otherwise false: any(lhs.xyz == rhs.xyz)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL vector_any_equal3(vector4d_arg0 lhs, vector4d_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_eq_pd = _mm_cmpeq_pd(lhs.xy, rhs.xy);
		__m128d zw_eq_pd = _mm_cmpeq_pd(lhs.zw, rhs.zw);
		return _mm_movemask_pd(xy_eq_pd) != 0 || (_mm_movemask_pd(zw_eq_pd) & 1) != 0;
#elif defined(RTM_WASM128_INTRINSICS)
		return rtm_impl::mask4q_any_true3(wasm_f64x2_eq(lhs.xy, rhs.xy), wasm_f64x2_eq(lhs.zw, rhs.zw));
#elif defined(RTM_NEON64_INTRINSICS)
		return rtm_impl::mask4q_any_true3(vceqq_f64(lhs.xy, rhs.xy), vceqq_f64(lhs.zw, rhs.zw));
#else
		return lhs.x == rhs.x || lhs.y == rhs.y || lhs.z == rhs.z;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all [xyzw] components are not equal, otherwise false: all(lhs.xyzw != rhs.xyzw)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL vector_all_not_equal(vector4d_arg0 lhs, vector4d_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_eq_pd = _mm_cmpneq_pd(lhs.xy, rhs.xy);
		__m128d zw_eq_pd = _mm_cmpneq_pd(lhs.zw, rhs.zw);
		return (_mm_movemask_pd(xy_eq_pd) & _mm_movemask_pd(zw_eq_pd)) == 3;
#elif defined(RTM_WASM128_INTRINSICS)
		// All the components are not equal if none of the components are equal
		return !rtm_impl::mask4q_any_true(wasm_f64x2_eq(lhs.xy, rhs.xy), wasm_f64x2_eq(lhs.zw, rhs.zw));
#elif defined(RTM_NEON64_INTRINSICS)
		// All the components are not equal if none of the components are equal
		return !rtm_impl::mask4q_any_true(vceqq_f64(lhs.xy, rhs.xy), vceqq_f64(lhs.zw, rhs.zw));
#else
		return lhs.x != rhs.x && lhs.y != rhs.y && lhs.z != rhs.z && lhs.w != rhs.w;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all [xy] components are not equal, otherwise false: all(lhs.xy != rhs.xy)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL vector_all_not_equal2(vector4d_arg0 lhs, vector4d_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_eq_pd = _mm_cmpneq_pd(lhs.xy, rhs.xy);
		return _mm_movemask_pd(xy_eq_pd) == 3;
#elif defined(RTM_WASM128_INTRINSICS)
		// All the components are not equal if none of the components are equal
		return !rtm_impl::mask2q_any_true(wasm_f64x2_eq(lhs.xy, rhs.xy));
#elif defined(RTM_NEON64_INTRINSICS)
		// All the components are not equal if none of the components are equal
		return !rtm_impl::mask2q_any_true(vceqq_f64(lhs.xy, rhs.xy));
#else
		return lhs.x != rhs.x && lhs.y != rhs.y;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all [xyz] components are not equal, otherwise false: all(lhs.xyz != rhs.xyz)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL vector_all_not_equal3(vector4d_arg0 lhs, vector4d_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_eq_pd = _mm_cmpneq_pd(lhs.xy, rhs.xy);
		__m128d zw_eq_pd = _mm_cmpneq_pd(lhs.zw, rhs.zw);
		return _mm_movemask_pd(xy_eq_pd) == 3 && (_mm_movemask_pd(zw_eq_pd) & 1) != 0;
#elif defined(RTM_WASM128_INTRINSICS)
		// All the components are not equal if none of the components are equal
		return !rtm_impl::mask4q_any_true3(wasm_f64x2_eq(lhs.xy, rhs.xy), wasm_f64x2_eq(lhs.zw, rhs.zw));
#elif defined(RTM_NEON64_INTRINSICS)
		// All the components are not equal if none of the components are equal
		return !rtm_impl::mask4q_any_true3(vceqq_f64(lhs.xy, rhs.xy), vceqq_f64(lhs.zw, rhs.zw));
#else
		return lhs.x != rhs.x && lhs.y != rhs.y && lhs.z != rhs.z;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if any [xyzw] components are not equal, otherwise false: any(lhs.xyzw != rhs.xyzw)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL vector_any_not_equal(vector4d_arg0 lhs, vector4d_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_eq_pd = _mm_cmpneq_pd(lhs.xy, rhs.xy);
		__m128d zw_eq_pd = _mm_cmpneq_pd(lhs.zw, rhs.zw);
		return (_mm_movemask_pd(xy_eq_pd) | _mm_movemask_pd(zw_eq_pd)) != 0;
#elif defined(RTM_WASM128_INTRINSICS)
		// One or more components are not equal if not all the components are equal
		return !rtm_impl::mask4q_all_true(wasm_f64x2_eq(lhs.xy, rhs.xy), wasm_f64x2_eq(lhs.zw, rhs.zw));
#elif defined(RTM_NEON64_INTRINSICS)
		// One or more components are not equal if not all the components are equal
		return !rtm_impl::mask4q_all_true(vceqq_f64(lhs.xy, rhs.xy), vceqq_f64(lhs.zw, rhs.zw));
#else
		return lhs.x != rhs.x || lhs.y != rhs.y || lhs.z != rhs.z || lhs.w != rhs.w;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if any [xy] components are not equal, otherwise false: any(lhs.xy != rhs.xy)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL vector_any_not_equal2(vector4d_arg0 lhs, vector4d_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_eq_pd = _mm_cmpneq_pd(lhs.xy, rhs.xy);
		return _mm_movemask_pd(xy_eq_pd) != 0;
#elif defined(RTM_WASM128_INTRINSICS)
		// One or more components are not equal if not all the components are equal
		return !rtm_impl::mask2q_all_true(wasm_f64x2_eq(lhs.xy, rhs.xy));
#elif defined(RTM_NEON64_INTRINSICS)
		// One or more components are not equal if not all the components are equal
		return !rtm_impl::mask2q_all_true(vceqq_f64(lhs.xy, rhs.xy));
#else
		return lhs.x != rhs.x || lhs.y != rhs.y;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if any [xyz] components are not equal, otherwise false: any(lhs.xyz != rhs.xyz)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL vector_any_not_equal3(vector4d_arg0 lhs, vector4d_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_eq_pd = _mm_cmpneq_pd(lhs.xy, rhs.xy);
		__m128d zw_eq_pd = _mm_cmpneq_pd(lhs.zw, rhs.zw);
		return _mm_movemask_pd(xy_eq_pd) != 0 || (_mm_movemask_pd(zw_eq_pd) & 1) != 0;
#elif defined(RTM_WASM128_INTRINSICS)
		// One or more components are not equal if not all the components are equal
		return !rtm_impl::mask4q_all_true3(wasm_f64x2_eq(lhs.xy, rhs.xy), wasm_f64x2_eq(lhs.zw, rhs.zw));
#elif defined(RTM_NEON64_INTRINSICS)
		// One or more components are not equal if not all the components are equal
		return !rtm_impl::mask4q_all_true3(vceqq_f64(lhs.xy, rhs.xy), vceqq_f64(lhs.zw, rhs.zw));
#else
		return lhs.x != rhs.x || lhs.y != rhs.y || lhs.z != rhs.z;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all 4 components are near equal, otherwise false: all(abs(lhs - rhs).xyzw <= threshold)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL vector_all_near_equal(vector4d_arg0 lhs, vector4d_arg1 rhs, double threshold = 0.00001) RTM_NO_EXCEPT
	{
		return vector_all_less_equal(vector_abs(vector_sub(lhs, rhs)), vector_set(threshold));
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all [xy] components are near equal, otherwise false: all(abs(lhs - rhs).xy <= threshold)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL vector_all_near_equal2(vector4d_arg0 lhs, vector4d_arg1 rhs, double threshold = 0.00001) RTM_NO_EXCEPT
	{
		return vector_all_less_equal2(vector_abs(vector_sub(lhs, rhs)), vector_set(threshold));
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all [xyz] components are near equal, otherwise false: all(abs(lhs - rhs).xyz <= threshold)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL vector_all_near_equal3(vector4d_arg0 lhs, vector4d_arg1 rhs, double threshold = 0.00001) RTM_NO_EXCEPT
	{
		return vector_all_less_equal3(vector_abs(vector_sub(lhs, rhs)), vector_set(threshold));
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if any 4 components are near equal, otherwise false: any(abs(lhs - rhs).xyzw <= threshold)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL vector_any_near_equal(vector4d_arg0 lhs, vector4d_arg1 rhs, double threshold = 0.00001) RTM_NO_EXCEPT
	{
		return vector_any_less_equal(vector_abs(vector_sub(lhs, rhs)), vector_set(threshold));
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if any [xy] components are near equal, otherwise false: any(abs(lhs - rhs).xy <= threshold)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL vector_any_near_equal2(vector4d_arg0 lhs, vector4d_arg1 rhs, double threshold = 0.00001) RTM_NO_EXCEPT
	{
		return vector_any_less_equal2(vector_abs(vector_sub(lhs, rhs)), vector_set(threshold));
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if any [xyz] components are near equal, otherwise false: any(abs(lhs - rhs).xyz <= threshold)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL vector_any_near_equal3(vector4d_arg0 lhs, vector4d_arg1 rhs, double threshold = 0.00001) RTM_NO_EXCEPT
	{
		return vector_any_less_equal3(vector_abs(vector_sub(lhs, rhs)), vector_set(threshold));
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns per component ~0 if input is finite, otherwise 0: finite(input) ? ~0 : 0
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE mask4d RTM_SIMD_CALL vector_finite(vector4d_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		const __m128i abs_mask = _mm_set_epi64x(0x7FFFFFFFFFFFFFFFULL, 0x7FFFFFFFFFFFFFFFULL);
		__m128d abs_input_xy = _mm_and_pd(input.xy, _mm_castsi128_pd(abs_mask));
		__m128d abs_input_zw = _mm_and_pd(input.zw, _mm_castsi128_pd(abs_mask));

		const __m128d infinity = _mm_set1_pd(std::numeric_limits<double>::infinity());
		__m128d is_not_infinity_xy = _mm_cmpneq_pd(abs_input_xy, infinity);
		__m128d is_not_infinity_zw = _mm_cmpneq_pd(abs_input_zw, infinity);

		__m128d is_nan_xy = _mm_cmpneq_pd(input.xy, input.xy);
		__m128d is_nan_zw = _mm_cmpneq_pd(input.zw, input.zw);

		__m128d is_finite_xy = _mm_andnot_pd(is_nan_xy, is_not_infinity_xy);
		__m128d is_finite_zw = _mm_andnot_pd(is_nan_zw, is_not_infinity_zw);
		return mask4d{ is_finite_xy, is_finite_zw };
#elif defined(RTM_WASM128_INTRINSICS)
		// The absolute value is less than infinity only if the value is finite: NaN and infinity are false
		const v128_t infinity = wasm_f64x2_splat(std::numeric_limits<double>::infinity());
		return mask4d{ wasm_f64x2_lt(wasm_f64x2_abs(input.xy), infinity), wasm_f64x2_lt(wasm_f64x2_abs(input.zw), infinity) };
#elif defined(RTM_NEON64_INTRINSICS)
		// The absolute value is less than infinity only if the value is finite: NaN and infinity are false
		const float64x2_t infinity = vdupq_n_f64(std::numeric_limits<double>::infinity());
		return mask4d{ vcltq_f64(vabsq_f64(input.xy), infinity), vcltq_f64(vabsq_f64(input.zw), infinity) };
#else
		return mask4d{
			rtm_impl::get_mask_value(scalar_is_finite(vector_get_x(input))),
			rtm_impl::get_mask_value(scalar_is_finite(vector_get_y(input))),
			rtm_impl::get_mask_value(scalar_is_finite(vector_get_z(input))),
			rtm_impl::get_mask_value(scalar_is_finite(vector_get_w(input)))
			};
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all 4 components are finite (not NaN/Inf), otherwise false: all(finite(input))
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL vector_is_finite(vector4d_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		const __m128i abs_mask = _mm_set_epi64x(0x7FFFFFFFFFFFFFFFULL, 0x7FFFFFFFFFFFFFFFULL);
		__m128d abs_input_xy = _mm_and_pd(input.xy, _mm_castsi128_pd(abs_mask));
		__m128d abs_input_zw = _mm_and_pd(input.zw, _mm_castsi128_pd(abs_mask));

		const __m128d infinity = _mm_set1_pd(std::numeric_limits<double>::infinity());
		__m128d is_infinity_xy = _mm_cmpeq_pd(abs_input_xy, infinity);
		__m128d is_infinity_zw = _mm_cmpeq_pd(abs_input_zw, infinity);

		__m128d is_nan_xy = _mm_cmpneq_pd(input.xy, input.xy);
		__m128d is_nan_zw = _mm_cmpneq_pd(input.zw, input.zw);

		__m128d is_not_finite_xy = _mm_or_pd(is_infinity_xy, is_nan_xy);
		__m128d is_not_finite_zw = _mm_or_pd(is_infinity_zw, is_nan_zw);
		__m128d is_not_finite = _mm_or_pd(is_not_finite_xy, is_not_finite_zw);
		return _mm_movemask_pd(is_not_finite) == 0x0;
#elif defined(RTM_WASM128_INTRINSICS)
		// The absolute value is less than infinity only if the value is finite: NaN and infinity are false
		const v128_t infinity = wasm_f64x2_splat(std::numeric_limits<double>::infinity());
		return rtm_impl::mask4q_all_true(wasm_f64x2_lt(wasm_f64x2_abs(input.xy), infinity), wasm_f64x2_lt(wasm_f64x2_abs(input.zw), infinity));
#elif defined(RTM_NEON64_INTRINSICS)
		// The absolute value is less than infinity only if the value is finite: NaN and infinity are false
		const float64x2_t infinity = vdupq_n_f64(std::numeric_limits<double>::infinity());
		return rtm_impl::mask4q_all_true(vcltq_f64(vabsq_f64(input.xy), infinity), vcltq_f64(vabsq_f64(input.zw), infinity));
#else
		return scalar_is_finite(vector_get_x(input)) && scalar_is_finite(vector_get_y(input)) && scalar_is_finite(vector_get_z(input)) && scalar_is_finite(vector_get_w(input));
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all [xy] components are finite (not NaN/Inf), otherwise false: all(finite(input))
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL vector_is_finite2(vector4d_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		const __m128i abs_mask = _mm_set_epi64x(0x7FFFFFFFFFFFFFFFULL, 0x7FFFFFFFFFFFFFFFULL);
		__m128d abs_input_xy = _mm_and_pd(input.xy, _mm_castsi128_pd(abs_mask));

		const __m128d infinity = _mm_set1_pd(std::numeric_limits<double>::infinity());
		__m128d is_infinity_xy = _mm_cmpeq_pd(abs_input_xy, infinity);

		__m128d is_nan_xy = _mm_cmpneq_pd(input.xy, input.xy);

		__m128d is_not_finite_xy = _mm_or_pd(is_infinity_xy, is_nan_xy);
		return _mm_movemask_pd(is_not_finite_xy) == 0x0;
#elif defined(RTM_WASM128_INTRINSICS)
		// The absolute value is less than infinity only if the value is finite: NaN and infinity are false
		const v128_t infinity = wasm_f64x2_splat(std::numeric_limits<double>::infinity());
		return rtm_impl::mask2q_all_true(wasm_f64x2_lt(wasm_f64x2_abs(input.xy), infinity));
#elif defined(RTM_NEON64_INTRINSICS)
		// The absolute value is less than infinity only if the value is finite: NaN and infinity are false
		const float64x2_t infinity = vdupq_n_f64(std::numeric_limits<double>::infinity());
		return rtm_impl::mask2q_all_true(vcltq_f64(vabsq_f64(input.xy), infinity));
#else
		return scalar_is_finite(vector_get_x(input)) && scalar_is_finite(vector_get_y(input));
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all [xyz] components are finite (not NaN/Inf), otherwise false: all(finite(input))
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL vector_is_finite3(vector4d_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		const __m128i abs_mask = _mm_set_epi64x(0x7FFFFFFFFFFFFFFFULL, 0x7FFFFFFFFFFFFFFFULL);
		__m128d abs_input_xy = _mm_and_pd(input.xy, _mm_castsi128_pd(abs_mask));
		__m128d abs_input_zw = _mm_and_pd(input.zw, _mm_castsi128_pd(abs_mask));

		const __m128d infinity = _mm_set1_pd(std::numeric_limits<double>::infinity());
		__m128d is_infinity_xy = _mm_cmpeq_pd(abs_input_xy, infinity);
		__m128d is_infinity_zw = _mm_cmpeq_pd(abs_input_zw, infinity);

		__m128d is_nan_xy = _mm_cmpneq_pd(input.xy, input.xy);
		__m128d is_nan_zw = _mm_cmpneq_pd(input.zw, input.zw);

		__m128d is_not_finite_xy = _mm_or_pd(is_infinity_xy, is_nan_xy);
		__m128d is_not_finite_zw = _mm_or_pd(is_infinity_zw, is_nan_zw);
		return _mm_movemask_pd(is_not_finite_xy) == 0 && (_mm_movemask_pd(is_not_finite_zw) & 0x1) == 0;
#elif defined(RTM_WASM128_INTRINSICS)
		// The absolute value is less than infinity only if the value is finite: NaN and infinity are false
		const v128_t infinity = wasm_f64x2_splat(std::numeric_limits<double>::infinity());
		return rtm_impl::mask4q_all_true3(wasm_f64x2_lt(wasm_f64x2_abs(input.xy), infinity), wasm_f64x2_lt(wasm_f64x2_abs(input.zw), infinity));
#elif defined(RTM_NEON64_INTRINSICS)
		// The absolute value is less than infinity only if the value is finite: NaN and infinity are false
		const float64x2_t infinity = vdupq_n_f64(std::numeric_limits<double>::infinity());
		return rtm_impl::mask4q_all_true3(vcltq_f64(vabsq_f64(input.xy), infinity), vcltq_f64(vabsq_f64(input.zw), infinity));
#else
		return scalar_is_finite(vector_get_x(input)) && scalar_is_finite(vector_get_y(input)) && scalar_is_finite(vector_get_z(input));
#endif
	}



	//////////////////////////////////////////////////////////////////////////
	// Swizzling, permutations, and mixing
	//////////////////////////////////////////////////////////////////////////


	//////////////////////////////////////////////////////////////////////////
	// Per component selection depending on the mask: mask != 0 ? if_true : if_false
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_select(mask4d_arg0 mask, vector4d_arg1 if_true, vector4d_arg2 if_false) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy = RTM_VECTOR2D_SELECT(mask.xy, if_true.xy, if_false.xy);
		__m128d zw = RTM_VECTOR2D_SELECT(mask.zw, if_true.zw, if_false.zw);
		return vector4d{ xy, zw };
#elif defined(RTM_WASM128_INTRINSICS)
		v128_t xy = RTM_VECTOR2D_SELECT(mask.xy, if_true.xy, if_false.xy);
		v128_t zw = RTM_VECTOR2D_SELECT(mask.zw, if_true.zw, if_false.zw);
		return vector4d{ xy, zw };
#elif defined(RTM_NEON64_INTRINSICS)
		float64x2_t xy = RTM_VECTOR2D_SELECT(mask.xy, if_true.xy, if_false.xy);
		float64x2_t zw = RTM_VECTOR2D_SELECT(mask.zw, if_true.zw, if_false.zw);
		return vector4d{ xy, zw };
#else
		return vector4d{ rtm_impl::select(mask.x, if_true.x, if_false.x), rtm_impl::select(mask.y, if_true.y, if_false.y), rtm_impl::select(mask.z, if_true.z, if_false.z), rtm_impl::select(mask.w, if_true.w, if_false.w) };
#endif
	}

#if defined(RTM_NEON64_INTRINSICS)
	namespace rtm_impl
	{
		//////////////////////////////////////////////////////////////////////////
		// Returns the 2-lane register that holds the mix4 component: input0.xy, input0.zw, input1.xy, or input1.zw.
		//////////////////////////////////////////////////////////////////////////
		template<mix4 component>
		RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE float64x2_t RTM_SIMD_CALL vector_mix_get_pair(const vector4d& input0, const vector4d& input1) RTM_NO_EXCEPT
		{
			const vector4d& input = is_mix_xyzw(component) ? input0 : input1;
			return (uint32_t(component) % 4) < 2 ? input.xy : input.zw;
		}
	}
#elif defined(RTM_WASM128_INTRINSICS)
	namespace rtm_impl
	{
		//////////////////////////////////////////////////////////////////////////
		// Returns the 2-lane register that holds the mix4 component: input0.xy, input0.zw, input1.xy, or input1.zw.
		//////////////////////////////////////////////////////////////////////////
		template<mix4 component>
		RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE v128_t RTM_SIMD_CALL vector_mix_get_pair(const vector4d& input0, const vector4d& input1) RTM_NO_EXCEPT
		{
			const vector4d& input = is_mix_xyzw(component) ? input0 : input1;
			return (uint32_t(component) % 4) < 2 ? input.xy : input.zw;
		}
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Mixes two inputs and returns the desired components.
	// [xyzw] indexes into the first input while [abcd] indexes in the second.
	//////////////////////////////////////////////////////////////////////////
	template<mix4 comp0, mix4 comp1, mix4 comp2, mix4 comp3>
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_mix(vector4d_arg0 input0, vector4d_arg1 input1) RTM_NO_EXCEPT
	{
		// Exactly input 0
		if (rtm_impl::static_condition<comp0 == mix4::x && comp1 == mix4::y && comp2 == mix4::z && comp3 == mix4::w>::test())
			return input0;

		// Exactly input 1
		if (rtm_impl::static_condition<comp0 == mix4::a && comp1 == mix4::b && comp2 == mix4::c && comp3 == mix4::d>::test())
			return input1;

#if defined(RTM_SSE2_INTRINSICS)
		// We do the same treatment for each half of the result pair

		__m128d result_xy;
		__m128d result_zw;

		if (rtm_impl::static_condition<comp0 == mix4::x && comp1 == mix4::y>::test())
		{
			result_xy = input0.xy;
		}

		else if (rtm_impl::static_condition<comp0 == mix4::z && comp1 == mix4::w>::test())
		{
			result_xy = input0.zw;
		}

		else if (rtm_impl::static_condition<comp0 == mix4::a && comp1 == mix4::b>::test())
		{
			result_xy = input1.xy;
		}

		else if (rtm_impl::static_condition<comp0 == mix4::c && comp1 == mix4::d>::test())
		{
			result_xy = input1.zw;
		}

	#if defined(RTM_SSE4_INTRINSICS)
		// xy0 xy0	duplicate
		// xy0 zw0
		else if (rtm_impl::static_condition<(comp0 == mix4::x || comp0 == mix4::z) && (comp1 == mix4::y || comp1 == mix4::w)>::test())
		{
			constexpr int mask = (comp0 == mix4::z ? 1 : 0) | (comp1 == mix4::w ? 2 : 0);
			result_xy = _mm_blend_pd(input0.xy, input0.zw, mask);
		}

		// xy0 xy1
		else if (rtm_impl::static_condition<(comp0 == mix4::x || comp0 == mix4::a) && (comp1 == mix4::y || comp1 == mix4::b)>::test())
		{
			constexpr int mask = (comp0 == mix4::a ? 1 : 0) | (comp1 == mix4::b ? 2 : 0);
			result_xy = _mm_blend_pd(input0.xy, input1.xy, mask);
		}

		// xy0 zw1
		else if (rtm_impl::static_condition<(comp0 == mix4::x || comp0 == mix4::c) && (comp1 == mix4::y || comp1 == mix4::d)>::test())
		{
			constexpr int mask = (comp0 == mix4::c ? 1 : 0) | (comp1 == mix4::d ? 2 : 0);
			result_xy = _mm_blend_pd(input0.xy, input1.zw, mask);
		}

		// xy1 xy0	duplicate
		// xy1 zw0
		else if (rtm_impl::static_condition<(comp0 == mix4::a || comp0 == mix4::z) && (comp1 == mix4::b || comp1 == mix4::w)>::test())
		{
			constexpr int mask = (comp0 == mix4::z ? 1 : 0) | (comp1 == mix4::w ? 2 : 0);
			result_xy = _mm_blend_pd(input1.xy, input0.zw, mask);
		}

		// xy1 xy1	duplicate
		// xy1 zw1
		else if (rtm_impl::static_condition<(comp0 == mix4::a || comp0 == mix4::c) && (comp1 == mix4::b || comp1 == mix4::d)>::test())
		{
			constexpr int mask = (comp0 == mix4::c ? 1 : 0) | (comp1 == mix4::d ? 2 : 0);
			result_xy = _mm_blend_pd(input1.xy, input1.zw, mask);
		}
	#endif // defined(RTM_SSE4_INTRINSICS)

	#if defined(RTM_SSE3_INTRINSICS)
		else if (rtm_impl::static_condition<comp0 == mix4::x && comp1 == mix4::x>::test())
			result_xy = _mm_movedup_pd(input0.xy);

		else if (rtm_impl::static_condition<comp0 == mix4::z && comp1 == mix4::z>::test())
			result_xy = _mm_movedup_pd(input0.zw);

		else if (rtm_impl::static_condition<comp0 == mix4::a && comp1 == mix4::a>::test())
			result_xy = _mm_movedup_pd(input1.xy);

		else if (rtm_impl::static_condition<comp0 == mix4::c && comp1 == mix4::c>::test())
			result_xy = _mm_movedup_pd(input1.zw);
	#endif // defined(RTM_SSE3_INTRINSICS)

		// xy0 xy0	duplicate
		// xy0 zw0
		else if (rtm_impl::static_condition<comp0 == mix4::z && comp1 == mix4::y>::test())
			result_xy = _mm_move_sd(input0.xy, input0.zw);

		// xy0 xy1
		else if (rtm_impl::static_condition<comp0 == mix4::a && comp1 == mix4::y>::test())
			result_xy = _mm_move_sd(input0.xy, input1.xy);

		// xy0 zw1
		else if (rtm_impl::static_condition<comp0 == mix4::c && comp1 == mix4::y>::test())
			result_xy = _mm_move_sd(input0.xy, input1.zw);

		// xy1 xy0
		else if (rtm_impl::static_condition<comp0 == mix4::x && comp1 == mix4::b>::test())
			result_xy = _mm_move_sd(input1.xy, input0.xy);

		// xy1 zw0
		else if (rtm_impl::static_condition<comp0 == mix4::z && comp1 == mix4::b>::test())
			result_xy = _mm_move_sd(input1.xy, input0.zw);

		// xy1 xy1	duplicate
		// xy1 zw1
		else if (rtm_impl::static_condition<comp0 == mix4::c && comp1 == mix4::b>::test())
			result_xy = _mm_move_sd(input1.xy, input1.zw);

		// zw0 xy0
		else if (rtm_impl::static_condition<comp0 == mix4::x && comp1 == mix4::w>::test())
			result_xy = _mm_move_sd(input0.zw, input0.xy);

		// zw0 zw0 duplicate
		// zw0 xy1
		else if (rtm_impl::static_condition<comp0 == mix4::a && comp1 == mix4::w>::test())
			result_xy = _mm_move_sd(input0.zw, input1.xy);

		// zw0 zw1
		else if (rtm_impl::static_condition<comp0 == mix4::c && comp1 == mix4::w>::test())
			result_xy = _mm_move_sd(input0.zw, input1.zw);

		// zw1 xy0
		else if (rtm_impl::static_condition<comp0 == mix4::x && comp1 == mix4::d>::test())
			result_xy = _mm_move_sd(input1.zw, input0.xy);

		// zw1 zw0
		else if (rtm_impl::static_condition<comp0 == mix4::z && comp1 == mix4::d>::test())
			result_xy = _mm_move_sd(input1.zw, input0.zw);

		// zw1 xy1
		// zw1 zw1 duplicate
		else if (rtm_impl::static_condition<comp0 == mix4::a && comp1 == mix4::d>::test())
			result_xy = _mm_move_sd(input1.zw, input1.xy);

		// xy0 xy0
		else if (rtm_impl::static_condition<comp0 == mix4::y && comp1 == mix4::y>::test())
			result_xy = _mm_unpackhi_pd(input0.xy, input0.xy);

		// xy0 zw0
		else if (rtm_impl::static_condition<comp0 == mix4::y && comp1 == mix4::w>::test())
			result_xy = _mm_unpackhi_pd(input0.xy, input0.zw);

		// xy0 xy1
		else if (rtm_impl::static_condition<comp0 == mix4::y && comp1 == mix4::b>::test())
			result_xy = _mm_unpackhi_pd(input0.xy, input1.xy);

		// xy0 zw1
		else if (rtm_impl::static_condition<comp0 == mix4::y && comp1 == mix4::d>::test())
			result_xy = _mm_unpackhi_pd(input0.xy, input1.zw);

		// xy1 xy0
		else if (rtm_impl::static_condition<comp0 == mix4::b && comp1 == mix4::y>::test())
			result_xy = _mm_unpackhi_pd(input1.xy, input0.xy);

		// xy1 zw0
		else if (rtm_impl::static_condition<comp0 == mix4::b && comp1 == mix4::w>::test())
			result_xy = _mm_unpackhi_pd(input1.xy, input0.zw);

		// xy1 xy1
		else if (rtm_impl::static_condition<comp0 == mix4::b && comp1 == mix4::b>::test())
			result_xy = _mm_unpackhi_pd(input1.xy, input1.xy);

		// xy1 zw1
		else if (rtm_impl::static_condition<comp0 == mix4::b && comp1 == mix4::d>::test())
			result_xy = _mm_unpackhi_pd(input1.xy, input1.zw);

		// zw0 xy0
		else if (rtm_impl::static_condition<comp0 == mix4::w && comp1 == mix4::y>::test())
			result_xy = _mm_unpackhi_pd(input0.zw, input0.xy);

		// zw0 zw0
		else if (rtm_impl::static_condition<comp0 == mix4::w && comp1 == mix4::w>::test())
			result_xy = _mm_unpackhi_pd(input0.zw, input0.zw);

		// zw0 xy1
		else if (rtm_impl::static_condition<comp0 == mix4::w && comp1 == mix4::b>::test())
			result_xy = _mm_unpackhi_pd(input0.zw, input1.xy);

		// zw0 zw1
		else if (rtm_impl::static_condition<comp0 == mix4::w && comp1 == mix4::d>::test())
			result_xy = _mm_unpackhi_pd(input0.zw, input1.zw);

		// zw1 xy0
		else if (rtm_impl::static_condition<comp0 == mix4::d && comp1 == mix4::y>::test())
			result_xy = _mm_unpackhi_pd(input1.zw, input0.xy);

		// zw1 zw0
		else if (rtm_impl::static_condition<comp0 == mix4::d && comp1 == mix4::w>::test())
			result_xy = _mm_unpackhi_pd(input1.zw, input0.zw);

		// zw1 xy1
		else if (rtm_impl::static_condition<comp0 == mix4::d && comp1 == mix4::b>::test())
			result_xy = _mm_unpackhi_pd(input1.zw, input1.xy);

		// zw1 zw1
		else if (rtm_impl::static_condition<comp0 == mix4::d && comp1 == mix4::d>::test())
			result_xy = _mm_unpackhi_pd(input1.zw, input1.zw);

		// xy0 xy0
		else if (rtm_impl::static_condition<comp0 == mix4::x && comp1 == mix4::x>::test())
			result_xy = _mm_unpacklo_pd(input0.xy, input0.xy);

		// xy0 zw0
		else if (rtm_impl::static_condition<comp0 == mix4::x && comp1 == mix4::z>::test())
			result_xy = _mm_unpacklo_pd(input0.xy, input0.zw);

		// xy0 xy1
		else if (rtm_impl::static_condition<comp0 == mix4::x && comp1 == mix4::a>::test())
			result_xy = _mm_unpacklo_pd(input0.xy, input1.xy);

		// xy0 zw1
		else if (rtm_impl::static_condition<comp0 == mix4::x && comp1 == mix4::c>::test())
			result_xy = _mm_unpacklo_pd(input0.xy, input1.zw);

		// xy1 xy0
		else if (rtm_impl::static_condition<comp0 == mix4::a && comp1 == mix4::x>::test())
			result_xy = _mm_unpacklo_pd(input1.xy, input0.xy);

		// xy1 zw0
		else if (rtm_impl::static_condition<comp0 == mix4::a && comp1 == mix4::z>::test())
			result_xy = _mm_unpacklo_pd(input1.xy, input0.zw);

		// xy1 xy1
		else if (rtm_impl::static_condition<comp0 == mix4::a && comp1 == mix4::a>::test())
			result_xy = _mm_unpacklo_pd(input1.xy, input1.xy);

		// xy1 zw1
		else if (rtm_impl::static_condition<comp0 == mix4::a && comp1 == mix4::c>::test())
			result_xy = _mm_unpacklo_pd(input1.xy, input1.zw);

		// zw0 xy0
		else if (rtm_impl::static_condition<comp0 == mix4::z && comp1 == mix4::x>::test())
			result_xy = _mm_unpacklo_pd(input0.zw, input0.xy);

		// zw0 zw0
		else if (rtm_impl::static_condition<comp0 == mix4::z && comp1 == mix4::z>::test())
			result_xy = _mm_unpacklo_pd(input0.zw, input0.zw);

		// zw0 xy1
		else if (rtm_impl::static_condition<comp0 == mix4::z && comp1 == mix4::a>::test())
			result_xy = _mm_unpacklo_pd(input0.zw, input1.xy);

		// zw0 zw1
		else if (rtm_impl::static_condition<comp0 == mix4::z && comp1 == mix4::c>::test())
			result_xy = _mm_unpacklo_pd(input0.zw, input1.zw);

		// zw1 xy0
		else if (rtm_impl::static_condition<comp0 == mix4::c && comp1 == mix4::x>::test())
			result_xy = _mm_unpacklo_pd(input1.zw, input0.xy);

		// zw1 zw0
		else if (rtm_impl::static_condition<comp0 == mix4::c && comp1 == mix4::z>::test())
			result_xy = _mm_unpacklo_pd(input1.zw, input0.zw);

		// zw1 xy1
		else if (rtm_impl::static_condition<comp0 == mix4::c && comp1 == mix4::a>::test())
			result_xy = _mm_unpacklo_pd(input1.zw, input1.xy);

		// zw1 zw1
		else if (rtm_impl::static_condition<comp0 == mix4::c && comp1 == mix4::c>::test())
			result_xy = _mm_unpacklo_pd(input1.zw, input1.zw);

		// xy0 xy0
		else if (rtm_impl::static_condition<(comp0 == mix4::x || comp0 == mix4::y) && (comp1 == mix4::x || comp1 == mix4::y)>::test())
			result_xy = _mm_shuffle_pd(input0.xy, input0.xy, _MM_SHUFFLE2(int(comp1) % 2, int(comp0) % 2));

		// xy0 zw0
		else if (rtm_impl::static_condition<(comp0 == mix4::x || comp0 == mix4::y) && (comp1 == mix4::z || comp1 == mix4::w)>::test())
			result_xy = _mm_shuffle_pd(input0.xy, input0.zw, _MM_SHUFFLE2(int(comp1) % 2, int(comp0) % 2));

		// xy0 xy1
		else if (rtm_impl::static_condition<(comp0 == mix4::x || comp0 == mix4::y) && (comp1 == mix4::a || comp1 == mix4::b)>::test())
			result_xy = _mm_shuffle_pd(input0.xy, input1.xy, _MM_SHUFFLE2(int(comp1) % 2, int(comp0) % 2));

		// xy0 zw1
		else if (rtm_impl::static_condition<(comp0 == mix4::x || comp0 == mix4::y) && (comp1 == mix4::c || comp1 == mix4::d)>::test())
			result_xy = _mm_shuffle_pd(input0.xy, input1.zw, _MM_SHUFFLE2(int(comp1) % 2, int(comp0) % 2));

		// xy1 xy0
		else if (rtm_impl::static_condition<(comp0 == mix4::a || comp0 == mix4::b) && (comp1 == mix4::x || comp1 == mix4::y)>::test())
			result_xy = _mm_shuffle_pd(input1.xy, input0.xy, _MM_SHUFFLE2(int(comp1) % 2, int(comp0) % 2));

		// xy1 zw0
		else if (rtm_impl::static_condition<(comp0 == mix4::a || comp0 == mix4::b) && (comp1 == mix4::z || comp1 == mix4::w)>::test())
			result_xy = _mm_shuffle_pd(input1.xy, input0.zw, _MM_SHUFFLE2(int(comp1) % 2, int(comp0) % 2));

		// xy1 xy1
		else if (rtm_impl::static_condition<(comp0 == mix4::a || comp0 == mix4::b) && (comp1 == mix4::a || comp1 == mix4::b)>::test())
			result_xy = _mm_shuffle_pd(input1.xy, input1.xy, _MM_SHUFFLE2(int(comp1) % 2, int(comp0) % 2));

		// xy1 zw1
		else if (rtm_impl::static_condition<(comp0 == mix4::a || comp0 == mix4::b) && (comp1 == mix4::c || comp1 == mix4::d)>::test())
			result_xy = _mm_shuffle_pd(input1.xy, input1.zw, _MM_SHUFFLE2(int(comp1) % 2, int(comp0) % 2));

		// zw0 xy0
		else if (rtm_impl::static_condition<(comp0 == mix4::z || comp0 == mix4::w) && (comp1 == mix4::x || comp1 == mix4::y)>::test())
			result_xy = _mm_shuffle_pd(input0.zw, input0.xy, _MM_SHUFFLE2(int(comp1) % 2, int(comp0) % 2));

		// zw0 zw0
		else if (rtm_impl::static_condition<(comp0 == mix4::z || comp0 == mix4::w) && (comp1 == mix4::z || comp1 == mix4::w)>::test())
			result_xy = _mm_shuffle_pd(input0.zw, input0.zw, _MM_SHUFFLE2(int(comp1) % 2, int(comp0) % 2));

		// zw0 xy1
		else if (rtm_impl::static_condition<(comp0 == mix4::z || comp0 == mix4::w) && (comp1 == mix4::a || comp1 == mix4::b)>::test())
			result_xy = _mm_shuffle_pd(input0.zw, input1.xy, _MM_SHUFFLE2(int(comp1) % 2, int(comp0) % 2));

		// zw0 zw1
		else if (rtm_impl::static_condition<(comp0 == mix4::z || comp0 == mix4::w) && (comp1 == mix4::c || comp1 == mix4::d)>::test())
			result_xy = _mm_shuffle_pd(input0.zw, input1.zw, _MM_SHUFFLE2(int(comp1) % 2, int(comp0) % 2));

		// zw1 xy0
		else if (rtm_impl::static_condition<(comp0 == mix4::c || comp0 == mix4::d) && (comp1 == mix4::x || comp1 == mix4::y)>::test())
			result_xy = _mm_shuffle_pd(input1.zw, input0.xy, _MM_SHUFFLE2(int(comp1) % 2, int(comp0) % 2));

		// zw1 zw0
		else if (rtm_impl::static_condition<(comp0 == mix4::c || comp0 == mix4::d) && (comp1 == mix4::z || comp1 == mix4::w)>::test())
			result_xy = _mm_shuffle_pd(input1.zw, input0.zw, _MM_SHUFFLE2(int(comp1) % 2, int(comp0) % 2));

		// zw1 xy1
		else if (rtm_impl::static_condition<(comp0 == mix4::c || comp0 == mix4::d) && (comp1 == mix4::a || comp1 == mix4::b)>::test())
			result_xy = _mm_shuffle_pd(input1.zw, input1.xy, _MM_SHUFFLE2(int(comp1) % 2, int(comp0) % 2));

		// zw1 zw1
		else if (rtm_impl::static_condition<(comp0 == mix4::c || comp0 == mix4::d) && (comp1 == mix4::c || comp1 == mix4::d)>::test())
			result_xy = _mm_shuffle_pd(input1.zw, input1.zw, _MM_SHUFFLE2(int(comp1) % 2, int(comp0) % 2));

		else
			result_xy = _mm_setzero_pd();	// Set a dummy value, never used

		if (rtm_impl::static_condition<comp2 == mix4::x && comp3 == mix4::y>::test())
		{
			result_zw = input0.xy;
		}

		else if (rtm_impl::static_condition<comp2 == mix4::z && comp3 == mix4::w>::test())
		{
			result_zw = input0.zw;
		}

		else if (rtm_impl::static_condition<comp2 == mix4::a && comp3 == mix4::b>::test())
		{
			result_zw = input1.xy;
		}

		else if (rtm_impl::static_condition<comp2 == mix4::c && comp3 == mix4::d>::test())
		{
			result_zw = input1.zw;
		}

	#if defined(RTM_SSE4_INTRINSICS)
		// xy0 xy0	duplicate
		// xy0 zw0
		else if (rtm_impl::static_condition<(comp2 == mix4::x || comp2 == mix4::z) && (comp3 == mix4::y || comp3 == mix4::w)>::test())
		{
			constexpr int mask = (comp2 == mix4::z ? 1 : 0) | (comp3 == mix4::w ? 2 : 0);
			result_zw = _mm_blend_pd(input0.xy, input0.zw, mask);
		}

		// xy0 xy1
		else if (rtm_impl::static_condition<(comp2 == mix4::x || comp2 == mix4::a) && (comp3 == mix4::y || comp3 == mix4::b)>::test())
		{
			constexpr int mask = (comp2 == mix4::a ? 1 : 0) | (comp3 == mix4::b ? 2 : 0);
			result_zw = _mm_blend_pd(input0.xy, input1.xy, mask);
		}

		// xy0 zw1
		else if (rtm_impl::static_condition<(comp2 == mix4::x || comp2 == mix4::c) && (comp3 == mix4::y || comp3 == mix4::d)>::test())
		{
			constexpr int mask = (comp2 == mix4::c ? 1 : 0) | (comp3 == mix4::d ? 2 : 0);
			result_zw = _mm_blend_pd(input0.xy, input1.zw, mask);
		}

		// xy1 xy0	duplicate
		// xy1 zw0
		else if (rtm_impl::static_condition<(comp2 == mix4::a || comp2 == mix4::z) && (comp3 == mix4::b || comp3 == mix4::w)>::test())
		{
			constexpr int mask = (comp2 == mix4::z ? 1 : 0) | (comp3 == mix4::w ? 2 : 0);
			result_zw = _mm_blend_pd(input1.xy, input0.zw, mask);
		}

		// xy1 xy1	duplicate
		// xy1 zw1
		else if (rtm_impl::static_condition<(comp2 == mix4::a || comp2 == mix4::c) && (comp3 == mix4::b || comp3 == mix4::d)>::test())
		{
			constexpr int mask = (comp2 == mix4::c ? 1 : 0) | (comp3 == mix4::d ? 2 : 0);
			result_zw = _mm_blend_pd(input1.xy, input1.zw, mask);
		}
	#endif // defined(RTM_SSE4_INTRINSICS)

	#if defined(RTM_SSE3_INTRINSICS)
		else if (rtm_impl::static_condition<comp2 == mix4::x && comp3 == mix4::x>::test())
			result_zw = _mm_movedup_pd(input0.xy);

		else if (rtm_impl::static_condition<comp2 == mix4::z && comp3 == mix4::z>::test())
			result_zw = _mm_movedup_pd(input0.zw);

		else if (rtm_impl::static_condition<comp2 == mix4::a && comp3 == mix4::a>::test())
			result_zw = _mm_movedup_pd(input1.xy);

		else if (rtm_impl::static_condition<comp2 == mix4::c && comp3 == mix4::c>::test())
			result_zw = _mm_movedup_pd(input1.zw);
	#endif // defined(RTM_SSE3_INTRINSICS)

		// xy0 xy0	duplicate
		// xy0 zw0
		else if (rtm_impl::static_condition<comp2 == mix4::z && comp3 == mix4::y>::test())
			result_zw = _mm_move_sd(input0.xy, input0.zw);

		// xy0 xy1
		else if (rtm_impl::static_condition<comp2 == mix4::a && comp3 == mix4::y>::test())
			result_zw = _mm_move_sd(input0.xy, input1.xy);

		// xy0 zw1
		else if (rtm_impl::static_condition<comp2 == mix4::c && comp3 == mix4::y>::test())
			result_zw = _mm_move_sd(input0.xy, input1.zw);

		// xy1 xy0
		else if (rtm_impl::static_condition<comp2 == mix4::x && comp3 == mix4::b>::test())
			result_zw = _mm_move_sd(input1.xy, input0.xy);

		// xy1 zw0
		else if (rtm_impl::static_condition<comp2 == mix4::z && comp3 == mix4::b>::test())
			result_zw = _mm_move_sd(input1.xy, input0.zw);

		// xy1 xy1	duplicate
		// xy1 zw1
		else if (rtm_impl::static_condition<comp2 == mix4::c && comp3 == mix4::b>::test())
			result_zw = _mm_move_sd(input1.xy, input1.zw);

		// zw0 xy0
		else if (rtm_impl::static_condition<comp2 == mix4::x && comp3 == mix4::w>::test())
			result_zw = _mm_move_sd(input0.zw, input0.xy);

		// zw0 zw0 duplicate
		// zw0 xy1
		else if (rtm_impl::static_condition<comp2 == mix4::a && comp3 == mix4::w>::test())
			result_zw = _mm_move_sd(input0.zw, input1.xy);

		// zw0 zw1
		else if (rtm_impl::static_condition<comp2 == mix4::c && comp3 == mix4::w>::test())
			result_zw = _mm_move_sd(input0.zw, input1.zw);

		// zw1 xy0
		else if (rtm_impl::static_condition<comp2 == mix4::x && comp3 == mix4::d>::test())
			result_zw = _mm_move_sd(input1.zw, input0.xy);

		// zw1 zw0
		else if (rtm_impl::static_condition<comp2 == mix4::z && comp3 == mix4::d>::test())
			result_zw = _mm_move_sd(input1.zw, input0.zw);

		// zw1 xy1
		// zw1 zw1 duplicate
		else if (rtm_impl::static_condition<comp2 == mix4::a && comp3 == mix4::d>::test())
			result_zw = _mm_move_sd(input1.zw, input1.xy);

		// xy0 xy0
		else if (rtm_impl::static_condition<comp2 == mix4::y && comp3 == mix4::y>::test())
			result_zw = _mm_unpackhi_pd(input0.xy, input0.xy);

		// xy0 zw0
		else if (rtm_impl::static_condition<comp2 == mix4::y && comp3 == mix4::w>::test())
			result_zw = _mm_unpackhi_pd(input0.xy, input0.zw);

		// xy0 xy1
		else if (rtm_impl::static_condition<comp2 == mix4::y && comp3 == mix4::b>::test())
			result_zw = _mm_unpackhi_pd(input0.xy, input1.xy);

		// xy0 zw1
		else if (rtm_impl::static_condition<comp2 == mix4::y && comp3 == mix4::d>::test())
			result_zw = _mm_unpackhi_pd(input0.xy, input1.zw);

		// xy1 xy0
		else if (rtm_impl::static_condition<comp2 == mix4::b && comp3 == mix4::y>::test())
			result_zw = _mm_unpackhi_pd(input1.xy, input0.xy);

		// xy1 zw0
		else if (rtm_impl::static_condition<comp2 == mix4::b && comp3 == mix4::w>::test())
			result_zw = _mm_unpackhi_pd(input1.xy, input0.zw);

		// xy1 xy1
		else if (rtm_impl::static_condition<comp2 == mix4::b && comp3 == mix4::b>::test())
			result_zw = _mm_unpackhi_pd(input1.xy, input1.xy);

		// xy1 zw1
		else if (rtm_impl::static_condition<comp2 == mix4::b && comp3 == mix4::d>::test())
			result_zw = _mm_unpackhi_pd(input1.xy, input1.zw);

		// zw0 xy0
		else if (rtm_impl::static_condition<comp2 == mix4::w && comp3 == mix4::y>::test())
			result_zw = _mm_unpackhi_pd(input0.zw, input0.xy);

		// zw0 zw0
		else if (rtm_impl::static_condition<comp2 == mix4::w && comp3 == mix4::w>::test())
			result_zw = _mm_unpackhi_pd(input0.zw, input0.zw);

		// zw0 xy1
		else if (rtm_impl::static_condition<comp2 == mix4::w && comp3 == mix4::b>::test())
			result_zw = _mm_unpackhi_pd(input0.zw, input1.xy);

		// zw0 zw1
		else if (rtm_impl::static_condition<comp2 == mix4::w && comp3 == mix4::d>::test())
			result_zw = _mm_unpackhi_pd(input0.zw, input1.zw);

		// zw1 xy0
		else if (rtm_impl::static_condition<comp2 == mix4::d && comp3 == mix4::y>::test())
			result_zw = _mm_unpackhi_pd(input1.zw, input0.xy);

		// zw1 zw0
		else if (rtm_impl::static_condition<comp2 == mix4::d && comp3 == mix4::w>::test())
			result_zw = _mm_unpackhi_pd(input1.zw, input0.zw);

		// zw1 xy1
		else if (rtm_impl::static_condition<comp2 == mix4::d && comp3 == mix4::b>::test())
			result_zw = _mm_unpackhi_pd(input1.zw, input1.xy);

		// zw1 zw1
		else if (rtm_impl::static_condition<comp2 == mix4::d && comp3 == mix4::d>::test())
			result_zw = _mm_unpackhi_pd(input1.zw, input1.zw);

		// xy0 xy0
		else if (rtm_impl::static_condition<comp2 == mix4::x && comp3 == mix4::x>::test())
			result_zw = _mm_unpacklo_pd(input0.xy, input0.xy);

		// xy0 zw0
		else if (rtm_impl::static_condition<comp2 == mix4::x && comp3 == mix4::z>::test())
			result_zw = _mm_unpacklo_pd(input0.xy, input0.zw);

		// xy0 xy1
		else if (rtm_impl::static_condition<comp2 == mix4::x && comp3 == mix4::a>::test())
			result_zw = _mm_unpacklo_pd(input0.xy, input1.xy);

		// xy0 zw1
		else if (rtm_impl::static_condition<comp2 == mix4::x && comp3 == mix4::c>::test())
			result_zw = _mm_unpacklo_pd(input0.xy, input1.zw);

		// xy1 xy0
		else if (rtm_impl::static_condition<comp2 == mix4::a && comp3 == mix4::x>::test())
			result_zw = _mm_unpacklo_pd(input1.xy, input0.xy);

		// xy1 zw0
		else if (rtm_impl::static_condition<comp2 == mix4::a && comp3 == mix4::z>::test())
			result_zw = _mm_unpacklo_pd(input1.xy, input0.zw);

		// xy1 xy1
		else if (rtm_impl::static_condition<comp2 == mix4::a && comp3 == mix4::a>::test())
			result_zw = _mm_unpacklo_pd(input1.xy, input1.xy);

		// xy1 zw1
		else if (rtm_impl::static_condition<comp2 == mix4::a && comp3 == mix4::c>::test())
			result_zw = _mm_unpacklo_pd(input1.xy, input1.zw);

		// zw0 xy0
		else if (rtm_impl::static_condition<comp2 == mix4::z && comp3 == mix4::x>::test())
			result_zw = _mm_unpacklo_pd(input0.zw, input0.xy);

		// zw0 zw0
		else if (rtm_impl::static_condition<comp2 == mix4::z && comp3 == mix4::z>::test())
			result_zw = _mm_unpacklo_pd(input0.zw, input0.zw);

		// zw0 xy1
		else if (rtm_impl::static_condition<comp2 == mix4::z && comp3 == mix4::a>::test())
			result_zw = _mm_unpacklo_pd(input0.zw, input1.xy);

		// zw0 zw1
		else if (rtm_impl::static_condition<comp2 == mix4::z && comp3 == mix4::c>::test())
			result_zw = _mm_unpacklo_pd(input0.zw, input1.zw);

		// zw1 xy0
		else if (rtm_impl::static_condition<comp2 == mix4::c && comp3 == mix4::x>::test())
			result_zw = _mm_unpacklo_pd(input1.zw, input0.xy);

		// zw1 zw0
		else if (rtm_impl::static_condition<comp2 == mix4::c && comp3 == mix4::z>::test())
			result_zw = _mm_unpacklo_pd(input1.zw, input0.zw);

		// zw1 xy1
		else if (rtm_impl::static_condition<comp2 == mix4::c && comp3 == mix4::a>::test())
			result_zw = _mm_unpacklo_pd(input1.zw, input1.xy);

		// zw1 zw1
		else if (rtm_impl::static_condition<comp2 == mix4::c && comp3 == mix4::c>::test())
			result_zw = _mm_unpacklo_pd(input1.zw, input1.zw);

		// xy0 xy0
		else if (rtm_impl::static_condition<(comp2 == mix4::x || comp2 == mix4::y) && (comp3 == mix4::x || comp3 == mix4::y)>::test())
			result_zw = _mm_shuffle_pd(input0.xy, input0.xy, _MM_SHUFFLE2(int(comp3) % 2, int(comp2) % 2));

		// xy0 zw0
		else if (rtm_impl::static_condition<(comp2 == mix4::x || comp2 == mix4::y) && (comp3 == mix4::z || comp3 == mix4::w)>::test())
			result_zw = _mm_shuffle_pd(input0.xy, input0.zw, _MM_SHUFFLE2(int(comp3) % 2, int(comp2) % 2));

		// xy0 xy1
		else if (rtm_impl::static_condition<(comp2 == mix4::x || comp2 == mix4::y) && (comp3 == mix4::a || comp3 == mix4::b)>::test())
			result_zw = _mm_shuffle_pd(input0.xy, input1.xy, _MM_SHUFFLE2(int(comp3) % 2, int(comp2) % 2));

		// xy0 zw1
		else if (rtm_impl::static_condition<(comp2 == mix4::x || comp2 == mix4::y) && (comp3 == mix4::c || comp3 == mix4::d)>::test())
			result_zw = _mm_shuffle_pd(input0.xy, input1.zw, _MM_SHUFFLE2(int(comp3) % 2, int(comp2) % 2));

		// xy1 xy0
		else if (rtm_impl::static_condition<(comp2 == mix4::a || comp2 == mix4::b) && (comp3 == mix4::x || comp3 == mix4::y)>::test())
			result_zw = _mm_shuffle_pd(input1.xy, input0.xy, _MM_SHUFFLE2(int(comp3) % 2, int(comp2) % 2));

		// xy1 zw0
		else if (rtm_impl::static_condition<(comp2 == mix4::a || comp2 == mix4::b) && (comp3 == mix4::z || comp3 == mix4::w)>::test())
			result_zw = _mm_shuffle_pd(input1.xy, input0.zw, _MM_SHUFFLE2(int(comp3) % 2, int(comp2) % 2));

		// xy1 xy1
		else if (rtm_impl::static_condition<(comp2 == mix4::a || comp2 == mix4::b) && (comp3 == mix4::a || comp3 == mix4::b)>::test())
			result_zw = _mm_shuffle_pd(input1.xy, input1.xy, _MM_SHUFFLE2(int(comp3) % 2, int(comp2) % 2));

		// xy1 zw1
		else if (rtm_impl::static_condition<(comp2 == mix4::a || comp2 == mix4::b) && (comp3 == mix4::c || comp3 == mix4::d)>::test())
			result_zw = _mm_shuffle_pd(input1.xy, input1.zw, _MM_SHUFFLE2(int(comp3) % 2, int(comp2) % 2));

		// zw0 xy0
		else if (rtm_impl::static_condition<(comp2 == mix4::z || comp2 == mix4::w) && (comp3 == mix4::x || comp3 == mix4::y)>::test())
			result_zw = _mm_shuffle_pd(input0.zw, input0.xy, _MM_SHUFFLE2(int(comp3) % 2, int(comp2) % 2));

		// zw0 zw0
		else if (rtm_impl::static_condition<(comp2 == mix4::z || comp2 == mix4::w) && (comp3 == mix4::z || comp3 == mix4::w)>::test())
			result_zw = _mm_shuffle_pd(input0.zw, input0.zw, _MM_SHUFFLE2(int(comp3) % 2, int(comp2) % 2));

		// zw0 xy1
		else if (rtm_impl::static_condition<(comp2 == mix4::z || comp2 == mix4::w) && (comp3 == mix4::a || comp3 == mix4::b)>::test())
			result_zw = _mm_shuffle_pd(input0.zw, input1.xy, _MM_SHUFFLE2(int(comp3) % 2, int(comp2) % 2));

		// zw0 zw1
		else if (rtm_impl::static_condition<(comp2 == mix4::z || comp2 == mix4::w) && (comp3 == mix4::c || comp3 == mix4::d)>::test())
			result_zw = _mm_shuffle_pd(input0.zw, input1.zw, _MM_SHUFFLE2(int(comp3) % 2, int(comp2) % 2));

		// zw1 xy0
		else if (rtm_impl::static_condition<(comp2 == mix4::c || comp2 == mix4::d) && (comp3 == mix4::x || comp3 == mix4::y)>::test())
			result_zw = _mm_shuffle_pd(input1.zw, input0.xy, _MM_SHUFFLE2(int(comp3) % 2, int(comp2) % 2));

		// zw1 zw0
		else if (rtm_impl::static_condition<(comp2 == mix4::c || comp2 == mix4::d) && (comp3 == mix4::z || comp3 == mix4::w)>::test())
			result_zw = _mm_shuffle_pd(input1.zw, input0.zw, _MM_SHUFFLE2(int(comp3) % 2, int(comp2) % 2));

		// zw1 xy1
		else if (rtm_impl::static_condition<(comp2 == mix4::c || comp2 == mix4::d) && (comp3 == mix4::a || comp3 == mix4::b)>::test())
			result_zw = _mm_shuffle_pd(input1.zw, input1.xy, _MM_SHUFFLE2(int(comp3) % 2, int(comp2) % 2));

		// zw1 zw1
		else if (rtm_impl::static_condition<(comp2 == mix4::c || comp2 == mix4::d) && (comp3 == mix4::c || comp3 == mix4::d)>::test())
			result_zw = _mm_shuffle_pd(input1.zw, input1.zw, _MM_SHUFFLE2(int(comp3) % 2, int(comp2) % 2));

		else
			result_zw = _mm_setzero_pd();	// Set a dummy value, never used


		return vector4d{ result_xy, result_zw };
#elif defined(RTM_WASM128_INTRINSICS)
		// Each pair of result lanes comes from two of the four 2-lane registers: input0.xy, input0.zw, input1.xy, input1.zw
		// Thus, a single shuffle can build each pair
		const v128_t src0 = rtm_impl::vector_mix_get_pair<comp0>(input0, input1);
		const v128_t src1 = rtm_impl::vector_mix_get_pair<comp1>(input0, input1);
		const v128_t src2 = rtm_impl::vector_mix_get_pair<comp2>(input0, input1);
		const v128_t src3 = rtm_impl::vector_mix_get_pair<comp3>(input0, input1);

		const v128_t result_xy = wasm_i64x2_shuffle(src0, src1, int(comp0) % 2, 2 + (int(comp1) % 2));
		const v128_t result_zw = wasm_i64x2_shuffle(src2, src3, int(comp2) % 2, 2 + (int(comp3) % 2));
		return vector4d{ result_xy, result_zw };
#elif defined(RTM_NEON64_INTRINSICS)
		// Each result lane is a lane of one of the four 2-lane registers: input0.xy, input0.zw, input1.xy, input1.zw
		// The compiler usually replaces the lane copies with one or two shuffle instructions
		const float64x2_t src0 = rtm_impl::vector_mix_get_pair<comp0>(input0, input1);
		const float64x2_t src1 = rtm_impl::vector_mix_get_pair<comp1>(input0, input1);
		const float64x2_t src2 = rtm_impl::vector_mix_get_pair<comp2>(input0, input1);
		const float64x2_t src3 = rtm_impl::vector_mix_get_pair<comp3>(input0, input1);

		const float64x2_t result_xy = vsetq_lane_f64(vgetq_lane_f64(src1, int(comp1) % 2), vdupq_laneq_f64(src0, int(comp0) % 2), 1);
		const float64x2_t result_zw = vsetq_lane_f64(vgetq_lane_f64(src3, int(comp3) % 2), vdupq_laneq_f64(src2, int(comp2) % 2), 1);
		return vector4d{ result_xy, result_zw };
#else
		// Non-simd variant
		constexpr component4 component0 = rtm_impl::mix_to_component(comp0);
		constexpr component4 component1 = rtm_impl::mix_to_component(comp1);
		constexpr component4 component2 = rtm_impl::mix_to_component(comp2);
		constexpr component4 component3 = rtm_impl::mix_to_component(comp3);

		const double x = rtm_impl::is_mix_xyzw(comp0) ? vector_get_component(input0, component0) : vector_get_component(input1, component0);
		const double y = rtm_impl::is_mix_xyzw(comp1) ? vector_get_component(input0, component1) : vector_get_component(input1, component1);
		const double z = rtm_impl::is_mix_xyzw(comp2) ? vector_get_component(input0, component2) : vector_get_component(input1, component2);
		const double w = rtm_impl::is_mix_xyzw(comp3) ? vector_get_component(input0, component3) : vector_get_component(input1, component3);

		return vector_set(x, y, z, w);
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Replicates the [x] component in all components.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_dup_x(vector4d_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d value = _mm_shuffle_pd(input.xy, input.xy, _MM_SHUFFLE2(0, 0));
		return vector4d{ value, value };
#elif defined(RTM_WASM128_INTRINSICS)
		const v128_t value = wasm_i64x2_shuffle(input.xy, input.xy, 0, 0);
		return vector4d{ value, value };
#elif defined(RTM_NEON64_INTRINSICS)
		const float64x2_t value = vdupq_laneq_f64(input.xy, 0);
		return vector4d{ value, value };
#else
		return vector4d{ input.x, input.x, input.x, input.x };
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Replicates the [y] component in all components.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_dup_y(vector4d_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d value = _mm_shuffle_pd(input.xy, input.xy, _MM_SHUFFLE2(1, 1));
		return vector4d{ value, value };
#elif defined(RTM_WASM128_INTRINSICS)
		const v128_t value = wasm_i64x2_shuffle(input.xy, input.xy, 1, 1);
		return vector4d{ value, value };
#elif defined(RTM_NEON64_INTRINSICS)
		const float64x2_t value = vdupq_laneq_f64(input.xy, 1);
		return vector4d{ value, value };
#else
		return vector4d{ input.y, input.y, input.y, input.y };
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Replicates the [z] component in all components.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_dup_z(vector4d_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d value = _mm_shuffle_pd(input.zw, input.zw, _MM_SHUFFLE2(0, 0));
		return vector4d{ value, value };
#elif defined(RTM_WASM128_INTRINSICS)
		const v128_t value = wasm_i64x2_shuffle(input.zw, input.zw, 0, 0);
		return vector4d{ value, value };
#elif defined(RTM_NEON64_INTRINSICS)
		const float64x2_t value = vdupq_laneq_f64(input.zw, 0);
		return vector4d{ value, value };
#else
		return vector4d{ input.z, input.z, input.z, input.z };
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Replicates the [w] component in all components.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_dup_w(vector4d_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d value = _mm_shuffle_pd(input.zw, input.zw, _MM_SHUFFLE2(1, 1));
		return vector4d{ value, value };
#elif defined(RTM_WASM128_INTRINSICS)
		const v128_t value = wasm_i64x2_shuffle(input.zw, input.zw, 1, 1);
		return vector4d{ value, value };
#elif defined(RTM_NEON64_INTRINSICS)
		const float64x2_t value = vdupq_laneq_f64(input.zw, 1);
		return vector4d{ value, value };
#else
		return vector4d{ input.w, input.w, input.w, input.w };
#endif
	}


	//////////////////////////////////////////////////////////////////////////
	// Logical
	//////////////////////////////////////////////////////////////////////////

	//////////////////////////////////////////////////////////////////////////
	// Per component logical AND between the inputs: input0 & input1
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vector4d RTM_SIMD_CALL vector_and(vector4d_arg0 input0, vector4d_arg1 input1) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy = _mm_and_pd(input0.xy, input1.xy);
		__m128d zw = _mm_and_pd(input0.zw, input1.zw);
		return vector4d{ xy, zw };
#elif defined(RTM_WASM128_INTRINSICS)
		const v128_t xy = wasm_v128_and(input0.xy, input1.xy);
		const v128_t zw = wasm_v128_and(input0.zw, input1.zw);
		return vector4d{ xy, zw };
#elif defined(RTM_NEON64_INTRINSICS)
		const float64x2_t xy = vreinterpretq_f64_u64(vandq_u64(vreinterpretq_u64_f64(input0.xy), vreinterpretq_u64_f64(input1.xy)));
		const float64x2_t zw = vreinterpretq_f64_u64(vandq_u64(vreinterpretq_u64_f64(input0.zw), vreinterpretq_u64_f64(input1.zw)));
		return vector4d{ xy, zw };
#else
		const uint64_t* input0_ = rtm_impl::bit_cast<const uint64_t*>(&input0);
		const uint64_t* input1_ = rtm_impl::bit_cast<const uint64_t*>(&input1);

		vector4d result = input0;
		uint64_t* result_ = rtm_impl::bit_cast<uint64_t*>(&result);

		result_[0] = input0_[0] & input1_[0];
		result_[1] = input0_[1] & input1_[1];
		result_[2] = input0_[2] & input1_[2];
		result_[3] = input0_[3] & input1_[3];

		return result;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Per component logical OR between the inputs: input0 | input1
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vector4d RTM_SIMD_CALL vector_or(vector4d_arg0 input0, vector4d_arg1 input1) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy = _mm_or_pd(input0.xy, input1.xy);
		__m128d zw = _mm_or_pd(input0.zw, input1.zw);
		return vector4d{ xy, zw };
#elif defined(RTM_WASM128_INTRINSICS)
		const v128_t xy = wasm_v128_or(input0.xy, input1.xy);
		const v128_t zw = wasm_v128_or(input0.zw, input1.zw);
		return vector4d{ xy, zw };
#elif defined(RTM_NEON64_INTRINSICS)
		const float64x2_t xy = vreinterpretq_f64_u64(vorrq_u64(vreinterpretq_u64_f64(input0.xy), vreinterpretq_u64_f64(input1.xy)));
		const float64x2_t zw = vreinterpretq_f64_u64(vorrq_u64(vreinterpretq_u64_f64(input0.zw), vreinterpretq_u64_f64(input1.zw)));
		return vector4d{ xy, zw };
#else
		const uint64_t* input0_ = rtm_impl::bit_cast<const uint64_t*>(&input0);
		const uint64_t* input1_ = rtm_impl::bit_cast<const uint64_t*>(&input1);

		vector4d result = input0;
		uint64_t* result_ = rtm_impl::bit_cast<uint64_t*>(&result);

		result_[0] = input0_[0] | input1_[0];
		result_[1] = input0_[1] | input1_[1];
		result_[2] = input0_[2] | input1_[2];
		result_[3] = input0_[3] | input1_[3];

		return result;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Per component logical XOR between the inputs: input0 ^ input1
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vector4d RTM_SIMD_CALL vector_xor(vector4d_arg0 input0, vector4d_arg1 input1) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy = _mm_xor_pd(input0.xy, input1.xy);
		__m128d zw = _mm_xor_pd(input0.zw, input1.zw);
		return vector4d{ xy, zw };
#elif defined(RTM_WASM128_INTRINSICS)
		const v128_t xy = wasm_v128_xor(input0.xy, input1.xy);
		const v128_t zw = wasm_v128_xor(input0.zw, input1.zw);
		return vector4d{ xy, zw };
#elif defined(RTM_NEON64_INTRINSICS)
		const float64x2_t xy = vreinterpretq_f64_u64(veorq_u64(vreinterpretq_u64_f64(input0.xy), vreinterpretq_u64_f64(input1.xy)));
		const float64x2_t zw = vreinterpretq_f64_u64(veorq_u64(vreinterpretq_u64_f64(input0.zw), vreinterpretq_u64_f64(input1.zw)));
		return vector4d{ xy, zw };
#else
		const uint64_t* input0_ = rtm_impl::bit_cast<const uint64_t*>(&input0);
		const uint64_t* input1_ = rtm_impl::bit_cast<const uint64_t*>(&input1);

		vector4d result = input0;
		uint64_t* result_ = rtm_impl::bit_cast<uint64_t*>(&result);

		result_[0] = input0_[0] ^ input1_[0];
		result_[1] = input0_[1] ^ input1_[1];
		result_[2] = input0_[2] ^ input1_[2];
		result_[3] = input0_[3] ^ input1_[3];

		return result;
#endif
	}


	//////////////////////////////////////////////////////////////////////////
	// Miscellaneous
	//////////////////////////////////////////////////////////////////////////


	//////////////////////////////////////////////////////////////////////////
	// Returns per component the sign of the input vector: input >= 0.0 ? 1.0 : -1.0
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_sign(vector4d_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		const __m128d signs = _mm_set1_pd(-0.0);
		const __m128d one = _mm_set1_pd(1.0);
		// Mask out the sign bit
		const __m128d sign_bits_xy = _mm_and_pd(input.xy, signs);
		const __m128d sign_bits_zw = _mm_and_pd(input.zw, signs);
		// Copy the sign bit onto +-1.0f
		return vector4d{ _mm_or_pd(sign_bits_xy, one), _mm_or_pd(sign_bits_zw, one) };
#elif defined(RTM_WASM128_INTRINSICS)
		const v128_t signs = wasm_f64x2_splat(-0.0);
		const v128_t one = wasm_f64x2_splat(1.0);
		// Mask out the sign bit
		const v128_t sign_bits_xy = wasm_v128_and(input.xy, signs);
		const v128_t sign_bits_zw = wasm_v128_and(input.zw, signs);
		// Copy the sign bit onto +-1.0f
		return vector4d{ wasm_v128_or(sign_bits_xy, one), wasm_v128_or(sign_bits_zw, one) };
#else
		const mask4d mask = vector_greater_equal(input, vector_zero());
		return vector_select(mask, vector_set(1.0), vector_set(-1.0));
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns per component the input with the sign of the control value.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_copy_sign(vector4d_arg0 input, vector4d_arg1 control_sign) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		const __m128d sign_bit = _mm_set1_pd(-0.0);
		__m128d signs_xy = _mm_and_pd(sign_bit, control_sign.xy);
		__m128d signs_zw = _mm_and_pd(sign_bit, control_sign.zw);
		__m128d abs_input_xy = _mm_andnot_pd(sign_bit, input.xy);
		__m128d abs_input_zw = _mm_andnot_pd(sign_bit, input.zw);
		__m128d xy = _mm_or_pd(abs_input_xy, signs_xy);
		__m128d zw = _mm_or_pd(abs_input_zw, signs_zw);
		return vector4d{ xy, zw };
#elif defined(RTM_WASM128_INTRINSICS)
		// Select the sign bit from the control value and the other bits from the input
		const v128_t sign_bit = wasm_u64x2_splat(0x8000000000000000ULL);
		return vector4d{ wasm_v128_bitselect(control_sign.xy, input.xy, sign_bit), wasm_v128_bitselect(control_sign.zw, input.zw, sign_bit) };
#elif defined(RTM_NEON64_INTRINSICS)
		// Select the sign bit from the control value and the other bits from the input
		const uint64x2_t sign_bit = vdupq_n_u64(0x8000000000000000ULL);
		return vector4d{ vbslq_f64(sign_bit, control_sign.xy, input.xy), vbslq_f64(sign_bit, control_sign.zw, input.zw) };
#else
		double x = vector_get_x(input);
		double y = vector_get_y(input);
		double z = vector_get_z(input);
		double w = vector_get_w(input);

		double x_sign = vector_get_x(control_sign);
		double y_sign = vector_get_y(control_sign);
		double z_sign = vector_get_z(control_sign);
		double w_sign = vector_get_w(control_sign);

		return vector_set(rtm_impl::copysign(x, x_sign), rtm_impl::copysign(y, y_sign), rtm_impl::copysign(z, z_sign), rtm_impl::copysign(w, w_sign));
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns per component the rounded input using a symmetric algorithm.
	// vector_round_symmetric(1.5) = 2.0
	// vector_round_symmetric(1.2) = 1.0
	// vector_round_symmetric(-1.5) = -2.0
	// vector_round_symmetric(-1.2) = -1.0
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vector4d RTM_SIMD_CALL vector_round_symmetric(vector4d_arg0 input) RTM_NO_EXCEPT
	{
		// We do not add a bias of 0.5 and then truncate: that addition can round.
		// For example, 0.49999999999999994 + 0.5 rounds to 1.0, and 4503599627370497.0 + 0.5 rounds to 4503599627370498.0.
		// We calculate the integer part and the fractional part with exact operations instead.

#if defined(RTM_SSE4_INTRINSICS)
		const __m128d sign_mask = _mm_set_pd(-0.0, -0.0);
		__m128d sign_xy = _mm_and_pd(input.xy, sign_mask);
		__m128d sign_zw = _mm_and_pd(input.zw, sign_mask);

		// NaN, +- Infinity, and numbers larger or equal to 2^52 remain unchanged
		// since they have no fractional part.
		__m128d integer_part_xy = _mm_round_pd(input.xy, _MM_FROUND_TO_ZERO | _MM_FROUND_NO_EXC);
		__m128d integer_part_zw = _mm_round_pd(input.zw, _MM_FROUND_TO_ZERO | _MM_FROUND_NO_EXC);

		// If the absolute value of the fractional part is 0.5 or more, we add 1.0 with the sign of the input.
		// The fractional part is NaN for NaN and infinities, the comparison is false for them.
		const __m128d half = _mm_set1_pd(0.5);
		__m128d abs_fraction_xy = _mm_andnot_pd(sign_mask, _mm_sub_pd(input.xy, integer_part_xy));
		__m128d abs_fraction_zw = _mm_andnot_pd(sign_mask, _mm_sub_pd(input.zw, integer_part_zw));
		__m128d is_fraction_half_or_more_xy = _mm_cmpge_pd(abs_fraction_xy, half);
		__m128d is_fraction_half_or_more_zw = _mm_cmpge_pd(abs_fraction_zw, half);

		// The bias is +- 1.0 with the sign of the input, or 0.0
		const __m128d one = _mm_set1_pd(1.0);
		__m128d bias_xy = _mm_and_pd(is_fraction_half_or_more_xy, _mm_or_pd(sign_xy, one));
		__m128d bias_zw = _mm_and_pd(is_fraction_half_or_more_zw, _mm_or_pd(sign_zw, one));
		__m128d result_xy = _mm_add_pd(integer_part_xy, bias_xy);
		__m128d result_zw = _mm_add_pd(integer_part_zw, bias_zw);

		// Keep the sign of the input as std::round does, thus round(-0.0) = -0.0 and round(-0.3) = -0.0
		result_xy = _mm_or_pd(result_xy, sign_xy);
		result_zw = _mm_or_pd(result_zw, sign_zw);
		return vector4d{ result_xy, result_zw };
#elif defined(RTM_SSE2_INTRINSICS)
		const __m128i abs_mask = _mm_set_epi64x(0x7FFFFFFFFFFFFFFFULL, 0x7FFFFFFFFFFFFFFFULL);
		const __m128d fractional_limit = _mm_set1_pd(4503599627370496.0); // 2^52

		// NaN, +- Infinity, and numbers larger or equal to 2^52 remain unchanged
		// since they have no fractional part.
		// The mask is true only for the smaller values. It is false for NaN.
		__m128d abs_input_xy = _mm_and_pd(input.xy, _mm_castsi128_pd(abs_mask));
		__m128d abs_input_zw = _mm_and_pd(input.zw, _mm_castsi128_pd(abs_mask));
		__m128d is_input_small_xy = _mm_cmplt_pd(abs_input_xy, fractional_limit);
		__m128d is_input_small_zw = _mm_cmplt_pd(abs_input_zw, fractional_limit);

		const __m128d sign_mask = _mm_set_pd(-0.0, -0.0);
		__m128d sign_xy = _mm_and_pd(input.xy, sign_mask);
		__m128d sign_zw = _mm_and_pd(input.zw, sign_mask);

		// Round the absolute value to the nearest integer with 2^52: the addition rounds half to even.
		// A conversion to a 32 bit integer would overflow for the values between 2^31 and 2^52.
		__m128d rounded_xy = _mm_sub_pd(_mm_add_pd(abs_input_xy, fractional_limit), fractional_limit);
		__m128d rounded_zw = _mm_sub_pd(_mm_add_pd(abs_input_zw, fractional_limit), fractional_limit);

		// The difference is exact. It is 0.5 only for a tie that rounds down to the even value.
		// In that case, add 1.0 to round half away from zero.
		const __m128d half = _mm_set1_pd(0.5);
		const __m128d one = _mm_set1_pd(1.0);
		__m128d is_tie_rounded_down_xy = _mm_cmpge_pd(_mm_sub_pd(abs_input_xy, rounded_xy), half);
		__m128d is_tie_rounded_down_zw = _mm_cmpge_pd(_mm_sub_pd(abs_input_zw, rounded_zw), half);
		rounded_xy = _mm_add_pd(rounded_xy, _mm_and_pd(is_tie_rounded_down_xy, one));
		rounded_zw = _mm_add_pd(rounded_zw, _mm_and_pd(is_tie_rounded_down_zw, one));

		// Restore the sign of the input as std::round does, thus round(-0.0) = -0.0 and round(-0.3) = -0.0
		__m128d integer_part_xy = _mm_or_pd(rounded_xy, sign_xy);
		__m128d integer_part_zw = _mm_or_pd(rounded_zw, sign_zw);

		__m128d result_xy = _mm_or_pd(_mm_and_pd(is_input_small_xy, integer_part_xy), _mm_andnot_pd(is_input_small_xy, input.xy));
		__m128d result_zw = _mm_or_pd(_mm_and_pd(is_input_small_zw, integer_part_zw), _mm_andnot_pd(is_input_small_zw, input.zw));

		return vector4d{ result_xy, result_zw };
#elif defined(RTM_WASM128_INTRINSICS)
		// Truncate and add one with the sign of the input when the fractional part is 0.5 or more.
		// The subtraction of the truncated value is exact, unlike the addition of a 0.5 bias.
		// NaN and +- Infinity remain unchanged since their fractional part is NaN.
		const v128_t sign_mask = wasm_f64x2_splat(-0.0);
		const v128_t half = wasm_f64x2_splat(0.5);
		const v128_t one = wasm_f64x2_splat(1.0);

		const v128_t sign_xy = wasm_v128_and(input.xy, sign_mask);
		const v128_t sign_zw = wasm_v128_and(input.zw, sign_mask);
		const v128_t truncated_xy = wasm_f64x2_trunc(input.xy);
		const v128_t truncated_zw = wasm_f64x2_trunc(input.zw);
		const v128_t is_half_or_more_xy = wasm_f64x2_ge(wasm_f64x2_abs(wasm_f64x2_sub(input.xy, truncated_xy)), half);
		const v128_t is_half_or_more_zw = wasm_f64x2_ge(wasm_f64x2_abs(wasm_f64x2_sub(input.zw, truncated_zw)), half);
		const v128_t result_xy = wasm_f64x2_add(truncated_xy, wasm_v128_and(is_half_or_more_xy, wasm_v128_or(sign_xy, one)));
		const v128_t result_zw = wasm_f64x2_add(truncated_zw, wasm_v128_and(is_half_or_more_zw, wasm_v128_or(sign_zw, one)));

		// Keep the sign of the input as std::round does, thus round(-0.0) = -0.0 and round(-0.3) = -0.0
		return vector4d{ wasm_v128_or(result_xy, sign_xy), wasm_v128_or(result_zw, sign_zw) };
#elif defined(RTM_NEON64_INTRINSICS)
		// This instruction rounds half away from zero, as std::round does
		return vector4d{ vrndaq_f64(input.xy), vrndaq_f64(input.zw) };
#else
		scalard x = scalar_round_symmetric(scalard(vector_get_x(input)));
		scalard y = scalar_round_symmetric(scalard(vector_get_y(input)));
		scalard z = scalar_round_symmetric(scalard(vector_get_z(input)));
		scalard w = scalar_round_symmetric(scalard(vector_get_w(input)));
		return vector_set(x, y, z, w);
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns per component the rounded input using banker's rounding (half to even).
	// vector_round_bankers(2.5) = 2.0
	// vector_round_bankers(1.5) = 2.0
	// vector_round_bankers(1.2) = 1.0
	// vector_round_bankers(-2.5) = -2.0
	// vector_round_bankers(-1.5) = -2.0
	// vector_round_bankers(-1.2) = -1.0
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL vector_round_bankers(vector4d_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE4_INTRINSICS)
		return vector4d{ _mm_round_pd(input.xy, _MM_FROUND_TO_NEAREST_INT | _MM_FROUND_NO_EXC), _mm_round_pd(input.zw, _MM_FROUND_TO_NEAREST_INT | _MM_FROUND_NO_EXC) };
#elif defined(RTM_SSE2_INTRINSICS)
		const __m128i abs_mask = _mm_set_epi64x(0x7FFFFFFFFFFFFFFFULL, 0x7FFFFFFFFFFFFFFFULL);

		const __m128d sign_mask = _mm_set_pd(-0.0, -0.0);
		__m128d sign_xy = _mm_and_pd(input.xy, sign_mask);
		__m128d sign_zw = _mm_and_pd(input.zw, sign_mask);

		// We add the largest integer that a 64 bit floating point number can represent and subtract it afterwards.
		// This relies on the fact that if we had a fractional part, the new value cannot be represented accurately
		// and IEEE 754 will perform rounding for us. The default rounding mode is Banker's rounding.
		// This has the effect of removing the fractional part while simultaneously rounding.
		// Use the same sign as the input value to make sure we handle positive and negative values.
		const __m128d fractional_limit = _mm_set1_pd(4503599627370496.0); // 2^52
		__m128d truncating_offset_xy = _mm_or_pd(sign_xy, fractional_limit);
		__m128d truncating_offset_zw = _mm_or_pd(sign_zw, fractional_limit);
		__m128d integer_part_xy = _mm_sub_pd(_mm_add_pd(input.xy, truncating_offset_xy), truncating_offset_xy);
		__m128d integer_part_zw = _mm_sub_pd(_mm_add_pd(input.zw, truncating_offset_zw), truncating_offset_zw);

		// Keep the sign of the input as std::nearbyint does, thus round(-0.0) = -0.0 and round(-0.3) = -0.0
		integer_part_xy = _mm_or_pd(integer_part_xy, sign_xy);
		integer_part_zw = _mm_or_pd(integer_part_zw, sign_zw);

		__m128d abs_input_xy = _mm_and_pd(input.xy, _mm_castsi128_pd(abs_mask));
		__m128d abs_input_zw = _mm_and_pd(input.zw, _mm_castsi128_pd(abs_mask));
		__m128d is_input_large_xy = _mm_cmpge_pd(abs_input_xy, fractional_limit);
		__m128d is_input_large_zw = _mm_cmpge_pd(abs_input_zw, fractional_limit);
		__m128d result_xy = _mm_or_pd(_mm_and_pd(is_input_large_xy, input.xy), _mm_andnot_pd(is_input_large_xy, integer_part_xy));
		__m128d result_zw = _mm_or_pd(_mm_and_pd(is_input_large_zw, input.zw), _mm_andnot_pd(is_input_large_zw, integer_part_zw));
		return vector4d{ result_xy, result_zw };
#elif defined(RTM_WASM128_INTRINSICS)
		return vector4d{ wasm_f64x2_nearest(input.xy), wasm_f64x2_nearest(input.zw) };
#elif defined(RTM_NEON64_INTRINSICS)
		return vector4d{ vrndnq_f64(input.xy), vrndnq_f64(input.zw) };
#else
		scalard x = scalar_round_bankers(scalard(vector_get_x(input)));
		scalard y = scalar_round_bankers(scalard(vector_get_y(input)));
		scalard z = scalar_round_bankers(scalard(vector_get_z(input)));
		scalard w = scalar_round_bankers(scalard(vector_get_w(input)));
		return vector_set(x, y, z, w);
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns per component the sine of the input angle.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vector4d RTM_SIMD_CALL vector_sin(vector4d_arg0 input) RTM_NO_EXCEPT
	{
		scalard x = scalar_sin(vector_get_x_as_scalar(input));
		scalard y = scalar_sin(vector_get_y_as_scalar(input));
		scalard z = scalar_sin(vector_get_z_as_scalar(input));
		scalard w = scalar_sin(vector_get_w_as_scalar(input));
		return vector_set(x, y, z, w);
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns per component the arc-sine of the input.
	// Input value must be in the range [-1.0, 1.0].
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vector4d RTM_SIMD_CALL vector_asin(vector4d_arg0 input) RTM_NO_EXCEPT
	{
		scalard x = scalar_asin(vector_get_x_as_scalar(input));
		scalard y = scalar_asin(vector_get_y_as_scalar(input));
		scalard z = scalar_asin(vector_get_z_as_scalar(input));
		scalard w = scalar_asin(vector_get_w_as_scalar(input));
		return vector_set(x, y, z, w);
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns per component the cosine of the input angle.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vector4d RTM_SIMD_CALL vector_cos(vector4d_arg0 input) RTM_NO_EXCEPT
	{
		scalard x = scalar_cos(vector_get_x_as_scalar(input));
		scalard y = scalar_cos(vector_get_y_as_scalar(input));
		scalard z = scalar_cos(vector_get_z_as_scalar(input));
		scalard w = scalar_cos(vector_get_w_as_scalar(input));
		return vector_set(x, y, z, w);
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns per component the arc-cosine of the input.
	// Input value must be in the range [-1.0, 1.0].
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vector4d RTM_SIMD_CALL vector_acos(vector4d_arg0 input) RTM_NO_EXCEPT
	{
		scalard x = scalar_acos(vector_get_x_as_scalar(input));
		scalard y = scalar_acos(vector_get_y_as_scalar(input));
		scalard z = scalar_acos(vector_get_z_as_scalar(input));
		scalard w = scalar_acos(vector_get_w_as_scalar(input));
		return vector_set(x, y, z, w);
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns per component the sine and cosine of the input angle.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline void RTM_SIMD_CALL vector_sincos(vector4d_arg0 input, vector4d& out_sine, vector4d& out_cosine) RTM_NO_EXCEPT
	{
		const vector4d x = scalar_sincos(vector_get_x_as_scalar(input));
		const vector4d y = scalar_sincos(vector_get_y_as_scalar(input));
		const vector4d z = scalar_sincos(vector_get_z_as_scalar(input));
		const vector4d w = scalar_sincos(vector_get_w_as_scalar(input));

		const vector4d cos_xy = vector_mix<mix4::y, mix4::b, mix4::y, mix4::b>(x, y);
		const vector4d cos_zw = vector_mix<mix4::y, mix4::b, mix4::y, mix4::b>(z, w);
		out_cosine = vector_mix<mix4::x, mix4::y, mix4::a, mix4::b>(cos_xy, cos_zw);

		const vector4d sin_xy = vector_mix<mix4::x, mix4::a, mix4::x, mix4::a>(x, y);
		const vector4d sin_zw = vector_mix<mix4::x, mix4::a, mix4::x, mix4::a>(z, w);
		out_sine = vector_mix<mix4::x, mix4::y, mix4::a, mix4::b>(sin_xy, sin_zw);
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns per component the tangent of the input angle.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vector4d RTM_SIMD_CALL vector_tan(vector4d_arg0 angle) RTM_NO_EXCEPT
	{
		// Use the identity: tan(angle) = sin(angle) / cos(angle)
		vector4d sin_;
		vector4d cos_;
		vector_sincos(angle, sin_, cos_);

		const mask4d is_cos_zero = vector_equal(cos_, vector_zero());
		const vector4d signed_infinity = vector_copy_sign(vector_set(std::numeric_limits<double>::infinity()), angle);
		const vector4d result = vector_div(sin_, cos_);
		return vector_select(is_cos_zero, signed_infinity, result);
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns per component the arc-tangent of the input.
	// Note that due to the sign ambiguity, atan cannot determine which quadrant
	// the value resides in.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vector4d RTM_SIMD_CALL vector_atan(vector4d_arg0 input) RTM_NO_EXCEPT
	{
		scalard x = scalar_atan(vector_get_x_as_scalar(input));
		scalard y = scalar_atan(vector_get_y_as_scalar(input));
		scalard z = scalar_atan(vector_get_z_as_scalar(input));
		scalard w = scalar_atan(vector_get_w_as_scalar(input));
		return vector_set(x, y, z, w);
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns per component the arc-tangent of [y/x] using the sign of the arguments to
	// determine the correct quadrant.
	// Y represents the proportion of the y-coordinate.
	// X represents the proportion of the x-coordinate.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vector4d RTM_SIMD_CALL vector_atan2(vector4d_arg0 y, vector4d_arg1 x) RTM_NO_EXCEPT
	{
		scalard x_ = scalar_atan2(vector_get_x_as_scalar(y), vector_get_x_as_scalar(x));
		scalard y_ = scalar_atan2(vector_get_y_as_scalar(y), vector_get_y_as_scalar(x));
		scalard z_ = scalar_atan2(vector_get_z_as_scalar(y), vector_get_z_as_scalar(x));
		scalard w_ = scalar_atan2(vector_get_w_as_scalar(y), vector_get_w_as_scalar(x));
		return vector_set(x_, y_, z_, w_);
	}

	RTM_IMPL_VERSION_NAMESPACE_END
}

RTM_IMPL_FILE_PRAGMA_POP
