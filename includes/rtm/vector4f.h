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
#include "rtm/scalarf.h"
#include "rtm/impl/compiler_utils.h"
#include "rtm/impl/memory_utils.h"
#include "rtm/impl/vector_common.h"

RTM_IMPL_FILE_PRAGMA_PUSH

namespace rtm
{
	//////////////////////////////////////////////////////////////////////////
	// Setters, getters, and casts
	//////////////////////////////////////////////////////////////////////////


	//////////////////////////////////////////////////////////////////////////
	// Loads an unaligned vector4 from memory.
	//////////////////////////////////////////////////////////////////////////
	inline vector4f RTM_SIMD_CALL vector_load(const float* input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_loadu_ps(input);
#elif defined(RTM_NEON_INTRINSICS)
		return vld1q_f32(input);
#else
		return vector_set(input[0], input[1], input[2], input[3]);
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Loads an unaligned vector1 from memory and leaves the [yzw] components undefined.
	//////////////////////////////////////////////////////////////////////////
	inline vector4f RTM_SIMD_CALL vector_load1(const float* input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_load_ps1(input);
#elif defined(RTM_NEON_INTRINSICS)
		return vld1q_dup_f32(input);
#else
		return vector_set(input[0]);
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Loads an unaligned vector2 from memory and leaves the [zw] components undefined.
	//////////////////////////////////////////////////////////////////////////
	inline vector4f RTM_SIMD_CALL vector_load2(const float* input) RTM_NO_EXCEPT
	{
		return vector_set(input[0], input[1], 0.0F, 0.0F);
	}

	//////////////////////////////////////////////////////////////////////////
	// Loads an unaligned vector3 from memory and leaves the [w] component undefined.
	//////////////////////////////////////////////////////////////////////////
	inline vector4f RTM_SIMD_CALL vector_load3(const float* input) RTM_NO_EXCEPT
	{
		return vector_set(input[0], input[1], input[2], 0.0F);
	}

	//////////////////////////////////////////////////////////////////////////
	// Loads an unaligned vector4 from memory.
	//////////////////////////////////////////////////////////////////////////
	inline vector4f RTM_SIMD_CALL vector_load(const float4f* input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_loadu_ps(&input->x);
#elif defined(RTM_NEON_INTRINSICS)
		return vld1q_f32(&input->x);
#else
		return vector_set(input->x, input->y, input->z, input->w);
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Loads an unaligned vector2 from memory and leaves the [zw] components undefined.
	//////////////////////////////////////////////////////////////////////////
	inline vector4f RTM_SIMD_CALL vector_load2(const float2f* input) RTM_NO_EXCEPT
	{
		return vector_set(input->x, input->y, 0.0F, 0.0F);
	}

	//////////////////////////////////////////////////////////////////////////
	// Loads an unaligned vector3 from memory and leaves the [w] component undefined.
	//////////////////////////////////////////////////////////////////////////
	inline vector4f RTM_SIMD_CALL vector_load3(const float3f* input) RTM_NO_EXCEPT
	{
		return vector_set(input->x, input->y, input->z, 0.0F);
	}

	//////////////////////////////////////////////////////////////////////////
	// Casts a quaternion to a vector4.
	//////////////////////////////////////////////////////////////////////////
	inline vector4f RTM_SIMD_CALL quat_to_vector(quatf_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS) || defined(RTM_NEON_INTRINSICS)
		return input;
#else
		return vector4f{ input.x, input.y, input.z, input.w };
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Casts a vector4 float64 variant to a float32 variant.
	//////////////////////////////////////////////////////////////////////////
	inline vector4f RTM_SIMD_CALL vector_cast(const vector4d& input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_shuffle_ps(_mm_cvtpd_ps(input.xy), _mm_cvtpd_ps(input.zw), _MM_SHUFFLE(1, 0, 1, 0));
#else
		return vector_set(float(input.x), float(input.y), float(input.z), float(input.w));
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the vector4 [x] component.
	//////////////////////////////////////////////////////////////////////////
	inline float RTM_SIMD_CALL vector_get_x(vector4f_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_cvtss_f32(input);
#elif defined(RTM_NEON_INTRINSICS)
		return vgetq_lane_f32(input, 0);
#else
		return input.x;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the vector4 [y] component.
	//////////////////////////////////////////////////////////////////////////
	inline float RTM_SIMD_CALL vector_get_y(vector4f_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_cvtss_f32(_mm_shuffle_ps(input, input, _MM_SHUFFLE(1, 1, 1, 1)));
#elif defined(RTM_NEON_INTRINSICS)
		return vgetq_lane_f32(input, 1);
#else
		return input.y;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the vector4 [z] component.
	//////////////////////////////////////////////////////////////////////////
	inline float RTM_SIMD_CALL vector_get_z(vector4f_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_cvtss_f32(_mm_shuffle_ps(input, input, _MM_SHUFFLE(2, 2, 2, 2)));
#elif defined(RTM_NEON_INTRINSICS)
		return vgetq_lane_f32(input, 2);
#else
		return input.z;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the vector4 [w] component.
	//////////////////////////////////////////////////////////////////////////
	inline float RTM_SIMD_CALL vector_get_w(vector4f_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_cvtss_f32(_mm_shuffle_ps(input, input, _MM_SHUFFLE(3, 3, 3, 3)));
#elif defined(RTM_NEON_INTRINSICS)
		return vgetq_lane_f32(input, 3);
#else
		return input.w;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the vector4 desired component.
	//////////////////////////////////////////////////////////////////////////
	template<mix4 component>
	inline float RTM_SIMD_CALL vector_get_component(vector4f_arg0 input) RTM_NO_EXCEPT
	{
		switch (mix4(int(component) % 4))
		{
		case mix4::x: return vector_get_x(input);
		case mix4::y: return vector_get_y(input);
		case mix4::z: return vector_get_z(input);
		case mix4::w: return vector_get_w(input);
		default:
			RTM_ASSERT(false, "Invalid component");
			return 0.0F;
		}
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the vector4 desired component.
	//////////////////////////////////////////////////////////////////////////
	inline float RTM_SIMD_CALL vector_get_component(vector4f_arg0 input, mix4 component) RTM_NO_EXCEPT
	{
		switch (mix4(int(component) % 4))
		{
		case mix4::x: return vector_get_x(input);
		case mix4::y: return vector_get_y(input);
		case mix4::z: return vector_get_z(input);
		case mix4::w: return vector_get_w(input);
		default:
			RTM_ASSERT(false, "Invalid component");
			return 0.0F;
		}
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the smallest component in the input vector as a scalar.
	//////////////////////////////////////////////////////////////////////////
	constexpr rtm_impl::vector4f_get_min_component RTM_SIMD_CALL vector_get_min_component(vector4f_arg0 input) RTM_NO_EXCEPT
	{
		return rtm_impl::vector4f_get_min_component{ input };
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the largest component in the input vector as a scalar.
	//////////////////////////////////////////////////////////////////////////
	constexpr rtm_impl::vector4f_get_max_component RTM_SIMD_CALL vector_get_max_component(vector4f_arg0 input) RTM_NO_EXCEPT
	{
		return rtm_impl::vector4f_get_max_component{ input };
	}

	//////////////////////////////////////////////////////////////////////////
	// Sets the vector4 [x] component and returns the new value.
	//////////////////////////////////////////////////////////////////////////
	inline vector4f RTM_SIMD_CALL vector_set_x(vector4f_arg0 input, float lane_value) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_move_ss(input, _mm_set_ss(lane_value));
#elif defined(RTM_NEON_INTRINSICS)
		return vsetq_lane_f32(lane_value, input, 0);
#else
		return vector4f{ lane_value, input.y, input.z, input.w };
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Sets the vector4 [y] component and returns the new value.
	//////////////////////////////////////////////////////////////////////////
	inline vector4f RTM_SIMD_CALL vector_set_y(vector4f_arg0 input, float lane_value) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE4_INTRINSICS)
		return _mm_insert_ps(input, _mm_set_ss(lane_value), 0x10);
#elif defined(RTM_SSE2_INTRINSICS)
		const __m128 yxzw = _mm_shuffle_ps(input, input, _MM_SHUFFLE(3, 2, 0, 1));
		const __m128 vxzw = _mm_move_ss(yxzw, _mm_set_ss(lane_value));
		return _mm_shuffle_ps(vxzw, vxzw, _MM_SHUFFLE(3, 2, 0, 1));
#elif defined(RTM_NEON_INTRINSICS)
		return vsetq_lane_f32(lane_value, input, 1);
#else
		return vector4f{ input.x, lane_value, input.z, input.w };
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Sets the vector4 [z] component and returns the new value.
	//////////////////////////////////////////////////////////////////////////
	inline vector4f RTM_SIMD_CALL vector_set_z(vector4f_arg0 input, float lane_value) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE4_INTRINSICS)
		return _mm_insert_ps(input, _mm_set_ss(lane_value), 0x20);
#elif defined(RTM_SSE2_INTRINSICS)
		const __m128 zyxw = _mm_shuffle_ps(input, input, _MM_SHUFFLE(3, 0, 1, 2));
		const __m128 vyxw = _mm_move_ss(zyxw, _mm_set_ss(lane_value));
		return _mm_shuffle_ps(vyxw, vyxw, _MM_SHUFFLE(3, 0, 1, 2));
#elif defined(RTM_NEON_INTRINSICS)
		return vsetq_lane_f32(lane_value, input, 2);
#else
		return vector4f{ input.x, input.y, lane_value, input.w };
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Sets the vector4 [w] component and returns the new value.
	//////////////////////////////////////////////////////////////////////////
	inline vector4f RTM_SIMD_CALL vector_set_w(vector4f_arg0 input, float lane_value) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE4_INTRINSICS)
		return _mm_insert_ps(input, _mm_set_ss(lane_value), 0x30);
#elif defined(RTM_SSE2_INTRINSICS)
		const __m128 wyzx = _mm_shuffle_ps(input, input, _MM_SHUFFLE(0, 2, 1, 3));
		const __m128 vyzx = _mm_move_ss(wyzx, _mm_set_ss(lane_value));
		return _mm_shuffle_ps(vyzx, vyzx, _MM_SHUFFLE(0, 2, 1, 3));
#elif defined(RTM_NEON_INTRINSICS)
		return vsetq_lane_f32(lane_value, input, 3);
#else
		return vector4f{ input.x, input.y, input.z, lane_value };
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns a floating point pointer to the vector4 data.
	//////////////////////////////////////////////////////////////////////////
	inline const float* RTM_SIMD_CALL vector_to_pointer(const vector4f& input) RTM_NO_EXCEPT
	{
		return reinterpret_cast<const float*>(&input);
	}

	//////////////////////////////////////////////////////////////////////////
	// Writes a vector4 to unaligned memory.
	//////////////////////////////////////////////////////////////////////////
	inline void RTM_SIMD_CALL vector_store(vector4f_arg0 input, float* output) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		_mm_storeu_ps(output, input);
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
	inline void RTM_SIMD_CALL vector_store1(vector4f_arg0 input, float* output) RTM_NO_EXCEPT
	{
		output[0] = vector_get_x(input);
	}

	//////////////////////////////////////////////////////////////////////////
	// Writes a vector2 to unaligned memory.
	//////////////////////////////////////////////////////////////////////////
	inline void RTM_SIMD_CALL vector_store2(vector4f_arg0 input, float* output) RTM_NO_EXCEPT
	{
		output[0] = vector_get_x(input);
		output[1] = vector_get_y(input);
	}

	//////////////////////////////////////////////////////////////////////////
	// Writes a vector3 to unaligned memory.
	//////////////////////////////////////////////////////////////////////////
	inline void RTM_SIMD_CALL vector_store3(vector4f_arg0 input, float* output) RTM_NO_EXCEPT
	{
		output[0] = vector_get_x(input);
		output[1] = vector_get_y(input);
		output[2] = vector_get_z(input);
	}

	//////////////////////////////////////////////////////////////////////////
	// Writes a vector4 to unaligned memory.
	//////////////////////////////////////////////////////////////////////////
	inline void RTM_SIMD_CALL vector_store(vector4f_arg0 input, uint8_t* output) RTM_NO_EXCEPT
	{
		std::memcpy(output, &input, sizeof(vector4f));
	}

	//////////////////////////////////////////////////////////////////////////
	// Writes a vector1 to unaligned memory.
	//////////////////////////////////////////////////////////////////////////
	inline void RTM_SIMD_CALL vector_store1(vector4f_arg0 input, uint8_t* output) RTM_NO_EXCEPT
	{
		std::memcpy(output, &input, sizeof(float) * 1);
	}

	//////////////////////////////////////////////////////////////////////////
	// Writes a vector2 to unaligned memory.
	//////////////////////////////////////////////////////////////////////////
	inline void RTM_SIMD_CALL vector_store2(vector4f_arg0 input, uint8_t* output) RTM_NO_EXCEPT
	{
		std::memcpy(output, &input, sizeof(float) * 2);
	}

	//////////////////////////////////////////////////////////////////////////
	// Writes a vector3 to unaligned memory.
	//////////////////////////////////////////////////////////////////////////
	inline void RTM_SIMD_CALL vector_store3(vector4f_arg0 input, uint8_t* output) RTM_NO_EXCEPT
	{
		std::memcpy(output, &input, sizeof(float) * 3);
	}

	//////////////////////////////////////////////////////////////////////////
	// Writes a vector4 to unaligned memory.
	//////////////////////////////////////////////////////////////////////////
	inline void RTM_SIMD_CALL vector_store(vector4f_arg0 input, float4f* output) RTM_NO_EXCEPT
	{
		output->x = vector_get_x(input);
		output->y = vector_get_y(input);
		output->z = vector_get_z(input);
		output->w = vector_get_w(input);
	}

	//////////////////////////////////////////////////////////////////////////
	// Writes a vector2 to unaligned memory.
	//////////////////////////////////////////////////////////////////////////
	inline void RTM_SIMD_CALL vector_store2(vector4f_arg0 input, float2f* output) RTM_NO_EXCEPT
	{
		output->x = vector_get_x(input);
		output->y = vector_get_y(input);
	}

	//////////////////////////////////////////////////////////////////////////
	// Writes a vector3 to unaligned memory.
	//////////////////////////////////////////////////////////////////////////
	inline void RTM_SIMD_CALL vector_store3(vector4f_arg0 input, float3f* output) RTM_NO_EXCEPT
	{
		output->x = vector_get_x(input);
		output->y = vector_get_y(input);
		output->z = vector_get_z(input);
	}



	//////////////////////////////////////////////////////////////////////////
	// Arithmetic
	//////////////////////////////////////////////////////////////////////////


	//////////////////////////////////////////////////////////////////////////
	// Per component addition of the two inputs: lhs + rhs
	//////////////////////////////////////////////////////////////////////////
	inline vector4f RTM_SIMD_CALL vector_add(vector4f_arg0 lhs, vector4f_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_add_ps(lhs, rhs);
#elif defined(RTM_NEON_INTRINSICS)
		return vaddq_f32(lhs, rhs);
#else
		return vector_set(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z, lhs.w + rhs.w);
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Per component subtraction of the two inputs: lhs - rhs
	//////////////////////////////////////////////////////////////////////////
	inline vector4f RTM_SIMD_CALL vector_sub(vector4f_arg0 lhs, vector4f_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_sub_ps(lhs, rhs);
#elif defined(RTM_NEON_INTRINSICS)
		return vsubq_f32(lhs, rhs);
#else
		return vector_set(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z, lhs.w - rhs.w);
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Per component multiplication of the two inputs: lhs * rhs
	//////////////////////////////////////////////////////////////////////////
	inline vector4f RTM_SIMD_CALL vector_mul(vector4f_arg0 lhs, vector4f_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_mul_ps(lhs, rhs);
#elif defined(RTM_NEON_INTRINSICS)
		return vmulq_f32(lhs, rhs);
#else
		return vector_set(lhs.x * rhs.x, lhs.y * rhs.y, lhs.z * rhs.z, lhs.w * rhs.w);
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Per component multiplication of the vector by a scalar: lhs * rhs
	//////////////////////////////////////////////////////////////////////////
	inline vector4f RTM_SIMD_CALL vector_mul(vector4f_arg0 lhs, float rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_NEON_INTRINSICS)
		return vmulq_n_f32(lhs, rhs);
#else
		return vector_mul(lhs, vector_set(rhs));
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Per component division of the two inputs: lhs / rhs
	//////////////////////////////////////////////////////////////////////////
	inline vector4f RTM_SIMD_CALL vector_div(vector4f_arg0 lhs, vector4f_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_div_ps(lhs, rhs);
#elif defined (RTM_NEON64_INTRINSICS)
		return vdivq_f32(lhs, rhs);
#elif defined(RTM_NEON_INTRINSICS)
		// Perform two passes of Newton-Raphson iteration on the hardware estimate
		float32x4_t x0 = vrecpeq_f32(rhs);

		// First iteration
		float32x4_t x1 = vmulq_f32(x0, vrecpsq_f32(x0, rhs));

		// Second iteration
		float32x4_t x2 = vmulq_f32(x1, vrecpsq_f32(x1, rhs));
		return vmulq_f32(lhs, x2);
#else
		return vector_set(lhs.x / rhs.x, lhs.y / rhs.y, lhs.z / rhs.z, lhs.w / rhs.w);
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Per component maximum of the two inputs: max(lhs, rhs)
	//////////////////////////////////////////////////////////////////////////
	inline vector4f RTM_SIMD_CALL vector_max(vector4f_arg0 lhs, vector4f_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_max_ps(lhs, rhs);
#elif defined(RTM_NEON_INTRINSICS)
		return vmaxq_f32(lhs, rhs);
#else
		return vector_set(scalar_max(lhs.x, rhs.x), scalar_max(lhs.y, rhs.y), scalar_max(lhs.z, rhs.z), scalar_max(lhs.w, rhs.w));
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Per component minimum of the two inputs: min(lhs, rhs)
	//////////////////////////////////////////////////////////////////////////
	inline vector4f RTM_SIMD_CALL vector_min(vector4f_arg0 lhs, vector4f_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_min_ps(lhs, rhs);
#elif defined(RTM_NEON_INTRINSICS)
		return vminq_f32(lhs, rhs);
#else
		return vector_set(scalar_min(lhs.x, rhs.x), scalar_min(lhs.y, rhs.y), scalar_min(lhs.z, rhs.z), scalar_min(lhs.w, rhs.w));
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Per component clamping of an input between a minimum and a maximum value: min(max_value, max(min_value, input))
	//////////////////////////////////////////////////////////////////////////
	inline vector4f RTM_SIMD_CALL vector_clamp(vector4f_arg0 input, vector4f_arg1 min_value, vector4f_arg2 max_value) RTM_NO_EXCEPT
	{
		return vector_min(max_value, vector_max(min_value, input));
	}

	//////////////////////////////////////////////////////////////////////////
	// Per component absolute of the input: abs(input)
	//////////////////////////////////////////////////////////////////////////
	inline vector4f RTM_SIMD_CALL vector_abs(vector4f_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
#if defined(_MSC_VER)
		constexpr __m128i masks = { 0xFFU, 0xFFU, 0xFFU, 0x7FU, 0xFFU, 0xFFU, 0xFFU, 0x7FU, 0xFFU, 0xFFU, 0xFFU, 0x7FU, 0xFFU, 0xFFU, 0xFFU, 0x7FU };
#else
		constexpr __m128i masks = { 0x7FFFFFFF7FFFFFFFULL, 0x7FFFFFFF7FFFFFFFULL };
#endif
		return _mm_and_ps(input, _mm_castsi128_ps(masks));
#elif defined(RTM_NEON_INTRINSICS)
		return vabsq_f32(input);
#else
		return vector_set(scalar_abs(input.x), scalar_abs(input.y), scalar_abs(input.z), scalar_abs(input.w));
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Per component negation of the input: -input
	//////////////////////////////////////////////////////////////////////////
	inline vector4f RTM_SIMD_CALL vector_neg(vector4f_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		constexpr __m128 signs = { -0.0F, -0.0F, -0.0F, -0.0F };
		return _mm_xor_ps(input, signs);
#elif defined(RTM_NEON_INTRINSICS)
		return vnegq_f32(input);
#else
		return vector_mul(input, -1.0f);
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Per component reciprocal of the input: 1.0 / input
	//////////////////////////////////////////////////////////////////////////
	inline vector4f RTM_SIMD_CALL vector_reciprocal(vector4f_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		// Perform two passes of Newton-Raphson iteration on the hardware estimate
		__m128 x0 = _mm_rcp_ps(input);

		// First iteration
		__m128 x1 = _mm_sub_ps(_mm_add_ps(x0, x0), _mm_mul_ps(input, _mm_mul_ps(x0, x0)));

		// Second iteration
		__m128 x2 = _mm_sub_ps(_mm_add_ps(x1, x1), _mm_mul_ps(input, _mm_mul_ps(x1, x1)));
		return x2;
#elif defined(RTM_NEON_INTRINSICS)
		// Perform two passes of Newton-Raphson iteration on the hardware estimate
		float32x4_t x0 = vrecpeq_f32(input);

		// First iteration
		float32x4_t x1 = vmulq_f32(x0, vrecpsq_f32(x0, input));

		// Second iteration
		float32x4_t x2 = vmulq_f32(x1, vrecpsq_f32(x1, input));
		return x2;
#else
		return vector_div(vector_set(1.0F), input);
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Per component returns the smallest integer value not less than the input.
	// vector_ceil([1.8, 1.0, -1.8, -1.0]) = [2.0, 1.0, -1.0, -1.0]
	//////////////////////////////////////////////////////////////////////////
	inline vector4f RTM_SIMD_CALL vector_ceil(vector4f_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE4_INTRINSICS)
		return _mm_ceil_ps(input);
#else
		return vector_set(scalar_ceil(vector_get_x(input)), scalar_ceil(vector_get_y(input)), scalar_ceil(vector_get_z(input)), scalar_ceil(vector_get_w(input)));
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Per component returns the largest integer value not greater than the input.
	// vector_floor([1.8, 1.0, -1.8, -1.0]) = [1.0, 1.0, -2.0, -1.0]
	//////////////////////////////////////////////////////////////////////////
	inline vector4f RTM_SIMD_CALL vector_floor(vector4f_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE4_INTRINSICS)
		return _mm_floor_ps(input);
#else
		return vector_set(scalar_floor(vector_get_x(input)), scalar_floor(vector_get_y(input)), scalar_floor(vector_get_z(input)), scalar_floor(vector_get_w(input)));
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// 3D cross product: lhs x rhs
	//////////////////////////////////////////////////////////////////////////
	inline vector4f RTM_SIMD_CALL vector_cross3(vector4f_arg0 lhs, vector4f_arg1 rhs) RTM_NO_EXCEPT
	{
		return vector_set(vector_get_y(lhs) * vector_get_z(rhs) - vector_get_z(lhs) * vector_get_y(rhs),
						  vector_get_z(lhs) * vector_get_x(rhs) - vector_get_x(lhs) * vector_get_z(rhs),
						  vector_get_x(lhs) * vector_get_y(rhs) - vector_get_y(lhs) * vector_get_x(rhs));
	}

	//////////////////////////////////////////////////////////////////////////
	// 4D dot product: lhs . rhs
	//////////////////////////////////////////////////////////////////////////
	inline float RTM_SIMD_CALL vector_dot(vector4f_arg0 lhs, vector4f_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE4_INTRINSICS) && 0
		// SSE4 dot product instruction isn't precise enough
		return _mm_cvtss_f32(_mm_dp_ps(lhs, rhs, 0xFF));
#elif defined(RTM_SSE2_INTRINSICS)
		__m128 x2_y2_z2_w2 = _mm_mul_ps(lhs, rhs);
		__m128 z2_w2_0_0 = _mm_shuffle_ps(x2_y2_z2_w2, x2_y2_z2_w2, _MM_SHUFFLE(0, 0, 3, 2));
		__m128 x2z2_y2w2_0_0 = _mm_add_ps(x2_y2_z2_w2, z2_w2_0_0);
		__m128 y2w2_0_0_0 = _mm_shuffle_ps(x2z2_y2w2_0_0, x2z2_y2w2_0_0, _MM_SHUFFLE(0, 0, 0, 1));
		__m128 x2y2z2w2_0_0_0 = _mm_add_ps(x2z2_y2w2_0_0, y2w2_0_0_0);
		return _mm_cvtss_f32(x2y2z2w2_0_0_0);
#elif defined(RTM_NEON_INTRINSICS)
		float32x4_t x2_y2_z2_w2 = vmulq_f32(lhs, rhs);
		float32x2_t x2_y2 = vget_low_f32(x2_y2_z2_w2);
		float32x2_t z2_w2 = vget_high_f32(x2_y2_z2_w2);
		float32x2_t x2z2_y2w2 = vadd_f32(x2_y2, z2_w2);
		float32x2_t x2y2z2w2 = vpadd_f32(x2z2_y2w2, x2z2_y2w2);
		return vget_lane_f32(x2y2z2w2, 0);
#else
		return (vector_get_x(lhs) * vector_get_x(rhs)) + (vector_get_y(lhs) * vector_get_y(rhs)) + (vector_get_z(lhs) * vector_get_z(rhs)) + (vector_get_w(lhs) * vector_get_w(rhs));
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// 4D dot product: lhs . rhs
	//////////////////////////////////////////////////////////////////////////
	inline scalarf RTM_SIMD_CALL vector_dot_as_scalar(vector4f_arg0 lhs, vector4f_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE4_INTRINSICS) && 0
		// SSE4 dot product instruction isn't precise enough
		return _mm_cvtss_f32(_mm_dp_ps(lhs, rhs, 0xFF));
#elif defined(RTM_SSE2_INTRINSICS)
		__m128 x2_y2_z2_w2 = _mm_mul_ps(lhs, rhs);
		__m128 z2_w2_0_0 = _mm_shuffle_ps(x2_y2_z2_w2, x2_y2_z2_w2, _MM_SHUFFLE(0, 0, 3, 2));
		__m128 x2z2_y2w2_0_0 = _mm_add_ps(x2_y2_z2_w2, z2_w2_0_0);
		__m128 y2w2_0_0_0 = _mm_shuffle_ps(x2z2_y2w2_0_0, x2z2_y2w2_0_0, _MM_SHUFFLE(0, 0, 0, 1));
		__m128 x2y2z2w2_0_0_0 = _mm_add_ps(x2z2_y2w2_0_0, y2w2_0_0_0);
		return x2y2z2w2_0_0_0;
#elif defined(RTM_NEON_INTRINSICS)
		float32x4_t x2_y2_z2_w2 = vmulq_f32(lhs, rhs);
		float32x2_t x2_y2 = vget_low_f32(x2_y2_z2_w2);
		float32x2_t z2_w2 = vget_high_f32(x2_y2_z2_w2);
		float32x2_t x2z2_y2w2 = vadd_f32(x2_y2, z2_w2);
		float32x2_t x2y2z2w2 = vpadd_f32(x2z2_y2w2, x2z2_y2w2);
		return vget_lane_f32(x2y2z2w2, 0);
#else
		return (vector_get_x(lhs) * vector_get_x(rhs)) + (vector_get_y(lhs) * vector_get_y(rhs)) + (vector_get_z(lhs) * vector_get_z(rhs)) + (vector_get_w(lhs) * vector_get_w(rhs));
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// 4D dot product replicated in all components: lhs . rhs
	//////////////////////////////////////////////////////////////////////////
	inline vector4f RTM_SIMD_CALL vector_dot_as_vector(vector4f_arg0 lhs, vector4f_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE4_INTRINSICS) && 0
		// SSE4 dot product instruction isn't precise enough
		return _mm_dp_ps(lhs, rhs, 0xFF);
#elif defined(RTM_SSE2_INTRINSICS)
		__m128 x2_y2_z2_w2 = _mm_mul_ps(lhs, rhs);
		__m128 z2_w2_0_0 = _mm_shuffle_ps(x2_y2_z2_w2, x2_y2_z2_w2, _MM_SHUFFLE(0, 0, 3, 2));
		__m128 x2z2_y2w2_0_0 = _mm_add_ps(x2_y2_z2_w2, z2_w2_0_0);
		__m128 y2w2_0_0_0 = _mm_shuffle_ps(x2z2_y2w2_0_0, x2z2_y2w2_0_0, _MM_SHUFFLE(0, 0, 0, 1));
		__m128 x2y2z2w2_0_0_0 = _mm_add_ps(x2z2_y2w2_0_0, y2w2_0_0_0);
		return _mm_shuffle_ps(x2y2z2w2_0_0_0, x2y2z2w2_0_0_0, _MM_SHUFFLE(0, 0, 0, 0));
#elif defined(RTM_NEON_INTRINSICS)
		float32x4_t x2_y2_z2_w2 = vmulq_f32(lhs, rhs);
		float32x2_t x2_y2 = vget_low_f32(x2_y2_z2_w2);
		float32x2_t z2_w2 = vget_high_f32(x2_y2_z2_w2);
		float32x2_t x2z2_y2w2 = vadd_f32(x2_y2, z2_w2);
		float32x2_t x2y2z2w2 = vpadd_f32(x2z2_y2w2, x2z2_y2w2);
		return vcombine_f32(x2y2z2w2, x2y2z2w2);
#else
		return vector_set(vector_dot(lhs, rhs));
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
		struct vector4f_vector_dot3
		{
			inline RTM_SIMD_CALL operator float() const RTM_NO_EXCEPT
			{
#if defined(RTM_SSE4_INTRINSICS) && 0
				// SSE4 dot product instruction isn't precise enough
				return _mm_cvtss_f32(_mm_dp_ps(lhs, rhs, 0x7F));
#elif defined(RTM_SSE2_INTRINSICS)
				__m128 x2_y2_z2_w2 = _mm_mul_ps(lhs, rhs);
				__m128 y2_0_0_0 = _mm_shuffle_ps(x2_y2_z2_w2, x2_y2_z2_w2, _MM_SHUFFLE(0, 0, 0, 1));
				__m128 x2y2_0_0_0 = _mm_add_ss(x2_y2_z2_w2, y2_0_0_0);
				__m128 z2_0_0_0 = _mm_shuffle_ps(x2_y2_z2_w2, x2_y2_z2_w2, _MM_SHUFFLE(0, 0, 0, 2));
				__m128 x2y2z2_0_0_0 = _mm_add_ss(x2y2_0_0_0, z2_0_0_0);
				return _mm_cvtss_f32(x2y2z2_0_0_0);
#elif defined(RTM_NEON_INTRINSICS)
				float32x4_t x2_y2_z2_w2 = vmulq_f32(lhs, rhs);
				float32x2_t x2_y2 = vget_low_f32(x2_y2_z2_w2);
				float32x2_t z2_w2 = vget_high_f32(x2_y2_z2_w2);
				float32x2_t x2y2_x2y2 = vpadd_f32(x2_y2, x2_y2);
				float32x2_t z2_z2 = vdup_lane_f32(z2_w2, 0);
				float32x2_t x2y2z2_x2y2z2 = vadd_f32(x2y2_x2y2, z2_z2);
				return vget_lane_f32(x2y2z2_x2y2z2, 0);
#else
				return (vector_get_x(lhs) * vector_get_x(rhs)) + (vector_get_y(lhs) * vector_get_y(rhs)) + (vector_get_z(lhs) * vector_get_z(rhs));
#endif
			}

#if defined(RTM_SSE2_INTRINSICS)
			inline RTM_SIMD_CALL operator scalarf() const RTM_NO_EXCEPT
			{
				__m128 x2_y2_z2_w2 = _mm_mul_ps(lhs, rhs);
				__m128 y2_0_0_0 = _mm_shuffle_ps(x2_y2_z2_w2, x2_y2_z2_w2, _MM_SHUFFLE(0, 0, 0, 1));
				__m128 x2y2_0_0_0 = _mm_add_ss(x2_y2_z2_w2, y2_0_0_0);
				__m128 z2_0_0_0 = _mm_shuffle_ps(x2_y2_z2_w2, x2_y2_z2_w2, _MM_SHUFFLE(0, 0, 0, 2));
				return _mm_add_ss(x2y2_0_0_0, z2_0_0_0);
			}
#endif

			vector4f lhs;
			vector4f rhs;
		};
	}

	//////////////////////////////////////////////////////////////////////////
	// 3D dot product: lhs . rhs
	//////////////////////////////////////////////////////////////////////////
	constexpr rtm_impl::vector4f_vector_dot3 RTM_SIMD_CALL vector_dot3(vector4f_arg0 lhs, vector4f_arg1 rhs) RTM_NO_EXCEPT
	{
		return rtm_impl::vector4f_vector_dot3{ lhs, rhs };
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the squared length/norm of the vector4.
	//////////////////////////////////////////////////////////////////////////
	inline float RTM_SIMD_CALL vector_length_squared(vector4f_arg0 input) RTM_NO_EXCEPT
	{
		return vector_dot(input, input);
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the squared length/norm of the vector3.
	//////////////////////////////////////////////////////////////////////////
	constexpr rtm_impl::vector4f_vector_dot3 RTM_SIMD_CALL vector_length_squared3(vector4f_arg0 input) RTM_NO_EXCEPT
	{
		return rtm_impl::vector4f_vector_dot3{ input, input };
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the length/norm of the vector4.
	//////////////////////////////////////////////////////////////////////////
	inline float RTM_SIMD_CALL vector_length(vector4f_arg0 input) RTM_NO_EXCEPT
	{
		return scalar_sqrt(vector_length_squared(input));
	}

	namespace rtm_impl
	{
		//////////////////////////////////////////////////////////////////////////
		// This is a helper struct to allow a single consistent API between
		// various vector types when the semantics are identical but the return
		// type differs. Implicit coercion is used to return the desired value
		// at the call site.
		//////////////////////////////////////////////////////////////////////////
		struct vector4f_vector_length3
		{
			inline RTM_SIMD_CALL operator float() const RTM_NO_EXCEPT
			{
				const scalarf len_sq = vector_length_squared3(input);
				return scalar_cast(scalar_sqrt(len_sq));
			}

#if defined(RTM_SSE2_INTRINSICS)
			inline RTM_SIMD_CALL operator scalarf() const RTM_NO_EXCEPT
			{
				const scalarf len_sq = vector_length_squared3(input);
				return scalar_sqrt(len_sq);
			}
#endif

			vector4f input;
		};
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the length/norm of the vector3.
	//////////////////////////////////////////////////////////////////////////
	constexpr rtm_impl::vector4f_vector_length3 RTM_SIMD_CALL vector_length3(vector4f_arg0 input) RTM_NO_EXCEPT
	{
		return rtm_impl::vector4f_vector_length3{ input };
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the reciprocal length/norm of the vector4.
	//////////////////////////////////////////////////////////////////////////
	inline float RTM_SIMD_CALL vector_length_reciprocal(vector4f_arg0 input) RTM_NO_EXCEPT
	{
		return scalar_sqrt_reciprocal(vector_length_squared(input));
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the reciprocal length/norm of the vector3.
	//////////////////////////////////////////////////////////////////////////
	inline float RTM_SIMD_CALL vector_length_reciprocal3(vector4f_arg0 input) RTM_NO_EXCEPT
	{
		return scalar_sqrt_reciprocal(vector_length_squared3(input));
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the distance between two 3D points.
	//////////////////////////////////////////////////////////////////////////
	inline rtm_impl::vector4f_vector_length3 RTM_SIMD_CALL vector_distance3(vector4f_arg0 lhs, vector4f_arg1 rhs) RTM_NO_EXCEPT
	{
		const vector4f difference = vector_sub(lhs, rhs);
		return rtm_impl::vector4f_vector_length3{ difference };
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns a normalized vector3.
	// If the length of the input is below the supplied threshold, the
	// fall back value is returned instead.
	//////////////////////////////////////////////////////////////////////////
	inline vector4f RTM_SIMD_CALL vector_normalize3(vector4f_arg0 input, vector4f_arg1 fallback, float threshold = 1.0E-8F) RTM_NO_EXCEPT
	{
		// Reciprocal is more accurate to normalize with
		const float len_sq = vector_length_squared3(input);
		if (len_sq >= threshold)
			return vector_mul(input, scalar_sqrt_reciprocal(len_sq));
		else
			return fallback;
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns per component the fractional part of the input.
	//////////////////////////////////////////////////////////////////////////
	inline vector4f RTM_SIMD_CALL vector_fraction(vector4f_arg0 input) RTM_NO_EXCEPT
	{
		return vector_set(scalar_fraction(vector_get_x(input)), scalar_fraction(vector_get_y(input)), scalar_fraction(vector_get_z(input)), scalar_fraction(vector_get_w(input)));
	}

	//////////////////////////////////////////////////////////////////////////
	// Per component multiplication/addition of the three inputs: v2 + (v0 * v1)
	//////////////////////////////////////////////////////////////////////////
	inline vector4f RTM_SIMD_CALL vector_mul_add(vector4f_arg0 v0, vector4f_arg1 v1, vector4f_arg2 v2) RTM_NO_EXCEPT
	{
#if defined(RTM_NEON64_INTRINSICS)
		return vfmaq_f32(v2, v0, v1);
#elif defined(RTM_NEON_INTRINSICS)
		return vmlaq_f32(v2, v0, v1);
#else
		return vector_add(vector_mul(v0, v1), v2);
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Per component multiplication/addition of the three inputs: v2 + (v0 * s1)
	//////////////////////////////////////////////////////////////////////////
	inline vector4f RTM_SIMD_CALL vector_mul_add(vector4f_arg0 v0, float s1, vector4f_arg2 v2) RTM_NO_EXCEPT
	{
#if defined(RTM_NEON64_INTRINSICS)
		return vfmaq_n_f32(v2, v0, s1);
#elif defined(RTM_NEON_INTRINSICS)
		return vmlaq_n_f32(v2, v0, s1);
#else
		return vector_add(vector_mul(v0, s1), v2);
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Per component negative multiplication/subtraction of the three inputs: -((v0 * v1) - v2)
	// This is mathematically equivalent to: v2 - (v0 * v1)
	//////////////////////////////////////////////////////////////////////////
	inline vector4f RTM_SIMD_CALL vector_neg_mul_sub(vector4f_arg0 v0, vector4f_arg1 v1, vector4f_arg2 v2) RTM_NO_EXCEPT
	{
#if defined(RTM_NEON64_INTRINSICS)
		return vfmsq_f32(v2, v0, v1);
#elif defined(RTM_NEON_INTRINSICS)
		return vmlsq_f32(v2, v0, v1);
#else
		return vector_sub(v2, vector_mul(v0, v1));
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Per component negative multiplication/subtraction of the three inputs: -((v0 * s1) - v2)
	// This is mathematically equivalent to: v2 - (v0 * s1)
	//////////////////////////////////////////////////////////////////////////
	inline vector4f RTM_SIMD_CALL vector_neg_mul_sub(vector4f_arg0 v0, float s1, vector4f_arg2 v2) RTM_NO_EXCEPT
	{
#if defined(RTM_NEON64_INTRINSICS)
		return vfmsq_n_f32(v2, v0, s1);
#elif defined(RTM_NEON_INTRINSICS)
		return vmlsq_n_f32(v2, v0, s1);
#else
		return vector_sub(v2, vector_mul(v0, s1));
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Per component linear interpolation of the two inputs at the specified alpha.
	//////////////////////////////////////////////////////////////////////////
	inline vector4f RTM_SIMD_CALL vector_lerp(vector4f_arg0 start, vector4f_arg1 end, float alpha) RTM_NO_EXCEPT
	{
		return vector_mul_add(vector_sub(end, start), alpha, start);
	}



	//////////////////////////////////////////////////////////////////////////
	// Comparisons and masking
	//////////////////////////////////////////////////////////////////////////


	//////////////////////////////////////////////////////////////////////////
	// Returns per component ~0 if less than, otherwise 0: lhs < rhs ? ~0 : 0
	//////////////////////////////////////////////////////////////////////////
	inline mask4i RTM_SIMD_CALL vector_less_than(vector4f_arg0 lhs, vector4f_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_cmplt_ps(lhs, rhs);
#elif defined(RTM_NEON_INTRINSICS)
		return vcltq_f32(lhs, rhs);
#else
		return mask4i{ rtm_impl::get_mask_value(lhs.x < rhs.x), rtm_impl::get_mask_value(lhs.y < rhs.y), rtm_impl::get_mask_value(lhs.z < rhs.z), rtm_impl::get_mask_value(lhs.w < rhs.w) };
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns per component ~0 if less equal, otherwise 0: lhs <= rhs ? ~0 : 0
	//////////////////////////////////////////////////////////////////////////
	inline mask4i RTM_SIMD_CALL vector_less_equal(vector4f_arg0 lhs, vector4f_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_cmple_ps(lhs, rhs);
#elif defined(RTM_NEON_INTRINSICS)
		return vcleq_f32(lhs, rhs);
#else
		return mask4i{ rtm_impl::get_mask_value(lhs.x <= rhs.x), rtm_impl::get_mask_value(lhs.y <= rhs.y), rtm_impl::get_mask_value(lhs.z <= rhs.z), rtm_impl::get_mask_value(lhs.w <= rhs.w) };
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns per component ~0 if greater equal, otherwise 0: lhs >= rhs ? ~0 : 0
	//////////////////////////////////////////////////////////////////////////
	inline mask4i RTM_SIMD_CALL vector_greater_equal(vector4f_arg0 lhs, vector4f_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_cmpge_ps(lhs, rhs);
#elif defined(RTM_NEON_INTRINSICS)
		return vcgeq_f32(lhs, rhs);
#else
		return mask4i{ rtm_impl::get_mask_value(lhs.x >= rhs.x), rtm_impl::get_mask_value(lhs.y >= rhs.y), rtm_impl::get_mask_value(lhs.z >= rhs.z), rtm_impl::get_mask_value(lhs.w >= rhs.w) };
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all 4 components are less than, otherwise false: all(lhs < rhs)
	//////////////////////////////////////////////////////////////////////////
	inline bool RTM_SIMD_CALL vector_all_less_than(vector4f_arg0 lhs, vector4f_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_movemask_ps(_mm_cmplt_ps(lhs, rhs)) == 0xF;
#elif defined(RTM_NEON_INTRINSICS)
		uint32x4_t mask = vcltq_f32(lhs, rhs);
		uint8x8x2_t mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15 = vzip_u8(vget_low_u8(mask), vget_high_u8(mask));
		uint16x4x2_t mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15 = vzip_u16(mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[0], mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[1]);
		return vget_lane_u32(mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15.val[0], 0) == 0xFFFFFFFFU;
#else
		return lhs.x < rhs.x && lhs.y < rhs.y && lhs.z < rhs.z && lhs.w < rhs.w;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all 2 components are less than, otherwise false: all(lhs < rhs)
	//////////////////////////////////////////////////////////////////////////
	inline bool RTM_SIMD_CALL vector_all_less_than2(vector4f_arg0 lhs, vector4f_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return (_mm_movemask_ps(_mm_cmplt_ps(lhs, rhs)) & 0x3) == 0x3;
#elif defined(RTM_NEON_INTRINSICS)
		uint32x2_t mask = vclt_f32(vget_low_f32(lhs), vget_low_f32(rhs));
		return vget_lane_u64(mask, 0) == 0xFFFFFFFFFFFFFFFFu;
#else
		return lhs.x < rhs.x && lhs.y < rhs.y;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all 3 components are less than, otherwise false: all(lhs < rhs)
	//////////////////////////////////////////////////////////////////////////
	inline bool RTM_SIMD_CALL vector_all_less_than3(vector4f_arg0 lhs, vector4f_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return (_mm_movemask_ps(_mm_cmplt_ps(lhs, rhs)) & 0x7) == 0x7;
#elif defined(RTM_NEON_INTRINSICS)
		uint32x4_t mask = vcltq_f32(lhs, rhs);
		uint8x8x2_t mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15 = vzip_u8(vget_low_u8(mask), vget_high_u8(mask));
		uint16x4x2_t mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15 = vzip_u16(mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[0], mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[1]);
		return (vget_lane_u32(mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15.val[0], 0) & 0x00FFFFFFU) == 0x00FFFFFFU;
#else
		return lhs.x < rhs.x && lhs.y < rhs.y && lhs.z < rhs.z;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if any 4 components are less than, otherwise false: any(lhs < rhs)
	//////////////////////////////////////////////////////////////////////////
	inline bool RTM_SIMD_CALL vector_any_less_than(vector4f_arg0 lhs, vector4f_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_movemask_ps(_mm_cmplt_ps(lhs, rhs)) != 0;
#elif defined(RTM_NEON_INTRINSICS)
		uint32x4_t mask = vcltq_f32(lhs, rhs);
		uint8x8x2_t mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15 = vzip_u8(vget_low_u8(mask), vget_high_u8(mask));
		uint16x4x2_t mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15 = vzip_u16(mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[0], mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[1]);
		return vget_lane_u32(mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15.val[0], 0) != 0;
#else
		return lhs.x < rhs.x || lhs.y < rhs.y || lhs.z < rhs.z || lhs.w < rhs.w;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if any 2 components are less than, otherwise false: any(lhs < rhs)
	//////////////////////////////////////////////////////////////////////////
	inline bool RTM_SIMD_CALL vector_any_less_than2(vector4f_arg0 lhs, vector4f_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return (_mm_movemask_ps(_mm_cmplt_ps(lhs, rhs)) & 0x3) != 0;
#elif defined(RTM_NEON_INTRINSICS)
		uint32x2_t mask = vclt_f32(vget_low_f32(lhs), vget_low_f32(rhs));
		return vget_lane_u64(mask, 0) != 0;
#else
		return lhs.x < rhs.x || lhs.y < rhs.y;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if any 3 components are less than, otherwise false: any(lhs < rhs)
	//////////////////////////////////////////////////////////////////////////
	inline bool RTM_SIMD_CALL vector_any_less_than3(vector4f_arg0 lhs, vector4f_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return (_mm_movemask_ps(_mm_cmplt_ps(lhs, rhs)) & 0x7) != 0;
#elif defined(RTM_NEON_INTRINSICS)
		uint32x4_t mask = vcltq_f32(lhs, rhs);
		uint8x8x2_t mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15 = vzip_u8(vget_low_u8(mask), vget_high_u8(mask));
		uint16x4x2_t mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15 = vzip_u16(mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[0], mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[1]);
		return (vget_lane_u32(mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15.val[0], 0) & 0x00FFFFFFU) != 0;
#else
		return lhs.x < rhs.x || lhs.y < rhs.y || lhs.z < rhs.z;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all 4 components are less equal, otherwise false: all(lhs <= rhs)
	//////////////////////////////////////////////////////////////////////////
	inline bool RTM_SIMD_CALL vector_all_less_equal(vector4f_arg0 lhs, vector4f_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_movemask_ps(_mm_cmple_ps(lhs, rhs)) == 0xF;
#elif defined(RTM_NEON_INTRINSICS)
		uint32x4_t mask = vcleq_f32(lhs, rhs);
		uint8x8x2_t mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15 = vzip_u8(vget_low_u8(mask), vget_high_u8(mask));
		uint16x4x2_t mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15 = vzip_u16(mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[0], mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[1]);
		return vget_lane_u32(mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15.val[0], 0) == 0xFFFFFFFFU;
#else
		return lhs.x <= rhs.x && lhs.y <= rhs.y && lhs.z <= rhs.z && lhs.w <= rhs.w;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all 2 components are less equal, otherwise false: all(lhs <= rhs)
	//////////////////////////////////////////////////////////////////////////
	inline bool RTM_SIMD_CALL vector_all_less_equal2(vector4f_arg0 lhs, vector4f_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return (_mm_movemask_ps(_mm_cmple_ps(lhs, rhs)) & 0x3) == 0x3;
#elif defined(RTM_NEON_INTRINSICS)
		uint32x2_t mask = vcle_f32(vget_low_f32(lhs), vget_low_f32(rhs));
		return vget_lane_u64(mask, 0) == 0xFFFFFFFFFFFFFFFFULL;
#else
		return lhs.x <= rhs.x && lhs.y <= rhs.y;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all 3 components are less equal, otherwise false: all(lhs <= rhs)
	//////////////////////////////////////////////////////////////////////////
	inline bool RTM_SIMD_CALL vector_all_less_equal3(vector4f_arg0 lhs, vector4f_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return (_mm_movemask_ps(_mm_cmple_ps(lhs, rhs)) & 0x7) == 0x7;
#elif defined(RTM_NEON_INTRINSICS)
		uint32x4_t mask = vcleq_f32(lhs, rhs);
		uint8x8x2_t mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15 = vzip_u8(vget_low_u8(mask), vget_high_u8(mask));
		uint16x4x2_t mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15 = vzip_u16(mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[0], mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[1]);
		return (vget_lane_u32(mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15.val[0], 0) & 0x00FFFFFFU) == 0x00FFFFFFU;
#else
		return lhs.x <= rhs.x && lhs.y <= rhs.y && lhs.z <= rhs.z;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if any 4 components are less equal, otherwise false: any(lhs <= rhs)
	//////////////////////////////////////////////////////////////////////////
	inline bool RTM_SIMD_CALL vector_any_less_equal(vector4f_arg0 lhs, vector4f_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_movemask_ps(_mm_cmple_ps(lhs, rhs)) != 0;
#elif defined(RTM_NEON_INTRINSICS)
		uint32x4_t mask = vcleq_f32(lhs, rhs);
		uint8x8x2_t mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15 = vzip_u8(vget_low_u8(mask), vget_high_u8(mask));
		uint16x4x2_t mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15 = vzip_u16(mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[0], mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[1]);
		return vget_lane_u32(mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15.val[0], 0) != 0;
#else
		return lhs.x <= rhs.x || lhs.y <= rhs.y || lhs.z <= rhs.z || lhs.w <= rhs.w;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if any 2 components are less equal, otherwise false: any(lhs <= rhs)
	//////////////////////////////////////////////////////////////////////////
	inline bool RTM_SIMD_CALL vector_any_less_equal2(vector4f_arg0 lhs, vector4f_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return (_mm_movemask_ps(_mm_cmple_ps(lhs, rhs)) & 0x3) != 0;
#elif defined(RTM_NEON_INTRINSICS)
		uint32x2_t mask = vcle_f32(vget_low_f32(lhs), vget_low_f32(rhs));
		return vget_lane_u64(mask, 0) != 0;
#else
		return lhs.x <= rhs.x || lhs.y <= rhs.y;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if any 3 components are less equal, otherwise false: any(lhs <= rhs)
	//////////////////////////////////////////////////////////////////////////
	inline bool RTM_SIMD_CALL vector_any_less_equal3(vector4f_arg0 lhs, vector4f_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return (_mm_movemask_ps(_mm_cmple_ps(lhs, rhs)) & 0x7) != 0;
#elif defined(RTM_NEON_INTRINSICS)
		uint32x4_t mask = vcleq_f32(lhs, rhs);
		uint8x8x2_t mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15 = vzip_u8(vget_low_u8(mask), vget_high_u8(mask));
		uint16x4x2_t mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15 = vzip_u16(mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[0], mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[1]);
		return (vget_lane_u32(mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15.val[0], 0) & 0x00FFFFFFU) != 0;
#else
		return lhs.x <= rhs.x || lhs.y <= rhs.y || lhs.z <= rhs.z;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all 4 components are greater equal, otherwise false: all(lhs >= rhs)
	//////////////////////////////////////////////////////////////////////////
	inline bool RTM_SIMD_CALL vector_all_greater_equal(vector4f_arg0 lhs, vector4f_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_movemask_ps(_mm_cmpge_ps(lhs, rhs)) == 0xF;
#elif defined(RTM_NEON_INTRINSICS)
		uint32x4_t mask = vcgeq_f32(lhs, rhs);
		uint8x8x2_t mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15 = vzip_u8(vget_low_u8(mask), vget_high_u8(mask));
		uint16x4x2_t mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15 = vzip_u16(mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[0], mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[1]);
		return vget_lane_u32(mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15.val[0], 0) == 0xFFFFFFFFU;
#else
		return lhs.x >= rhs.x && lhs.y >= rhs.y && lhs.z >= rhs.z && lhs.w >= rhs.w;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all 2 components are greater equal, otherwise false: all(lhs >= rhs)
	//////////////////////////////////////////////////////////////////////////
	inline bool RTM_SIMD_CALL vector_all_greater_equal2(vector4f_arg0 lhs, vector4f_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return (_mm_movemask_ps(_mm_cmpge_ps(lhs, rhs)) & 0x3) == 0x3;
#elif defined(RTM_NEON_INTRINSICS)
		uint32x2_t mask = vcge_f32(vget_low_f32(lhs), vget_low_f32(rhs));
		return vget_lane_u64(mask, 0) == 0xFFFFFFFFFFFFFFFFULL;
#else
		return lhs.x >= rhs.x && lhs.y >= rhs.y;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all 3 components are greater equal, otherwise false: all(lhs >= rhs)
	//////////////////////////////////////////////////////////////////////////
	inline bool RTM_SIMD_CALL vector_all_greater_equal3(vector4f_arg0 lhs, vector4f_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return (_mm_movemask_ps(_mm_cmpge_ps(lhs, rhs)) & 0x7) == 0x7;
#elif defined(RTM_NEON_INTRINSICS)
		uint32x4_t mask = vcgeq_f32(lhs, rhs);
		uint8x8x2_t mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15 = vzip_u8(vget_low_u8(mask), vget_high_u8(mask));
		uint16x4x2_t mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15 = vzip_u16(mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[0], mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[1]);
		return (vget_lane_u32(mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15.val[0], 0) & 0x00FFFFFFU) == 0x00FFFFFFU;
#else
		return lhs.x >= rhs.x && lhs.y >= rhs.y && lhs.z >= rhs.z;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if any 4 components are greater equal, otherwise false: any(lhs >= rhs)
	//////////////////////////////////////////////////////////////////////////
	inline bool RTM_SIMD_CALL vector_any_greater_equal(vector4f_arg0 lhs, vector4f_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_movemask_ps(_mm_cmpge_ps(lhs, rhs)) != 0;
#elif defined(RTM_NEON_INTRINSICS)
		uint32x4_t mask = vcgeq_f32(lhs, rhs);
		uint8x8x2_t mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15 = vzip_u8(vget_low_u8(mask), vget_high_u8(mask));
		uint16x4x2_t mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15 = vzip_u16(mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[0], mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[1]);
		return vget_lane_u32(mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15.val[0], 0) != 0;
#else
		return lhs.x >= rhs.x || lhs.y >= rhs.y || lhs.z >= rhs.z || lhs.w >= rhs.w;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if any 2 components are greater equal, otherwise false: any(lhs >= rhs)
	//////////////////////////////////////////////////////////////////////////
	inline bool RTM_SIMD_CALL vector_any_greater_equal2(vector4f_arg0 lhs, vector4f_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return (_mm_movemask_ps(_mm_cmpge_ps(lhs, rhs)) & 0x3) != 0;
#elif defined(RTM_NEON_INTRINSICS)
		uint32x2_t mask = vcge_f32(vget_low_f32(lhs), vget_low_f32(rhs));
		return vget_lane_u64(mask, 0) != 0;
#else
		return lhs.x >= rhs.x || lhs.y >= rhs.y;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if any 3 components are greater equal, otherwise false: any(lhs >= rhs)
	//////////////////////////////////////////////////////////////////////////
	inline bool RTM_SIMD_CALL vector_any_greater_equal3(vector4f_arg0 lhs, vector4f_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return (_mm_movemask_ps(_mm_cmpge_ps(lhs, rhs)) & 0x7) != 0;
#elif defined(RTM_NEON_INTRINSICS)
		uint32x4_t mask = vcgeq_f32(lhs, rhs);
		uint8x8x2_t mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15 = vzip_u8(vget_low_u8(mask), vget_high_u8(mask));
		uint16x4x2_t mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15 = vzip_u16(mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[0], mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[1]);
		return (vget_lane_u32(mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15.val[0], 0) & 0x00FFFFFFU) != 0;
#else
		return lhs.x >= rhs.x || lhs.y >= rhs.y || lhs.z >= rhs.z;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all 4 components are near equal, otherwise false: all(abs(lhs - rhs) < threshold)
	//////////////////////////////////////////////////////////////////////////
	inline bool RTM_SIMD_CALL vector_all_near_equal(vector4f_arg0 lhs, vector4f_arg1 rhs, float threshold = 0.00001F) RTM_NO_EXCEPT
	{
		return vector_all_less_equal(vector_abs(vector_sub(lhs, rhs)), vector_set(threshold));
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all 2 components are near equal, otherwise false: all(abs(lhs - rhs) < threshold)
	//////////////////////////////////////////////////////////////////////////
	inline bool RTM_SIMD_CALL vector_all_near_equal2(vector4f_arg0 lhs, vector4f_arg1 rhs, float threshold = 0.00001F) RTM_NO_EXCEPT
	{
		return vector_all_less_equal2(vector_abs(vector_sub(lhs, rhs)), vector_set(threshold));
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all 3 components are near equal, otherwise false: all(abs(lhs - rhs) < threshold)
	//////////////////////////////////////////////////////////////////////////
	inline bool RTM_SIMD_CALL vector_all_near_equal3(vector4f_arg0 lhs, vector4f_arg1 rhs, float threshold = 0.00001F) RTM_NO_EXCEPT
	{
		return vector_all_less_equal3(vector_abs(vector_sub(lhs, rhs)), vector_set(threshold));
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if any 4 components are near equal, otherwise false: any(abs(lhs - rhs) < threshold)
	//////////////////////////////////////////////////////////////////////////
	inline bool RTM_SIMD_CALL vector_any_near_equal(vector4f_arg0 lhs, vector4f_arg1 rhs, float threshold = 0.00001F) RTM_NO_EXCEPT
	{
		return vector_any_less_equal(vector_abs(vector_sub(lhs, rhs)), vector_set(threshold));
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if any 2 components are near equal, otherwise false: any(abs(lhs - rhs) < threshold)
	//////////////////////////////////////////////////////////////////////////
	inline bool RTM_SIMD_CALL vector_any_near_equal2(vector4f_arg0 lhs, vector4f_arg1 rhs, float threshold = 0.00001F) RTM_NO_EXCEPT
	{
		return vector_any_less_equal2(vector_abs(vector_sub(lhs, rhs)), vector_set(threshold));
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if any 3 components are near equal, otherwise false: any(abs(lhs - rhs) < threshold)
	//////////////////////////////////////////////////////////////////////////
	inline bool RTM_SIMD_CALL vector_any_near_equal3(vector4f_arg0 lhs, vector4f_arg1 rhs, float threshold = 0.00001F) RTM_NO_EXCEPT
	{
		return vector_any_less_equal3(vector_abs(vector_sub(lhs, rhs)), vector_set(threshold));
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all 4 components are finite (not NaN/Inf), otherwise false: all(finite(input))
	//////////////////////////////////////////////////////////////////////////
	inline bool RTM_SIMD_CALL vector_is_finite(vector4f_arg0 input) RTM_NO_EXCEPT
	{
		return scalar_is_finite(vector_get_x(input)) && scalar_is_finite(vector_get_y(input)) && scalar_is_finite(vector_get_z(input)) && scalar_is_finite(vector_get_w(input));
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all 2 components are finite (not NaN/Inf), otherwise false: all(finite(input))
	//////////////////////////////////////////////////////////////////////////
	inline bool RTM_SIMD_CALL vector_is_finite2(vector4f_arg0 input) RTM_NO_EXCEPT
	{
		return scalar_is_finite(vector_get_x(input)) && scalar_is_finite(vector_get_y(input));
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all 3 components are finite (not NaN/Inf), otherwise false: all(finite(input))
	//////////////////////////////////////////////////////////////////////////
	inline bool RTM_SIMD_CALL vector_is_finite3(vector4f_arg0 input) RTM_NO_EXCEPT
	{
		return scalar_is_finite(vector_get_x(input)) && scalar_is_finite(vector_get_y(input)) && scalar_is_finite(vector_get_z(input));
	}



	//////////////////////////////////////////////////////////////////////////
	// Swizzling, permutations, and mixing
	//////////////////////////////////////////////////////////////////////////


	//////////////////////////////////////////////////////////////////////////
	// Per component selection depending on the mask: mask != 0 ? if_true : if_false
	//////////////////////////////////////////////////////////////////////////
	inline vector4f RTM_SIMD_CALL vector_select(mask4i_arg0 mask, vector4f_arg1 if_true, vector4f_arg2 if_false) RTM_NO_EXCEPT
	{
#if defined(RTM_AVX_INTRINSICS)
		return _mm_blendv_ps(if_false, if_true, mask);
#elif defined(RTM_SSE2_INTRINSICS)
		return _mm_or_ps(_mm_andnot_ps(mask, if_false), _mm_and_ps(if_true, mask));
#elif defined(RTM_NEON_INTRINSICS)
		return vbslq_f32(mask, if_true, if_false);
#else
		return vector4f{ rtm_impl::select(mask.x, if_true.x, if_false.x), rtm_impl::select(mask.y, if_true.y, if_false.y), rtm_impl::select(mask.z, if_true.z, if_false.z), rtm_impl::select(mask.w, if_true.w, if_false.w) };
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Mixes two inputs and returns the desired components.
	// [xyzw] indexes into the first input while [abcd] indexes in the second.
	//////////////////////////////////////////////////////////////////////////
	template<mix4 comp0, mix4 comp1, mix4 comp2, mix4 comp3>
	inline vector4f RTM_SIMD_CALL vector_mix(vector4f_arg0 input0, vector4f_arg1 input1) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		// All four components come from input 0
		if (rtm_impl::is_mix_xyzw(comp0) && rtm_impl::is_mix_xyzw(comp1) && rtm_impl::is_mix_xyzw(comp2) && rtm_impl::is_mix_xyzw(comp3))
			return _mm_shuffle_ps(input0, input0, _MM_SHUFFLE(int(comp3) % 4, int(comp2) % 4, int(comp1) % 4, int(comp0) % 4));

		// All four components come from input 1
		if (rtm_impl::is_mix_abcd(comp0) && rtm_impl::is_mix_abcd(comp1) && rtm_impl::is_mix_abcd(comp2) && rtm_impl::is_mix_abcd(comp3))
			return _mm_shuffle_ps(input1, input1, _MM_SHUFFLE(int(comp3) % 4, int(comp2) % 4, int(comp1) % 4, int(comp0) % 4));

		// First two components come from input 0, second two come from input 1
		if (rtm_impl::is_mix_xyzw(comp0) && rtm_impl::is_mix_xyzw(comp1) && rtm_impl::is_mix_abcd(comp2) && rtm_impl::is_mix_abcd(comp3))
			return _mm_shuffle_ps(input0, input1, _MM_SHUFFLE(int(comp3) % 4, int(comp2) % 4, int(comp1) % 4, int(comp0) % 4));

		// First two components come from input 1, second two come from input 0
		if (rtm_impl::is_mix_abcd(comp0) && rtm_impl::is_mix_abcd(comp1) && rtm_impl::is_mix_xyzw(comp2) && rtm_impl::is_mix_xyzw(comp3))
			return _mm_shuffle_ps(input1, input0, _MM_SHUFFLE(int(comp3) % 4, int(comp2) % 4, int(comp1) % 4, int(comp0) % 4));

		// Low words from both inputs are interleaved
		if (rtm_impl::static_condition<comp0 == mix4::x && comp1 == mix4::a && comp2 == mix4::y && comp3 == mix4::b>::test())
			return _mm_unpacklo_ps(input0, input1);

		// Low words from both inputs are interleaved
		if (rtm_impl::static_condition<comp0 == mix4::a && comp1 == mix4::x && comp2 == mix4::b && comp3 == mix4::y>::test())
			return _mm_unpacklo_ps(input1, input0);

		// High words from both inputs are interleaved
		if (rtm_impl::static_condition<comp0 == mix4::z && comp1 == mix4::c && comp2 == mix4::w && comp3 == mix4::d>::test())
			return _mm_unpackhi_ps(input0, input1);

		// High words from both inputs are interleaved
		if (rtm_impl::static_condition<comp0 == mix4::c && comp1 == mix4::z && comp2 == mix4::d && comp3 == mix4::w>::test())
			return _mm_unpackhi_ps(input1, input0);
#endif	// defined(RTM_SSE2_INTRINSICS)

		// Slow code path, not yet optimized or not using intrinsics
		const float x = rtm_impl::is_mix_xyzw(comp0) ? vector_get_component<comp0>(input0) : vector_get_component<comp0>(input1);
		const float y = rtm_impl::is_mix_xyzw(comp1) ? vector_get_component<comp1>(input0) : vector_get_component<comp1>(input1);
		const float z = rtm_impl::is_mix_xyzw(comp2) ? vector_get_component<comp2>(input0) : vector_get_component<comp2>(input1);
		const float w = rtm_impl::is_mix_xyzw(comp3) ? vector_get_component<comp3>(input0) : vector_get_component<comp3>(input1);
		return vector_set(x, y, z, w);
	}

	//////////////////////////////////////////////////////////////////////////
	// Replicates the [x] component in all components.
	//////////////////////////////////////////////////////////////////////////
	inline vector4f RTM_SIMD_CALL vector_dup_x(vector4f_arg0 input) RTM_NO_EXCEPT { return vector_mix<mix4::x, mix4::x, mix4::x, mix4::x>(input, input); }

	//////////////////////////////////////////////////////////////////////////
	// Replicates the [y] component in all components.
	//////////////////////////////////////////////////////////////////////////
	inline vector4f RTM_SIMD_CALL vector_dup_y(vector4f_arg0 input) RTM_NO_EXCEPT { return vector_mix<mix4::y, mix4::y, mix4::y, mix4::y>(input, input); }

	//////////////////////////////////////////////////////////////////////////
	// Replicates the [z] component in all components.
	//////////////////////////////////////////////////////////////////////////
	inline vector4f RTM_SIMD_CALL vector_dup_z(vector4f_arg0 input) RTM_NO_EXCEPT { return vector_mix<mix4::z, mix4::z, mix4::z, mix4::z>(input, input); }

	//////////////////////////////////////////////////////////////////////////
	// Replicates the [w] component in all components.
	//////////////////////////////////////////////////////////////////////////
	inline vector4f RTM_SIMD_CALL vector_dup_w(vector4f_arg0 input) RTM_NO_EXCEPT { return vector_mix<mix4::w, mix4::w, mix4::w, mix4::w>(input, input); }


	//////////////////////////////////////////////////////////////////////////
	// Miscellaneous
	//////////////////////////////////////////////////////////////////////////


	//////////////////////////////////////////////////////////////////////////
	// Returns per component the sign of the input vector: input >= 0.0 ? 1.0 : -1.0
	//////////////////////////////////////////////////////////////////////////
	inline vector4f RTM_SIMD_CALL vector_sign(vector4f_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		constexpr __m128 signs = { -0.0F, -0.0F, -0.0F, -0.0F };
		constexpr __m128 one = { 1.0F, 1.0F, 1.0F, 1.0F };
		const __m128 sign_bits = _mm_and_ps(input, signs);	// Mask out the sign bit
		return _mm_or_ps(sign_bits, one);					// Copy the sign bit onto +-1.0f
#else
		const mask4i mask = vector_greater_equal(input, vector_zero());
		return vector_select(mask, vector_set(1.0F), vector_set(-1.0F));
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns per component the rounded input using a symmetric algorithm.
	// symmetric_round(1.5) = 2.0
	// symmetric_round(1.2) = 1.0
	// symmetric_round(-1.5) = -2.0
	// symmetric_round(-1.2) = -1.0
	//////////////////////////////////////////////////////////////////////////
	inline vector4f RTM_SIMD_CALL vector_symmetric_round(vector4f_arg0 input) RTM_NO_EXCEPT
	{
		const vector4f half = vector_set(0.5F);
		const vector4f floored = vector_floor(vector_add(input, half));
		const vector4f ceiled = vector_ceil(vector_sub(input, half));
		const mask4i is_greater_equal = vector_greater_equal(input, vector_zero());
		return vector_select(is_greater_equal, floored, ceiled);
	}
}

RTM_IMPL_FILE_PRAGMA_POP
