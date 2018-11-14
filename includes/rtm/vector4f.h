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

#include "rtm/error.h"
#include "rtm/math.h"
#include "rtm/scalar_32.h"
#include "rtm/impl/memory_utils.h"

namespace rtm
{
	//////////////////////////////////////////////////////////////////////////
	// Setters, getters, and casts

	inline vector4f RTM_SIMD_CALL vector_set(float x, float y, float z, float w)
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

	inline vector4f RTM_SIMD_CALL vector_set(float x, float y, float z)
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

	inline vector4f RTM_SIMD_CALL vector_set(float xyzw)
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_set_ps1(xyzw);
#elif defined(RTM_NEON_INTRINSICS)
		return vdupq_n_f32(xyzw);
#else
		return vector4f{ xyzw, xyzw, xyzw, xyzw };
#endif
	}

	inline vector4f RTM_SIMD_CALL vector_unaligned_load(const float* input)
	{
		RTM_ASSERT(rtm_impl::is_aligned(input), "Invalid alignment");
		return vector_set(input[0], input[1], input[2], input[3]);
	}

	inline vector4f RTM_SIMD_CALL vector_unaligned_load3(const float* input)
	{
		RTM_ASSERT(rtm_impl::is_aligned(input), "Invalid alignment");
		return vector_set(input[0], input[1], input[2], 0.0f);
	}

	inline vector4f RTM_SIMD_CALL vector_unaligned_load_32(const uint8_t* input)
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_loadu_ps((const float*)input);
#elif defined(RTM_NEON_INTRINSICS)
		return vreinterpretq_f32_u8(vld1q_u8(input));
#else
		vector4f result;
		memcpy(&result, input, sizeof(vector4f));
		return result;
#endif
	}

	inline vector4f RTM_SIMD_CALL vector_unaligned_load3_32(const uint8_t* input)
	{
		float input_f[3];
		memcpy(&input_f[0], input, sizeof(float) * 3);
		return vector_set(input_f[0], input_f[1], input_f[2], 0.0f);
	}

	inline vector4f RTM_SIMD_CALL vector_zero_32()
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_setzero_ps();
#else
		return vector_set(0.0f);
#endif
	}

	inline vector4f RTM_SIMD_CALL quat_to_vector(quatf_arg0 input)
	{
#if defined(RTM_SSE2_INTRINSICS) || defined(RTM_NEON_INTRINSICS)
		return input;
#else
		return vector4f{ input.x, input.y, input.z, input.w };
#endif
	}

	inline vector4f RTM_SIMD_CALL vector_cast(const vector4d& input)
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_shuffle_ps(_mm_cvtpd_ps(input.xy), _mm_cvtpd_ps(input.zw), _MM_SHUFFLE(1, 0, 1, 0));
#else
		return vector4f{ float(input.x), float(input.y), float(input.z), float(input.w) };
#endif
	}

	inline float RTM_SIMD_CALL vector_get_x(vector4f_arg0 input)
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_cvtss_f32(input);
#elif defined(RTM_NEON_INTRINSICS)
		return vgetq_lane_f32(input, 0);
#else
		return input.x;
#endif
	}

	inline float RTM_SIMD_CALL vector_get_y(vector4f_arg0 input)
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_cvtss_f32(_mm_shuffle_ps(input, input, _MM_SHUFFLE(1, 1, 1, 1)));
#elif defined(RTM_NEON_INTRINSICS)
		return vgetq_lane_f32(input, 1);
#else
		return input.y;
#endif
	}

	inline float RTM_SIMD_CALL vector_get_z(vector4f_arg0 input)
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_cvtss_f32(_mm_shuffle_ps(input, input, _MM_SHUFFLE(2, 2, 2, 2)));
#elif defined(RTM_NEON_INTRINSICS)
		return vgetq_lane_f32(input, 2);
#else
		return input.z;
#endif
	}

	inline float RTM_SIMD_CALL vector_get_w(vector4f_arg0 input)
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_cvtss_f32(_mm_shuffle_ps(input, input, _MM_SHUFFLE(3, 3, 3, 3)));
#elif defined(RTM_NEON_INTRINSICS)
		return vgetq_lane_f32(input, 3);
#else
		return input.w;
#endif
	}

	template<VectorMix component_index>
	inline float RTM_SIMD_CALL vector_get_component(vector4f_arg0 input)
	{
		switch (component_index)
		{
		case VectorMix::A:
		case VectorMix::X: return vector_get_x(input);
		case VectorMix::B:
		case VectorMix::Y: return vector_get_y(input);
		case VectorMix::C:
		case VectorMix::Z: return vector_get_z(input);
		case VectorMix::D:
		case VectorMix::W: return vector_get_w(input);
		default:
			RTM_ASSERT(false, "Invalid component index");
			return 0.0f;
		}
	}

	inline float RTM_SIMD_CALL vector_get_component(vector4f_arg0 input, VectorMix component_index)
	{
		switch (component_index)
		{
		case VectorMix::A:
		case VectorMix::X: return vector_get_x(input);
		case VectorMix::B:
		case VectorMix::Y: return vector_get_y(input);
		case VectorMix::C:
		case VectorMix::Z: return vector_get_z(input);
		case VectorMix::D:
		case VectorMix::W: return vector_get_w(input);
		default:
			RTM_ASSERT(false, "Invalid component index");
			return 0.0f;
		}
	}

	inline const float* RTM_SIMD_CALL vector_as_float_ptr(const vector4f& input)
	{
		return reinterpret_cast<const float*>(&input);
	}

	inline void RTM_SIMD_CALL vector_unaligned_write(vector4f_arg0 input, float* output)
	{
		RTM_ASSERT(rtm_impl::is_aligned(output), "Invalid alignment");
		output[0] = vector_get_x(input);
		output[1] = vector_get_y(input);
		output[2] = vector_get_z(input);
		output[3] = vector_get_w(input);
	}

	inline void RTM_SIMD_CALL vector_unaligned_write3(vector4f_arg0 input, float* output)
	{
		RTM_ASSERT(rtm_impl::is_aligned(output), "Invalid alignment");
		output[0] = vector_get_x(input);
		output[1] = vector_get_y(input);
		output[2] = vector_get_z(input);
	}

	inline void RTM_SIMD_CALL vector_unaligned_write(vector4f_arg0 input, uint8_t* output)
	{
		memcpy(output, &input, sizeof(vector4f));
	}

	inline void RTM_SIMD_CALL vector_unaligned_write3(vector4f_arg0 input, uint8_t* output)
	{
		memcpy(output, &input, sizeof(float) * 3);
	}

	//////////////////////////////////////////////////////////////////////////
	// Arithmetic

	inline vector4f RTM_SIMD_CALL vector_add(vector4f_arg0 lhs, vector4f_arg1 rhs)
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_add_ps(lhs, rhs);
#elif defined(RTM_NEON_INTRINSICS)
		return vaddq_f32(lhs, rhs);
#else
		return vector_set(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z, lhs.w + rhs.w);
#endif
	}

	inline vector4f RTM_SIMD_CALL vector_sub(vector4f_arg0 lhs, vector4f_arg1 rhs)
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_sub_ps(lhs, rhs);
#elif defined(RTM_NEON_INTRINSICS)
		return vsubq_f32(lhs, rhs);
#else
		return vector_set(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z, lhs.w - rhs.w);
#endif
	}

	inline vector4f RTM_SIMD_CALL vector_mul(vector4f_arg0 lhs, vector4f_arg1 rhs)
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_mul_ps(lhs, rhs);
#elif defined(RTM_NEON_INTRINSICS)
		return vmulq_f32(lhs, rhs);
#else
		return vector_set(lhs.x * rhs.x, lhs.y * rhs.y, lhs.z * rhs.z, lhs.w * rhs.w);
#endif
	}

	inline vector4f RTM_SIMD_CALL vector_mul(vector4f_arg0 lhs, float rhs)
	{
#if defined(RTM_NEON_INTRINSICS)
		return vmulq_n_f32(lhs, rhs);
#else
		return vector_mul(lhs, vector_set(rhs));
#endif
	}

	inline vector4f RTM_SIMD_CALL vector_div(vector4f_arg0 lhs, vector4f_arg1 rhs)
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

	inline vector4f RTM_SIMD_CALL vector_max(vector4f_arg0 lhs, vector4f_arg1 rhs)
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_max_ps(lhs, rhs);
#elif defined(RTM_NEON_INTRINSICS)
		return vmaxq_f32(lhs, rhs);
#else
		return vector_set(scalar_max(lhs.x, rhs.x), scalar_max(lhs.y, rhs.y), scalar_max(lhs.z, rhs.z), scalar_max(lhs.w, rhs.w));
#endif
	}

	inline vector4f RTM_SIMD_CALL vector_min(vector4f_arg0 lhs, vector4f_arg1 rhs)
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_min_ps(lhs, rhs);
#elif defined(RTM_NEON_INTRINSICS)
		return vminq_f32(lhs, rhs);
#else
		return vector_set(scalar_min(lhs.x, rhs.x), scalar_min(lhs.y, rhs.y), scalar_min(lhs.z, rhs.z), scalar_min(lhs.w, rhs.w));
#endif
	}

	inline vector4f RTM_SIMD_CALL vector_abs(vector4f_arg0 input)
	{
#if defined(RTM_SSE2_INTRINSICS)
		return vector_max(vector_sub(_mm_setzero_ps(), input), input);
#elif defined(RTM_NEON_INTRINSICS)
		return vabsq_f32(input);
#else
		return vector_set(scalar_abs(input.x), scalar_abs(input.y), scalar_abs(input.z), scalar_abs(input.w));
#endif
	}

	inline vector4f RTM_SIMD_CALL vector_neg(vector4f_arg0 input)
	{
#if defined(RTM_NEON_INTRINSICS)
		return vnegq_f32(input);
#else
		return vector_mul(input, -1.0f);
#endif
	}

	inline vector4f RTM_SIMD_CALL vector_reciprocal(vector4f_arg0 input)
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
		return vector_div(vector_set(1.0f), input);
#endif
	}

	inline vector4f RTM_SIMD_CALL vector_cross3(vector4f_arg0 lhs, vector4f_arg1 rhs)
	{
		return vector_set(vector_get_y(lhs) * vector_get_z(rhs) - vector_get_z(lhs) * vector_get_y(rhs),
						  vector_get_z(lhs) * vector_get_x(rhs) - vector_get_x(lhs) * vector_get_z(rhs),
						  vector_get_x(lhs) * vector_get_y(rhs) - vector_get_y(lhs) * vector_get_x(rhs));
	}

	inline float RTM_SIMD_CALL vector_dot(vector4f_arg0 lhs, vector4f_arg1 rhs)
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

	inline vector4f RTM_SIMD_CALL vector_vdot(vector4f_arg0 lhs, vector4f_arg1 rhs)
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

	inline float RTM_SIMD_CALL vector_dot3(vector4f_arg0 lhs, vector4f_arg1 rhs)
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

	inline float RTM_SIMD_CALL vector_length_squared(vector4f_arg0 input)
	{
		return vector_dot(input, input);
	}

	inline float RTM_SIMD_CALL vector_length_squared3(vector4f_arg0 input)
	{
		return vector_dot3(input, input);
	}

	inline float RTM_SIMD_CALL vector_length(vector4f_arg0 input)
	{
		return scalar_sqrt(vector_length_squared(input));
	}

	inline float RTM_SIMD_CALL vector_length3(vector4f_arg0 input)
	{
		return scalar_sqrt(vector_length_squared3(input));
	}

	inline float RTM_SIMD_CALL vector_length_reciprocal(vector4f_arg0 input)
	{
		return scalar_sqrt_reciprocal(vector_length_squared(input));
	}

	inline float RTM_SIMD_CALL vector_length_reciprocal3(vector4f_arg0 input)
	{
		return scalar_sqrt_reciprocal(vector_length_squared3(input));
	}

	inline float RTM_SIMD_CALL vector_distance3(vector4f_arg0 lhs, vector4f_arg1 rhs)
	{
		return vector_length3(vector_sub(rhs, lhs));
	}

	inline vector4f RTM_SIMD_CALL vector_normalize3(vector4f_arg0 input, float threshold = 1.0e-8f)
	{
		// Reciprocal is more accurate to normalize with
		const float len_sq = vector_length_squared3(input);
		if (len_sq >= threshold)
			return vector_mul(input, scalar_sqrt_reciprocal(len_sq));
		else
			return input;
	}

	inline vector4f RTM_SIMD_CALL vector_fraction(vector4f_arg0 input)
	{
		return vector_set(scalar_fraction(vector_get_x(input)), scalar_fraction(vector_get_y(input)), scalar_fraction(vector_get_z(input)), scalar_fraction(vector_get_w(input)));
	}

	// output = (input * scale) + offset
	inline vector4f RTM_SIMD_CALL vector_mul_add(vector4f_arg0 input, vector4f_arg1 scale, vector4f_arg2 offset)
	{
#if defined(RTM_NEON_INTRINSICS)
		return vmlaq_f32(offset, input, scale);
#else
		return vector_add(vector_mul(input, scale), offset);
#endif
	}

	inline vector4f RTM_SIMD_CALL vector_mul_add(vector4f_arg0 input, float scale, vector4f_arg2 offset)
	{
#if defined(RTM_NEON_INTRINSICS)
		return vmlaq_n_f32(offset, input, scale);
#else
		return vector_add(vector_mul(input, scale), offset);
#endif
	}

	// output = offset - (input * scale)
	inline vector4f RTM_SIMD_CALL vector_neg_mul_sub(vector4f_arg0 input, vector4f_arg1 scale, vector4f_arg2 offset)
	{
#if defined(RTM_NEON_INTRINSICS)
		return vmlsq_f32(offset, input, scale);
#else
		return vector_sub(offset, vector_mul(input, scale));
#endif
	}

	inline vector4f RTM_SIMD_CALL vector_lerp(vector4f_arg0 start, vector4f_arg1 end, float alpha)
	{
		return vector_mul_add(vector_sub(end, start), alpha, start);
	}

	//////////////////////////////////////////////////////////////////////////
	// Comparisons and masking

	inline vector4f RTM_SIMD_CALL vector_less_than(vector4f_arg0 lhs, vector4f_arg1 rhs)
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_cmplt_ps(lhs, rhs);
#elif defined(RTM_NEON_INTRINSICS)
		return vcltq_f32(lhs, rhs);
#else
		return vector4f{ rtm_impl::get_mask_value(lhs.x < rhs.x), rtm_impl::get_mask_value(lhs.y < rhs.y), rtm_impl::get_mask_value(lhs.z < rhs.z), rtm_impl::get_mask_value(lhs.w < rhs.w) };
#endif
	}

	inline vector4f RTM_SIMD_CALL vector_greater_equal(vector4f_arg0 lhs, vector4f_arg1 rhs)
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_cmpge_ps(lhs, rhs);
#elif defined(RTM_NEON_INTRINSICS)
		return vcgeq_f32(lhs, rhs);
#else
		return vector4f{ rtm_impl::get_mask_value(lhs.x >= rhs.x), rtm_impl::get_mask_value(lhs.y >= rhs.y), rtm_impl::get_mask_value(lhs.z >= rhs.z), rtm_impl::get_mask_value(lhs.w >= rhs.w) };
#endif
	}

	inline bool RTM_SIMD_CALL vector_all_less_than(vector4f_arg0 lhs, vector4f_arg1 rhs)
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_movemask_ps(_mm_cmplt_ps(lhs, rhs)) == 0xF;
#elif defined(RTM_NEON_INTRINSICS)
		uint32x4_t mask = vcltq_f32(lhs, rhs);
		uint8x8x2_t mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15 = vzip_u8(vget_low_u8(mask), vget_high_u8(mask));
		uint16x4x2_t mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15 = vzip_u16(mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[0], mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[1]);
		return vget_lane_u32(mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15.val[0], 0) == 0xFFFFFFFFu;
#else
		return lhs.x < rhs.x && lhs.y < rhs.y && lhs.z < rhs.z && lhs.w < rhs.w;
#endif
	}

	inline bool RTM_SIMD_CALL vector_all_less_than3(vector4f_arg0 lhs, vector4f_arg1 rhs)
	{
#if defined(RTM_SSE2_INTRINSICS)
		return (_mm_movemask_ps(_mm_cmplt_ps(lhs, rhs)) & 0x7) == 0x7;
#elif defined(RTM_NEON_INTRINSICS)
		uint32x4_t mask = vcltq_f32(lhs, rhs);
		uint8x8x2_t mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15 = vzip_u8(vget_low_u8(mask), vget_high_u8(mask));
		uint16x4x2_t mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15 = vzip_u16(mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[0], mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[1]);
		return (vget_lane_u32(mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15.val[0], 0) & 0x00FFFFFFu) == 0x00FFFFFFu;
#else
		return lhs.x < rhs.x && lhs.y < rhs.y && lhs.z < rhs.z;
#endif
	}

	inline bool RTM_SIMD_CALL vector_any_less_than(vector4f_arg0 lhs, vector4f_arg1 rhs)
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

	inline bool RTM_SIMD_CALL vector_any_less_than3(vector4f_arg0 lhs, vector4f_arg1 rhs)
	{
#if defined(RTM_SSE2_INTRINSICS)
		return (_mm_movemask_ps(_mm_cmplt_ps(lhs, rhs)) & 0x7) != 0;
#elif defined(RTM_NEON_INTRINSICS)
		uint32x4_t mask = vcltq_f32(lhs, rhs);
		uint8x8x2_t mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15 = vzip_u8(vget_low_u8(mask), vget_high_u8(mask));
		uint16x4x2_t mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15 = vzip_u16(mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[0], mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[1]);
		return (vget_lane_u32(mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15.val[0], 0) & 0x00FFFFFFu) != 0;
#else
		return lhs.x < rhs.x || lhs.y < rhs.y || lhs.z < rhs.z;
#endif
	}

	inline bool RTM_SIMD_CALL vector_all_less_equal(vector4f_arg0 lhs, vector4f_arg1 rhs)
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_movemask_ps(_mm_cmple_ps(lhs, rhs)) == 0xF;
#elif defined(RTM_NEON_INTRINSICS)
		uint32x4_t mask = vcleq_f32(lhs, rhs);
		uint8x8x2_t mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15 = vzip_u8(vget_low_u8(mask), vget_high_u8(mask));
		uint16x4x2_t mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15 = vzip_u16(mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[0], mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[1]);
		return vget_lane_u32(mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15.val[0], 0) == 0xFFFFFFFFu;
#else
		return lhs.x <= rhs.x && lhs.y <= rhs.y && lhs.z <= rhs.z && lhs.w <= rhs.w;
#endif
	}

	inline bool RTM_SIMD_CALL vector_all_less_equal3(vector4f_arg0 lhs, vector4f_arg1 rhs)
	{
#if defined(RTM_SSE2_INTRINSICS)
		return (_mm_movemask_ps(_mm_cmple_ps(lhs, rhs)) & 0x7) == 0x7;
#elif defined(RTM_NEON_INTRINSICS)
		uint32x4_t mask = vcleq_f32(lhs, rhs);
		uint8x8x2_t mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15 = vzip_u8(vget_low_u8(mask), vget_high_u8(mask));
		uint16x4x2_t mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15 = vzip_u16(mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[0], mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[1]);
		return (vget_lane_u32(mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15.val[0], 0) & 0x00FFFFFFu) == 0x00FFFFFFu;
#else
		return lhs.x <= rhs.x && lhs.y <= rhs.y && lhs.z <= rhs.z;
#endif
	}

	inline bool RTM_SIMD_CALL vector_any_less_equal(vector4f_arg0 lhs, vector4f_arg1 rhs)
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

	inline bool RTM_SIMD_CALL vector_any_less_equal3(vector4f_arg0 lhs, vector4f_arg1 rhs)
	{
#if defined(RTM_SSE2_INTRINSICS)
		return (_mm_movemask_ps(_mm_cmple_ps(lhs, rhs)) & 0x7) != 0;
#elif defined(RTM_NEON_INTRINSICS)
		uint32x4_t mask = vcleq_f32(lhs, rhs);
		uint8x8x2_t mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15 = vzip_u8(vget_low_u8(mask), vget_high_u8(mask));
		uint16x4x2_t mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15 = vzip_u16(mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[0], mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[1]);
		return (vget_lane_u32(mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15.val[0], 0) & 0x00FFFFFFu) != 0;
#else
		return lhs.x <= rhs.x || lhs.y <= rhs.y || lhs.z <= rhs.z;
#endif
	}

	inline bool RTM_SIMD_CALL vector_all_greater_equal(vector4f_arg0 lhs, vector4f_arg1 rhs)
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_movemask_ps(_mm_cmpge_ps(lhs, rhs)) == 0xF;
#elif defined(RTM_NEON_INTRINSICS)
		uint32x4_t mask = vcgeq_f32(lhs, rhs);
		uint8x8x2_t mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15 = vzip_u8(vget_low_u8(mask), vget_high_u8(mask));
		uint16x4x2_t mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15 = vzip_u16(mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[0], mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[1]);
		return vget_lane_u32(mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15.val[0], 0) == 0xFFFFFFFFu;
#else
		return lhs.x >= rhs.x && lhs.y >= rhs.y && lhs.z >= rhs.z && lhs.w >= rhs.w;
#endif
	}

	inline bool RTM_SIMD_CALL vector_all_greater_equal3(vector4f_arg0 lhs, vector4f_arg1 rhs)
	{
#if defined(RTM_SSE2_INTRINSICS)
		return (_mm_movemask_ps(_mm_cmpge_ps(lhs, rhs)) & 0x7) == 0x7;
#elif defined(RTM_NEON_INTRINSICS)
		uint32x4_t mask = vcgeq_f32(lhs, rhs);
		uint8x8x2_t mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15 = vzip_u8(vget_low_u8(mask), vget_high_u8(mask));
		uint16x4x2_t mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15 = vzip_u16(mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[0], mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[1]);
		return (vget_lane_u32(mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15.val[0], 0) & 0x00FFFFFFu) == 0x00FFFFFFu;
#else
		return lhs.x >= rhs.x && lhs.y >= rhs.y && lhs.z >= rhs.z;
#endif
	}

	inline bool RTM_SIMD_CALL vector_any_greater_equal(vector4f_arg0 lhs, vector4f_arg1 rhs)
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

	inline bool RTM_SIMD_CALL vector_any_greater_equal3(vector4f_arg0 lhs, vector4f_arg1 rhs)
	{
#if defined(RTM_SSE2_INTRINSICS)
		return (_mm_movemask_ps(_mm_cmpge_ps(lhs, rhs)) & 0x7) != 0;
#elif defined(RTM_NEON_INTRINSICS)
		uint32x4_t mask = vcgeq_f32(lhs, rhs);
		uint8x8x2_t mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15 = vzip_u8(vget_low_u8(mask), vget_high_u8(mask));
		uint16x4x2_t mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15 = vzip_u16(mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[0], mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[1]);
		return (vget_lane_u32(mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15.val[0], 0) & 0x00FFFFFFu) != 0;
#else
		return lhs.x >= rhs.x || lhs.y >= rhs.y || lhs.z >= rhs.z;
#endif
	}

	inline bool RTM_SIMD_CALL vector_all_near_equal(vector4f_arg0 lhs, vector4f_arg1 rhs, float threshold = 0.00001f)
	{
		return vector_all_less_equal(vector_abs(vector_sub(lhs, rhs)), vector_set(threshold));
	}

	inline bool RTM_SIMD_CALL vector_all_near_equal3(vector4f_arg0 lhs, vector4f_arg1 rhs, float threshold = 0.00001f)
	{
		return vector_all_less_equal3(vector_abs(vector_sub(lhs, rhs)), vector_set(threshold));
	}

	inline bool RTM_SIMD_CALL vector_any_near_equal(vector4f_arg0 lhs, vector4f_arg1 rhs, float threshold = 0.00001f)
	{
		return vector_any_less_equal(vector_abs(vector_sub(lhs, rhs)), vector_set(threshold));
	}

	inline bool RTM_SIMD_CALL vector_any_near_equal3(vector4f_arg0 lhs, vector4f_arg1 rhs, float threshold = 0.00001f)
	{
		return vector_any_less_equal3(vector_abs(vector_sub(lhs, rhs)), vector_set(threshold));
	}

	inline bool RTM_SIMD_CALL vector_is_finite(vector4f_arg0 input)
	{
		return scalar_is_finite(vector_get_x(input)) && scalar_is_finite(vector_get_y(input)) && scalar_is_finite(vector_get_z(input)) && scalar_is_finite(vector_get_w(input));
	}

	inline bool RTM_SIMD_CALL vector_is_finite3(vector4f_arg0 input)
	{
		return scalar_is_finite(vector_get_x(input)) && scalar_is_finite(vector_get_y(input)) && scalar_is_finite(vector_get_z(input));
	}

	//////////////////////////////////////////////////////////////////////////
	// Swizzling, permutations, and mixing

	inline vector4f RTM_SIMD_CALL vector_blend(vector4f_arg0 mask, vector4f_arg1 if_true, vector4f_arg2 if_false)
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_or_ps(_mm_andnot_ps(mask, if_false), _mm_and_ps(if_true, mask));
#elif defined(RTM_NEON_INTRINSICS)
		return vbslq_f32(mask, if_true, if_false);
#else
		return vector4f{ rtm_impl::select(mask.x, if_true.x, if_false.x), rtm_impl::select(mask.y, if_true.y, if_false.y), rtm_impl::select(mask.z, if_true.z, if_false.z), rtm_impl::select(mask.w, if_true.w, if_false.w) };
#endif
	}

	template<VectorMix comp0, VectorMix comp1, VectorMix comp2, VectorMix comp3>
	inline vector4f RTM_SIMD_CALL vector_mix(vector4f_arg0 input0, vector4f_arg1 input1)
	{
		if (rtm_impl::is_vector_mix_arg_xyzw(comp0) && rtm_impl::is_vector_mix_arg_xyzw(comp1) && rtm_impl::is_vector_mix_arg_xyzw(comp2) && rtm_impl::is_vector_mix_arg_xyzw(comp3))
		{
			// All four components come from input 0
#if defined(RTM_SSE2_INTRINSICS)
			return _mm_shuffle_ps(input0, input0, _MM_SHUFFLE(RTM_GET_VECTOR_MIX_COMPONENT_INDEX(comp3), RTM_GET_VECTOR_MIX_COMPONENT_INDEX(comp2), RTM_GET_VECTOR_MIX_COMPONENT_INDEX(comp1), RTM_GET_VECTOR_MIX_COMPONENT_INDEX(comp0)));
#else
			return vector_set(vector_get_component(input0, comp0), vector_get_component(input0, comp1), vector_get_component(input0, comp2), vector_get_component(input0, comp3));
#endif
		}

		if (rtm_impl::is_vector_mix_arg_abcd(comp0) && rtm_impl::is_vector_mix_arg_abcd(comp1) && rtm_impl::is_vector_mix_arg_abcd(comp2) && rtm_impl::is_vector_mix_arg_abcd(comp3))
		{
			// All four components come from input 1
#if defined(RTM_SSE2_INTRINSICS)
			return _mm_shuffle_ps(input1, input1, _MM_SHUFFLE(RTM_GET_VECTOR_MIX_COMPONENT_INDEX(comp3), RTM_GET_VECTOR_MIX_COMPONENT_INDEX(comp2), RTM_GET_VECTOR_MIX_COMPONENT_INDEX(comp1), RTM_GET_VECTOR_MIX_COMPONENT_INDEX(comp0)));
#else
			return vector_set(vector_get_component(input1, comp0), vector_get_component(input1, comp1), vector_get_component(input1, comp2), vector_get_component(input1, comp3));
#endif
		}

		if (rtm_impl::static_condition<(comp0 == VectorMix::X || comp0 == VectorMix::Y) && (comp1 == VectorMix::X || comp1 == VectorMix::Y) && (comp2 == VectorMix::A || comp2 == VectorMix::B) && (comp3 == VectorMix::A && comp3 == VectorMix::B)>::test())
		{
			// First two components come from input 0, second two come from input 1
#if defined(RTM_SSE2_INTRINSICS)
			return _mm_shuffle_ps(input0, input1, _MM_SHUFFLE(RTM_GET_VECTOR_MIX_COMPONENT_INDEX(comp3), RTM_GET_VECTOR_MIX_COMPONENT_INDEX(comp2), RTM_GET_VECTOR_MIX_COMPONENT_INDEX(comp1), RTM_GET_VECTOR_MIX_COMPONENT_INDEX(comp0)));
#else
			return vector_set(vector_get_component(input0, comp0), vector_get_component(input0, comp1), vector_get_component(input1, comp2), vector_get_component(input1, comp3));
#endif
		}

		if (rtm_impl::static_condition<(comp0 == VectorMix::A || comp0 == VectorMix::B) && (comp1 == VectorMix::A && comp1 == VectorMix::B) && (comp2 == VectorMix::X || comp2 == VectorMix::Y) && (comp3 == VectorMix::X || comp3 == VectorMix::Y)>::test())
		{
			// First two components come from input 1, second two come from input 0
#if defined(RTM_SSE2_INTRINSICS)
			return _mm_shuffle_ps(input1, input0, _MM_SHUFFLE(RTM_GET_VECTOR_MIX_COMPONENT_INDEX(comp3), RTM_GET_VECTOR_MIX_COMPONENT_INDEX(comp2), RTM_GET_VECTOR_MIX_COMPONENT_INDEX(comp1), RTM_GET_VECTOR_MIX_COMPONENT_INDEX(comp0)));
#else
			return vector_set(vector_get_component(input1, comp0), vector_get_component(input1, comp1), vector_get_component(input0, comp2), vector_get_component(input0, comp3));
#endif
		}

		if (rtm_impl::static_condition<comp0 == VectorMix::X && comp1 == VectorMix::A && comp2 == VectorMix::Y && comp3 == VectorMix::B>::test())
		{
			// Low words from both inputs are interleaved
#if defined(RTM_SSE2_INTRINSICS)
			return _mm_unpacklo_ps(input0, input1);
#else
			return vector_set(vector_get_component(input0, comp0), vector_get_component(input1, comp1), vector_get_component(input0, comp2), vector_get_component(input1, comp3));
#endif
		}

		if (rtm_impl::static_condition<comp0 == VectorMix::A && comp1 == VectorMix::X && comp2 == VectorMix::B && comp3 == VectorMix::Y>::test())
		{
			// Low words from both inputs are interleaved
#if defined(RTM_SSE2_INTRINSICS)
			return _mm_unpacklo_ps(input1, input0);
#else
			return vector_set(vector_get_component(input1, comp0), vector_get_component(input0, comp1), vector_get_component(input1, comp2), vector_get_component(input0, comp3));
#endif
		}

		if (rtm_impl::static_condition<comp0 == VectorMix::Z && comp1 == VectorMix::C && comp2 == VectorMix::W && comp3 == VectorMix::D>::test())
		{
			// High words from both inputs are interleaved
#if defined(RTM_SSE2_INTRINSICS)
			return _mm_unpackhi_ps(input0, input1);
#else
			return vector_set(vector_get_component(input0, comp0), vector_get_component(input1, comp1), vector_get_component(input0, comp2), vector_get_component(input1, comp3));
#endif
		}

		if (rtm_impl::static_condition<comp0 == VectorMix::C && comp1 == VectorMix::Z && comp2 == VectorMix::D && comp3 == VectorMix::W>::test())
		{
			// High words from both inputs are interleaved
#if defined(RTM_SSE2_INTRINSICS)
			return _mm_unpackhi_ps(input1, input0);
#else
			return vector_set(vector_get_component(input1, comp0), vector_get_component(input0, comp1), vector_get_component(input1, comp2), vector_get_component(input0, comp3));
#endif
		}

		// Slow code path, not yet optimized
		//RTM_ASSERT(false, "vector_mix permutation not handled");
		const float x = rtm_impl::is_vector_mix_arg_xyzw(comp0) ? vector_get_component<comp0>(input0) : vector_get_component<comp0>(input1);
		const float y = rtm_impl::is_vector_mix_arg_xyzw(comp1) ? vector_get_component<comp1>(input0) : vector_get_component<comp1>(input1);
		const float z = rtm_impl::is_vector_mix_arg_xyzw(comp2) ? vector_get_component<comp2>(input0) : vector_get_component<comp2>(input1);
		const float w = rtm_impl::is_vector_mix_arg_xyzw(comp3) ? vector_get_component<comp3>(input0) : vector_get_component<comp3>(input1);
		return vector_set(x, y, z, w);
	}

	inline vector4f RTM_SIMD_CALL vector_mix_xxxx(vector4f_arg0 input) { return vector_mix<VectorMix::X, VectorMix::X, VectorMix::X, VectorMix::X>(input, input); }
	inline vector4f RTM_SIMD_CALL vector_mix_yyyy(vector4f_arg0 input) { return vector_mix<VectorMix::Y, VectorMix::Y, VectorMix::Y, VectorMix::Y>(input, input); }
	inline vector4f RTM_SIMD_CALL vector_mix_zzzz(vector4f_arg0 input) { return vector_mix<VectorMix::Z, VectorMix::Z, VectorMix::Z, VectorMix::Z>(input, input); }
	inline vector4f RTM_SIMD_CALL vector_mix_wwww(vector4f_arg0 input) { return vector_mix<VectorMix::W, VectorMix::W, VectorMix::W, VectorMix::W>(input, input); }

	inline vector4f RTM_SIMD_CALL vector_mix_xxyy(vector4f_arg0 input) { return vector_mix<VectorMix::X, VectorMix::X, VectorMix::Y, VectorMix::Y>(input, input); }
	inline vector4f RTM_SIMD_CALL vector_mix_xzyw(vector4f_arg0 input) { return vector_mix<VectorMix::X, VectorMix::Z, VectorMix::Y, VectorMix::W>(input, input); }
	inline vector4f RTM_SIMD_CALL vector_mix_yzxy(vector4f_arg0 input) { return vector_mix<VectorMix::Y, VectorMix::Z, VectorMix::X, VectorMix::Y>(input, input); }
	inline vector4f RTM_SIMD_CALL vector_mix_ywxz(vector4f_arg0 input) { return vector_mix<VectorMix::Y, VectorMix::W, VectorMix::X, VectorMix::Z>(input, input); }
	inline vector4f RTM_SIMD_CALL vector_mix_zxyx(vector4f_arg0 input) { return vector_mix<VectorMix::Z, VectorMix::X, VectorMix::Y, VectorMix::X>(input, input); }
	inline vector4f RTM_SIMD_CALL vector_mix_zwyz(vector4f_arg0 input) { return vector_mix<VectorMix::Z, VectorMix::W, VectorMix::Y, VectorMix::Z>(input, input); }
	inline vector4f RTM_SIMD_CALL vector_mix_zwzw(vector4f_arg0 input) { return vector_mix<VectorMix::Z, VectorMix::W, VectorMix::Z, VectorMix::W>(input, input); }
	inline vector4f RTM_SIMD_CALL vector_mix_wxwx(vector4f_arg0 input) { return vector_mix<VectorMix::W, VectorMix::X, VectorMix::W, VectorMix::X>(input, input); }
	inline vector4f RTM_SIMD_CALL vector_mix_wzwy(vector4f_arg0 input) { return vector_mix<VectorMix::W, VectorMix::Z, VectorMix::W, VectorMix::Y>(input, input); }

	inline vector4f RTM_SIMD_CALL vector_mix_xyab(vector4f_arg0 input0, vector4f_arg1 input1) { return vector_mix<VectorMix::X, VectorMix::Y, VectorMix::A, VectorMix::B>(input0, input1); }
	inline vector4f RTM_SIMD_CALL vector_mix_xzac(vector4f_arg0 input0, vector4f_arg1 input1) { return vector_mix<VectorMix::X, VectorMix::Z, VectorMix::A, VectorMix::C>(input0, input1); }
	inline vector4f RTM_SIMD_CALL vector_mix_xbxb(vector4f_arg0 input0, vector4f_arg1 input1) { return vector_mix<VectorMix::X, VectorMix::B, VectorMix::X, VectorMix::B>(input0, input1); }
	inline vector4f RTM_SIMD_CALL vector_mix_xbzd(vector4f_arg0 input0, vector4f_arg1 input1) { return vector_mix<VectorMix::X, VectorMix::B, VectorMix::Z, VectorMix::D>(input0, input1); }
	inline vector4f RTM_SIMD_CALL vector_mix_ywbd(vector4f_arg0 input0, vector4f_arg1 input1) { return vector_mix<VectorMix::Y, VectorMix::W, VectorMix::B, VectorMix::D>(input0, input1); }
	inline vector4f RTM_SIMD_CALL vector_mix_zyax(vector4f_arg0 input0, vector4f_arg1 input1) { return vector_mix<VectorMix::Z, VectorMix::Y, VectorMix::A, VectorMix::X>(input0, input1); }
	inline vector4f RTM_SIMD_CALL vector_mix_zycx(vector4f_arg0 input0, vector4f_arg1 input1) { return vector_mix<VectorMix::Z, VectorMix::Y, VectorMix::C, VectorMix::X>(input0, input1); }
	inline vector4f RTM_SIMD_CALL vector_mix_zwcd(vector4f_arg0 input0, vector4f_arg1 input1) { return vector_mix<VectorMix::Z, VectorMix::W, VectorMix::C, VectorMix::D>(input0, input1); }
	inline vector4f RTM_SIMD_CALL vector_mix_zbaz(vector4f_arg0 input0, vector4f_arg1 input1) { return vector_mix<VectorMix::Z, VectorMix::B, VectorMix::A, VectorMix::Z>(input0, input1); }
	inline vector4f RTM_SIMD_CALL vector_mix_zdcz(vector4f_arg0 input0, vector4f_arg1 input1) { return vector_mix<VectorMix::Z, VectorMix::D, VectorMix::C, VectorMix::Z>(input0, input1); }
	inline vector4f RTM_SIMD_CALL vector_mix_wxya(vector4f_arg0 input0, vector4f_arg1 input1) { return vector_mix<VectorMix::W, VectorMix::X, VectorMix::Y, VectorMix::A>(input0, input1); }
	inline vector4f RTM_SIMD_CALL vector_mix_wxyc(vector4f_arg0 input0, vector4f_arg1 input1) { return vector_mix<VectorMix::W, VectorMix::X, VectorMix::Y, VectorMix::C>(input0, input1); }
	inline vector4f RTM_SIMD_CALL vector_mix_wbyz(vector4f_arg0 input0, vector4f_arg1 input1) { return vector_mix<VectorMix::W, VectorMix::B, VectorMix::Y, VectorMix::Z>(input0, input1); }
	inline vector4f RTM_SIMD_CALL vector_mix_wdyz(vector4f_arg0 input0, vector4f_arg1 input1) { return vector_mix<VectorMix::W, VectorMix::D, VectorMix::Y, VectorMix::Z>(input0, input1); }
	inline vector4f RTM_SIMD_CALL vector_mix_bxwa(vector4f_arg0 input0, vector4f_arg1 input1) { return vector_mix<VectorMix::B, VectorMix::X, VectorMix::W, VectorMix::A>(input0, input1); }
	inline vector4f RTM_SIMD_CALL vector_mix_bywx(vector4f_arg0 input0, vector4f_arg1 input1) { return vector_mix<VectorMix::B, VectorMix::Y, VectorMix::W, VectorMix::X>(input0, input1); }
	inline vector4f RTM_SIMD_CALL vector_mix_dxwc(vector4f_arg0 input0, vector4f_arg1 input1) { return vector_mix<VectorMix::D, VectorMix::X, VectorMix::W, VectorMix::C>(input0, input1); }
	inline vector4f RTM_SIMD_CALL vector_mix_dywx(vector4f_arg0 input0, vector4f_arg1 input1) { return vector_mix<VectorMix::D, VectorMix::Y, VectorMix::W, VectorMix::X>(input0, input1); }

	//////////////////////////////////////////////////////////////////////////
	// Misc

	inline vector4f RTM_SIMD_CALL vector_sign(vector4f_arg0 input)
	{
		vector4f mask = vector_greater_equal(input, vector_zero_32());
		return vector_blend(mask, vector_set(1.0f), vector_set(-1.0f));
	}
}
