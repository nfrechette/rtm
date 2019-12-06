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
	inline vector4d vector_load(const double* input) RTM_NO_EXCEPT
	{
		return vector_set(input[0], input[1], input[2], input[3]);
	}

	//////////////////////////////////////////////////////////////////////////
	// Loads an unaligned vector1 from memory and leaves the [yzw] components undefined.
	//////////////////////////////////////////////////////////////////////////
	inline vector4d vector_load1(const double* input) RTM_NO_EXCEPT
	{
		return vector_set(input[0]);
	}

	//////////////////////////////////////////////////////////////////////////
	// Loads an unaligned vector2 from memory and leaves the [zw] components undefined.
	//////////////////////////////////////////////////////////////////////////
	inline vector4d vector_load2(const double* input) RTM_NO_EXCEPT
	{
		return vector_set(input[0], input[1], 0.0, 0.0);
	}

	//////////////////////////////////////////////////////////////////////////
	// Loads an unaligned vector3 from memory and leaves the [w] component undefined.
	//////////////////////////////////////////////////////////////////////////
	inline vector4d vector_load3(const double* input) RTM_NO_EXCEPT
	{
		return vector_set(input[0], input[1], input[2], 0.0);
	}

	//////////////////////////////////////////////////////////////////////////
	// Loads an unaligned vector4 from memory.
	//////////////////////////////////////////////////////////////////////////
	inline vector4d vector_load(const float4d* input) RTM_NO_EXCEPT
	{
		return vector_set(input->x, input->y, input->z, input->w);
	}

	//////////////////////////////////////////////////////////////////////////
	// Loads an unaligned vector2 from memory and leaves the [zw] components undefined.
	//////////////////////////////////////////////////////////////////////////
	inline vector4d vector_load2(const float2d* input) RTM_NO_EXCEPT
	{
		return vector_set(input->x, input->y, 0.0, 0.0);
	}

	//////////////////////////////////////////////////////////////////////////
	// Loads an unaligned vector3 from memory and leaves the [w] component undefined.
	//////////////////////////////////////////////////////////////////////////
	inline vector4d vector_load3(const float3d* input) RTM_NO_EXCEPT
	{
		return vector_set(input->x, input->y, input->z, 0.0);
	}

	//////////////////////////////////////////////////////////////////////////
	// Casts a quaternion to a vector4.
	//////////////////////////////////////////////////////////////////////////
	inline vector4d quat_to_vector(const quatd& input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return vector4d{ input.xy, input.zw };
#else
		return vector4d{ input.x, input.y, input.z, input.w };
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Casts a vector4 float32 variant to a float64 variant.
	//////////////////////////////////////////////////////////////////////////
	inline vector4d vector_cast(const vector4f& input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return vector4d{ _mm_cvtps_pd(input), _mm_cvtps_pd(_mm_shuffle_ps(input, input, _MM_SHUFFLE(3, 2, 3, 2))) };
#elif defined(RTM_NEON_INTRINSICS)
		return vector4d{ double(vgetq_lane_f32(input, 0)), double(vgetq_lane_f32(input, 1)), double(vgetq_lane_f32(input, 2)), double(vgetq_lane_f32(input, 3)) };
#else
		return vector4d{ double(input.x), double(input.y), double(input.z), double(input.w) };
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the vector4 [x] component.
	//////////////////////////////////////////////////////////////////////////
	inline double vector_get_x(const vector4d& input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_cvtsd_f64(input.xy);
#else
		return input.x;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the vector4 [y] component.
	//////////////////////////////////////////////////////////////////////////
	inline double vector_get_y(const vector4d& input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_cvtsd_f64(_mm_shuffle_pd(input.xy, input.xy, 1));
#else
		return input.y;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the vector4 [z] component.
	//////////////////////////////////////////////////////////////////////////
	inline double vector_get_z(const vector4d& input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_cvtsd_f64(input.zw);
#else
		return input.z;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the vector4 [w] component.
	//////////////////////////////////////////////////////////////////////////
	inline double vector_get_w(const vector4d& input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_cvtsd_f64(_mm_shuffle_pd(input.zw, input.zw, 1));
#else
		return input.w;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the vector4 desired component.
	//////////////////////////////////////////////////////////////////////////
	template<mix4 component>
	inline double vector_get_component(const vector4d& input) RTM_NO_EXCEPT
	{
		switch (mix4(int(component) % 4))
		{
		case mix4::x: return vector_get_x(input);
		case mix4::y: return vector_get_y(input);
		case mix4::z: return vector_get_z(input);
		case mix4::w: return vector_get_w(input);
		default:
			RTM_ASSERT(false, "Invalid component");
			return 0.0;
		}
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the vector4 desired component.
	//////////////////////////////////////////////////////////////////////////
	inline double vector_get_component(const vector4d& input, mix4 component) RTM_NO_EXCEPT
	{
		switch (mix4(int(component) % 4))
		{
		case mix4::x: return vector_get_x(input);
		case mix4::y: return vector_get_y(input);
		case mix4::z: return vector_get_z(input);
		case mix4::w: return vector_get_w(input);
		default:
			RTM_ASSERT(false, "Invalid component");
			return 0.0;
		}
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the smallest component in the input vector as a scalar.
	//////////////////////////////////////////////////////////////////////////
	constexpr rtm_impl::vector4d_get_min_component vector_get_min_component(const vector4d& input) RTM_NO_EXCEPT
	{
		return rtm_impl::vector4d_get_min_component{ input };
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the largest component in the input vector as a scalar.
	//////////////////////////////////////////////////////////////////////////
	constexpr rtm_impl::vector4d_get_max_component vector_get_max_component(const vector4d& input) RTM_NO_EXCEPT
	{
		return rtm_impl::vector4d_get_max_component{ input };
	}

	//////////////////////////////////////////////////////////////////////////
	// Sets the vector4 [x] component and returns the new value.
	//////////////////////////////////////////////////////////////////////////
	inline vector4d vector_set_x(const vector4d& input, double lane_value) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return vector4d{ _mm_move_sd(input.xy, _mm_set_sd(lane_value)), input.zw };
#else
		return vector4d{ lane_value, input.y, input.z, input.w };
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Sets the vector4 [y] component and returns the new value.
	//////////////////////////////////////////////////////////////////////////
	inline vector4d vector_set_y(const vector4d& input, double lane_value) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return vector4d{ _mm_shuffle_pd(input.xy, _mm_set_sd(lane_value), 0), input.zw };
#else
		return vector4d{ input.x, lane_value, input.z, input.w };
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Sets the vector4 [z] component and returns the new value.
	//////////////////////////////////////////////////////////////////////////
	inline vector4d vector_set_z(const vector4d& input, double lane_value) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return vector4d{ input.xy, _mm_move_sd(input.zw, _mm_set_sd(lane_value)) };
#else
		return vector4d{ input.x, input.y, lane_value, input.w };
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Sets the vector4 [w] component and returns the new value.
	//////////////////////////////////////////////////////////////////////////
	inline vector4d vector_set_w(const vector4d& input, double lane_value) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return vector4d{ input.xy, _mm_shuffle_pd(input.zw, _mm_set_sd(lane_value), 0) };
#else
		return vector4d{ input.x, input.y, input.z, lane_value };
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns a floating point pointer to the vector4 data.
	//////////////////////////////////////////////////////////////////////////
	inline const double* vector_to_pointer(const vector4d& input) RTM_NO_EXCEPT
	{
		return reinterpret_cast<const double*>(&input);
	}

	//////////////////////////////////////////////////////////////////////////
	// Writes a vector4 to unaligned memory.
	//////////////////////////////////////////////////////////////////////////
	inline void vector_store(const vector4d& input, double* output) RTM_NO_EXCEPT
	{
		output[0] = vector_get_x(input);
		output[1] = vector_get_y(input);
		output[2] = vector_get_z(input);
		output[3] = vector_get_w(input);
	}

	//////////////////////////////////////////////////////////////////////////
	// Writes a vector1 to unaligned memory.
	//////////////////////////////////////////////////////////////////////////
	inline void vector_store1(const vector4d& input, double* output) RTM_NO_EXCEPT
	{
		output[0] = vector_get_x(input);
	}

	//////////////////////////////////////////////////////////////////////////
	// Writes a vector2 to unaligned memory.
	//////////////////////////////////////////////////////////////////////////
	inline void vector_store2(const vector4d& input, double* output) RTM_NO_EXCEPT
	{
		output[0] = vector_get_x(input);
		output[1] = vector_get_y(input);
	}

	//////////////////////////////////////////////////////////////////////////
	// Writes a vector3 to unaligned memory.
	//////////////////////////////////////////////////////////////////////////
	inline void vector_store3(const vector4d& input, double* output) RTM_NO_EXCEPT
	{
		output[0] = vector_get_x(input);
		output[1] = vector_get_y(input);
		output[2] = vector_get_z(input);
	}

	//////////////////////////////////////////////////////////////////////////
	// Writes a vector4 to unaligned memory.
	//////////////////////////////////////////////////////////////////////////
	inline void vector_store(const vector4d& input, uint8_t* output) RTM_NO_EXCEPT
	{
		std::memcpy(output, &input, sizeof(vector4d));
	}

	//////////////////////////////////////////////////////////////////////////
	// Writes a vector1 to unaligned memory.
	//////////////////////////////////////////////////////////////////////////
	inline void vector_store1(const vector4d& input, uint8_t* output)
	{
		std::memcpy(output, &input, sizeof(double) * 1);
	}

	//////////////////////////////////////////////////////////////////////////
	// Writes a vector2 to unaligned memory.
	//////////////////////////////////////////////////////////////////////////
	inline void vector_store2(const vector4d& input, uint8_t* output)
	{
		std::memcpy(output, &input, sizeof(double) * 2);
	}

	//////////////////////////////////////////////////////////////////////////
	// Writes a vector3 to unaligned memory.
	//////////////////////////////////////////////////////////////////////////
	inline void vector_store3(const vector4d& input, uint8_t* output)
	{
		std::memcpy(output, &input, sizeof(double) * 3);
	}

	//////////////////////////////////////////////////////////////////////////
	// Writes a vector4 to unaligned memory.
	//////////////////////////////////////////////////////////////////////////
	inline void vector_store(const vector4d& input, float4d* output) RTM_NO_EXCEPT
	{
		output->x = vector_get_x(input);
		output->y = vector_get_y(input);
		output->z = vector_get_z(input);
		output->w = vector_get_w(input);
	}

	//////////////////////////////////////////////////////////////////////////
	// Writes a vector2 to unaligned memory.
	//////////////////////////////////////////////////////////////////////////
	inline void vector_store2(const vector4d& input, float2d* output) RTM_NO_EXCEPT
	{
		output->x = vector_get_x(input);
		output->y = vector_get_y(input);
	}

	//////////////////////////////////////////////////////////////////////////
	// Writes a vector3 to unaligned memory.
	//////////////////////////////////////////////////////////////////////////
	inline void vector_store3(const vector4d& input, float3d* output) RTM_NO_EXCEPT
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
	inline vector4d vector_add(const vector4d& lhs, const vector4d& rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return vector4d{ _mm_add_pd(lhs.xy, rhs.xy), _mm_add_pd(lhs.zw, rhs.zw) };
#else
		return vector_set(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z, lhs.w + rhs.w);
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Per component subtraction of the two inputs: lhs - rhs
	//////////////////////////////////////////////////////////////////////////
	inline vector4d vector_sub(const vector4d& lhs, const vector4d& rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return vector4d{ _mm_sub_pd(lhs.xy, rhs.xy), _mm_sub_pd(lhs.zw, rhs.zw) };
#else
		return vector_set(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z, lhs.w - rhs.w);
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Per component multiplication of the two inputs: lhs * rhs
	//////////////////////////////////////////////////////////////////////////
	inline vector4d vector_mul(const vector4d& lhs, const vector4d& rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return vector4d{ _mm_mul_pd(lhs.xy, rhs.xy), _mm_mul_pd(lhs.zw, rhs.zw) };
#else
		return vector_set(lhs.x * rhs.x, lhs.y * rhs.y, lhs.z * rhs.z, lhs.w * rhs.w);
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Per component multiplication of the vector by a scalar: lhs * rhs
	//////////////////////////////////////////////////////////////////////////
	inline vector4d vector_mul(const vector4d& lhs, double rhs) RTM_NO_EXCEPT
	{
		return vector_mul(lhs, vector_set(rhs));
	}

	//////////////////////////////////////////////////////////////////////////
	// Per component division of the two inputs: lhs / rhs
	//////////////////////////////////////////////////////////////////////////
	inline vector4d vector_div(const vector4d& lhs, const vector4d& rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return vector4d{ _mm_div_pd(lhs.xy, rhs.xy), _mm_div_pd(lhs.zw, rhs.zw) };
#else
		return vector_set(lhs.x / rhs.x, lhs.y / rhs.y, lhs.z / rhs.z, lhs.w / rhs.w);
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Per component maximum of the two inputs: max(lhs, rhs)
	//////////////////////////////////////////////////////////////////////////
	inline vector4d vector_max(const vector4d& lhs, const vector4d& rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return vector4d{ _mm_max_pd(lhs.xy, rhs.xy), _mm_max_pd(lhs.zw, rhs.zw) };
#else
		return vector_set(scalar_max(lhs.x, rhs.x), scalar_max(lhs.y, rhs.y), scalar_max(lhs.z, rhs.z), scalar_max(lhs.w, rhs.w));
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Per component minimum of the two inputs: min(lhs, rhs)
	//////////////////////////////////////////////////////////////////////////
	inline vector4d vector_min(const vector4d& lhs, const vector4d& rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return vector4d{ _mm_min_pd(lhs.xy, rhs.xy), _mm_min_pd(lhs.zw, rhs.zw) };
#else
		return vector_set(scalar_min(lhs.x, rhs.x), scalar_min(lhs.y, rhs.y), scalar_min(lhs.z, rhs.z), scalar_min(lhs.w, rhs.w));
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Per component clamping of an input between a minimum and a maximum value: min(max_value, max(min_value, input))
	//////////////////////////////////////////////////////////////////////////
	inline vector4d vector_clamp(const vector4d& input, const vector4d& min_value, const vector4d& max_value) RTM_NO_EXCEPT
	{
		return vector_min(max_value, vector_max(min_value, input));
	}

	//////////////////////////////////////////////////////////////////////////
	// Per component absolute of the input: abs(input)
	//////////////////////////////////////////////////////////////////////////
	inline vector4d vector_abs(const vector4d& input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		vector4d zero{ _mm_setzero_pd(), _mm_setzero_pd() };
		return vector_max(vector_sub(zero, input), input);
#else
		return vector_set(scalar_abs(input.x), scalar_abs(input.y), scalar_abs(input.z), scalar_abs(input.w));
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Per component negation of the input: -input
	//////////////////////////////////////////////////////////////////////////
	inline vector4d vector_neg(const vector4d& input) RTM_NO_EXCEPT
	{
		return vector_mul(input, -1.0);
	}

	//////////////////////////////////////////////////////////////////////////
	// Per component reciprocal of the input: 1.0 / input
	//////////////////////////////////////////////////////////////////////////
	inline vector4d vector_reciprocal(const vector4d& input) RTM_NO_EXCEPT
	{
		return vector_div(vector_set(1.0), input);
	}

	//////////////////////////////////////////////////////////////////////////
	// Per component returns the smallest integer value not less than the input.
	// vector_ceil([1.8, 1.0, -1.8, -1.0]) = [2.0, 1.0, -1.0, -1.0]
	//////////////////////////////////////////////////////////////////////////
	inline vector4d vector_ceil(const vector4d& input) RTM_NO_EXCEPT
	{
		return vector_set(scalar_ceil(vector_get_x(input)), scalar_ceil(vector_get_y(input)), scalar_ceil(vector_get_z(input)), scalar_ceil(vector_get_w(input)));
	}

	//////////////////////////////////////////////////////////////////////////
	// Per component returns the largest integer value not greater than the input.
	// vector_floor([1.8, 1.0, -1.8, -1.0]) = [1.0, 1.0, -2.0, -1.0]
	//////////////////////////////////////////////////////////////////////////
	inline vector4d vector_floor(const vector4d& input) RTM_NO_EXCEPT
	{
		return vector_set(scalar_floor(vector_get_x(input)), scalar_floor(vector_get_y(input)), scalar_floor(vector_get_z(input)), scalar_floor(vector_get_w(input)));
	}

	//////////////////////////////////////////////////////////////////////////
	// 3D cross product: lhs x rhs
	//////////////////////////////////////////////////////////////////////////
	inline vector4d vector_cross3(const vector4d& lhs, const vector4d& rhs) RTM_NO_EXCEPT
	{
		return vector_set(vector_get_y(lhs) * vector_get_z(rhs) - vector_get_z(lhs) * vector_get_y(rhs),
						  vector_get_z(lhs) * vector_get_x(rhs) - vector_get_x(lhs) * vector_get_z(rhs),
						  vector_get_x(lhs) * vector_get_y(rhs) - vector_get_y(lhs) * vector_get_x(rhs));
	}

	//////////////////////////////////////////////////////////////////////////
	// 4D dot product: lhs . rhs
	//////////////////////////////////////////////////////////////////////////
	inline double vector_dot(const vector4d& lhs, const vector4d& rhs) RTM_NO_EXCEPT
	{
		return (vector_get_x(lhs) * vector_get_x(rhs)) + (vector_get_y(lhs) * vector_get_y(rhs)) + (vector_get_z(lhs) * vector_get_z(rhs)) + (vector_get_w(lhs) * vector_get_w(rhs));
	}

	//////////////////////////////////////////////////////////////////////////
	// 4D dot product: lhs . rhs
	//////////////////////////////////////////////////////////////////////////
	inline scalard vector_dot_as_scalar(const vector4d& lhs, const vector4d& rhs) RTM_NO_EXCEPT
	{
		return scalar_set((vector_get_x(lhs) * vector_get_x(rhs)) + (vector_get_y(lhs) * vector_get_y(rhs)) + (vector_get_z(lhs) * vector_get_z(rhs)) + (vector_get_w(lhs) * vector_get_w(rhs)));
	}

	//////////////////////////////////////////////////////////////////////////
	// 4D dot product replicated in all components: lhs . rhs
	//////////////////////////////////////////////////////////////////////////
	inline vector4d vector_dot_as_vector(const vector4d& lhs, const vector4d& rhs) RTM_NO_EXCEPT
	{
		return vector_set(vector_dot(lhs, rhs));
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
			inline RTM_SIMD_CALL operator double() const RTM_NO_EXCEPT
			{
#if defined(RTM_SSE2_INTRINSICS)
				__m128d x2_y2 = _mm_mul_pd(lhs.xy, rhs.xy);
				__m128d z2_w2 = _mm_mul_pd(lhs.zw, rhs.zw);
				__m128d y2 = _mm_shuffle_pd(x2_y2, x2_y2, 1);
				__m128d x2y2 = _mm_add_sd(x2_y2, y2);
				return _mm_cvtsd_f64(_mm_add_sd(x2y2, z2_w2));
#else
				return (vector_get_x(lhs) * vector_get_x(rhs)) + (vector_get_y(lhs) * vector_get_y(rhs)) + (vector_get_z(lhs) * vector_get_z(rhs));
#endif
			}

#if defined(RTM_SSE2_INTRINSICS)
			inline RTM_SIMD_CALL operator scalard() const RTM_NO_EXCEPT
			{
				__m128d x2_y2 = _mm_mul_pd(lhs.xy, rhs.xy);
				__m128d z2_w2 = _mm_mul_pd(lhs.zw, rhs.zw);
				__m128d y2 = _mm_shuffle_pd(x2_y2, x2_y2, 1);
				__m128d x2y2 = _mm_add_sd(x2_y2, y2);
				return _mm_add_sd(x2y2, z2_w2);
			}
#endif

			vector4d lhs;
			vector4d rhs;
		};
	}

	//////////////////////////////////////////////////////////////////////////
	// 3D dot product: lhs . rhs
	//////////////////////////////////////////////////////////////////////////
	constexpr rtm_impl::vector4d_vector_dot3 vector_dot3(const vector4d& lhs, const vector4d& rhs) RTM_NO_EXCEPT
	{
		return rtm_impl::vector4d_vector_dot3{ lhs, rhs };
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the squared length/norm of the vector4.
	//////////////////////////////////////////////////////////////////////////
	inline double vector_length_squared(const vector4d& input) RTM_NO_EXCEPT
	{
		return vector_dot(input, input);
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the squared length/norm of the vector3.
	//////////////////////////////////////////////////////////////////////////
	constexpr rtm_impl::vector4d_vector_dot3 vector_length_squared3(const vector4d& input) RTM_NO_EXCEPT
	{
		return rtm_impl::vector4d_vector_dot3{ input, input };
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the length/norm of the vector4.
	//////////////////////////////////////////////////////////////////////////
	inline double vector_length(const vector4d& input) RTM_NO_EXCEPT
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
		struct vector4d_vector_length3
		{
			inline RTM_SIMD_CALL operator double() const RTM_NO_EXCEPT
			{
				const scalard len_sq = vector_length_squared3(input);
				return scalar_cast(scalar_sqrt(len_sq));
			}

#if defined(RTM_SSE2_INTRINSICS)
			inline RTM_SIMD_CALL operator scalard() const RTM_NO_EXCEPT
			{
				const scalard len_sq = vector_length_squared3(input);
				return scalar_sqrt(len_sq);
			}
#endif

			vector4d input;
		};
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the length/norm of the vector3.
	//////////////////////////////////////////////////////////////////////////
	constexpr rtm_impl::vector4d_vector_length3 vector_length3(const vector4d& input) RTM_NO_EXCEPT
	{
		return rtm_impl::vector4d_vector_length3{ input };
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the reciprocal length/norm of the vector4.
	//////////////////////////////////////////////////////////////////////////
	inline double vector_length_reciprocal(const vector4d& input) RTM_NO_EXCEPT
	{
		return 1.0 / vector_length(input);
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the reciprocal length/norm of the vector3.
	//////////////////////////////////////////////////////////////////////////
	inline double vector_length_reciprocal3(const vector4d& input) RTM_NO_EXCEPT
	{
		return 1.0 / vector_length3(input);
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the distance between two 3D points.
	//////////////////////////////////////////////////////////////////////////
	inline rtm_impl::vector4d_vector_length3 vector_distance3(const vector4d& lhs, const vector4d& rhs) RTM_NO_EXCEPT
	{
		const vector4d difference = vector_sub(lhs, rhs);
		return rtm_impl::vector4d_vector_length3{ difference };
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns a normalized vector3.
	// If the length of the input is below the supplied threshold, the
	// fall back value is returned instead.
	//////////////////////////////////////////////////////////////////////////
	inline vector4d vector_normalize3(const vector4d& input, const vector4d& fallback, double threshold = 1.0E-8) RTM_NO_EXCEPT
	{
		// Reciprocal is more accurate to normalize with
		const double len_sq = vector_length_squared3(input);
		if (len_sq >= threshold)
			return vector_mul(input, scalar_sqrt_reciprocal(len_sq));
		else
			return fallback;
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns per component the fractional part of the input.
	//////////////////////////////////////////////////////////////////////////
	inline vector4d vector_fraction(const vector4d& input) RTM_NO_EXCEPT
	{
		return vector_set(scalar_fraction(vector_get_x(input)), scalar_fraction(vector_get_y(input)), scalar_fraction(vector_get_z(input)), scalar_fraction(vector_get_w(input)));
	}

	//////////////////////////////////////////////////////////////////////////
	// Per component multiplication/addition of the three inputs: v2 + (v0 * v1)
	//////////////////////////////////////////////////////////////////////////
	inline vector4d vector_mul_add(const vector4d& v0, const vector4d& v1, const vector4d& v2) RTM_NO_EXCEPT
	{
		return vector_add(vector_mul(v0, v1), v2);
	}

	//////////////////////////////////////////////////////////////////////////
	// Per component multiplication/addition of the three inputs: v2 + (v0 * s1)
	//////////////////////////////////////////////////////////////////////////
	inline vector4d vector_mul_add(const vector4d& v0, double s1, const vector4d& v2) RTM_NO_EXCEPT
	{
		return vector_add(vector_mul(v0, s1), v2);
	}

	//////////////////////////////////////////////////////////////////////////
	// Per component negative multiplication/subtraction of the three inputs: -((v0 * v1) - v2)
	// This is mathematically equivalent to: v2 - (v0 * v1)
	//////////////////////////////////////////////////////////////////////////
	inline vector4d vector_neg_mul_sub(const vector4d& v0, const vector4d& v1, const vector4d& v2) RTM_NO_EXCEPT
	{
		return vector_sub(v2, vector_mul(v0, v1));
	}

	//////////////////////////////////////////////////////////////////////////
	// Per component negative multiplication/subtraction of the three inputs: -((v0 * s1) - v2)
	// This is mathematically equivalent to: v2 - (v0 * s1)
	//////////////////////////////////////////////////////////////////////////
	inline vector4d vector_neg_mul_sub(const vector4d& v0, double s1, const vector4d& v2) RTM_NO_EXCEPT
	{
		return vector_sub(v2, vector_mul(v0, s1));
	}

	//////////////////////////////////////////////////////////////////////////
	// Per component linear interpolation of the two inputs at the specified alpha.
	//////////////////////////////////////////////////////////////////////////
	inline vector4d vector_lerp(const vector4d& start, const vector4d& end, double alpha) RTM_NO_EXCEPT
	{
		return vector_mul_add(vector_sub(end, start), alpha, start);
	}



	//////////////////////////////////////////////////////////////////////////
	// Comparisons and masking
	//////////////////////////////////////////////////////////////////////////


	//////////////////////////////////////////////////////////////////////////
	// Returns per component ~0 if less than, otherwise 0: lhs < rhs ? ~0 : 0
	//////////////////////////////////////////////////////////////////////////
	inline mask4q vector_less_than(const vector4d& lhs, const vector4d& rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_lt_pd = _mm_cmplt_pd(lhs.xy, rhs.xy);
		__m128d zw_lt_pd = _mm_cmplt_pd(lhs.zw, rhs.zw);
		return mask4q{xy_lt_pd, zw_lt_pd};
#else
		return mask4q{rtm_impl::get_mask_value(lhs.x < rhs.x), rtm_impl::get_mask_value(lhs.y < rhs.y), rtm_impl::get_mask_value(lhs.z < rhs.z), rtm_impl::get_mask_value(lhs.w < rhs.w)};
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns per component ~0 if less equal, otherwise 0: lhs <= rhs ? ~0 : 0
	//////////////////////////////////////////////////////////////////////////
	inline mask4q vector_less_equal(const vector4d& lhs, const vector4d& rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_lt_pd = _mm_cmple_pd(lhs.xy, rhs.xy);
		__m128d zw_lt_pd = _mm_cmple_pd(lhs.zw, rhs.zw);
		return mask4q{ xy_lt_pd, zw_lt_pd };
#else
		return mask4q{ rtm_impl::get_mask_value(lhs.x <= rhs.x), rtm_impl::get_mask_value(lhs.y <= rhs.y), rtm_impl::get_mask_value(lhs.z <= rhs.z), rtm_impl::get_mask_value(lhs.w <= rhs.w) };
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns per component ~0 if greater equal, otherwise 0: lhs >= rhs ? ~0 : 0
	//////////////////////////////////////////////////////////////////////////
	inline mask4q vector_greater_equal(const vector4d& lhs, const vector4d& rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_ge_pd = _mm_cmpge_pd(lhs.xy, rhs.xy);
		__m128d zw_ge_pd = _mm_cmpge_pd(lhs.zw, rhs.zw);
		return mask4q{ xy_ge_pd, zw_ge_pd };
#else
		return mask4q{ rtm_impl::get_mask_value(lhs.x >= rhs.x), rtm_impl::get_mask_value(lhs.y >= rhs.y), rtm_impl::get_mask_value(lhs.z >= rhs.z), rtm_impl::get_mask_value(lhs.w >= rhs.w) };
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all 4 components are less than, otherwise false: all(lhs < rhs)
	//////////////////////////////////////////////////////////////////////////
	inline bool vector_all_less_than(const vector4d& lhs, const vector4d& rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_lt_pd = _mm_cmplt_pd(lhs.xy, rhs.xy);
		__m128d zw_lt_pd = _mm_cmplt_pd(lhs.zw, rhs.zw);
		return (_mm_movemask_pd(xy_lt_pd) & _mm_movemask_pd(zw_lt_pd)) == 3;
#else
		return lhs.x < rhs.x && lhs.y < rhs.y && lhs.z < rhs.z && lhs.w < rhs.w;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all 2 components are less than, otherwise false: all(lhs < rhs)
	//////////////////////////////////////////////////////////////////////////
	inline bool vector_all_less_than2(const vector4d& lhs, const vector4d& rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_lt_pd = _mm_cmplt_pd(lhs.xy, rhs.xy);
		return _mm_movemask_pd(xy_lt_pd) == 3;
#else
		return lhs.x < rhs.x && lhs.y < rhs.y;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all 3 components are less than, otherwise false: all(lhs < rhs)
	//////////////////////////////////////////////////////////////////////////
	inline bool vector_all_less_than3(const vector4d& lhs, const vector4d& rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_lt_pd = _mm_cmplt_pd(lhs.xy, rhs.xy);
		__m128d zw_lt_pd = _mm_cmplt_pd(lhs.zw, rhs.zw);
		return _mm_movemask_pd(xy_lt_pd) == 3 && (_mm_movemask_pd(zw_lt_pd) & 1) == 1;
#else
		return lhs.x < rhs.x && lhs.y < rhs.y && lhs.z < rhs.z;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if any 4 components are less than, otherwise false: any(lhs < rhs)
	//////////////////////////////////////////////////////////////////////////
	inline bool vector_any_less_than(const vector4d& lhs, const vector4d& rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_lt_pd = _mm_cmplt_pd(lhs.xy, rhs.xy);
		__m128d zw_lt_pd = _mm_cmplt_pd(lhs.zw, rhs.zw);
		return (_mm_movemask_pd(xy_lt_pd) | _mm_movemask_pd(zw_lt_pd)) != 0;
#else
		return lhs.x < rhs.x || lhs.y < rhs.y || lhs.z < rhs.z || lhs.w < rhs.w;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if any 2 components are less than, otherwise false: any(lhs < rhs)
	//////////////////////////////////////////////////////////////////////////
	inline bool vector_any_less_than2(const vector4d& lhs, const vector4d& rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_lt_pd = _mm_cmplt_pd(lhs.xy, rhs.xy);
		return _mm_movemask_pd(xy_lt_pd) != 0;
#else
		return lhs.x < rhs.x || lhs.y < rhs.y;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if any 3 components are less than, otherwise false: any(lhs < rhs)
	//////////////////////////////////////////////////////////////////////////
	inline bool vector_any_less_than3(const vector4d& lhs, const vector4d& rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_lt_pd = _mm_cmplt_pd(lhs.xy, rhs.xy);
		__m128d zw_lt_pd = _mm_cmplt_pd(lhs.zw, rhs.zw);
		return _mm_movemask_pd(xy_lt_pd) != 0 || (_mm_movemask_pd(zw_lt_pd) & 0x1) != 0;
#else
		return lhs.x < rhs.x || lhs.y < rhs.y || lhs.z < rhs.z;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all 4 components are less equal, otherwise false: all(lhs <= rhs)
	//////////////////////////////////////////////////////////////////////////
	inline bool vector_all_less_equal(const vector4d& lhs, const vector4d& rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_le_pd = _mm_cmple_pd(lhs.xy, rhs.xy);
		__m128d zw_le_pd = _mm_cmple_pd(lhs.zw, rhs.zw);
		return (_mm_movemask_pd(xy_le_pd) & _mm_movemask_pd(zw_le_pd)) == 3;
#else
		return lhs.x <= rhs.x && lhs.y <= rhs.y && lhs.z <= rhs.z && lhs.w <= rhs.w;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all 2 components are less equal, otherwise false: all(lhs <= rhs)
	//////////////////////////////////////////////////////////////////////////
	inline bool vector_all_less_equal2(const vector4d& lhs, const vector4d& rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_le_pd = _mm_cmple_pd(lhs.xy, rhs.xy);
		return _mm_movemask_pd(xy_le_pd) == 3;
#else
		return lhs.x <= rhs.x && lhs.y <= rhs.y;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all 3 components are less equal, otherwise false: all(lhs <= rhs)
	//////////////////////////////////////////////////////////////////////////
	inline bool vector_all_less_equal3(const vector4d& lhs, const vector4d& rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_le_pd = _mm_cmple_pd(lhs.xy, rhs.xy);
		__m128d zw_le_pd = _mm_cmple_pd(lhs.zw, rhs.zw);
		return _mm_movemask_pd(xy_le_pd) == 3 && (_mm_movemask_pd(zw_le_pd) & 1) != 0;
#else
		return lhs.x <= rhs.x && lhs.y <= rhs.y && lhs.z <= rhs.z;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if any 4 components are less equal, otherwise false: any(lhs <= rhs)
	//////////////////////////////////////////////////////////////////////////
	inline bool vector_any_less_equal(const vector4d& lhs, const vector4d& rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_le_pd = _mm_cmple_pd(lhs.xy, rhs.xy);
		__m128d zw_le_pd = _mm_cmple_pd(lhs.zw, rhs.zw);
		return (_mm_movemask_pd(xy_le_pd) | _mm_movemask_pd(zw_le_pd)) != 0;
#else
		return lhs.x <= rhs.x || lhs.y <= rhs.y || lhs.z <= rhs.z || lhs.w <= rhs.w;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if any 2 components are less equal, otherwise false: any(lhs <= rhs)
	//////////////////////////////////////////////////////////////////////////
	inline bool vector_any_less_equal2(const vector4d& lhs, const vector4d& rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_le_pd = _mm_cmple_pd(lhs.xy, rhs.xy);
		return _mm_movemask_pd(xy_le_pd) != 0;
#else
		return lhs.x <= rhs.x || lhs.y <= rhs.y;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if any 3 components are less equal, otherwise false: any(lhs <= rhs)
	//////////////////////////////////////////////////////////////////////////
	inline bool vector_any_less_equal3(const vector4d& lhs, const vector4d& rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_le_pd = _mm_cmple_pd(lhs.xy, rhs.xy);
		__m128d zw_le_pd = _mm_cmple_pd(lhs.zw, rhs.zw);
		return _mm_movemask_pd(xy_le_pd) != 0 || (_mm_movemask_pd(zw_le_pd) & 1) != 0;
#else
		return lhs.x <= rhs.x || lhs.y <= rhs.y || lhs.z <= rhs.z;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all 4 components are greater equal, otherwise false: all(lhs >= rhs)
	//////////////////////////////////////////////////////////////////////////
	inline bool vector_all_greater_equal(const vector4d& lhs, const vector4d& rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_ge_pd = _mm_cmpge_pd(lhs.xy, rhs.xy);
		__m128d zw_ge_pd = _mm_cmpge_pd(lhs.zw, rhs.zw);
		return (_mm_movemask_pd(xy_ge_pd) & _mm_movemask_pd(zw_ge_pd)) == 3;
#else
		return lhs.x >= rhs.x && lhs.y >= rhs.y && lhs.z >= rhs.z && lhs.w >= rhs.w;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all 2 components are greater equal, otherwise false: all(lhs >= rhs)
	//////////////////////////////////////////////////////////////////////////
	inline bool vector_all_greater_equal2(const vector4d& lhs, const vector4d& rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_ge_pd = _mm_cmpge_pd(lhs.xy, rhs.xy);
		return _mm_movemask_pd(xy_ge_pd) == 3;
#else
		return lhs.x >= rhs.x && lhs.y >= rhs.y;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all 3 components are greater equal, otherwise false: all(lhs >= rhs)
	//////////////////////////////////////////////////////////////////////////
	inline bool vector_all_greater_equal3(const vector4d& lhs, const vector4d& rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_ge_pd = _mm_cmpge_pd(lhs.xy, rhs.xy);
		__m128d zw_ge_pd = _mm_cmpge_pd(lhs.zw, rhs.zw);
		return _mm_movemask_pd(xy_ge_pd) == 3 && (_mm_movemask_pd(zw_ge_pd) & 1) != 0;
#else
		return lhs.x >= rhs.x && lhs.y >= rhs.y && lhs.z >= rhs.z;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if any 4 components are greater equal, otherwise false: any(lhs >= rhs)
	//////////////////////////////////////////////////////////////////////////
	inline bool vector_any_greater_equal(const vector4d& lhs, const vector4d& rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_ge_pd = _mm_cmpge_pd(lhs.xy, rhs.xy);
		__m128d zw_ge_pd = _mm_cmpge_pd(lhs.zw, rhs.zw);
		return (_mm_movemask_pd(xy_ge_pd) | _mm_movemask_pd(zw_ge_pd)) != 0;
#else
		return lhs.x >= rhs.x || lhs.y >= rhs.y || lhs.z >= rhs.z || lhs.w >= rhs.w;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if any 2 components are greater equal, otherwise false: any(lhs >= rhs)
	//////////////////////////////////////////////////////////////////////////
	inline bool vector_any_greater_equal2(const vector4d& lhs, const vector4d& rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_ge_pd = _mm_cmpge_pd(lhs.xy, rhs.xy);
		return _mm_movemask_pd(xy_ge_pd) != 0;
#else
		return lhs.x >= rhs.x || lhs.y >= rhs.y;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if any 3 components are greater equal, otherwise false: any(lhs >= rhs)
	//////////////////////////////////////////////////////////////////////////
	inline bool vector_any_greater_equal3(const vector4d& lhs, const vector4d& rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_ge_pd = _mm_cmpge_pd(lhs.xy, rhs.xy);
		__m128d zw_ge_pd = _mm_cmpge_pd(lhs.zw, rhs.zw);
		return _mm_movemask_pd(xy_ge_pd) != 0 || (_mm_movemask_pd(zw_ge_pd) & 1) != 0;
#else
		return lhs.x >= rhs.x || lhs.y >= rhs.y || lhs.z >= rhs.z;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all 4 components are near equal, otherwise false: all(abs(lhs - rhs) < threshold)
	//////////////////////////////////////////////////////////////////////////
	inline bool vector_all_near_equal(const vector4d& lhs, const vector4d& rhs, double threshold = 0.00001) RTM_NO_EXCEPT
	{
		return vector_all_less_equal(vector_abs(vector_sub(lhs, rhs)), vector_set(threshold));
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all 2 components are near equal, otherwise false: all(abs(lhs - rhs) < threshold)
	//////////////////////////////////////////////////////////////////////////
	inline bool vector_all_near_equal2(const vector4d& lhs, const vector4d& rhs, double threshold = 0.00001) RTM_NO_EXCEPT
	{
		return vector_all_less_equal2(vector_abs(vector_sub(lhs, rhs)), vector_set(threshold));
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all 3 components are near equal, otherwise false: all(abs(lhs - rhs) < threshold)
	//////////////////////////////////////////////////////////////////////////
	inline bool vector_all_near_equal3(const vector4d& lhs, const vector4d& rhs, double threshold = 0.00001) RTM_NO_EXCEPT
	{
		return vector_all_less_equal3(vector_abs(vector_sub(lhs, rhs)), vector_set(threshold));
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if any 4 components are near equal, otherwise false: any(abs(lhs - rhs) < threshold)
	//////////////////////////////////////////////////////////////////////////
	inline bool vector_any_near_equal(const vector4d& lhs, const vector4d& rhs, double threshold = 0.00001) RTM_NO_EXCEPT
	{
		return vector_any_less_equal(vector_abs(vector_sub(lhs, rhs)), vector_set(threshold));
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if any 2 components are near equal, otherwise false: any(abs(lhs - rhs) < threshold)
	//////////////////////////////////////////////////////////////////////////
	inline bool vector_any_near_equal2(const vector4d& lhs, const vector4d& rhs, double threshold = 0.00001) RTM_NO_EXCEPT
	{
		return vector_any_less_equal2(vector_abs(vector_sub(lhs, rhs)), vector_set(threshold));
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if any 3 components are near equal, otherwise false: any(abs(lhs - rhs) < threshold)
	//////////////////////////////////////////////////////////////////////////
	inline bool vector_any_near_equal3(const vector4d& lhs, const vector4d& rhs, double threshold = 0.00001) RTM_NO_EXCEPT
	{
		return vector_any_less_equal3(vector_abs(vector_sub(lhs, rhs)), vector_set(threshold));
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all 4 components are finite (not NaN/Inf), otherwise false: all(finite(input))
	//////////////////////////////////////////////////////////////////////////
	inline bool vector_is_finite(const vector4d& input) RTM_NO_EXCEPT
	{
		return scalar_is_finite(vector_get_x(input)) && scalar_is_finite(vector_get_y(input)) && scalar_is_finite(vector_get_z(input)) && scalar_is_finite(vector_get_w(input));
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all 2 components are finite (not NaN/Inf), otherwise false: all(finite(input))
	//////////////////////////////////////////////////////////////////////////
	inline bool vector_is_finite2(const vector4d& input) RTM_NO_EXCEPT
	{
		return scalar_is_finite(vector_get_x(input)) && scalar_is_finite(vector_get_y(input));
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all 3 components are finite (not NaN/Inf), otherwise false: all(finite(input))
	//////////////////////////////////////////////////////////////////////////
	inline bool vector_is_finite3(const vector4d& input) RTM_NO_EXCEPT
	{
		return scalar_is_finite(vector_get_x(input)) && scalar_is_finite(vector_get_y(input)) && scalar_is_finite(vector_get_z(input));
	}



	//////////////////////////////////////////////////////////////////////////
	// Swizzling, permutations, and mixing
	//////////////////////////////////////////////////////////////////////////


	//////////////////////////////////////////////////////////////////////////
	// Per component selection depending on the mask: mask != 0 ? if_true : if_false
	//////////////////////////////////////////////////////////////////////////
	inline vector4d vector_select(const mask4q& mask, const vector4d& if_true, const vector4d& if_false) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy = _mm_or_pd(_mm_andnot_pd(mask.xy, if_false.xy), _mm_and_pd(if_true.xy, mask.xy));
		__m128d zw = _mm_or_pd(_mm_andnot_pd(mask.zw, if_false.zw), _mm_and_pd(if_true.zw, mask.zw));
		return vector4d{ xy, zw };
#else
		return vector4d{ rtm_impl::select(mask.x, if_true.x, if_false.x), rtm_impl::select(mask.y, if_true.y, if_false.y), rtm_impl::select(mask.z, if_true.z, if_false.z), rtm_impl::select(mask.w, if_true.w, if_false.w) };
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Mixes two inputs and returns the desired components.
	// [xyzw] indexes into the first input while [abcd] indexes in the second.
	//////////////////////////////////////////////////////////////////////////
	template<mix4 comp0, mix4 comp1, mix4 comp2, mix4 comp3>
	inline vector4d vector_mix(const vector4d& input0, const vector4d& input1) RTM_NO_EXCEPT
	{
		// Slow code path, not yet optimized or not using intrinsics
		const double x = rtm_impl::is_mix_xyzw(comp0) ? vector_get_component<comp0>(input0) : vector_get_component<comp0>(input1);
		const double y = rtm_impl::is_mix_xyzw(comp1) ? vector_get_component<comp1>(input0) : vector_get_component<comp1>(input1);
		const double z = rtm_impl::is_mix_xyzw(comp2) ? vector_get_component<comp2>(input0) : vector_get_component<comp2>(input1);
		const double w = rtm_impl::is_mix_xyzw(comp3) ? vector_get_component<comp3>(input0) : vector_get_component<comp3>(input1);
		return vector_set(x, y, z, w);
	}

	//////////////////////////////////////////////////////////////////////////
	// Replicates the [x] component in all components.
	//////////////////////////////////////////////////////////////////////////
	inline vector4d vector_dup_x(const vector4d& input) RTM_NO_EXCEPT { return vector_mix<mix4::x, mix4::x, mix4::x, mix4::x>(input, input); }

	//////////////////////////////////////////////////////////////////////////
	// Replicates the [y] component in all components.
	//////////////////////////////////////////////////////////////////////////
	inline vector4d vector_dup_y(const vector4d& input) RTM_NO_EXCEPT { return vector_mix<mix4::y, mix4::y, mix4::y, mix4::y>(input, input); }

	//////////////////////////////////////////////////////////////////////////
	// Replicates the [z] component in all components.
	//////////////////////////////////////////////////////////////////////////
	inline vector4d vector_dup_z(const vector4d& input) RTM_NO_EXCEPT { return vector_mix<mix4::z, mix4::z, mix4::z, mix4::z>(input, input); }

	//////////////////////////////////////////////////////////////////////////
	// Replicates the [w] component in all components.
	//////////////////////////////////////////////////////////////////////////
	inline vector4d vector_dup_w(const vector4d& input) RTM_NO_EXCEPT { return vector_mix<mix4::w, mix4::w, mix4::w, mix4::w>(input, input); }


	//////////////////////////////////////////////////////////////////////////
	// Miscellaneous
	//////////////////////////////////////////////////////////////////////////


	//////////////////////////////////////////////////////////////////////////
	// Returns per component the sign of the input vector: input >= 0.0 ? 1.0 : -1.0
	//////////////////////////////////////////////////////////////////////////
	inline vector4d vector_sign(const vector4d& input) RTM_NO_EXCEPT
	{
		const mask4q mask = vector_greater_equal(input, vector_zero());
		return vector_select(mask, vector_set(1.0), vector_set(-1.0));
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns per component the rounded input using a symmetric algorithm.
	// symmetric_round(1.5) = 2.0
	// symmetric_round(1.2) = 1.0
	// symmetric_round(-1.5) = -2.0
	// symmetric_round(-1.2) = -1.0
	//////////////////////////////////////////////////////////////////////////
	inline vector4d vector_symmetric_round(const vector4d& input) RTM_NO_EXCEPT
	{
		const vector4d half = vector_set(0.5);
		const vector4d floored = vector_floor(vector_add(input, half));
		const vector4d ceiled = vector_ceil(vector_sub(input, half));
		const mask4q is_greater_equal = vector_greater_equal(input, vector_zero());
		return vector_select(is_greater_equal, floored, ceiled);
	}
}

RTM_IMPL_FILE_PRAGMA_POP
