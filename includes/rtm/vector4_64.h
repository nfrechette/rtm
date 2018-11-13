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
#include "rtm/scalar_64.h"
#include "rtm/impl/memory_utils.h"

namespace rtm
{
	//////////////////////////////////////////////////////////////////////////
	// Setters, getters, and casts

	inline vector4d vector_set(double x, double y, double z, double w)
	{
#if defined(RTM_SSE2_INTRINSICS)
		return vector4d{ _mm_set_pd(y, x), _mm_set_pd(w, z) };
#else
		return vector4d{ x, y, z, w };
#endif
	}

	inline vector4d vector_set(double x, double y, double z)
	{
#if defined(RTM_SSE2_INTRINSICS)
		return vector4d{ _mm_set_pd(y, x), _mm_set_pd(0.0, z) };
#else
		return vector4d{ x, y, z, 0.0 };
#endif
	}

	inline vector4d vector_set(double xyzw)
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xyzw_pd = _mm_set1_pd(xyzw);
		return vector4d{ xyzw_pd, xyzw_pd };
#else
		return vector4d{ xyzw, xyzw, xyzw, xyzw };
#endif
	}

	inline vector4d vector_unaligned_load(const double* input)
	{
		RTM_ASSERT(rtm_impl::is_aligned_to(input, 4), "Invalid alignment");
		return vector_set(input[0], input[1], input[2], input[3]);
	}

	inline vector4d vector_unaligned_load3(const double* input)
	{
		RTM_ASSERT(rtm_impl::is_aligned_to(input, 4), "Invalid alignment");
		return vector_set(input[0], input[1], input[2], 0.0);
	}

	inline vector4d vector_unaligned_load_64(const uint8_t* input)
	{
		vector4d result;
		memcpy(&result, input, sizeof(vector4d));
		return result;
	}

	inline vector4d vector_unaligned_load3_64(const uint8_t* input)
	{
		double input_f[3];
		memcpy(&input_f[0], input, sizeof(double) * 3);
		return vector_set(input_f[0], input_f[1], input_f[2], 0.0);
	}

	inline vector4d vector_zero_64()
	{
		return vector_set(0.0, 0.0, 0.0, 0.0);
	}

	inline vector4d quat_to_vector(const Quat_64& input)
	{
#if defined(RTM_SSE2_INTRINSICS)
		return vector4d{ input.xy, input.zw };
#else
		return vector4d{ input.x, input.y, input.z, input.w };
#endif
	}

	inline vector4d vector_cast(const vector4f& input)
	{
#if defined(RTM_SSE2_INTRINSICS)
		return vector4d{ _mm_cvtps_pd(input), _mm_cvtps_pd(_mm_shuffle_ps(input, input, _MM_SHUFFLE(3, 2, 3, 2))) };
#elif defined(RTM_NEON_INTRINSICS)
		return vector4d{ double(vgetq_lane_f32(input, 0)), double(vgetq_lane_f32(input, 1)), double(vgetq_lane_f32(input, 2)), double(vgetq_lane_f32(input, 3)) };
#else
		return vector4d{ double(input.x), double(input.y), double(input.z), double(input.w) };
#endif
	}

	inline double vector_get_x(const vector4d& input)
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_cvtsd_f64(input.xy);
#else
		return input.x;
#endif
	}

	inline double vector_get_y(const vector4d& input)
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_cvtsd_f64(_mm_shuffle_pd(input.xy, input.xy, 1));
#else
		return input.y;
#endif
	}

	inline double vector_get_z(const vector4d& input)
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_cvtsd_f64(input.zw);
#else
		return input.z;
#endif
	}

	inline double vector_get_w(const vector4d& input)
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_cvtsd_f64(_mm_shuffle_pd(input.zw, input.zw, 1));
#else
		return input.w;
#endif
	}

	template<VectorMix component_index>
	inline double vector_get_component(const vector4d& input)
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
			return 0.0;
		}
	}

	inline double vector_get_component(const vector4d& input, VectorMix component_index)
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
			return 0.0;
		}
	}

	inline const double* vector_as_double_ptr(const vector4d& input)
	{
		return reinterpret_cast<const double*>(&input);
	}

	inline void vector_unaligned_write(const vector4d& input, double* output)
	{
		RTM_ASSERT(rtm_impl::is_aligned_to(output, 4), "Invalid alignment");
		output[0] = vector_get_x(input);
		output[1] = vector_get_y(input);
		output[2] = vector_get_z(input);
		output[3] = vector_get_w(input);
	}

	inline void vector_unaligned_write3(const vector4d& input, double* output)
	{
		RTM_ASSERT(rtm_impl::is_aligned_to(output, 4), "Invalid alignment");
		output[0] = vector_get_x(input);
		output[1] = vector_get_y(input);
		output[2] = vector_get_z(input);
	}

	inline void vector_unaligned_write(const vector4d& input, uint8_t* output)
	{
		memcpy(output, &input, sizeof(vector4d));
	}

	inline void vector_unaligned_write3(const vector4d& input, uint8_t* output)
	{
		memcpy(output, &input, sizeof(double) * 3);
	}

	//////////////////////////////////////////////////////////////////////////
	// Arithmetic

	inline vector4d vector_add(const vector4d& lhs, const vector4d& rhs)
	{
#if defined(RTM_SSE2_INTRINSICS)
		return vector4d{ _mm_add_pd(lhs.xy, rhs.xy), _mm_add_pd(lhs.zw, rhs.zw) };
#else
		return vector_set(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z, lhs.w + rhs.w);
#endif
	}

	inline vector4d vector_sub(const vector4d& lhs, const vector4d& rhs)
	{
#if defined(RTM_SSE2_INTRINSICS)
		return vector4d{ _mm_sub_pd(lhs.xy, rhs.xy), _mm_sub_pd(lhs.zw, rhs.zw) };
#else
		return vector_set(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z, lhs.w - rhs.w);
#endif
	}

	inline vector4d vector_mul(const vector4d& lhs, const vector4d& rhs)
	{
#if defined(RTM_SSE2_INTRINSICS)
		return vector4d{ _mm_mul_pd(lhs.xy, rhs.xy), _mm_mul_pd(lhs.zw, rhs.zw) };
#else
		return vector_set(lhs.x * rhs.x, lhs.y * rhs.y, lhs.z * rhs.z, lhs.w * rhs.w);
#endif
	}

	inline vector4d vector_mul(const vector4d& lhs, double rhs)
	{
		return vector_mul(lhs, vector_set(rhs));
	}

	inline vector4d vector_div(const vector4d& lhs, const vector4d& rhs)
	{
#if defined(RTM_SSE2_INTRINSICS)
		return vector4d{ _mm_div_pd(lhs.xy, rhs.xy), _mm_div_pd(lhs.zw, rhs.zw) };
#else
		return vector_set(lhs.x / rhs.x, lhs.y / rhs.y, lhs.z / rhs.z, lhs.w / rhs.w);
#endif
	}

	inline vector4d vector_max(const vector4d& lhs, const vector4d& rhs)
	{
#if defined(RTM_SSE2_INTRINSICS)
		return vector4d{ _mm_max_pd(lhs.xy, rhs.xy), _mm_max_pd(lhs.zw, rhs.zw) };
#else
		return vector_set(max(lhs.x, rhs.x), max(lhs.y, rhs.y), max(lhs.z, rhs.z), max(lhs.w, rhs.w));
#endif
	}

	inline vector4d vector_min(const vector4d& lhs, const vector4d& rhs)
	{
#if defined(RTM_SSE2_INTRINSICS)
		return vector4d{ _mm_min_pd(lhs.xy, rhs.xy), _mm_min_pd(lhs.zw, rhs.zw) };
#else
		return vector_set(min(lhs.x, rhs.x), min(lhs.y, rhs.y), min(lhs.z, rhs.z), min(lhs.w, rhs.w));
#endif
	}

	inline vector4d vector_abs(const vector4d& input)
	{
#if defined(RTM_SSE2_INTRINSICS)
		vector4d zero{ _mm_setzero_pd(), _mm_setzero_pd() };
		return vector_max(vector_sub(zero, input), input);
#else
		return vector_set(abs(input.x), abs(input.y), abs(input.z), abs(input.w));
#endif
	}

	inline vector4d vector_neg(const vector4d& input)
	{
		return vector_mul(input, -1.0);
	}

	inline vector4d vector_reciprocal(const vector4d& input)
	{
		return vector_div(vector_set(1.0), input);
	}

	inline vector4d vector_cross3(const vector4d& lhs, const vector4d& rhs)
	{
		return vector_set(vector_get_y(lhs) * vector_get_z(rhs) - vector_get_z(lhs) * vector_get_y(rhs),
						  vector_get_z(lhs) * vector_get_x(rhs) - vector_get_x(lhs) * vector_get_z(rhs),
						  vector_get_x(lhs) * vector_get_y(rhs) - vector_get_y(lhs) * vector_get_x(rhs));
	}

	inline double vector_dot(const vector4d& lhs, const vector4d& rhs)
	{
		return (vector_get_x(lhs) * vector_get_x(rhs)) + (vector_get_y(lhs) * vector_get_y(rhs)) + (vector_get_z(lhs) * vector_get_z(rhs)) + (vector_get_w(lhs) * vector_get_w(rhs));
	}

	inline vector4d vector_vdot(const vector4d& lhs, const vector4d& rhs)
	{
		return vector_set(vector_dot(lhs, rhs));
	}

	inline double vector_dot3(const vector4d& lhs, const vector4d& rhs)
	{
		return (vector_get_x(lhs) * vector_get_x(rhs)) + (vector_get_y(lhs) * vector_get_y(rhs)) + (vector_get_z(lhs) * vector_get_z(rhs));
	}

	inline double vector_length_squared(const vector4d& input)
	{
		return vector_dot(input, input);
	}

	inline double vector_length_squared3(const vector4d& input)
	{
		return vector_dot3(input, input);
	}

	inline double vector_length(const vector4d& input)
	{
		return sqrt(vector_length_squared(input));
	}

	inline double vector_length3(const vector4d& input)
	{
		return sqrt(vector_length_squared3(input));
	}

	inline double vector_length_reciprocal(const vector4d& input)
	{
		return 1.0 / vector_length(input);
	}

	inline double vector_length_reciprocal3(const vector4d& input)
	{
		return 1.0 / vector_length3(input);
	}

	inline double vector_distance3(const vector4d& lhs, const vector4d& rhs)
	{
		return vector_length3(vector_sub(rhs, lhs));
	}

	inline vector4d vector_normalize3(const vector4d& input, double threshold = 1.0e-8)
	{
		// Reciprocal is more accurate to normalize with
		const double len_sq = vector_length_squared3(input);
		if (len_sq >= threshold)
			return vector_mul(input, sqrt_reciprocal(len_sq));
		else
			return input;
	}

	inline vector4d vector_fraction(const vector4d& input)
	{
		return vector_set(fraction(vector_get_x(input)), fraction(vector_get_y(input)), fraction(vector_get_z(input)), fraction(vector_get_w(input)));
	}

	// output = (input * scale) + offset
	inline vector4d vector_mul_add(const vector4d& input, const vector4d& scale, const vector4d& offset)
	{
		return vector_add(vector_mul(input, scale), offset);
	}

	inline vector4d vector_mul_add(const vector4d& input, double scale, const vector4d& offset)
	{
		return vector_add(vector_mul(input, scale), offset);
	}

	// output = offset - (input * scale)
	inline vector4d vector_neg_mul_sub(const vector4d& input, const vector4d& scale, const vector4d& offset)
	{
		return vector_sub(offset, vector_mul(input, scale));
	}

	inline vector4d vector_lerp(const vector4d& start, const vector4d& end, double alpha)
	{
		return vector_mul_add(vector_sub(end, start), alpha, start);
	}

	//////////////////////////////////////////////////////////////////////////
	// Comparisons and masking

	inline vector4d vector_less_than(const vector4d& lhs, const vector4d& rhs)
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_lt_pd = _mm_cmplt_pd(lhs.xy, rhs.xy);
		__m128d zw_lt_pd = _mm_cmplt_pd(lhs.zw, rhs.zw);
		return vector4d{xy_lt_pd, zw_lt_pd};
#else
		return vector4d{rtm_impl::get_mask_value(lhs.x < rhs.x), rtm_impl::get_mask_value(lhs.y < rhs.y), rtm_impl::get_mask_value(lhs.z < rhs.z), rtm_impl::get_mask_value(lhs.w < rhs.w)};
#endif
	}

	inline vector4d vector_greater_equal(const vector4d& lhs, const vector4d& rhs)
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_ge_pd = _mm_cmpge_pd(lhs.xy, rhs.xy);
		__m128d zw_ge_pd = _mm_cmpge_pd(lhs.zw, rhs.zw);
		return vector4d{ xy_ge_pd, zw_ge_pd };
#else
		return vector4d{ rtm_impl::get_mask_value(lhs.x >= rhs.x), rtm_impl::get_mask_value(lhs.y >= rhs.y), rtm_impl::get_mask_value(lhs.z >= rhs.z), rtm_impl::get_mask_value(lhs.w >= rhs.w) };
#endif
	}

	inline bool vector_all_less_than(const vector4d& lhs, const vector4d& rhs)
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_lt_pd = _mm_cmplt_pd(lhs.xy, rhs.xy);
		__m128d zw_lt_pd = _mm_cmplt_pd(lhs.zw, rhs.zw);
		return (_mm_movemask_pd(xy_lt_pd) & _mm_movemask_pd(zw_lt_pd)) == 3;
#else
		return lhs.x < rhs.x && lhs.y < rhs.y && lhs.z < rhs.z && lhs.w < rhs.w;
#endif
	}

	inline bool vector_all_less_than3(const vector4d& lhs, const vector4d& rhs)
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_lt_pd = _mm_cmplt_pd(lhs.xy, rhs.xy);
		__m128d zw_lt_pd = _mm_cmplt_pd(lhs.zw, rhs.zw);
		return _mm_movemask_pd(xy_lt_pd) == 3 && (_mm_movemask_pd(zw_lt_pd) & 1) == 1;
#else
		return lhs.x < rhs.x && lhs.y < rhs.y && lhs.z < rhs.z;
#endif
	}

	inline bool vector_any_less_than(const vector4d& lhs, const vector4d& rhs)
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_lt_pd = _mm_cmplt_pd(lhs.xy, rhs.xy);
		__m128d zw_lt_pd = _mm_cmplt_pd(lhs.zw, rhs.zw);
		return (_mm_movemask_pd(xy_lt_pd) | _mm_movemask_pd(zw_lt_pd)) != 0;
#else
		return lhs.x < rhs.x || lhs.y < rhs.y || lhs.z < rhs.z || lhs.w < rhs.w;
#endif
	}

	inline bool vector_any_less_than3(const vector4d& lhs, const vector4d& rhs)
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_lt_pd = _mm_cmplt_pd(lhs.xy, rhs.xy);
		__m128d zw_lt_pd = _mm_cmplt_pd(lhs.zw, rhs.zw);
		return _mm_movemask_pd(xy_lt_pd) != 0 || (_mm_movemask_pd(zw_lt_pd) & 0x1) != 0;
#else
		return lhs.x < rhs.x || lhs.y < rhs.y || lhs.z < rhs.z;
#endif
	}

	inline bool vector_all_less_equal(const vector4d& lhs, const vector4d& rhs)
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_le_pd = _mm_cmple_pd(lhs.xy, rhs.xy);
		__m128d zw_le_pd = _mm_cmple_pd(lhs.zw, rhs.zw);
		return (_mm_movemask_pd(xy_le_pd) & _mm_movemask_pd(zw_le_pd)) == 3;
#else
		return lhs.x <= rhs.x && lhs.y <= rhs.y && lhs.z <= rhs.z && lhs.w <= rhs.w;
#endif
	}

	inline bool vector_all_less_equal3(const vector4d& lhs, const vector4d& rhs)
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_le_pd = _mm_cmple_pd(lhs.xy, rhs.xy);
		__m128d zw_le_pd = _mm_cmple_pd(lhs.zw, rhs.zw);
		return _mm_movemask_pd(xy_le_pd) == 3 && (_mm_movemask_pd(zw_le_pd) & 1) != 0;
#else
		return lhs.x <= rhs.x && lhs.y <= rhs.y && lhs.z <= rhs.z;
#endif
	}

	inline bool vector_any_less_equal(const vector4d& lhs, const vector4d& rhs)
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_le_pd = _mm_cmple_pd(lhs.xy, rhs.xy);
		__m128d zw_le_pd = _mm_cmple_pd(lhs.zw, rhs.zw);
		return (_mm_movemask_pd(xy_le_pd) | _mm_movemask_pd(zw_le_pd)) != 0;
#else
		return lhs.x <= rhs.x || lhs.y <= rhs.y || lhs.z <= rhs.z || lhs.w <= rhs.w;
#endif
	}

	inline bool vector_any_less_equal3(const vector4d& lhs, const vector4d& rhs)
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_le_pd = _mm_cmple_pd(lhs.xy, rhs.xy);
		__m128d zw_le_pd = _mm_cmple_pd(lhs.zw, rhs.zw);
		return _mm_movemask_pd(xy_le_pd) != 0 || (_mm_movemask_pd(zw_le_pd) & 1) != 0;
#else
		return lhs.x <= rhs.x || lhs.y <= rhs.y || lhs.z <= rhs.z;
#endif
	}

	inline bool vector_all_greater_equal(const vector4d& lhs, const vector4d& rhs)
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_ge_pd = _mm_cmpge_pd(lhs.xy, rhs.xy);
		__m128d zw_ge_pd = _mm_cmpge_pd(lhs.zw, rhs.zw);
		return (_mm_movemask_pd(xy_ge_pd) & _mm_movemask_pd(zw_ge_pd)) == 3;
#else
		return lhs.x >= rhs.x && lhs.y >= rhs.y && lhs.z >= rhs.z && lhs.w >= rhs.w;
#endif
	}

	inline bool vector_all_greater_equal3(const vector4d& lhs, const vector4d& rhs)
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_ge_pd = _mm_cmpge_pd(lhs.xy, rhs.xy);
		__m128d zw_ge_pd = _mm_cmpge_pd(lhs.zw, rhs.zw);
		return _mm_movemask_pd(xy_ge_pd) == 3 && (_mm_movemask_pd(zw_ge_pd) & 1) != 0;
#else
		return lhs.x >= rhs.x && lhs.y >= rhs.y && lhs.z >= rhs.z;
#endif
	}

	inline bool vector_any_greater_equal(const vector4d& lhs, const vector4d& rhs)
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_ge_pd = _mm_cmpge_pd(lhs.xy, rhs.xy);
		__m128d zw_ge_pd = _mm_cmpge_pd(lhs.zw, rhs.zw);
		return (_mm_movemask_pd(xy_ge_pd) | _mm_movemask_pd(zw_ge_pd)) != 0;
#else
		return lhs.x >= rhs.x || lhs.y >= rhs.y || lhs.z >= rhs.z || lhs.w >= rhs.w;
#endif
	}

	inline bool vector_any_greater_equal3(const vector4d& lhs, const vector4d& rhs)
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_ge_pd = _mm_cmpge_pd(lhs.xy, rhs.xy);
		__m128d zw_ge_pd = _mm_cmpge_pd(lhs.zw, rhs.zw);
		return _mm_movemask_pd(xy_ge_pd) != 0 || (_mm_movemask_pd(zw_ge_pd) & 1) != 0;
#else
		return lhs.x >= rhs.x || lhs.y >= rhs.y || lhs.z >= rhs.z;
#endif
	}

	inline bool vector_all_near_equal(const vector4d& lhs, const vector4d& rhs, double threshold = 0.00001)
	{
		return vector_all_less_equal(vector_abs(vector_sub(lhs, rhs)), vector_set(threshold));
	}

	inline bool vector_all_near_equal3(const vector4d& lhs, const vector4d& rhs, double threshold = 0.00001)
	{
		return vector_all_less_equal3(vector_abs(vector_sub(lhs, rhs)), vector_set(threshold));
	}

	inline bool vector_any_near_equal(const vector4d& lhs, const vector4d& rhs, double threshold = 0.00001)
	{
		return vector_any_less_equal(vector_abs(vector_sub(lhs, rhs)), vector_set(threshold));
	}

	inline bool vector_any_near_equal3(const vector4d& lhs, const vector4d& rhs, double threshold = 0.00001)
	{
		return vector_any_less_equal3(vector_abs(vector_sub(lhs, rhs)), vector_set(threshold));
	}

	inline bool vector_is_finite(const vector4d& input)
	{
		return is_finite(vector_get_x(input)) && is_finite(vector_get_y(input)) && is_finite(vector_get_z(input)) && is_finite(vector_get_w(input));
	}

	inline bool vector_is_finite3(const vector4d& input)
	{
		return is_finite(vector_get_x(input)) && is_finite(vector_get_y(input)) && is_finite(vector_get_z(input));
	}

	//////////////////////////////////////////////////////////////////////////
	// Swizzling, permutations, and mixing

	inline vector4d vector_blend(const vector4d& mask, const vector4d& if_true, const vector4d& if_false)
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy = _mm_or_pd(_mm_andnot_pd(mask.xy, if_false.xy), _mm_and_pd(if_true.xy, mask.xy));
		__m128d zw = _mm_or_pd(_mm_andnot_pd(mask.zw, if_false.zw), _mm_and_pd(if_true.zw, mask.zw));
		return vector4d{ xy, zw };
#else
		return vector4d{ rtm_impl::select(mask.x, if_true.x, if_false.x), rtm_impl::select(mask.y, if_true.y, if_false.y), rtm_impl::select(mask.z, if_true.z, if_false.z), rtm_impl::select(mask.w, if_true.w, if_false.w) };
#endif
	}

	template<VectorMix comp0, VectorMix comp1, VectorMix comp2, VectorMix comp3>
	inline vector4d vector_mix(const vector4d& input0, const vector4d& input1)
	{
		if (rtm_impl::is_vector_mix_arg_xyzw(comp0) && rtm_impl::is_vector_mix_arg_xyzw(comp1) && rtm_impl::is_vector_mix_arg_xyzw(comp2) && rtm_impl::is_vector_mix_arg_xyzw(comp3))
		{
			// All four components come from input 0
			return vector_set(vector_get_component(input0, comp0), vector_get_component(input0, comp1), vector_get_component(input0, comp2), vector_get_component(input0, comp3));
		}

		if (rtm_impl::is_vector_mix_arg_abcd(comp0) && rtm_impl::is_vector_mix_arg_abcd(comp1) && rtm_impl::is_vector_mix_arg_abcd(comp2) && rtm_impl::is_vector_mix_arg_abcd(comp3))
		{
			// All four components come from input 1
			return vector_set(vector_get_component(input1, comp0), vector_get_component(input1, comp1), vector_get_component(input1, comp2), vector_get_component(input1, comp3));
		}

		if (rtm_impl::static_condition<(comp0 == VectorMix::X || comp0 == VectorMix::Y) && (comp1 == VectorMix::X || comp1 == VectorMix::Y) && (comp2 == VectorMix::A || comp2 == VectorMix::B) && (comp3 == VectorMix::A && comp3 == VectorMix::B)>::test())
		{
			// First two components come from input 0, second two come from input 1
			return vector_set(vector_get_component(input0, comp0), vector_get_component(input0, comp1), vector_get_component(input1, comp2), vector_get_component(input1, comp3));
		}

		if (rtm_impl::static_condition<(comp0 == VectorMix::A || comp0 == VectorMix::B) && (comp1 == VectorMix::A && comp1 == VectorMix::B) && (comp2 == VectorMix::X || comp2 == VectorMix::Y) && (comp3 == VectorMix::X || comp3 == VectorMix::Y)>::test())
		{
			// First two components come from input 1, second two come from input 0
			return vector_set(vector_get_component(input1, comp0), vector_get_component(input1, comp1), vector_get_component(input0, comp2), vector_get_component(input0, comp3));
		}

		if (rtm_impl::static_condition<comp0 == VectorMix::X && comp1 == VectorMix::A && comp2 == VectorMix::Y && comp3 == VectorMix::B>::test())
		{
			// Low words from both inputs are interleaved
			return vector_set(vector_get_component(input0, comp0), vector_get_component(input1, comp1), vector_get_component(input0, comp2), vector_get_component(input1, comp3));
		}

		if (rtm_impl::static_condition<comp0 == VectorMix::A && comp1 == VectorMix::X && comp2 == VectorMix::B && comp3 == VectorMix::Y>::test())
		{
			// Low words from both inputs are interleaved
			return vector_set(vector_get_component(input1, comp0), vector_get_component(input0, comp1), vector_get_component(input1, comp2), vector_get_component(input0, comp3));
		}

		if (rtm_impl::static_condition<comp0 == VectorMix::Z && comp1 == VectorMix::C && comp2 == VectorMix::W && comp3 == VectorMix::D>::test())
		{
			// High words from both inputs are interleaved
			return vector_set(vector_get_component(input0, comp0), vector_get_component(input1, comp1), vector_get_component(input0, comp2), vector_get_component(input1, comp3));
		}

		if (rtm_impl::static_condition<comp0 == VectorMix::C && comp1 == VectorMix::Z && comp2 == VectorMix::D && comp3 == VectorMix::W>::test())
		{
			// High words from both inputs are interleaved
			return vector_set(vector_get_component(input1, comp0), vector_get_component(input0, comp1), vector_get_component(input1, comp2), vector_get_component(input0, comp3));
		}

		// Slow code path, not yet optimized
		//RTM_ASSERT(false, "vector_mix permutation not handled");
		const double x = rtm_impl::is_vector_mix_arg_xyzw(comp0) ? vector_get_component<comp0>(input0) : vector_get_component<comp0>(input1);
		const double y = rtm_impl::is_vector_mix_arg_xyzw(comp1) ? vector_get_component<comp1>(input0) : vector_get_component<comp1>(input1);
		const double z = rtm_impl::is_vector_mix_arg_xyzw(comp2) ? vector_get_component<comp2>(input0) : vector_get_component<comp2>(input1);
		const double w = rtm_impl::is_vector_mix_arg_xyzw(comp3) ? vector_get_component<comp3>(input0) : vector_get_component<comp3>(input1);
		return vector_set(x, y, z, w);
	}

	inline vector4d vector_mix_xxxx(const vector4d& input) { return vector_mix<VectorMix::X, VectorMix::X, VectorMix::X, VectorMix::X>(input, input); }
	inline vector4d vector_mix_yyyy(const vector4d& input) { return vector_mix<VectorMix::Y, VectorMix::Y, VectorMix::Y, VectorMix::Y>(input, input); }
	inline vector4d vector_mix_zzzz(const vector4d& input) { return vector_mix<VectorMix::Z, VectorMix::Z, VectorMix::Z, VectorMix::Z>(input, input); }
	inline vector4d vector_mix_wwww(const vector4d& input) { return vector_mix<VectorMix::W, VectorMix::W, VectorMix::W, VectorMix::W>(input, input); }

	inline vector4d vector_mix_xxyy(const vector4d& input) { return vector_mix<VectorMix::X, VectorMix::X, VectorMix::Y, VectorMix::Y>(input, input); }
	inline vector4d vector_mix_xzyw(const vector4d& input) { return vector_mix<VectorMix::X, VectorMix::Z, VectorMix::Y, VectorMix::W>(input, input); }
	inline vector4d vector_mix_yzxy(const vector4d& input) { return vector_mix<VectorMix::Y, VectorMix::Z, VectorMix::X, VectorMix::Y>(input, input); }
	inline vector4d vector_mix_ywxz(const vector4d& input) { return vector_mix<VectorMix::Y, VectorMix::W, VectorMix::X, VectorMix::Z>(input, input); }
	inline vector4d vector_mix_zxyx(const vector4d& input) { return vector_mix<VectorMix::Z, VectorMix::X, VectorMix::Y, VectorMix::X>(input, input); }
	inline vector4d vector_mix_zwyz(const vector4d& input) { return vector_mix<VectorMix::Z, VectorMix::W, VectorMix::Y, VectorMix::Z>(input, input); }
	inline vector4d vector_mix_zwzw(const vector4d& input) { return vector_mix<VectorMix::Z, VectorMix::W, VectorMix::Z, VectorMix::W>(input, input); }
	inline vector4d vector_mix_wxwx(const vector4d& input) { return vector_mix<VectorMix::W, VectorMix::X, VectorMix::W, VectorMix::X>(input, input); }
	inline vector4d vector_mix_wzwy(const vector4d& input) { return vector_mix<VectorMix::W, VectorMix::Z, VectorMix::W, VectorMix::Y>(input, input); }

	inline vector4d vector_mix_xyab(const vector4d& input0, const vector4d& input1) { return vector_mix<VectorMix::X, VectorMix::Y, VectorMix::A, VectorMix::B>(input0, input1); }
	inline vector4d vector_mix_xzac(const vector4d& input0, const vector4d& input1) { return vector_mix<VectorMix::X, VectorMix::Z, VectorMix::A, VectorMix::C>(input0, input1); }
	inline vector4d vector_mix_xbxb(const vector4d& input0, const vector4d& input1) { return vector_mix<VectorMix::X, VectorMix::B, VectorMix::X, VectorMix::B>(input0, input1); }
	inline vector4d vector_mix_xbzd(const vector4d& input0, const vector4d& input1) { return vector_mix<VectorMix::X, VectorMix::B, VectorMix::Z, VectorMix::D>(input0, input1); }
	inline vector4d vector_mix_ywbd(const vector4d& input0, const vector4d& input1) { return vector_mix<VectorMix::Y, VectorMix::W, VectorMix::B, VectorMix::D>(input0, input1); }
	inline vector4d vector_mix_zyax(const vector4d& input0, const vector4d& input1) { return vector_mix<VectorMix::Z, VectorMix::Y, VectorMix::A, VectorMix::X>(input0, input1); }
	inline vector4d vector_mix_zycx(const vector4d& input0, const vector4d& input1) { return vector_mix<VectorMix::Z, VectorMix::Y, VectorMix::C, VectorMix::X>(input0, input1); }
	inline vector4d vector_mix_zwcd(const vector4d& input0, const vector4d& input1) { return vector_mix<VectorMix::Z, VectorMix::W, VectorMix::C, VectorMix::D>(input0, input1); }
	inline vector4d vector_mix_zbaz(const vector4d& input0, const vector4d& input1) { return vector_mix<VectorMix::Z, VectorMix::B, VectorMix::A, VectorMix::Z>(input0, input1); }
	inline vector4d vector_mix_zdcz(const vector4d& input0, const vector4d& input1) { return vector_mix<VectorMix::Z, VectorMix::D, VectorMix::C, VectorMix::Z>(input0, input1); }
	inline vector4d vector_mix_wxya(const vector4d& input0, const vector4d& input1) { return vector_mix<VectorMix::W, VectorMix::X, VectorMix::Y, VectorMix::A>(input0, input1); }
	inline vector4d vector_mix_wxyc(const vector4d& input0, const vector4d& input1) { return vector_mix<VectorMix::W, VectorMix::X, VectorMix::Y, VectorMix::C>(input0, input1); }
	inline vector4d vector_mix_wbyz(const vector4d& input0, const vector4d& input1) { return vector_mix<VectorMix::W, VectorMix::B, VectorMix::Y, VectorMix::Z>(input0, input1); }
	inline vector4d vector_mix_wdyz(const vector4d& input0, const vector4d& input1) { return vector_mix<VectorMix::W, VectorMix::D, VectorMix::Y, VectorMix::Z>(input0, input1); }
	inline vector4d vector_mix_bxwa(const vector4d& input0, const vector4d& input1) { return vector_mix<VectorMix::B, VectorMix::X, VectorMix::W, VectorMix::A>(input0, input1); }
	inline vector4d vector_mix_bywx(const vector4d& input0, const vector4d& input1) { return vector_mix<VectorMix::B, VectorMix::Y, VectorMix::W, VectorMix::X>(input0, input1); }
	inline vector4d vector_mix_dxwc(const vector4d& input0, const vector4d& input1) { return vector_mix<VectorMix::D, VectorMix::X, VectorMix::W, VectorMix::C>(input0, input1); }
	inline vector4d vector_mix_dywx(const vector4d& input0, const vector4d& input1) { return vector_mix<VectorMix::D, VectorMix::Y, VectorMix::W, VectorMix::X>(input0, input1); }

	//////////////////////////////////////////////////////////////////////////
	// Misc

	inline vector4d vector_sign(const vector4d& input)
	{
		vector4d mask = vector_greater_equal(input, vector_zero_64());
		return vector_blend(mask, vector_set(1.0), vector_set(-1.0));
	}
}
