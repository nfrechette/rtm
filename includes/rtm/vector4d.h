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
#include "rtm/scalard.h"
#include "rtm/impl/memory_utils.h"
#include "rtm/impl/vector_mix_common.h"

namespace rtm
{
	//////////////////////////////////////////////////////////////////////////
	// Setters, getters, and casts

	inline vector4d vector_set(double x, double y, double z, double w) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return vector4d{ _mm_set_pd(y, x), _mm_set_pd(w, z) };
#else
		return vector4d{ x, y, z, w };
#endif
	}

	inline vector4d vector_set(double x, double y, double z) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return vector4d{ _mm_set_pd(y, x), _mm_set_pd(0.0, z) };
#else
		return vector4d{ x, y, z, 0.0 };
#endif
	}

	inline vector4d vector_set(double xyzw) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xyzw_pd = _mm_set1_pd(xyzw);
		return vector4d{ xyzw_pd, xyzw_pd };
#else
		return vector4d{ xyzw, xyzw, xyzw, xyzw };
#endif
	}

	inline vector4d vector_unaligned_load(const double* input) RTM_NO_EXCEPT
	{
		RTM_ASSERT(rtm_impl::is_aligned_to(input, 4), "Invalid alignment");
		return vector_set(input[0], input[1], input[2], input[3]);
	}

	inline vector4d vector_unaligned_load3(const double* input) RTM_NO_EXCEPT
	{
		RTM_ASSERT(rtm_impl::is_aligned_to(input, 4), "Invalid alignment");
		return vector_set(input[0], input[1], input[2], 0.0);
	}

	inline vector4d vector_unaligned_load_64(const uint8_t* input) RTM_NO_EXCEPT
	{
		vector4d result;
		memcpy(&result, input, sizeof(vector4d));
		return result;
	}

	inline vector4d vector_unaligned_load3_64(const uint8_t* input) RTM_NO_EXCEPT
	{
		double input_f[3];
		memcpy(&input_f[0], input, sizeof(double) * 3);
		return vector_set(input_f[0], input_f[1], input_f[2], 0.0);
	}

	inline vector4d vector_zero_64() RTM_NO_EXCEPT
	{
		return vector_set(0.0, 0.0, 0.0, 0.0);
	}

	inline vector4d quat_to_vector(const quatd& input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return vector4d{ input.xy, input.zw };
#else
		return vector4d{ input.x, input.y, input.z, input.w };
#endif
	}

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

	inline double vector_get_x(const vector4d& input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_cvtsd_f64(input.xy);
#else
		return input.x;
#endif
	}

	inline double vector_get_y(const vector4d& input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_cvtsd_f64(_mm_shuffle_pd(input.xy, input.xy, 1));
#else
		return input.y;
#endif
	}

	inline double vector_get_z(const vector4d& input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_cvtsd_f64(input.zw);
#else
		return input.z;
#endif
	}

	inline double vector_get_w(const vector4d& input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_cvtsd_f64(_mm_shuffle_pd(input.zw, input.zw, 1));
#else
		return input.w;
#endif
	}

	template<mix4 component_index>
	inline double vector_get_component(const vector4d& input) RTM_NO_EXCEPT
	{
		switch (mix4(int(component_index) % 4))
		{
		case mix4::x: return vector_get_x(input);
		case mix4::y: return vector_get_y(input);
		case mix4::z: return vector_get_z(input);
		case mix4::w: return vector_get_w(input);
		default:
			RTM_ASSERT(false, "Invalid component index");
			return 0.0;
		}
	}

	inline double vector_get_component(const vector4d& input, mix4 component_index) RTM_NO_EXCEPT
	{
		switch (mix4(int(component_index) % 4))
		{
		case mix4::x: return vector_get_x(input);
		case mix4::y: return vector_get_y(input);
		case mix4::z: return vector_get_z(input);
		case mix4::w: return vector_get_w(input);
		default:
			RTM_ASSERT(false, "Invalid component index");
			return 0.0;
		}
	}

	inline const double* vector_as_double_ptr(const vector4d& input) RTM_NO_EXCEPT
	{
		return reinterpret_cast<const double*>(&input);
	}

	inline void vector_unaligned_write(const vector4d& input, double* output) RTM_NO_EXCEPT
	{
		RTM_ASSERT(rtm_impl::is_aligned_to(output, 4), "Invalid alignment");
		output[0] = vector_get_x(input);
		output[1] = vector_get_y(input);
		output[2] = vector_get_z(input);
		output[3] = vector_get_w(input);
	}

	inline void vector_unaligned_write3(const vector4d& input, double* output) RTM_NO_EXCEPT
	{
		RTM_ASSERT(rtm_impl::is_aligned_to(output, 4), "Invalid alignment");
		output[0] = vector_get_x(input);
		output[1] = vector_get_y(input);
		output[2] = vector_get_z(input);
	}

	inline void vector_unaligned_write(const vector4d& input, uint8_t* output) RTM_NO_EXCEPT
	{
		memcpy(output, &input, sizeof(vector4d));
	}

	inline void vector_unaligned_write3(const vector4d& input, uint8_t* output)
	{
		memcpy(output, &input, sizeof(double) * 3);
	}

	//////////////////////////////////////////////////////////////////////////
	// Arithmetic

	inline vector4d vector_add(const vector4d& lhs, const vector4d& rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return vector4d{ _mm_add_pd(lhs.xy, rhs.xy), _mm_add_pd(lhs.zw, rhs.zw) };
#else
		return vector_set(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z, lhs.w + rhs.w);
#endif
	}

	inline vector4d vector_sub(const vector4d& lhs, const vector4d& rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return vector4d{ _mm_sub_pd(lhs.xy, rhs.xy), _mm_sub_pd(lhs.zw, rhs.zw) };
#else
		return vector_set(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z, lhs.w - rhs.w);
#endif
	}

	inline vector4d vector_mul(const vector4d& lhs, const vector4d& rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return vector4d{ _mm_mul_pd(lhs.xy, rhs.xy), _mm_mul_pd(lhs.zw, rhs.zw) };
#else
		return vector_set(lhs.x * rhs.x, lhs.y * rhs.y, lhs.z * rhs.z, lhs.w * rhs.w);
#endif
	}

	inline vector4d vector_mul(const vector4d& lhs, double rhs) RTM_NO_EXCEPT
	{
		return vector_mul(lhs, vector_set(rhs));
	}

	inline vector4d vector_div(const vector4d& lhs, const vector4d& rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return vector4d{ _mm_div_pd(lhs.xy, rhs.xy), _mm_div_pd(lhs.zw, rhs.zw) };
#else
		return vector_set(lhs.x / rhs.x, lhs.y / rhs.y, lhs.z / rhs.z, lhs.w / rhs.w);
#endif
	}

	inline vector4d vector_max(const vector4d& lhs, const vector4d& rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return vector4d{ _mm_max_pd(lhs.xy, rhs.xy), _mm_max_pd(lhs.zw, rhs.zw) };
#else
		return vector_set(scalar_max(lhs.x, rhs.x), scalar_max(lhs.y, rhs.y), scalar_max(lhs.z, rhs.z), scalar_max(lhs.w, rhs.w));
#endif
	}

	inline vector4d vector_min(const vector4d& lhs, const vector4d& rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return vector4d{ _mm_min_pd(lhs.xy, rhs.xy), _mm_min_pd(lhs.zw, rhs.zw) };
#else
		return vector_set(scalar_min(lhs.x, rhs.x), scalar_min(lhs.y, rhs.y), scalar_min(lhs.z, rhs.z), scalar_min(lhs.w, rhs.w));
#endif
	}

	inline vector4d vector_abs(const vector4d& input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		vector4d zero{ _mm_setzero_pd(), _mm_setzero_pd() };
		return vector_max(vector_sub(zero, input), input);
#else
		return vector_set(scalar_abs(input.x), scalar_abs(input.y), scalar_abs(input.z), scalar_abs(input.w));
#endif
	}

	inline vector4d vector_neg(const vector4d& input) RTM_NO_EXCEPT
	{
		return vector_mul(input, -1.0);
	}

	inline vector4d vector_reciprocal(const vector4d& input) RTM_NO_EXCEPT
	{
		return vector_div(vector_set(1.0), input);
	}

	inline vector4d vector_cross3(const vector4d& lhs, const vector4d& rhs) RTM_NO_EXCEPT
	{
		return vector_set(vector_get_y(lhs) * vector_get_z(rhs) - vector_get_z(lhs) * vector_get_y(rhs),
						  vector_get_z(lhs) * vector_get_x(rhs) - vector_get_x(lhs) * vector_get_z(rhs),
						  vector_get_x(lhs) * vector_get_y(rhs) - vector_get_y(lhs) * vector_get_x(rhs));
	}

	inline double vector_dot(const vector4d& lhs, const vector4d& rhs) RTM_NO_EXCEPT
	{
		return (vector_get_x(lhs) * vector_get_x(rhs)) + (vector_get_y(lhs) * vector_get_y(rhs)) + (vector_get_z(lhs) * vector_get_z(rhs)) + (vector_get_w(lhs) * vector_get_w(rhs));
	}

	inline vector4d vector_vdot(const vector4d& lhs, const vector4d& rhs) RTM_NO_EXCEPT
	{
		return vector_set(vector_dot(lhs, rhs));
	}

	inline double vector_dot3(const vector4d& lhs, const vector4d& rhs) RTM_NO_EXCEPT
	{
		return (vector_get_x(lhs) * vector_get_x(rhs)) + (vector_get_y(lhs) * vector_get_y(rhs)) + (vector_get_z(lhs) * vector_get_z(rhs));
	}

	inline double vector_length_squared(const vector4d& input) RTM_NO_EXCEPT
	{
		return vector_dot(input, input);
	}

	inline double vector_length_squared3(const vector4d& input) RTM_NO_EXCEPT
	{
		return vector_dot3(input, input);
	}

	inline double vector_length(const vector4d& input) RTM_NO_EXCEPT
	{
		return scalar_sqrt(vector_length_squared(input));
	}

	inline double vector_length3(const vector4d& input) RTM_NO_EXCEPT
	{
		return scalar_sqrt(vector_length_squared3(input));
	}

	inline double vector_length_reciprocal(const vector4d& input) RTM_NO_EXCEPT
	{
		return 1.0 / vector_length(input);
	}

	inline double vector_length_reciprocal3(const vector4d& input) RTM_NO_EXCEPT
	{
		return 1.0 / vector_length3(input);
	}

	inline double vector_distance3(const vector4d& lhs, const vector4d& rhs) RTM_NO_EXCEPT
	{
		return vector_length3(vector_sub(rhs, lhs));
	}

	inline vector4d vector_normalize3(const vector4d& input, double threshold = 1.0e-8) RTM_NO_EXCEPT
	{
		// Reciprocal is more accurate to normalize with
		const double len_sq = vector_length_squared3(input);
		if (len_sq >= threshold)
			return vector_mul(input, scalar_sqrt_reciprocal(len_sq));
		else
			return input;
	}

	inline vector4d vector_fraction(const vector4d& input) RTM_NO_EXCEPT
	{
		return vector_set(scalar_fraction(vector_get_x(input)), scalar_fraction(vector_get_y(input)), scalar_fraction(vector_get_z(input)), scalar_fraction(vector_get_w(input)));
	}

	// output = (input * scale) + offset
	inline vector4d vector_mul_add(const vector4d& input, const vector4d& scale, const vector4d& offset) RTM_NO_EXCEPT
	{
		return vector_add(vector_mul(input, scale), offset);
	}

	inline vector4d vector_mul_add(const vector4d& input, double scale, const vector4d& offset) RTM_NO_EXCEPT
	{
		return vector_add(vector_mul(input, scale), offset);
	}

	// output = offset - (input * scale)
	inline vector4d vector_neg_mul_sub(const vector4d& input, const vector4d& scale, const vector4d& offset) RTM_NO_EXCEPT
	{
		return vector_sub(offset, vector_mul(input, scale));
	}

	inline vector4d vector_lerp(const vector4d& start, const vector4d& end, double alpha) RTM_NO_EXCEPT
	{
		return vector_mul_add(vector_sub(end, start), alpha, start);
	}

	//////////////////////////////////////////////////////////////////////////
	// Comparisons and masking

	inline vector4d vector_less_than(const vector4d& lhs, const vector4d& rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_lt_pd = _mm_cmplt_pd(lhs.xy, rhs.xy);
		__m128d zw_lt_pd = _mm_cmplt_pd(lhs.zw, rhs.zw);
		return vector4d{xy_lt_pd, zw_lt_pd};
#else
		return vector4d{rtm_impl::get_mask_value(lhs.x < rhs.x), rtm_impl::get_mask_value(lhs.y < rhs.y), rtm_impl::get_mask_value(lhs.z < rhs.z), rtm_impl::get_mask_value(lhs.w < rhs.w)};
#endif
	}

	inline vector4d vector_greater_equal(const vector4d& lhs, const vector4d& rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_ge_pd = _mm_cmpge_pd(lhs.xy, rhs.xy);
		__m128d zw_ge_pd = _mm_cmpge_pd(lhs.zw, rhs.zw);
		return vector4d{ xy_ge_pd, zw_ge_pd };
#else
		return vector4d{ rtm_impl::get_mask_value(lhs.x >= rhs.x), rtm_impl::get_mask_value(lhs.y >= rhs.y), rtm_impl::get_mask_value(lhs.z >= rhs.z), rtm_impl::get_mask_value(lhs.w >= rhs.w) };
#endif
	}

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

	inline bool vector_all_near_equal(const vector4d& lhs, const vector4d& rhs, double threshold = 0.00001) RTM_NO_EXCEPT
	{
		return vector_all_less_equal(vector_abs(vector_sub(lhs, rhs)), vector_set(threshold));
	}

	inline bool vector_all_near_equal3(const vector4d& lhs, const vector4d& rhs, double threshold = 0.00001) RTM_NO_EXCEPT
	{
		return vector_all_less_equal3(vector_abs(vector_sub(lhs, rhs)), vector_set(threshold));
	}

	inline bool vector_any_near_equal(const vector4d& lhs, const vector4d& rhs, double threshold = 0.00001) RTM_NO_EXCEPT
	{
		return vector_any_less_equal(vector_abs(vector_sub(lhs, rhs)), vector_set(threshold));
	}

	inline bool vector_any_near_equal3(const vector4d& lhs, const vector4d& rhs, double threshold = 0.00001) RTM_NO_EXCEPT
	{
		return vector_any_less_equal3(vector_abs(vector_sub(lhs, rhs)), vector_set(threshold));
	}

	inline bool vector_is_finite(const vector4d& input) RTM_NO_EXCEPT
	{
		return scalar_is_finite(vector_get_x(input)) && scalar_is_finite(vector_get_y(input)) && scalar_is_finite(vector_get_z(input)) && scalar_is_finite(vector_get_w(input));
	}

	inline bool vector_is_finite3(const vector4d& input) RTM_NO_EXCEPT
	{
		return scalar_is_finite(vector_get_x(input)) && scalar_is_finite(vector_get_y(input)) && scalar_is_finite(vector_get_z(input));
	}

	//////////////////////////////////////////////////////////////////////////
	// Swizzling, permutations, and mixing

	inline vector4d vector_blend(const vector4d& mask, const vector4d& if_true, const vector4d& if_false) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy = _mm_or_pd(_mm_andnot_pd(mask.xy, if_false.xy), _mm_and_pd(if_true.xy, mask.xy));
		__m128d zw = _mm_or_pd(_mm_andnot_pd(mask.zw, if_false.zw), _mm_and_pd(if_true.zw, mask.zw));
		return vector4d{ xy, zw };
#else
		return vector4d{ rtm_impl::select(mask.x, if_true.x, if_false.x), rtm_impl::select(mask.y, if_true.y, if_false.y), rtm_impl::select(mask.z, if_true.z, if_false.z), rtm_impl::select(mask.w, if_true.w, if_false.w) };
#endif
	}

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

	inline vector4d vector_dup_x(const vector4d& input) RTM_NO_EXCEPT { return vector_mix<mix4::x, mix4::x, mix4::x, mix4::x>(input, input); }
	inline vector4d vector_dup_y(const vector4d& input) RTM_NO_EXCEPT { return vector_mix<mix4::y, mix4::y, mix4::y, mix4::y>(input, input); }
	inline vector4d vector_dup_z(const vector4d& input) RTM_NO_EXCEPT { return vector_mix<mix4::z, mix4::z, mix4::z, mix4::z>(input, input); }
	inline vector4d vector_dup_w(const vector4d& input) RTM_NO_EXCEPT { return vector_mix<mix4::w, mix4::w, mix4::w, mix4::w>(input, input); }

	//////////////////////////////////////////////////////////////////////////
	// Misc

	inline vector4d vector_sign(const vector4d& input) RTM_NO_EXCEPT
	{
		vector4d mask = vector_greater_equal(input, vector_zero_64());
		return vector_blend(mask, vector_set(1.0), vector_set(-1.0));
	}
}
