////////////////////////////////////////////////////////////////////////////////
// The MIT License (MIT)
//
// Copyright (c) 2019 Nicholas Frechette & Realtime Math contributors
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

#include <benchmark/benchmark.h>

#include <rtm/quatf.h>

using namespace rtm;

inline quatf RTM_SIMD_CALL quat_mul_scalar(quatf_arg0 lhs, quatf_arg1 rhs) RTM_NO_EXCEPT
{
	const float lhs_x = quat_get_x(lhs);
	const float lhs_y = quat_get_y(lhs);
	const float lhs_z = quat_get_z(lhs);
	const float lhs_w = quat_get_w(lhs);

	const float rhs_x = quat_get_x(rhs);
	const float rhs_y = quat_get_y(rhs);
	const float rhs_z = quat_get_z(rhs);
	const float rhs_w = quat_get_w(rhs);

	const float x = (rhs_w * lhs_x) + (rhs_x * lhs_w) + (rhs_y * lhs_z) - (rhs_z * lhs_y);
	const float y = (rhs_w * lhs_y) - (rhs_x * lhs_z) + (rhs_y * lhs_w) + (rhs_z * lhs_x);
	const float z = (rhs_w * lhs_z) + (rhs_x * lhs_y) - (rhs_y * lhs_x) + (rhs_z * lhs_w);
	const float w = (rhs_w * lhs_w) - (rhs_x * lhs_x) - (rhs_y * lhs_y) - (rhs_z * lhs_z);

	return quat_set(x, y, z, w);
}

#if defined(RTM_FMA_INTRINSICS)
inline quatf RTM_SIMD_CALL quat_mul_fma_mul(quatf_arg0 lhs, quatf_arg1 rhs) RTM_NO_EXCEPT
{
	constexpr __m128 control_wzyx = { 1.0f,-1.0f, 1.0f,-1.0f };
	constexpr __m128 control_zwxy = { 1.0f, 1.0f,-1.0f,-1.0f };
	constexpr __m128 control_yxwz = { -1.0f, 1.0f, 1.0f,-1.0f };

	const __m128 r_xxxx = _mm_shuffle_ps(rhs, rhs, _MM_SHUFFLE(0, 0, 0, 0));
	const __m128 r_yyyy = _mm_shuffle_ps(rhs, rhs, _MM_SHUFFLE(1, 1, 1, 1));
	const __m128 r_zzzz = _mm_shuffle_ps(rhs, rhs, _MM_SHUFFLE(2, 2, 2, 2));
	const __m128 r_wwww = _mm_shuffle_ps(rhs, rhs, _MM_SHUFFLE(3, 3, 3, 3));

	const __m128 lxrw_lyrw_lzrw_lwrw = _mm_mul_ps(r_wwww, lhs);
	const __m128 l_wzyx = _mm_shuffle_ps(lhs, lhs, _MM_SHUFFLE(0, 1, 2, 3));

	const __m128 lwrx_lzrx_lyrx_lxrx = _mm_mul_ps(r_xxxx, l_wzyx);
	const __m128 l_zwxy = _mm_shuffle_ps(l_wzyx, l_wzyx, _MM_SHUFFLE(2, 3, 0, 1));

	const __m128 lzry_lwry_lxry_lyry = _mm_mul_ps(r_yyyy, l_zwxy);
	const __m128 l_yxwz = _mm_shuffle_ps(l_zwxy, l_zwxy, _MM_SHUFFLE(0, 1, 2, 3));

	const __m128 lyrz_lxrz_lwrz_lzrz = _mm_mul_ps(r_zzzz, l_yxwz);
	const __m128 result0 = _mm_fmadd_ps(lwrx_lzrx_lyrx_lxrx, control_wzyx, lxrw_lyrw_lzrw_lwrw);
	const __m128 result1 = _mm_fmadd_ps(lzry_lwry_lxry_lyry, control_zwxy, result0);
	return _mm_fmadd_ps(lyrz_lxrz_lwrz_lzrz, control_yxwz, result1);
}

inline quatf RTM_SIMD_CALL quat_mul_fma_xor(quatf_arg0 lhs, quatf_arg1 rhs) RTM_NO_EXCEPT
{
	constexpr __m128 control_wzyx = { 0.0f,-0.0f, 0.0f,-0.0f };
	constexpr __m128 control_zwxy = { 0.0f, 0.0f,-0.0f,-0.0f };
	constexpr __m128 control_yxwz = { -0.0f, 0.0f, 0.0f,-0.0f };

	const __m128 r_xxxx = _mm_shuffle_ps(rhs, rhs, _MM_SHUFFLE(0, 0, 0, 0));
	const __m128 r_yyyy = _mm_shuffle_ps(rhs, rhs, _MM_SHUFFLE(1, 1, 1, 1));
	const __m128 r_zzzz = _mm_shuffle_ps(rhs, rhs, _MM_SHUFFLE(2, 2, 2, 2));
	const __m128 r_wwww = _mm_shuffle_ps(rhs, rhs, _MM_SHUFFLE(3, 3, 3, 3));

	const __m128 l_wzyx = _mm_shuffle_ps(lhs, lhs, _MM_SHUFFLE(0, 1, 2, 3));

	const __m128 lwrx_lzrx_lyrx_lxrx = _mm_mul_ps(r_xxxx, l_wzyx);
	const __m128 l_zwxy = _mm_shuffle_ps(l_wzyx, l_wzyx, _MM_SHUFFLE(2, 3, 0, 1));

	const __m128 lwrx_nlzrx_lyrx_nlxrx = _mm_xor_ps(lwrx_lzrx_lyrx_lxrx, control_wzyx);

	const __m128 lzry_lwry_lxry_lyry = _mm_mul_ps(r_yyyy, l_zwxy);
	const __m128 l_yxwz = _mm_shuffle_ps(l_zwxy, l_zwxy, _MM_SHUFFLE(0, 1, 2, 3));

	const __m128 lzry_lwry_nlxry_nlyry = _mm_xor_ps(lzry_lwry_lxry_lyry, control_zwxy);

	const __m128 lyrz_lxrz_lwrz_lzrz = _mm_mul_ps(r_zzzz, l_yxwz);
	const __m128 result0 = _mm_fmadd_ps(r_wwww, lhs, lwrx_nlzrx_lyrx_nlxrx);

	const __m128 nlyrz_lxrz_lwrz_wlzrz = _mm_xor_ps(lyrz_lxrz_lwrz_lzrz, control_yxwz);
	const __m128 result1 = _mm_add_ps(lzry_lwry_nlxry_nlyry, nlyrz_lxrz_lwrz_wlzrz);
	return _mm_add_ps(result0, result1);
}
#endif

#if defined(RTM_SSE2_INTRINSICS)
inline quatf RTM_SIMD_CALL quat_mul_sse_mul(quatf_arg0 lhs, quatf_arg1 rhs) RTM_NO_EXCEPT
{
	constexpr __m128 control_wzyx = { 1.0f,-1.0f, 1.0f,-1.0f };
	constexpr __m128 control_zwxy = { 1.0f, 1.0f,-1.0f,-1.0f };
	constexpr __m128 control_yxwz = { -1.0f, 1.0f, 1.0f,-1.0f };

	const __m128 r_xxxx = _mm_shuffle_ps(rhs, rhs, _MM_SHUFFLE(0, 0, 0, 0));
	const __m128 r_yyyy = _mm_shuffle_ps(rhs, rhs, _MM_SHUFFLE(1, 1, 1, 1));
	const __m128 r_zzzz = _mm_shuffle_ps(rhs, rhs, _MM_SHUFFLE(2, 2, 2, 2));
	const __m128 r_wwww = _mm_shuffle_ps(rhs, rhs, _MM_SHUFFLE(3, 3, 3, 3));

	const __m128 lxrw_lyrw_lzrw_lwrw = _mm_mul_ps(r_wwww, lhs);
	const __m128 l_wzyx = _mm_shuffle_ps(lhs, lhs, _MM_SHUFFLE(0, 1, 2, 3));

	const __m128 lwrx_lzrx_lyrx_lxrx = _mm_mul_ps(r_xxxx, l_wzyx);
	const __m128 l_zwxy = _mm_shuffle_ps(l_wzyx, l_wzyx, _MM_SHUFFLE(2, 3, 0, 1));

	const __m128 lwrx_nlzrx_lyrx_nlxrx = _mm_mul_ps(lwrx_lzrx_lyrx_lxrx, control_wzyx);

	const __m128 lzry_lwry_lxry_lyry = _mm_mul_ps(r_yyyy, l_zwxy);
	const __m128 l_yxwz = _mm_shuffle_ps(l_zwxy, l_zwxy, _MM_SHUFFLE(0, 1, 2, 3));

	const __m128 lzry_lwry_nlxry_nlyry = _mm_mul_ps(lzry_lwry_lxry_lyry, control_zwxy);

	const __m128 lyrz_lxrz_lwrz_lzrz = _mm_mul_ps(r_zzzz, l_yxwz);
	const __m128 result0 = _mm_add_ps(lxrw_lyrw_lzrw_lwrw, lwrx_nlzrx_lyrx_nlxrx);

	const __m128 nlyrz_lxrz_lwrz_wlzrz = _mm_mul_ps(lyrz_lxrz_lwrz_lzrz, control_yxwz);
	const __m128 result1 = _mm_add_ps(lzry_lwry_nlxry_nlyry, nlyrz_lxrz_lwrz_wlzrz);
	return _mm_add_ps(result0, result1);
}

inline quatf RTM_SIMD_CALL quat_mul_sse_xor(quatf_arg0 lhs, quatf_arg1 rhs) RTM_NO_EXCEPT
{
	constexpr __m128 control_wzyx = { 0.0f,-0.0f, 0.0f,-0.0f };
	constexpr __m128 control_zwxy = { 0.0f, 0.0f,-0.0f,-0.0f };
	constexpr __m128 control_yxwz = { -0.0f, 0.0f, 0.0f,-0.0f };

	const __m128 r_xxxx = _mm_shuffle_ps(rhs, rhs, _MM_SHUFFLE(0, 0, 0, 0));
	const __m128 r_yyyy = _mm_shuffle_ps(rhs, rhs, _MM_SHUFFLE(1, 1, 1, 1));
	const __m128 r_zzzz = _mm_shuffle_ps(rhs, rhs, _MM_SHUFFLE(2, 2, 2, 2));
	const __m128 r_wwww = _mm_shuffle_ps(rhs, rhs, _MM_SHUFFLE(3, 3, 3, 3));

	const __m128 lxrw_lyrw_lzrw_lwrw = _mm_mul_ps(r_wwww, lhs);
	const __m128 l_wzyx = _mm_shuffle_ps(lhs, lhs, _MM_SHUFFLE(0, 1, 2, 3));

	const __m128 lwrx_lzrx_lyrx_lxrx = _mm_mul_ps(r_xxxx, l_wzyx);
	const __m128 l_zwxy = _mm_shuffle_ps(l_wzyx, l_wzyx, _MM_SHUFFLE(2, 3, 0, 1));

	const __m128 lwrx_nlzrx_lyrx_nlxrx = _mm_xor_ps(lwrx_lzrx_lyrx_lxrx, control_wzyx);

	const __m128 lzry_lwry_lxry_lyry = _mm_mul_ps(r_yyyy, l_zwxy);
	const __m128 l_yxwz = _mm_shuffle_ps(l_zwxy, l_zwxy, _MM_SHUFFLE(0, 1, 2, 3));

	const __m128 lzry_lwry_nlxry_nlyry = _mm_xor_ps(lzry_lwry_lxry_lyry, control_zwxy);

	const __m128 lyrz_lxrz_lwrz_lzrz = _mm_mul_ps(r_zzzz, l_yxwz);
	const __m128 result0 = _mm_add_ps(lxrw_lyrw_lzrw_lwrw, lwrx_nlzrx_lyrx_nlxrx);

	const __m128 nlyrz_lxrz_lwrz_wlzrz = _mm_xor_ps(lyrz_lxrz_lwrz_lzrz, control_yxwz);
	const __m128 result1 = _mm_add_ps(lzry_lwry_nlxry_nlyry, nlyrz_lxrz_lwrz_wlzrz);
	return _mm_add_ps(result0, result1);
}
#endif

#if defined(RTM_NEON_INTRINSICS)
inline quatf RTM_SIMD_CALL quat_mul_neon_mul(quatf_arg0 lhs, quatf_arg1 rhs) RTM_NO_EXCEPT
{
	alignas(16) constexpr float control_wzyx_f[4] = { 1.0f, -1.0f, 1.0f, -1.0f };
	alignas(16) constexpr float control_zwxy_f[4] = { 1.0f, 1.0f, -1.0f, -1.0f };
	alignas(16) constexpr float control_yxwz_f[4] = { -1.0f, 1.0f, 1.0f, -1.0f };

	const float32x4_t control_wzyx = *reinterpret_cast<const float32x4_t*>(&control_wzyx_f[0]);
	const float32x4_t control_zwxy = *reinterpret_cast<const float32x4_t*>(&control_zwxy_f[0]);
	const float32x4_t control_yxwz = *reinterpret_cast<const float32x4_t*>(&control_yxwz_f[0]);

	const float32x2_t r_xy = vget_low_f32(rhs);
	const float32x2_t r_zw = vget_high_f32(rhs);

	const float32x4_t lxrw_lyrw_lzrw_lwrw = vmulq_lane_f32(lhs, r_zw, 1);

	const float32x4_t l_yxwz = vrev64q_f32(lhs);
	const float32x4_t l_wzyx = vcombine_f32(vget_high_f32(l_yxwz), vget_low_f32(l_yxwz));
	const float32x4_t l_zwxy = vrev64q_f32(l_wzyx);

	const float32x4_t lwrx_lzrx_lyrx_lxrx = vmulq_lane_f32(l_wzyx, r_xy, 0);
	const float32x4_t lzry_lwry_lxry_lyry = vmulq_lane_f32(l_zwxy, r_xy, 1);
	const float32x4_t lyrz_lxrz_lwrz_lzrz = vmulq_lane_f32(l_yxwz, r_zw, 0);

#if defined(RTM_NEON64_INTRINSICS)
	const float32x4_t result0 = vfmaq_f32(lxrw_lyrw_lzrw_lwrw, lwrx_lzrx_lyrx_lxrx, control_wzyx);
	const float32x4_t result1 = vfmaq_f32(result0, lzry_lwry_lxry_lyry, control_zwxy);
	return vfmaq_f32(result1, lyrz_lxrz_lwrz_lzrz, control_yxwz);
#else
	const float32x4_t result0 = vmlaq_f32(lxrw_lyrw_lzrw_lwrw, lwrx_lzrx_lyrx_lxrx, control_wzyx);
	const float32x4_t result1 = vmlaq_f32(result0, lzry_lwry_lxry_lyry, control_zwxy);
	return vmlaq_f32(result1, lyrz_lxrz_lwrz_lzrz, control_yxwz);
#endif
}

inline quatf RTM_SIMD_CALL quat_mul_neon_xor(quatf_arg0 lhs, quatf_arg1 rhs) RTM_NO_EXCEPT
{
	alignas(16) constexpr uint32x4_t control_wzyx_f[4] = { 0, 0x80000000U, 0, 0x80000000U };
	alignas(16) constexpr uint32x4_t control_zwxy_f[4] = { 0, 0, 0x80000000U, 0x80000000U };
	alignas(16) constexpr uint32x4_t control_yxwz_f[4] = { 0x80000000U, 0, 0, 0x80000000U };

	const uint32x4_t control_wzyx = *reinterpret_cast<const uint32x4_t*>(&control_wzyx_f[0]);
	const uint32x4_t control_zwxy = *reinterpret_cast<const uint32x4_t*>(&control_zwxy_f[0]);
	const uint32x4_t control_yxwz = *reinterpret_cast<const uint32x4_t*>(&control_yxwz_f[0]);

	float32x2_t r_xy = vget_low_f32(rhs);
	float32x2_t r_zw = vget_high_f32(rhs);

	float32x4_t lxrw_lyrw_lzrw_lwrw = vmulq_lane_f32(lhs, r_zw, 1);

	float32x4_t l_yxwz = vrev64q_f32(lhs);
	float32x4_t l_wzyx = vcombine_f32(vget_high_f32(l_yxwz), vget_low_f32(l_yxwz));
	float32x4_t lwrx_lzrx_lyrx_lxrx = vmulq_lane_f32(l_wzyx, r_xy, 0);

	float32x4_t result0 = vaddq_f32(vreinterpretq_f32_u32(veorq_u32(vreinterpretq_u32_f32(lwrx_lzrx_lyrx_lxrx), control_wzyx)), lxrw_lyrw_lzrw_lwrw);

	float32x4_t l_zwxy = vrev64q_f32(l_wzyx);
	float32x4_t lzry_lwry_lxry_lyry = vmulq_lane_f32(l_zwxy, r_xy, 1);

	float32x4_t result1 = vaddq_f32(vreinterpretq_f32_u32(veorq_u32(vreinterpretq_u32_f32(lzry_lwry_lxry_lyry), control_zwxy)), result0);

	float32x4_t lyrz_lxrz_lwrz_lzrz = vmulq_lane_f32(l_yxwz, r_zw, 0);

	return vaddq_f32(vreinterpretq_f32_u32(veorq_u32(vreinterpretq_u32_f32(lyrz_lxrz_lwrz_lzrz), control_yxwz)), result1);
}

inline quatf RTM_SIMD_CALL quat_mul_neon_neg(quatf_arg0 lhs, quatf_arg1 rhs) RTM_NO_EXCEPT
{
	const float32x4_t neg_lhs = vnegq_f32(lhs);														// -t.x, -t.y, -t.z, -t.w
	const float32x4_t t_zwxy = vrev64q_f32(lhs);														// t.z, t.w, t.x, t.y
	const float32x4_t nt_zwxy = vrev64q_f32(neg_lhs);													// -t.z, -t.w, -t.x, -t.y

	const float32x2x2_t tmp1 = vzip_f32(vget_low_f32(t_zwxy), vget_high_f32(t_zwxy));					// t.z, t.x, t.w, t.y
	const float32x2x2_t tmp2 = vzip_f32(vget_low_f32(nt_zwxy), vget_high_f32(nt_zwxy));					// -t.z, -t.x, -t.w, -t.y
	const float32x2x2_t tmp3 = vzip_f32(tmp2.val[1], tmp1.val[0]);										// -t.w, t.z, -t.y, t.x
	const float32x2x2_t tmp4 = vzip_f32(tmp1.val[1], tmp2.val[0]);										// t.w, -t.z, t.y, -t.x

	const float32x4_t l_wzyx = vcombine_f32(tmp3.val[0], tmp3.val[1]);									// -t.w, t.z, -t.y, t.x
	const float32x4_t l_zwxy = vcombine_f32(vget_high_f32(neg_lhs), vget_low_f32(lhs));				// -t.z, -t.w, t.x, t.y
	const float32x4_t l_yxwz = vcombine_f32(tmp4.val[1], tmp3.val[0]);									// t.y, -t.x, -t.w, t.z

	const float32x2_t r_xy = vget_low_f32(rhs);
	const float32x2_t r_zw = vget_high_f32(rhs);

	const float32x4_t lxrw_lyrw_lzrw_lwrw = vmulq_lane_f32(lhs, r_zw, 1);

#if defined(RTM_NEON64_INTRINSICS)
	const float32x4_t result0 = vfmaq_lane_f32(lxrw_lyrw_lzrw_lwrw, l_wzyx, r_xy, 0);
	const float32x4_t result1 = vfmaq_lane_f32(result0, l_zwxy, r_xy, 1);
	return vfmaq_lane_f32(result1, l_yxwz, r_zw, 0);
#else
	const float32x4_t result0 = vmlaq_lane_f32(lxrw_lyrw_lzrw_lwrw, l_wzyx, r_xy, 0);
	const float32x4_t result1 = vmlaq_lane_f32(result0, l_zwxy, r_xy, 1);
	return vmlaq_lane_f32(result1, l_yxwz, r_zw, 0);
#endif
}
#endif

static void bm_quat_mul_scalar(benchmark::State& state)
{
	quatf q0 = quat_identity();

	for (auto _ : state)
		benchmark::DoNotOptimize(q0 = quat_mul_scalar(q0, q0));
}

BENCHMARK(bm_quat_mul_scalar);

#if defined(RTM_FMA_INTRINSICS)
static void bm_quat_mul_fma_mul(benchmark::State& state)
{
	quatf q0 = quat_identity();

	for (auto _ : state)
		benchmark::DoNotOptimize(q0 = quat_mul_fma_mul(q0, q0));
}

BENCHMARK(bm_quat_mul_fma_mul);

static void bm_quat_mul_fma_xor(benchmark::State& state)
{
	quatf q0 = quat_identity();

	for (auto _ : state)
		benchmark::DoNotOptimize(q0 = quat_mul_fma_xor(q0, q0));
}

BENCHMARK(bm_quat_mul_fma_xor);
#endif

#if defined(RTM_SSE2_INTRINSICS)
static void bm_quat_mul_sse_mul(benchmark::State& state)
{
	quatf q0 = quat_identity();

	for (auto _ : state)
		benchmark::DoNotOptimize(q0 = quat_mul_sse_mul(q0, q0));
}

BENCHMARK(bm_quat_mul_sse_mul);

static void bm_quat_mul_sse_xor(benchmark::State& state)
{
	quatf q0 = quat_identity();

	for (auto _ : state)
		benchmark::DoNotOptimize(q0 = quat_mul_sse_xor(q0, q0));
}

BENCHMARK(bm_quat_mul_sse_xor);
#endif

#if defined(RTM_NEON_INTRINSICS)
static void bm_quat_mul_neon_mul(benchmark::State& state)
{
	quatf q0 = quat_identity();

	for (auto _ : state)
		benchmark::DoNotOptimize(q0 = quat_mul_neon_mul(q0, q0));
}

BENCHMARK(bm_quat_mul_neon_mul);

static void bm_quat_mul_neon_xor(benchmark::State& state)
{
	quatf q0 = quat_identity();

	for (auto _ : state)
		benchmark::DoNotOptimize(q0 = quat_mul_neon_xor(q0, q0));
}

BENCHMARK(bm_quat_mul_neon_xor);

static void bm_quat_mul_neon_neg(benchmark::State& state)
{
	quatf q0 = quat_identity();

	for (auto _ : state)
		benchmark::DoNotOptimize(q0 = quat_mul_neon_neg(q0, q0));
}

BENCHMARK(bm_quat_mul_neon_neg);
#endif
