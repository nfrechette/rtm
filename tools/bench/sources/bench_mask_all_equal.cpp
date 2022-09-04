////////////////////////////////////////////////////////////////////////////////
// The MIT License (MIT)
//
// Copyright (c) 2022 Nicholas Frechette & Realtime Math contributors
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

#include <rtm/mask4f.h>

#include <cstring>

using namespace rtm;

RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_NOINLINE bool RTM_SIMD_CALL mask_all_equal_scalar4f(mask4f_arg0 lhs, mask4f_arg1 rhs) RTM_NO_EXCEPT
{
	return mask_get_x(lhs) == mask_get_x(rhs) &&
		mask_get_y(lhs) == mask_get_y(rhs) &&
		mask_get_z(lhs) == mask_get_z(rhs) &&
		mask_get_w(lhs) == mask_get_w(rhs);
}

RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_NOINLINE bool RTM_SIMD_CALL mask_all_equal_memcmp4f(mask4f_arg0 lhs, mask4f_arg1 rhs) RTM_NO_EXCEPT
{
	return std::memcmp(&lhs, &rhs, sizeof(uint32_t) * 4) == 0;
}

#if defined(RTM_SSE2_INTRINSICS) || defined(RTM_NEON_INTRINSICS)
RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_NOINLINE bool RTM_SIMD_CALL mask_all_equal_cmp4f(mask4f_arg0 lhs, mask4f_arg1 rhs) RTM_NO_EXCEPT
{
#if defined(RTM_SSE2_INTRINSICS)
	// Assembly for this is 16 bytes with SSE2/AVX: pcmpeqd, pmovmskb, cmp, sete, ret
	return _mm_movemask_epi8(_mm_cmpeq_epi32(_mm_castps_si128(lhs), _mm_castps_si128(rhs))) == 0xFFFF;
#elif defined(RTM_NEON_INTRINSICS)
	uint8x16_t mask = vreinterpretq_u8_u32(vceqq_u32(vreinterpretq_u32_f32(lhs), vreinterpretq_u32_f32(rhs)));
	uint8x8x2_t mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15 = vzip_u8(vget_low_u8(mask), vget_high_u8(mask));
	uint16x4x2_t mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15 = vzip_u16(vreinterpret_u16_u8(mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[0]), vreinterpret_u16_u8(mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[1]));
	return vget_lane_u32(vreinterpret_u32_u16(mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15.val[0]), 0) == 0xFFFFFFFFU;
#endif
}

// Wins with SSE2 on Ryzen 2990X, same performance as cmp4f but shorter assembly
// With AVX, cmp4f is a bit faster, reason unclear. Assembly remains shorter and xor
// can execute on more ports than pcmpeqd and reciprocal throughput is faster and
// it should avoid a domain change.
RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_NOINLINE bool RTM_SIMD_CALL mask_all_equal_xor4f(mask4f_arg0 lhs, mask4f_arg1 rhs) RTM_NO_EXCEPT
{
#if defined(RTM_SSE2_INTRINSICS)
	// Assembly for this is 11 bytes with SSE2 (13 bytes with AVX): xorps, movmskps, test, sete, ret
	return _mm_movemask_ps(_mm_xor_ps(lhs, rhs)) == 0;
#elif defined(RTM_NEON_INTRINSICS)
	// Generates the following assembly:
	// eor.16b v0, v1, v0
    // ext.16b v1, v0, v0, #0x8
    // zip1.8b v2, v0, v1
    // zip2.8b v0, v0, v1
    // zip1.4h v0, v2, v0
    // fmov   w8, s0
    // cmp    w8, #0x0
    // cset   w0, eq
    // ret

	uint8x16_t mask = vreinterpretq_u8_u32(veorq_u32(vreinterpretq_u32_f32(lhs), vreinterpretq_u32_f32(rhs)));
	uint8x8x2_t mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15 = vzip_u8(vget_low_u8(mask), vget_high_u8(mask));
	uint16x4x2_t mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15 = vzip_u16(vreinterpret_u16_u8(mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[0]), vreinterpret_u16_u8(mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[1]));
	return vget_lane_u32(vreinterpret_u32_u16(mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15.val[0]), 0) == 0;
#endif
}

#if defined(RTM_NEON_INTRINSICS)
// On an Apple M1, both mask_all_equal_xor4f and this variant have the same performance
// but this version has much short assembly
RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_NOINLINE bool RTM_SIMD_CALL mask_all_equal_xor_movn(mask4f_arg0 lhs, mask4f_arg1 rhs) RTM_NO_EXCEPT
{
	// Generates the following assembly:
	// eor.16b v0, v1, v0
	// xtn.4h v0, v0
	// fmov   x8, d0
	// cmp    x8, #0x0
	// cset   w0, eq
	// ret

	uint32x4_t mask = veorq_u32(vreinterpretq_u32_f32(lhs), vreinterpretq_u32_f32(rhs));
	uint16x4_t truncated_mask = vmovn_u32(mask);
	return vget_lane_u64(vreinterpret_u64_u16(truncated_mask), 0) == 0;
}
#endif
#endif

static void bm_mask_all_equal_scalar4f(benchmark::State& state)
{
	mask4f m0 = mask_set(true, false, true, false);
	mask4f m1 = mask_set(true, true, false, false);
	mask4f m2 = mask_set(false, true, false, true);
	mask4f m3 = mask_set(false, false, false, false);

	volatile bool r0 = false;
	volatile bool r1 = false;
	volatile bool r2 = false;
	volatile bool r3 = false;

	for (auto _ : state)
	{
		r0 ^= mask_all_equal_scalar4f(m0, m1);
		r1 ^= mask_all_equal_scalar4f(m1, m2);
		r2 ^= mask_all_equal_scalar4f(m2, m3);
		r3 ^= mask_all_equal_scalar4f(m3, m0);
	}

	benchmark::DoNotOptimize(r0);
	benchmark::DoNotOptimize(r1);
	benchmark::DoNotOptimize(r2);
	benchmark::DoNotOptimize(r3);
}

BENCHMARK(bm_mask_all_equal_scalar4f);

static void bm_mask_all_equal_memcmp4f(benchmark::State& state)
{
	mask4f m0 = mask_set(true, false, true, false);
	mask4f m1 = mask_set(true, true, false, false);
	mask4f m2 = mask_set(false, true, false, true);
	mask4f m3 = mask_set(false, false, false, false);

	volatile bool r0 = false;
	volatile bool r1 = false;
	volatile bool r2 = false;
	volatile bool r3 = false;

	for (auto _ : state)
	{
		r0 ^= mask_all_equal_memcmp4f(m0, m1);
		r1 ^= mask_all_equal_memcmp4f(m1, m2);
		r2 ^= mask_all_equal_memcmp4f(m2, m3);
		r3 ^= mask_all_equal_memcmp4f(m3, m0);
	}

	benchmark::DoNotOptimize(r0);
	benchmark::DoNotOptimize(r1);
	benchmark::DoNotOptimize(r2);
	benchmark::DoNotOptimize(r3);
}

BENCHMARK(bm_mask_all_equal_memcmp4f);

#if defined(RTM_SSE2_INTRINSICS) || defined(RTM_NEON_INTRINSICS)
static void bm_mask_all_equal_cmp4f(benchmark::State& state)
{
	mask4f m0 = mask_set(true, false, true, false);
	mask4f m1 = mask_set(true, true, false, false);
	mask4f m2 = mask_set(false, true, false, true);
	mask4f m3 = mask_set(false, false, false, false);

	volatile bool r0 = false;
	volatile bool r1 = false;
	volatile bool r2 = false;
	volatile bool r3 = false;

	for (auto _ : state)
	{
		r0 ^= mask_all_equal_cmp4f(m0, m1);
		r1 ^= mask_all_equal_cmp4f(m1, m2);
		r2 ^= mask_all_equal_cmp4f(m2, m3);
		r3 ^= mask_all_equal_cmp4f(m3, m0);
	}

	benchmark::DoNotOptimize(r0);
	benchmark::DoNotOptimize(r1);
	benchmark::DoNotOptimize(r2);
	benchmark::DoNotOptimize(r3);
}

BENCHMARK(bm_mask_all_equal_cmp4f);

static void bm_mask_all_equal_xor4f(benchmark::State& state)
{
	mask4f m0 = mask_set(true, false, true, false);
	mask4f m1 = mask_set(true, true, false, false);
	mask4f m2 = mask_set(false, true, false, true);
	mask4f m3 = mask_set(false, false, false, false);

	volatile bool r0 = false;
	volatile bool r1 = false;
	volatile bool r2 = false;
	volatile bool r3 = false;

	for (auto _ : state)
	{
		r0 ^= mask_all_equal_xor4f(m0, m1);
		r1 ^= mask_all_equal_xor4f(m1, m2);
		r2 ^= mask_all_equal_xor4f(m2, m3);
		r3 ^= mask_all_equal_xor4f(m3, m0);
	}

	benchmark::DoNotOptimize(r0);
	benchmark::DoNotOptimize(r1);
	benchmark::DoNotOptimize(r2);
	benchmark::DoNotOptimize(r3);
}

BENCHMARK(bm_mask_all_equal_xor4f);

#if defined(RTM_NEON_INTRINSICS)
static void bm_mask_all_equal_xor_movn(benchmark::State& state)
{
	mask4f m0 = mask_set(true, false, true, false);
	mask4f m1 = mask_set(true, true, false, false);
	mask4f m2 = mask_set(false, true, false, true);
	mask4f m3 = mask_set(false, false, false, false);

	volatile bool r0 = false;
	volatile bool r1 = false;
	volatile bool r2 = false;
	volatile bool r3 = false;

	for (auto _ : state)
	{
		r0 ^= mask_all_equal_xor_movn(m0, m1);
		r1 ^= mask_all_equal_xor_movn(m1, m2);
		r2 ^= mask_all_equal_xor_movn(m2, m3);
		r3 ^= mask_all_equal_xor_movn(m3, m0);
	}

	benchmark::DoNotOptimize(r0);
	benchmark::DoNotOptimize(r1);
	benchmark::DoNotOptimize(r2);
	benchmark::DoNotOptimize(r3);
}

BENCHMARK(bm_mask_all_equal_xor_movn);
#endif
#endif
