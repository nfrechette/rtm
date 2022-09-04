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

RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_NOINLINE bool RTM_SIMD_CALL mask_any_equal3_scalar4f(mask4f_arg0 lhs, mask4f_arg1 rhs) RTM_NO_EXCEPT
{
	return mask_get_x(lhs) == mask_get_x(rhs) &&
		mask_get_y(lhs) == mask_get_y(rhs) &&
		mask_get_z(lhs) == mask_get_z(rhs);
}

#if defined(RTM_NEON_INTRINSICS)
RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_NOINLINE bool RTM_SIMD_CALL mask_any_equal3_ref(mask4f_arg0 lhs, mask4f_arg1 rhs) RTM_NO_EXCEPT
{
	// Generates the following assembly:
	// eor.16b v0, v1, v0
    // xtn.4h v0, v0
    // fmov   x8, d0
    // mov    x9, #0xffffffffffff
    // bics   xzr, x9, x8
    // cset   w0, ne
    // ret

	uint32x4_t mask = veorq_u32(vreinterpretq_u32_f32(lhs), vreinterpretq_u32_f32(rhs));
	uint16x4_t truncated_mask = vmovn_u32(mask);
	return (vget_lane_u64(vreinterpret_u64_u16(truncated_mask), 0) & 0x0000FFFFFFFFFFFFULL) != 0x0000FFFFFFFFFFFFULL;
}

RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_NOINLINE bool RTM_SIMD_CALL mask_any_equal3_shl(mask4f_arg0 lhs, mask4f_arg1 rhs) RTM_NO_EXCEPT
{
	// Generates the following assembly:
	// eor.16b v0, v1, v0
    // xtn.4h v0, v0
    // fmov   x8, d0
    // mov    x9, #0xffffffffffff
    // bics   xzr, x9, x8
    // cset   w0, ne
    // ret

	uint32x4_t mask = veorq_u32(vreinterpretq_u32_f32(lhs), vreinterpretq_u32_f32(rhs));
	uint16x4_t truncated_mask = vmovn_u32(mask);
	return (~vget_lane_u64(vreinterpret_u64_u16(truncated_mask), 0) << 16) != 0;
}
#endif

static void bm_mask_any_equal3_scalar4f(benchmark::State& state)
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
		r0 ^= mask_any_equal3_scalar4f(m0, m1);
		r1 ^= mask_any_equal3_scalar4f(m1, m2);
		r2 ^= mask_any_equal3_scalar4f(m2, m3);
		r3 ^= mask_any_equal3_scalar4f(m3, m0);
	}

	benchmark::DoNotOptimize(r0);
	benchmark::DoNotOptimize(r1);
	benchmark::DoNotOptimize(r2);
	benchmark::DoNotOptimize(r3);
}

BENCHMARK(bm_mask_any_equal3_scalar4f);

#if defined(RTM_NEON_INTRINSICS)
static void bm_mask_any_equal3_ref(benchmark::State& state)
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
		r0 ^= mask_any_equal3_ref(m0, m1);
		r1 ^= mask_any_equal3_ref(m1, m2);
		r2 ^= mask_any_equal3_ref(m2, m3);
		r3 ^= mask_any_equal3_ref(m3, m0);
	}

	benchmark::DoNotOptimize(r0);
	benchmark::DoNotOptimize(r1);
	benchmark::DoNotOptimize(r2);
	benchmark::DoNotOptimize(r3);
}

BENCHMARK(bm_mask_any_equal3_ref);

static void bm_mask_any_equal3_shl(benchmark::State& state)
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
		r0 ^= mask_any_equal3_shl(m0, m1);
		r1 ^= mask_any_equal3_shl(m1, m2);
		r2 ^= mask_any_equal3_shl(m2, m3);
		r3 ^= mask_any_equal3_shl(m3, m0);
	}

	benchmark::DoNotOptimize(r0);
	benchmark::DoNotOptimize(r1);
	benchmark::DoNotOptimize(r2);
	benchmark::DoNotOptimize(r3);
}

BENCHMARK(bm_mask_any_equal3_shl);
#endif
