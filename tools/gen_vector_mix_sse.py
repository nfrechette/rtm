import sys

# This script is used to generate the vector_mix template specialization

# Note:
# I attempted to use this script to exhaustively specialize every template
# for vector_mix and while it worked fine, compilation was quite dramatically slower
# and as a result, this is not currently used

def idx_to_mix(idx):
	if idx == 0:
		return 'mix4::x'
	elif idx == 1:
		return 'mix4::y'
	elif idx == 2:
		return 'mix4::z'
	elif idx == 3:
		return 'mix4::w'
	elif idx == 4:
		return 'mix4::a'
	elif idx == 5:
		return 'mix4::b'
	elif idx == 6:
		return 'mix4::c'
	elif idx == 7:
		return 'mix4::d'

def idx_to_mix_short(idx):
	if idx == 0:
		return 'x'
	elif idx == 1:
		return 'y'
	elif idx == 2:
		return 'z'
	elif idx == 3:
		return 'w'
	elif idx == 4:
		return 'a'
	elif idx == 5:
		return 'b'
	elif idx == 6:
		return 'c'
	elif idx == 7:
		return 'd'

def is_mix_xyzw(idx):
	return idx >= 0 and idx <= 3

def is_mix_abcd(idx):
	return idx >= 4 and idx <= 7

def get_selector(idx):
	if (idx % 4) == 0:
		return 'x'
	elif (idx % 4) == 1:
		return 'y'
	elif (idx % 4) == 2:
		return 'z'
	elif (idx % 4) == 3:
		return 'w'

# Rule formats are tuples of the form: (inner txt, cpp macro guard)
# The cpp macro guard can be None
# Rule functions return a matching rule format on success, or None on failure
# Rules will match in the order defined in the rules list
# When a rule matches, no other rule with the same cpp macro guard can match afterwards
# A rule must always match, as such a fallback must be implemented for every architecture

def rule_self0(x, y, z, w):
	if x == 0 and y == 1 and z == 2 and w == 3:
		inner_txt='''		(void)input1;
		return input0;'''
		return (inner_txt, None)

def rule_self1(x, y, z, w):
	if x == 4 and y == 5 and z == 6 and w == 7:
		inner_txt='''		(void)input0;
		return input1;'''
		return (inner_txt, None)

def rule_sse4_blend(x, y, z, w):
	if (x % 4) == 0 and (y % 4) == 1 and (z % 4) == 2 and (w % 4) == 3:
		mask_x = 0 if x == 0 else 1
		mask_y = 0 if y == 1 else 2
		mask_z = 0 if z == 2 else 4
		mask_w = 0 if w == 3 else 8
		inner_txt='''		return _mm_blend_ps(input0, input1, {} | {} | {} | {});'''.format(mask_x, mask_y, mask_z, mask_w)
		return (inner_txt, 'RTM_SSE4_INTRINSICS')

def rule_sse4_insert(x, y, z, w):
	if is_mix_abcd(x) and y == 1 and z == 2 and w == 3:
		inner_txt='''		return _mm_insert_ps(input0, input1, ({} << 6) | (0 << 4));'''.format(x % 4)
		return (inner_txt, 'RTM_SSE4_INTRINSICS')
	if x == 0 and is_mix_abcd(y) and z == 2 and w == 3:
		inner_txt='''		return _mm_insert_ps(input0, input1, ({} << 6) | (1 << 4));'''.format(y % 4)
		return (inner_txt, 'RTM_SSE4_INTRINSICS')
	if x == 0 and y == 1 and is_mix_abcd(z) and w == 3:
		inner_txt='''		return _mm_insert_ps(input0, input1, ({} << 6) | (2 << 4));'''.format(z % 4)
		return (inner_txt, 'RTM_SSE4_INTRINSICS')
	if x == 0 and y == 1 and z == 2 and is_mix_abcd(w):
		inner_txt='''		return _mm_insert_ps(input0, input1, ({} << 6) | (3 << 4));'''.format(w % 4)
		return (inner_txt, 'RTM_SSE4_INTRINSICS')
	if is_mix_xyzw(x) and y == 5 and z == 6 and w == 7:
		inner_txt='''		return _mm_insert_ps(input1, input0, ({} << 6) | (0 << 4));'''.format(x % 4)
		return (inner_txt, 'RTM_SSE4_INTRINSICS')
	if x == 4 and is_mix_xyzw(y) and z == 6 and w == 7:
		inner_txt='''		return _mm_insert_ps(input1, input0, ({} << 6) | (1 << 4));'''.format(y % 4)
		return (inner_txt, 'RTM_SSE4_INTRINSICS')
	if x == 4 and y == 5 and is_mix_xyzw(z) and w == 7:
		inner_txt='''		return _mm_insert_ps(input1, input0, ({} << 6) | (2 << 4));'''.format(z % 4)
		return (inner_txt, 'RTM_SSE4_INTRINSICS')
	if x == 4 and y == 5 and z == 6 and is_mix_xyzw(w):
		inner_txt='''		return _mm_insert_ps(input1, input0, ({} << 6) | (3 << 4));'''.format(w % 4)
		return (inner_txt, 'RTM_SSE4_INTRINSICS')

def rule_sse3_moveldup(x, y, z, w):
	if x == 0 and y == 0 and z == 2 and w == 2:
		inner_txt='''		(void)input1;
		return _mm_moveldup_ps(input0);'''
		return (inner_txt, 'RTM_SSE3_INTRINSICS')
	if x == 4 and y == 4 and z == 6 and w == 6:
		inner_txt='''		(void)input0;
		return _mm_moveldup_ps(input1);'''
		return (inner_txt, 'RTM_SSE3_INTRINSICS')

def rule_sse3_movehdup(x, y, z, w):
	if x == 1 and y == 1 and z == 3 and w == 3:
		inner_txt='''		(void)input1;
		return _mm_movehdup_ps(input0);'''
		return (inner_txt, 'RTM_SSE3_INTRINSICS')
	if x == 5 and y == 5 and z == 7 and w == 7:
		inner_txt='''		(void)input0;
		return _mm_movehdup_ps(input1);'''
		return (inner_txt, 'RTM_SSE3_INTRINSICS')

def rule_sse2_unpacklo(x, y, z, w):
	if x == 0 and y == 4 and z == 1 and w == 5:
		inner_txt='''		return _mm_unpacklo_ps(input0, input1);'''
		return (inner_txt, 'RTM_SSE2_INTRINSICS')
	if x == 4 and y == 0 and z == 5 and w == 1:
		inner_txt='''		return _mm_unpacklo_ps(input1, input0);'''
		return (inner_txt, 'RTM_SSE2_INTRINSICS')
	if x == 0 and y == 0 and z == 1 and w == 1:
		inner_txt='''		(void)input1;
		return _mm_unpacklo_ps(input0, input0);'''
		return (inner_txt, 'RTM_SSE2_INTRINSICS')
	if x == 4 and y == 4 and z == 5 and w == 5:
		inner_txt='''		(void)input0;
		return _mm_unpacklo_ps(input1, input1);'''
		return (inner_txt, 'RTM_SSE2_INTRINSICS')

def rule_sse2_unpackhi(x, y, z, w):
	if x == 2 and y == 6 and z == 3 and w == 7:
		inner_txt='''		return _mm_unpackhi_ps(input0, input1);'''
		return (inner_txt, 'RTM_SSE2_INTRINSICS')
	if x == 6 and y == 2 and z == 7 and w == 3:
		inner_txt='''		return _mm_unpackhi_ps(input1, input0);'''
		return (inner_txt, 'RTM_SSE2_INTRINSICS')
	if x == 2 and y == 2 and z == 3 and w == 3:
		inner_txt='''		(void)input1;
		return _mm_unpackhi_ps(input0, input0);'''
		return (inner_txt, 'RTM_SSE2_INTRINSICS')
	if x == 6 and y == 6 and z == 7 and w == 7:
		inner_txt='''		(void)input0;
		return _mm_unpackhi_ps(input1, input1);'''
		return (inner_txt, 'RTM_SSE2_INTRINSICS')

def rule_sse2_movelh(x, y, z, w):
	if x == 0 and y == 1 and z == 0 and w == 1:
		inner_txt='''		(void)input1;
		return _mm_movelh_ps(input0, input0);'''
		return (inner_txt, 'RTM_SSE2_INTRINSICS')
	if x == 4 and y == 5 and z == 4 and w == 5:
		inner_txt='''		(void)input0;
		return _mm_movelh_ps(input1, input1);'''
		return (inner_txt, 'RTM_SSE2_INTRINSICS')

def rule_sse2_movehl(x, y, z, w):
	if x == 2 and y == 3 and z == 2 and w == 3:
		inner_txt='''		(void)input1;
		return _mm_movehl_ps(input0, input0);'''
		return (inner_txt, 'RTM_SSE2_INTRINSICS')
	if x == 6 and y == 7 and z == 6 and w == 7:
		inner_txt='''		(void)input0;
		return _mm_movehl_ps(input1, input1);'''
		return (inner_txt, 'RTM_SSE2_INTRINSICS')

def rule_sse2_shuffle(x, y, z, w):
	if is_mix_xyzw(x) and is_mix_xyzw(y) and is_mix_xyzw(z) and is_mix_xyzw(w):
		inner_txt='''		(void)input1;
		return _mm_shuffle_ps(input0, input0, _MM_SHUFFLE({}, {}, {}, {}));'''.format(w % 4, z % 4, y % 4, x % 4)
		return (inner_txt, 'RTM_SSE2_INTRINSICS')
	if is_mix_xyzw(x) and is_mix_xyzw(y) and is_mix_abcd(z) and is_mix_abcd(w):
		inner_txt='''		return _mm_shuffle_ps(input0, input1, _MM_SHUFFLE({}, {}, {}, {}));'''.format(w % 4, z % 4, y % 4, x % 4)
		return (inner_txt, 'RTM_SSE2_INTRINSICS')
	if is_mix_abcd(x) and is_mix_abcd(y) and is_mix_abcd(z) and is_mix_abcd(w):
		inner_txt='''		(void)input0;
		return _mm_shuffle_ps(input1, input1, _MM_SHUFFLE({}, {}, {}, {}));'''.format(w % 4, z % 4, y % 4, x % 4)
		return (inner_txt, 'RTM_SSE2_INTRINSICS')
	if is_mix_abcd(x) and is_mix_abcd(y) and is_mix_xyzw(z) and is_mix_xyzw(w):
		inner_txt='''		return _mm_shuffle_ps(input1, input0, _MM_SHUFFLE({}, {}, {}, {}));'''.format(w % 4, z % 4, y % 4, x % 4)
		return (inner_txt, 'RTM_SSE2_INTRINSICS')

	if is_mix_xyzw(x) and is_mix_xyzw(y) and is_mix_xyzw(z) and is_mix_abcd(w):
		inner_txt='''		const __m128 z0z0w1w1 = _mm_shuffle_ps(input0, input1, _MM_SHUFFLE({}, {}, {}, {}));
		return _mm_shuffle_ps(input0, z0z0w1w1, _MM_SHUFFLE({}, {}, {}, {}));'''.format(w % 4, w % 4, z % 4, z % 4, 2, 0, y % 4, x % 4)
		return (inner_txt, 'RTM_SSE2_INTRINSICS')
	if is_mix_xyzw(x) and is_mix_xyzw(y) and is_mix_abcd(z) and is_mix_xyzw(w):
		inner_txt='''		const __m128 z1z1w0w0 = _mm_shuffle_ps(input1, input0, _MM_SHUFFLE({}, {}, {}, {}));
		return _mm_shuffle_ps(input0, z1z1w0w0, _MM_SHUFFLE({}, {}, {}, {}));'''.format(w % 4, w % 4, z % 4, z % 4, 2, 0, y % 4, x % 4)
		return (inner_txt, 'RTM_SSE2_INTRINSICS')
	if is_mix_xyzw(x) and is_mix_abcd(y) and is_mix_xyzw(z) and is_mix_xyzw(w):
		inner_txt='''		const __m128 x0x0y1y1 = _mm_shuffle_ps(input0, input1, _MM_SHUFFLE({}, {}, {}, {}));
		return _mm_shuffle_ps(x0x0y1y1, input0, _MM_SHUFFLE({}, {}, {}, {}));'''.format(y % 4, y % 4, x % 4, x % 4, w % 4, z % 4, 2, 0)
		return (inner_txt, 'RTM_SSE2_INTRINSICS')
	if is_mix_abcd(x) and is_mix_xyzw(y) and is_mix_xyzw(z) and is_mix_xyzw(w):
		inner_txt='''		const __m128 x1x1y0y0 = _mm_shuffle_ps(input1, input0, _MM_SHUFFLE({}, {}, {}, {}));
		return _mm_shuffle_ps(x1x1y0y0, input0, _MM_SHUFFLE({}, {}, {}, {}));'''.format(y % 4, y % 4, x % 4, x % 4, w % 4, z % 4, 2, 0)
		return (inner_txt, 'RTM_SSE2_INTRINSICS')

	if is_mix_abcd(x) and is_mix_abcd(y) and is_mix_abcd(z) and is_mix_xyzw(w):
		inner_txt='''		const __m128 z1z1w0w0 = _mm_shuffle_ps(input1, input0, _MM_SHUFFLE({}, {}, {}, {}));
		return _mm_shuffle_ps(input1, z1z1w0w0, _MM_SHUFFLE({}, {}, {}, {}));'''.format(w % 4, w % 4, z % 4, z % 4, 2, 0, y % 4, x % 4)
		return (inner_txt, 'RTM_SSE2_INTRINSICS')
	if is_mix_abcd(x) and is_mix_abcd(y) and is_mix_xyzw(z) and is_mix_abcd(w):
		inner_txt='''		const __m128 z0z0w1w1 = _mm_shuffle_ps(input0, input1, _MM_SHUFFLE({}, {}, {}, {}));
		return _mm_shuffle_ps(input1, z0z0w1w1, _MM_SHUFFLE({}, {}, {}, {}));'''.format(w % 4, w % 4, z % 4, z % 4, 2, 0, y % 4, x % 4)
		return (inner_txt, 'RTM_SSE2_INTRINSICS')
	if is_mix_abcd(x) and is_mix_xyzw(y) and is_mix_abcd(z) and is_mix_abcd(w):
		inner_txt='''		const __m128 x1x1y0y0 = _mm_shuffle_ps(input1, input0, _MM_SHUFFLE({}, {}, {}, {}));
		return _mm_shuffle_ps(x1x1y0y0, input1, _MM_SHUFFLE({}, {}, {}, {}));'''.format(y % 4, y % 4, x % 4, x % 4, w % 4, z % 4, 2, 0)
		return (inner_txt, 'RTM_SSE2_INTRINSICS')
	if is_mix_xyzw(x) and is_mix_abcd(y) and is_mix_abcd(z) and is_mix_abcd(w):
		inner_txt='''		const __m128 x0x0y1y1 = _mm_shuffle_ps(input0, input1, _MM_SHUFFLE({}, {}, {}, {}));
		return _mm_shuffle_ps(x0x0y1y1, input1, _MM_SHUFFLE({}, {}, {}, {}));'''.format(y % 4, y % 4, x % 4, x % 4, w % 4, z % 4, 2, 0)
		return (inner_txt, 'RTM_SSE2_INTRINSICS')

	if is_mix_xyzw(x) and is_mix_abcd(y) and is_mix_xyzw(z) and is_mix_abcd(w):
		inner_txt='''		const __m128 x0x0y1y1 = _mm_shuffle_ps(input0, input1, _MM_SHUFFLE({}, {}, {}, {}));
		const __m128 z0z0w1w1 = _mm_shuffle_ps(input0, input1, _MM_SHUFFLE({}, {}, {}, {}));
		return _mm_shuffle_ps(x0x0y1y1, z0z0w1w1, _MM_SHUFFLE({}, {}, {}, {}));'''.format(y % 4, y % 4, x % 4, x % 4, w % 4, w % 4, z % 4, z % 4, 2, 0, 2, 0)
		return (inner_txt, 'RTM_SSE2_INTRINSICS')
	if is_mix_abcd(x) and is_mix_xyzw(y) and is_mix_abcd(z) and is_mix_xyzw(w):
		inner_txt='''		const __m128 x1x1y0y0 = _mm_shuffle_ps(input1, input0, _MM_SHUFFLE({}, {}, {}, {}));
		const __m128 z1z1w0w0 = _mm_shuffle_ps(input1, input0, _MM_SHUFFLE({}, {}, {}, {}));
		return _mm_shuffle_ps(x1x1y0y0, z1z1w0w0, _MM_SHUFFLE({}, {}, {}, {}));'''.format(y % 4, y % 4, x % 4, x % 4, w % 4, w % 4, z % 4, z % 4, 2, 0, 2, 0)
		return (inner_txt, 'RTM_SSE2_INTRINSICS')
	if is_mix_xyzw(x) and is_mix_abcd(y) and is_mix_abcd(z) and is_mix_xyzw(w):
		inner_txt='''		const __m128 x0x0y1y1 = _mm_shuffle_ps(input0, input1, _MM_SHUFFLE({}, {}, {}, {}));
		const __m128 z1z1w0w0 = _mm_shuffle_ps(input1, input0, _MM_SHUFFLE({}, {}, {}, {}));
		return _mm_shuffle_ps(x0x0y1y1, z1z1w0w0, _MM_SHUFFLE({}, {}, {}, {}));'''.format(y % 4, y % 4, x % 4, x % 4, w % 4, w % 4, z % 4, z % 4, 2, 0, 2, 0)
		return (inner_txt, 'RTM_SSE2_INTRINSICS')
	if is_mix_abcd(x) and is_mix_xyzw(y) and is_mix_xyzw(z) and is_mix_abcd(w):
		inner_txt='''		const __m128 x1x1y0y0 = _mm_shuffle_ps(input1, input0, _MM_SHUFFLE({}, {}, {}, {}));
		const __m128 z0z0w1w1 = _mm_shuffle_ps(input0, input1, _MM_SHUFFLE({}, {}, {}, {}));
		return _mm_shuffle_ps(x1x1y0y0, z0z0w1w1, _MM_SHUFFLE({}, {}, {}, {}));'''.format(y % 4, y % 4, x % 4, x % 4, w % 4, w % 4, z % 4, z % 4, 2, 0, 2, 0)
		return (inner_txt, 'RTM_SSE2_INTRINSICS')

def rule_neon64_zip1(x, y, z, w):
	if x == 0 and y == 4 and z == 1 and w == 5:
		inner_txt='''		return vzip1q_f32(input0, input1);'''
		return (inner_txt, 'RTM_NEON64_INTRINSICS')
	if x == 4 and y == 0 and z == 5 and w == 1:
		inner_txt='''		return vzip1q_f32(input1, input0);'''
		return (inner_txt, 'RTM_NEON64_INTRINSICS')

def rule_neon64_zip2(x, y, z, w):
	if x == 2 and y == 6 and z == 3 and w == 7:
		inner_txt='''		return vzip2q_f32(input0, input1);'''
		return (inner_txt, 'RTM_NEON64_INTRINSICS')
	if x == 6 and y == 2 and z == 7 and w == 3:
		inner_txt='''		return vzip2q_f32(input1, input0);'''
		return (inner_txt, 'RTM_NEON64_INTRINSICS')

def rule_neon64_uzp1(x, y, z, w):
	if x == 0 and y == 2 and z == 4 and w == 6:
		inner_txt='''		return vuzp1q_f32(input0, input1);'''
		return (inner_txt, 'RTM_NEON64_INTRINSICS')
	if x == 4 and y == 6 and z == 0 and w == 2:
		inner_txt='''		return vuzp1q_f32(input1, input0);'''
		return (inner_txt, 'RTM_NEON64_INTRINSICS')

def rule_neon64_uzp2(x, y, z, w):
	if x == 1 and y == 3 and z == 5 and w == 7:
		inner_txt='''		return vuzp2q_f32(input0, input1);'''
		return (inner_txt, 'RTM_NEON64_INTRINSICS')
	if x == 5 and y == 7 and z == 1 and w == 3:
		inner_txt='''		return vuzp2q_f32(input1, input0);'''
		return (inner_txt, 'RTM_NEON64_INTRINSICS')

def rule_neon64_trn1(x, y, z, w):
	if x == 0 and y == 4 and z == 2 and w == 6:
		inner_txt='''		return vtrn1q_f32(input0, input1);'''
		return (inner_txt, 'RTM_NEON64_INTRINSICS')
	if x == 4 and y == 0 and z == 6 and w == 2:
		inner_txt='''		return vtrn1q_f32(input1, input0);'''
		return (inner_txt, 'RTM_NEON64_INTRINSICS')

def rule_neon64_trn2(x, y, z, w):
	if x == 1 and y == 5 and z == 3 and w == 7:
		inner_txt='''		return vtrn2q_f32(input0, input1);'''
		return (inner_txt, 'RTM_NEON64_INTRINSICS')
	if x == 5 and y == 1 and z == 7 and w == 3:
		inner_txt='''		return vtrn2q_f32(input1, input0);'''
		return (inner_txt, 'RTM_NEON64_INTRINSICS')

def rule_neon_ext(x, y, z, w):
	if is_mix_xyzw(x) and (x + 1) == y and (y + 1) == z and (z + 1) == w:
		inner_txt='''		return vextq_f32(input0, input1, {});'''.format(x % 4)
		return (inner_txt, 'RTM_NEON_INTRINSICS')
	if is_mix_abcd(x) and ((x + 1) % 8 == y) and ((y + 1) % 8) == z and ((z + 1) % 8) == w:
		inner_txt='''		return vextq_f32(input1, input0, {});'''.format(x % 4)
		return (inner_txt, 'RTM_NEON_INTRINSICS')

def rule_neon_rev64(x, y, z, w):
	if x == 1 and y == 0 and z == 3 and w == 2:
		inner_txt='''		(void)input1;
		return vrev64q_f32(input0);'''
		return (inner_txt, 'RTM_NEON_INTRINSICS')
	if x == 5 and y == 4 and z == 7 and w == 6:
		inner_txt='''		(void)input0;
		return vrev64q_f32(input1);'''
		return (inner_txt, 'RTM_NEON_INTRINSICS')

def rule_neon_movn(x, y, z, w):
	if x == y and x == z and x == w and is_mix_xyzw(x):
		inner_txt='''		(void)input1;
		return vmovq_n_f32(vgetq_lane_f32(input0, {}));'''.format(x % 4)
		return (inner_txt, 'RTM_NEON_INTRINSICS')
	if x == y and x == z and x == w and is_mix_abcd(x):
		inner_txt='''		(void)input0;
		return vmovq_n_f32(vgetq_lane_f32(input1, {}));'''.format(x % 4)
		return (inner_txt, 'RTM_NEON_INTRINSICS')

def rule_neon_getset(x, y, z, w):
	if is_mix_abcd(x) and y == 1 and z == 2 and w == 3:
		inner_txt='''		return vsetq_lane_f32(vgetq_lane_f32(input1, {}), input0, 0);'''.format(x % 4)
		return (inner_txt, 'RTM_NEON_INTRINSICS')
	if x == 0 and is_mix_abcd(y) and z == 2 and w == 3:
		inner_txt='''		return vsetq_lane_f32(vgetq_lane_f32(input1, {}), input0, 1);'''.format(y % 4)
		return (inner_txt, 'RTM_NEON_INTRINSICS')
	if x == 0 and y == 1 and is_mix_abcd(z) and w == 3:
		inner_txt='''		return vsetq_lane_f32(vgetq_lane_f32(input1, {}), input0, 2);'''.format(z % 4)
		return (inner_txt, 'RTM_NEON_INTRINSICS')
	if x == 0 and y == 1 and z == 2 and is_mix_abcd(w):
		inner_txt='''		return vsetq_lane_f32(vgetq_lane_f32(input1, {}), input0, 3);'''.format(w % 4)
		return (inner_txt, 'RTM_NEON_INTRINSICS')

	if is_mix_xyzw(x) and y == 5 and z == 6 and w == 7:
		inner_txt='''		return vsetq_lane_f32(vgetq_lane_f32(input0, {}), input1, 0);'''.format(x % 4)
		return (inner_txt, 'RTM_NEON_INTRINSICS')
	if x == 4 and is_mix_xyzw(y) and z == 6 and w == 7:
		inner_txt='''		return vsetq_lane_f32(vgetq_lane_f32(input0, {}), input1, 1);'''.format(y % 4)
		return (inner_txt, 'RTM_NEON_INTRINSICS')
	if x == 4 and y == 5 and is_mix_xyzw(z) and w == 7:
		inner_txt='''		return vsetq_lane_f32(vgetq_lane_f32(input0, {}), input1, 2);'''.format(z % 4)
		return (inner_txt, 'RTM_NEON_INTRINSICS')
	if x == 4 and y == 5 and z == 6 and is_mix_xyzw(w):
		inner_txt='''		return vsetq_lane_f32(vgetq_lane_f32(input0, {}), input1, 3);'''.format(w % 4)
		return (inner_txt, 'RTM_NEON_INTRINSICS')

	if is_mix_xyzw(x) and y == 1 and z == 2 and w == 3:
		inner_txt='''		(void)input1;
		return vsetq_lane_f32(vgetq_lane_f32(input0, {}), input0, 0);'''.format(x % 4)
		return (inner_txt, 'RTM_NEON_INTRINSICS')
	if x == 0 and is_mix_xyzw(y) and z == 2 and w == 3:
		inner_txt='''		(void)input1;
		return vsetq_lane_f32(vgetq_lane_f32(input0, {}), input0, 1);'''.format(y % 4)
		return (inner_txt, 'RTM_NEON_INTRINSICS')
	if x == 0 and y == 1 and is_mix_xyzw(z) and w == 3:
		inner_txt='''		(void)input1;
		return vsetq_lane_f32(vgetq_lane_f32(input0, {}), input0, 2);'''.format(z % 4)
		return (inner_txt, 'RTM_NEON_INTRINSICS')
	if x == 0 and y == 1 and z == 2 and is_mix_xyzw(w):
		inner_txt='''		(void)input1;
		return vsetq_lane_f32(vgetq_lane_f32(input0, {}), input0, 3);'''.format(w % 4)
		return (inner_txt, 'RTM_NEON_INTRINSICS')

	if is_mix_abcd(x) and y == 5 and z == 6 and w == 7:
		inner_txt='''		(void)input0;
		return vsetq_lane_f32(vgetq_lane_f32(input1, {}), input1, 0);'''.format(x % 4)
		return (inner_txt, 'RTM_NEON_INTRINSICS')
	if x == 4 and is_mix_abcd(y) and z == 6 and w == 7:
		inner_txt='''		(void)input0;
		return vsetq_lane_f32(vgetq_lane_f32(input1, {}), input1, 1);'''.format(y % 4)
		return (inner_txt, 'RTM_NEON_INTRINSICS')
	if x == 4 and y == 5 and is_mix_abcd(z) and w == 7:
		inner_txt='''		(void)input0;
		return vsetq_lane_f32(vgetq_lane_f32(input1, {}), input1, 2);'''.format(z % 4)
		return (inner_txt, 'RTM_NEON_INTRINSICS')
	if x == 4 and y == 5 and z == 6 and is_mix_abcd(w):
		inner_txt='''		(void)input0;
		return vsetq_lane_f32(vgetq_lane_f32(input1, {}), input1, 3);'''.format(w % 4)
		return (inner_txt, 'RTM_NEON_INTRINSICS')

	inner_x = 'input0' if is_mix_xyzw(x) else 'input1'
	inner_y = 'input0' if is_mix_xyzw(y) else 'input1'
	inner_z = 'input0' if is_mix_xyzw(z) else 'input1'
	inner_w = 'input0' if is_mix_xyzw(w) else 'input1'

	inner_txt='''		(void)input0; (void)input1;
		const float x = vgetq_lane_f32({}, {});
		const float y = vgetq_lane_f32({}, {});
		const float z = vgetq_lane_f32({}, {});
		const float w = vgetq_lane_f32({}, {});
		return vector_set(x, y, z, w);'''.format(inner_x, x % 4, inner_y, y % 4, inner_z, z % 4, inner_w, w % 4)
	return (inner_txt, 'RTM_NEON_INTRINSICS')

rules = [
	rule_self0,
	rule_self1,
	rule_sse4_blend,
	rule_sse4_insert,
	rule_sse3_moveldup,
	rule_sse3_movehdup,
	rule_sse2_unpacklo,
	rule_sse2_unpackhi,
	rule_sse2_movelh,
	rule_sse2_movehl,
	rule_sse2_shuffle,
	rule_neon64_zip1,
	rule_neon64_zip2,
	rule_neon64_uzp1,
	rule_neon64_uzp2,
	rule_neon64_trn1,
	rule_neon64_trn2,
	rule_neon_ext,
	rule_neon_rev64,
	rule_neon_movn,
	rule_neon_getset,
]

def print_xyzw(x, y, z, w):
	inner_txt = ''

	matching_results = []
	is_match_exclusive = False
	matching_cpp_guards = set()

	for rule in rules:
		result = rule(x, y, z, w)
		if result:
			inner_txt, cpp_guard = result

			if cpp_guard in matching_cpp_guards:
				# We already got a match for this cpp guard
				continue

			if cpp_guard:
				continuation = 'el' if matching_results else ''
				guard_txt = '''\
#{}if defined({})
{}
'''
				inner_txt = guard_txt.format(continuation, cpp_guard, inner_txt)

			matching_results.append(inner_txt)
			matching_cpp_guards.add(cpp_guard)

			if not cpp_guard:
				# A trivial rule without a cpp guard has matched, these are exclusive
				is_match_exclusive = True
				break

	if not matching_results:
		print('No matching rule found for: {}, {}, {}, {}'.format(x, y, z, w))
		sys.exit(1)

	if not is_match_exclusive:
		# Add non-simd fallback
		inner_txt = '''\
#else
		(void)input0; (void)input1;
		return vector4f{{ {}.{}, {}.{}, {}.{}, {}.{} }};
#endif'''

		inner_x = 'input0' if is_mix_xyzw(x) else 'input1'
		inner_y = 'input0' if is_mix_xyzw(y) else 'input1'
		inner_z = 'input0' if is_mix_xyzw(z) else 'input1'
		inner_w = 'input0' if is_mix_xyzw(w) else 'input1'

		selector_x = get_selector(x)
		selector_y = get_selector(y)
		selector_z = get_selector(z)
		selector_w = get_selector(w)

		inner_txt = inner_txt.format(inner_x, selector_x, inner_y, selector_y, inner_z, selector_z, inner_w, selector_w)

		matching_results.append(inner_txt)

	vector_mix_inner_txt = ''.join(matching_results)

	txt ='''	template<>
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4f RTM_SIMD_CALL
		vector_mix<{}, {}, {}, {}>(vector4f_arg0 input0, vector4f_arg1 input1) RTM_NO_EXCEPT
	{{
		// [{}{}{}{}]
{}
	}}
	'''

	print(txt.format(idx_to_mix(x), idx_to_mix(y), idx_to_mix(z), idx_to_mix(w), idx_to_mix_short(x), idx_to_mix_short(y), idx_to_mix_short(z), idx_to_mix_short(w), vector_mix_inner_txt))

def print_xyz_(x, y, z):
	print_xyzw(x, y, z, 0)
	print_xyzw(x, y, z, 1)
	print_xyzw(x, y, z, 2)
	print_xyzw(x, y, z, 3)
	print_xyzw(x, y, z, 4)
	print_xyzw(x, y, z, 5)
	print_xyzw(x, y, z, 6)
	print_xyzw(x, y, z, 7)

def print_xy__(x, y):
	print_xyz_(x, y, 0)
	print_xyz_(x, y, 1)
	print_xyz_(x, y, 2)
	print_xyz_(x, y, 3)
	print_xyz_(x, y, 4)
	print_xyz_(x, y, 5)
	print_xyz_(x, y, 6)
	print_xyz_(x, y, 7)

def print_x___(x):
	print_xy__(x, 0)
	print_xy__(x, 1)
	print_xy__(x, 2)
	print_xy__(x, 3)
	print_xy__(x, 4)
	print_xy__(x, 5)
	print_xy__(x, 6)
	print_xy__(x, 7)

if __name__ == "__main__":
	print_x___(0)
	print_x___(1)
	print_x___(2)
	print_x___(3)
	print_x___(4)
	print_x___(5)
	print_x___(6)
	print_x___(7)
