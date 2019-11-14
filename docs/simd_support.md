# SIMD support

## x86 and x64

Various versions of SSE are supported: SSE2, SSE3, SSE4, AVX, AVX2, and FMA.

*Note that even when FMA is enabled, its intrinsics are not used because they appear slower on at least Haswell and Ryzen.*

## ARM

Both ARM NEON and ARM64 NEON are supported.

