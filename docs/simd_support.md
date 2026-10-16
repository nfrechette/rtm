# SIMD support

RTM does not use the reciprocal estimate and the reciprocal square root estimate instructions (for example `rcpps`, `rsqrtps`, and `vrecpe`). The precision of these instructions is not the same on all CPUs. RTM uses a division and a square root instead. Modern CPUs hide most of the latency of the division and the square root, and the code has fewer instructions.

## x86 and x64

Various versions of SSE are supported: SSE2, SSE3, SSE4, AVX, AVX2, and FMA.

*Note that even when FMA is enabled, its intrinsics are not used because they appear slower on at least Haswell and Ryzen.*

## ARM

Both ARM NEON and ARM64 NEON are supported.

## WebAssembly

WebAssembly SIMD128 is supported. To enable it, compile with `-msimd128`. Clang then defines `__wasm_simd128__` and RTM defines `RTM_WASM128_INTRINSICS`. Without `-msimd128`, RTM uses the scalar code path.

*  The relaxed SIMD instructions are not used. Thus, the results do not change between the browsers and the CPUs.
*  `vector_min` and `vector_max` use the `pmin` and `pmax` instructions. They have the same semantics as `std::min` and `std::max`, and they compile to a single instruction on x64.

