# Significant changes per release

## 2.4.0

### Breaking changes

*  Remove the deprecated `float_traits` type: use `related_types`
*  Remove the deprecated implicit coercion to `scalarf`, `scalard`, `vector4f`, and `vector4d`: use the functions with the `as_scalar` and `as_vector` suffixes
*  Code that forward declares struct qvf, qvd, qvsf, qvsd, qvvf, or qvvd does not compile: include rtm/fwd.h instead

### Changes in behavior

*  Symmetric and banker's rounding keep the sign of the input as std::round and std::nearbyint do, for example round(-0.0) = -0.0
*  Use a float64 default threshold in quat_near_identity (double): 2.82729655E-7 rad (an error of 1.0E-14 on [w]) replaces the float32 value of 0.00284714461 rad. To keep the previous behavior, give 0.00284714461 as the threshold
*  `vqv_slerp` and the deprecated `qvv_slerp` interpolate the scale with eerp instead of lerp. If the two scales of a component have a different sign or if one scale is zero, they use lerp for that component. `vqv_lerp` does not change
*  Use FMA on ARM64 for float64 multiply-add and quaternion multiplication, as float32 does: the results can differ in the last bits

### Deprecations

*  Rename qvf and qvd to vqf and vqd, and the qv functions to vq: the name follows the order of operations (rotation, then translation). The new headers are rtm/vqf.h and rtm/vqd.h
*  Rename qvsf and qvsd to vqsf and vqsd, and the qvs functions to vqs. The new headers are rtm/vqsf.h and rtm/vqsd.h
*  Rename qvvf and qvvd to vqvf and vqvd, and the qvv functions to vqv. The new headers are rtm/vqvf.h and rtm/vqvd.h
*  Deprecate the qv, qvs, and qvv names: the types, the functions, matrix_from_qv, matrix_from_qvs, matrix_from_qvv, the `_arg` types, related_types::qv, related_types::qvs, related_types::qvv, and the headers rtm/qvf.h, rtm/qvd.h, rtm/qvsf.h, rtm/qvsd.h, rtm/qvvf.h, and rtm/qvvd.h. Use the vq, vqs, and vqv names. To be removed in 2.5

### New features

*  Add the VQM transform type, `vqmf` and `vqmd`: a rotation quaternion, a translation, and a 3x3 scale/shear matrix. Unlike VQV, VQM can hold shear and forms a group. Include `rtm/vqmf.h` or `rtm/vqmd.h`. `rtm/types.h` declares `vqmf` and `vqmd`, `rtm/type_traits.h` adds `related_types::vqm`, and the functions take the `vqmf_arg` and `vqmd_arg` types, with the same choices as the 3x4 matrices
*  `vqm_from_matrix` finds the rotation with the Gram-Schmidt process, and the scale/shear part contains the reflection, if any. `vqm_slerp` interpolates the scale (the diagonal of the scale/shear part) with eerp and the remaining shear with lerp. `vqm_lerp` interpolates the scale/shear part with lerp. A `vqm_inverse` overload takes a fallback scale
*  A VQM uses 4 registers (64 bytes for vqmf): the [w] lanes of the translation and of the first two rows hold the third row of the scale/shear part. Use the vqm accessors, not the members
*  A VQM stores a flag in the m22 slot when the scale/shear part is exactly the identity: a quiet NaN with a fixed payload. `vqm_identity`, `vqm_set`, `vqm_set_scale`, and `vqm_set_scale_shear` set the flag. `vqm_mul`, `vqm_mul_point3`, `vqm_mul_vector3`, `vqm_inverse`, `vqm_lerp`, and `vqm_slerp` then skip the scale/shear part, and keep the flag when the result has the identity scale/shear part. When the scale/shear part of the second input of `vqm_mul` is the identity, the result has the exact scale/shear part of the first input
*  Add `scalar_exp`, `scalar_log`, `vector_exp`, and `vector_log` for float32 and float64. All the code paths use the same approximations (Cephes): for the same input, the scalar and the vector functions give the same result, and the result does not depend on the C runtime library. The results can differ in the last bit on ARM because the code uses FMA there. The largest error that we measured is 1 ulp for float32 and 2 ulp for float64
*  Add `scalar_eerp` and `vector_eerp` for float32 and float64: the exponential interpolation `start * (end / start)^alpha`
*  Add `quat_add`, `quat_select`, and the `quat_mul` overloads that multiply a quaternion by a scalar
*  Add `mask_true`, `mask_false`, and `mask_not`
*  Add `vector_not_equal`, `vector_any_not_equal`, and `vector_all_not_equal`, with their 2 and 3 component variants
*  Add `vector_distance_squared3`
*  Add the transpose macros `RTM_MATRIXF_TRANSPOSE_2X2`, `RTM_MATRIXD_TRANSPOSE_2X2`, `RTM_MATRIXD_TRANSPOSE_3X4`, `RTM_MATRIXD_TRANSPOSE_4X3`, and `RTM_MATRIXD_TRANSPOSE_4X4`
*  Add `rtm/config.h`: the header lists and documents the macros that configure RTM
*  Add `RTM_NO_BIT_CAST` to not use `std::bit_cast` with C++20 and later. RTM also defines it when the `<bit>` header is not available
*  Add an API reference site: Doxygen reads the `///` comments, and MkDocs makes the site for develop and for each release

### Platforms and tools

*  Add support for WebAssembly SIMD128: compile with `-msimd128` to enable it, RTM then defines `RTM_WASM128_INTRINSICS`
*  Add support for ARM64EC with MSVC: RTM defines `RTM_ARCH_ARM64` and uses the ARM64 NEON code path, and `make.py` adds `-cpu arm64ec`
*  Add support for VS2026: add `vs2026` and `vs2026-clang` to `make.py` and add `RTM_COMPILER_MSVC_2026`
*  Use `__vectorcall` with Clang on Windows, as with MSVC
*  Add GCC 14, GCC 15, and Clang 19 to 23 to `make.py`
*  Add support for native ARMv7 builds on Linux to `make.py`
*  Add support for Android builds on OS X and Linux to `make.py`: the build uses the CMake toolchain of the NDK and `-unit_test` runs the unit tests on the device with `adb`
*  Compile the Emscripten unit tests with WebAssembly SIMD by default in `make.py`, use `-nosimd` for the scalar code path
*  Add the `-analyze` action to `make.py` to run the clang static analyzer on the public headers
*  Update Google Benchmark to 1.9.5

### Fixes

*  Disable fast math with `float_control` pragma under Clang where the target supports it
*  Fix symmetric rounding for values near 0.5 and for odd values above 2^23 (float32) or 2^52 (float64) on all code paths, for example round(0.49999997) = 0.0
*  Fix float64 floor, ceil, and symmetric rounding for values between 2^31 and 2^52 with SSE2 without SSE4
*  Fix floor and ceil with SSE2 without SSE4 to keep the sign of a zero result, as the other code paths do
*  Fix float32 banker's rounding for values of 2^31 and larger without SIMD
*  Fix compilation errors with GCC on ARMv7 in vector floor, vector ceil, and the 2x2 transpose macro
*  Fix vector floor and ceil on ARMv7 to keep the sign of a zero result, as the other code paths do
*  Fix `quat_slerp` when the two quaternions are equal or nearly equal: the result was NaN. When the angle is near zero, the function uses the weights (1.0 - alpha) and alpha
*  Fix quat_mul (quatd, scalard) with SSE2: the function used lane 1 of the scalar to multiply [y] and [w]
*  Fix the argument type of the second input of vq_slerp, vqs_lerp, vqs_lerp_no_scale, vqs_slerp, quat_slerp (double), and vector_neg_mul_sub (vector4d, scalard)
*  Fix vector_cast and quat_cast (float64 to float32) with SSE2 and MSVC 19.34 to 19.44: when [xy] and [zw] are the same constant, the compiler sets [zw] to 0
*  Fix vector_atan2 (vector4d) with SSE2 and MSVC 19.51 for x64: the compiler uses the [xy] result for [zw]
*  Fix vector_mix and matrix_from_scale with MSVC 19.44 and 19.51 for x86 with AVX: when the mix takes only [x] from a zero input, the compiler removes the blend
*  Fix an internal compiler error with MSVC 19.44 for arm64 in some functions that call quat_set_w (quatd, double)
*  Detect the C++ version from `_MSVC_LANG` first with MSVC: by default, MSVC sets `__cplusplus` to an old value

### Performance and accuracy

*  Use float64 SIMD on ARM64: vector4d, quatd, mask4d, and mask4q are now two 2-lane registers, as with SSE2
*  Optimize many float64 functions with SSE2, vector_mix with SSE2 and NEON, the unaligned loads and stores, and vector_cross3 with NEON
*  Add the SSE4 code path to vector4d ceil
*  Use the ARM64 round instruction (half away from zero) for vector and scalar symmetric rounding
*  Pass vqd and vqsd by register on ARM64
*  Pass vqvd, matrix3x3d, matrix3x4d, and matrix4x4d by reference on ARM64 to remove a copy to the stack
*  Pass the third vector4d, quatd, mask4d, and mask4q argument by register with `__vectorcall`
*  Pass the vqd or vqsd of vq_mul_point3, vqs_mul_point3, and vqs_mul_point3_no_scale by register with `__vectorcall`
*  Pass the vector4d of matrix_set_axis (double) by register with `__vectorcall` and on ARM64
*  Pass the vector4d of vqv_inverse (double) and qvv_inverse (double) by register with `__vectorcall`
*  Use the type arg aliases in the casts from double to float types
*  Use a square root and a division instead of the reciprocal square root estimate in float32 quat_normalize and quat_lerp with SSE2: the results are more accurate and do not change between Intel and AMD CPUs
*  Use a division instead of the reciprocal estimate in vector_reciprocal on ARMv7: the results are more accurate
*  Use a square root instead of two reciprocals in quat_from_matrix
*  Divide by the length instead of a multiplication by the reciprocal length to normalize float32 vectors and quaternions with SSE2 and WebAssembly SIMD: the code is faster and more accurate. This applies to vector_normalize, vector_normalize2, vector_normalize3, quat_normalize, quat_lerp, quat_to_axis_angle, quat_get_axis, and the functions that call them
*  Divide by the sine of the half angle instead of a multiplication by its reciprocal in float32 quat_slerp with SSE2 and WebAssembly SIMD
*  Multiply by the reciprocal length instead of two vector divisions in float64 quat_normalize: the code is faster
*  Use one division instead of a reciprocal and a multiplication to scale the result of quat_rotation_log
*  Compute sin(half angle) in `quat_slerp` as sqrt((1 - cos) * (1 + cos)) instead of sqrt(1 - cos^2): the result is more accurate for small angles. For example, the float32 error for an angle of 0.0336 rad is 1.0E-7 instead of 3.0E-5
*  Replicate a lane with one NEON dup instruction in float32 vector_dup_x/y/z/w and when vector_mix uses the same lane four times: on ARMv7, GCC previously moved the lane through a general purpose register or used a table lookup (vtbl)

## 2.3.1

*  Fix missing macro error when deprecation is disabled

## 2.3.0

*  Added missing 2D/4D variants of some vector arithmetic
*  Added support for Windows min/max macros
*  Added vector_get_component/vector_set_component
*  Added matrix_get_axis/matrix_set_axis and component versions
*  Added qv/qvs/qvv from matrix
*  Improved type traits
*  Improved code generation for float64 types
*  Added camera related matrix utilities
*  Deprecated scalar/vector coersion for some arithmetic functions in favor of `as_scalar` and `as_vector` suffixes
*  Other improvements and cleanup

## 2.2.1

*  Fix matrix 4x4 multiplication and add unit tests

## 2.2.0

*  Added support for Clang 15
*  Added support for GCC 12 and 13
*  Added support for XCode 14
*  Added support for VS2022
*  CI now uses docker where possible
*  Updated Catch2 to 2.13.7
*  Tons of static analysis fixes and polish
*  Added new QV transform type
*  Added new QVS transform type
*  Tons of new minor additions
*  Tons of optimizations, especially on ARM64
*  Many other improvements and cleanup

## 2.1.5

*  Force macro expansion in version namespace identifier

## 2.1.4

*  Add support for clang 12, 13, and 14
*  Add support for GCC 11
*  Add support for XCode 12 and 13
*  Add support for Arm64 development on OS X and Linux
*  Misc CI improvements

## 2.1.3

*  Add versioned namespace to allow multiple versions to coexist within a binary

## 2.1.2

*  Fix MSVC/clang static analysis warnings
*  Improve MSVC ARM64 intrinsic support detection

## 2.1.1

*  Avoid enabling NEON when just `__arm__` is present

## 2.1.0

*  Force inlined most functions to improve MSVC code generation
*  Migrated from Travis to GitHub Actions
*  Added support for clang 11
*  Lots of minor additions, fixes, and improvements

## 2.0.0

*  Added support for GCC10, clang8, clang9, clang10, VS 2019 clang, and emscripten
*  Added a lot of matrix math
*  Added trigonometric functions (scalar and vector)
*  Angle types have been removed
*  Lots of optimizations and improvements
*  Tons of cleanup to ensure a consistent API

## 1.1.0

*  Added support for Windows ARM64
*  Added support for VS2019, GCC9, clang7, and Xcode 11
*  Added support for Intel FMA and ARM64 FMA
*  Many optimizations, minor fixes, and cleanup

## 1.0.0

Initial release migrating and improving the code from the Animation Compression Library.

