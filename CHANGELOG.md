# Significant changes per release

## Unreleased

*  Disable fast math with `float_control` pragma under Clang where the target supports it
*  Fix float64 floor, ceil, and symmetric rounding for values between 2^31 and 2^52 with SSE2 without SSE4
*  Fix floor and ceil with SSE2 without SSE4 to keep the sign of a zero result, as the other code paths do
*  Add the SSE4 code path to vector4d ceil
*  Symmetric and banker's rounding keep the sign of the input as std::round and std::nearbyint do, for example round(-0.0) = -0.0
*  Fix float32 banker's rounding for values of 2^31 and larger without SIMD
*  Fix compilation errors with GCC on ARMv7 in vector floor, vector ceil, and the 2x2 transpose macro
*  Fix vector floor and ceil on ARMv7 to keep the sign of a zero result, as the other code paths do
*  Add support for native ARMv7 builds on Linux to `make.py`
*  Add GCC 14, GCC 15, and Clang 19 to 23 to `make.py`
*  Update Google Benchmark to 1.9.5
*  Use float64 SIMD on ARM64: vector4d, quatd, mask4d, and mask4q are now two 2-lane registers, as with SSE2
*  Use FMA on ARM64 for float64 multiply-add and quaternion multiplication, as float32 does: the results can differ in the last bits
*  Pass qvd and qvsd by register on ARM64
*  Add support for WebAssembly SIMD128: compile with `-msimd128` to enable it, RTM then defines `RTM_WASM128_INTRINSICS`
*  Compile the Emscripten unit tests with WebAssembly SIMD by default in `make.py`, use `-nosimd` for the scalar code path
*  Fix symmetric rounding for values near 0.5 and for odd values above 2^23 (float32) or 2^52 (float64) on all code paths, for example round(0.49999997) = 0.0
*  Use the ARM64 round instruction (half away from zero) for vector and scalar symmetric rounding

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

