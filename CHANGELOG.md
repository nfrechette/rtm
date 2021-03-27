# Significant changes per release

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

