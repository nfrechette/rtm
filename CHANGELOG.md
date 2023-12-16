# Significant changes per release

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

