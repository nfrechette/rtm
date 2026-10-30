# Getting started

In order to contribute to RTM you will first need to setup your environment.

## Setting up your environment

### Windows, Linux, and OS X for x86 and x64

1. Install *CMake 3.2* or higher (*3.14* for Visual Studio 2019, *4.2* for Visual Studio 2026, or *3.10* on OS X with *Xcode 10*), *Python 2.7 or 3*, and the proper compiler for your platform.
2. Execute `git submodule update --init` to get the files of external submodules (e.g. Catch2).
3. Generate the IDE solution with: `python make.py`  
   The solution is generated under `./build`
4. Build the IDE solution with: `python make.py -build`
5. Run the unit tests with: `python make.py -unit_test`
6. Build and run benchmarks with the `-bench` switch
7. Run the clang static analyzer on the public headers with: `python make.py -analyze`

On all three platforms, *AVX* support can be enabled by using the `-avx` switch and *AVX2* with `-avx2`. Intrinsic usage can be turned off with `-nosimd`.

The `-analyze` switch runs the clang static analyzer on the include validation files, one file for each public header. It analyzes the functions in the headers and enables the array bound checker. It needs clang and a Makefile or Ninja generator: use the default compiler on OS X, or `-compiler clang19` (or a later version) on Linux. It prints each issue one time and returns an error code if it finds an issue. On GitHub Actions, it also prints each issue as an annotation on the line of the issue. The other switches apply: for example, `-nosimd` analyzes the scalar code path.

### Linux riscv64

For native *Linux on riscv64*, the steps are identical to *x86 and x64* and the generic scalar path is selected automatically. You can generate build files with: `python make.py -cpu riscv64`

Cross-compilation to `riscv64` is not handled by `make.py`. Use CMake directly with a proper RISC-V compiler and toolchain file instead.

### Linux ARMv7

For native *Linux on ARMv7*, the steps are identical to *x86 and x64* and `make.py` selects the `armv7` architecture automatically. You can generate build files with: `python make.py -cpu armv7`

With SIMD enabled, the build adds `-mfpu=neon` because the Linux `armhf` compilers do not enable NEON by default.

### Windows ARM64

For *Windows on ARM64*, the steps are identical to *x86 and x64* but you will need *CMake 3.13 or higher* and you must provide the architecture on the command line: `python make.py -compiler vs2017 -cpu arm64`

### Windows ARM64EC

For *Windows on ARM64EC*, the steps are identical to *Windows ARM64*. ARM64EC needs *Visual Studio 2019* or later. You must provide the architecture on the command line: `python make.py -compiler vs2022 -cpu arm64ec`

On an x64 host, `make.py` cross-compiles. It does not register the unit tests with CTest, thus `-unit_test` does not run them. To run the unit tests, use an ARM64 host.

### Android

For *Android*, the steps are identical to *Windows, Linux, and OS X* but you also need to install *Android NDK 21* (or higher). Use `-compiler android`. The default architecture is `arm64`. Use `-cpu armv7` for 32-bit ARM. Make sure that the `adb` executable is in your `PATH`. The `-tests_matching` switch is not supported on Android.

On *Windows*, the build uses `gradle`. The `-unit_test` and `-bench` switches install the app on the device and run it.

*Android Studio v3.5* can be used to launch and debug. After running *CMake* to build and generate everything, the *Android Studio* projects can be found under the `./build` directory.

Note that running the benchmark on Android can be done but it currently involves a number of manual steps.

On *OS X* and *Linux*, the build uses the CMake toolchain of the NDK. `make.py` finds the NDK with the `ANDROID_NDK_HOME` or `ANDROID_NDK_ROOT` environment variable. If neither is set, it uses the most recent version in the `ndk` directory of `ANDROID_HOME` or `ANDROID_SDK_ROOT`. The `-unit_test` switch copies the unit test executable to `/data/local/tmp` on the device with `adb` and runs it there. The output of the unit tests shows in the terminal, and `make.py` returns an error code if a test fails. Use the `ANDROID_SERIAL` environment variable to select a device when more than one is connected. The benchmark is not supported with this build.

For example, on *OS X* with *Homebrew*:

```bash
brew install --cask android-platform-tools android-ndk
export ANDROID_NDK_HOME="/opt/homebrew/share/android-ndk"
python3 make.py -compiler android -build -unit_test
```

### iOS

For *iOS*, the steps are identical to the other platforms but due to code signing, you will need to perform the builds from *Xcode* manually. Note that this is only an issue if you attempt to use the unit tests locally.

In order to run these manually:

*  Open the *Xcode* project for the appropriate tool
*  In the project settings, enable automatic code signing and select your development team
*  Build and run on your device

Note that *iOS* builds have never been tested on an emulator.

### Emscripten

Emscripten support currently only has been tested on OS X and Linux. To use it, make sure to install a recent version of Emscripten SDK 1.39.11+.

By default, `make.py` compiles the Emscripten unit tests with WebAssembly SIMD (`-msimd128`). To test the scalar code path, use `-nosimd`. The unit tests run with Node.js, which must support WebAssembly SIMD (version 16.4 or later).

## Commit message format

This library uses the [angular.js message format](https://github.com/angular/angular.js/blob/master/DEVELOPERS.md#commits) and it is enforced with commit linting through every pull request.
