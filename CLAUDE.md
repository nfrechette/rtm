# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

Realtime Math (RTM) is a **header-only C++11** SIMD math library (vectors, quaternions, matrices, QV/QVS/QVV transforms) for games/realtime apps. All library code lives in `includes/rtm/`; there is nothing to link. The only compiled code is unit tests (`tests/`) and benchmarks (`tools/bench/`). Code must stay valid C++11 (CI also tests 14/17/20).

## Build & test

Everything goes through `make.py` (wraps CMake + CTest). Run `git submodule update --init` first (Catch2, Google Benchmark in `external/`). Output goes to `./build`.

```bash
python3 make.py                      # generate project files only
python3 make.py -build -unit_test    # build + run all unit tests (Release by default)
python3 make.py -build -unit_test -config Debug
python3 make.py -unit_test -tests_matching "quat"   # ctest --tests-regex; tests are discovered per Catch2 TEST_CASE
python3 make.py -clean -build        # clean rebuild
python3 make.py -build -bench        # build benchmarks
```

Useful variants: `-compiler {osx,clang18,gcc13,vs2022,...}`, `-cpu {x64,x86,arm64,...}`, `-cpp_version {11,14,17,20}`, `-avx`, `-avx2`, `-nosimd` (scalar fallback path), `-vector_mix_test` (enables the slow exhaustive `vector_mix` tests via `RTM_IMPL_WITH_VECTOR_MIX_TESTS`). Changing these flags regenerates CMake; use `-clean` if the cache seems stale.

The test executable can also be run directly with Catch2 filters: `./build/bin/rtm_unit_tests "[quat]"` (`-build` installs it into `build/bin`).

Builds use `-Wall -Wextra -Werror` (GCC/Clang) and `/W4 /WX` (MSVC) -- warnings break the build. Tests define `RTM_ON_ASSERT_THROW` so asserts can be caught.

`tests/validate_includes` generates one `.cpp` per public header (`includes/rtm/*.h`, `includes/rtm/packing/*.h`) to verify each header compiles standalone -- every public header must include everything it uses.

## Architecture & conventions

- **Three code paths per function**: nearly every function has `#if defined(RTM_SSE2_INTRINSICS)` / `#elif defined(RTM_NEON_INTRINSICS)` / `#else` scalar fallback branches. Changes must be made (and stay consistent) across all paths; `-nosimd` exercises the scalar path. Feature/arch/compiler detection lives in `impl/detect_*.h`.
- **Type naming suffixes**: `f`=float32, `d`=float64, `i`=int32, `q`=int64 (e.g. `vector4f`, `quatd`, `mask4i`). Float and double variants live in separate headers (`vector4f.h` / `vector4d.h`) with largely mirrored APIs; shared helpers go in `impl/*_common.h`.
- **C-style API** (`vector_add(a, b)`, `quat_mul(...)`) for inlining/codegen. Function declaration boilerplate: `RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE <ret> RTM_SIMD_CALL name(args) RTM_NO_EXCEPT`.
- **Argument-passing aliases**: SIMD params use `vector4f_arg0..N`, `quatf_arg0`, etc. rather than raw types; these are defined per ABI in `impl/type_args.*.impl.h` (vectorcall, NEON, x64 gcc/clang, other) to maximize register passing. Use the correct arg index for the parameter position.
- **Header skeleton**: every header wraps content in `RTM_IMPL_FILE_PRAGMA_PUSH`/`POP` (forces non-fast-math) and `namespace rtm { RTM_IMPL_VERSION_NAMESPACE_BEGIN ... RTM_IMPL_VERSION_NAMESPACE_END }`. New types should also be forward declared in `fwd.h`.
- **Implementation details**: put code that is not public API in the nested `rtm_impl` namespace and/or in a header under `includes/rtm/impl/`. Client code must not use either.
- **Public API comments**: all public API must have a comment. Put it in the `//////` banner block above the function or type, as the existing headers do.
- **Return-type coercion helpers**: some functions return small `rtm_impl::*_impl`/`*_loader` structs with implicit conversion operators so one name can yield `float`, `scalarf`, or `vector4f` at the call site. Newer API prefers explicit `_as_scalar` / `_as_vector` suffixes; the implicit scalar coercions are deprecated (`RTM_DEPRECATED`, slated for removal in 2.4).
- **Math conventions**: row vectors (`v' = v * M`, `local_to_world = local_to_object * object_to_world`); left-handed frame with X forward, Y right, Z up; quaternion `[xyz]` = vector part, `[w]` = scalar part. Functions with a numeric suffix (`vector_dot3`) operate on fewer lanes and leave unused lanes undefined.
- **Asserts** (`impl/error.h`) are stripped by default; `RTM_ON_ASSERT_ABORT`, `RTM_ON_ASSERT_THROW`, or `RTM_ON_ASSERT_CUSTOM` enable them.
- `includes/rtm/experimental/` holds unstable APIs (e.g. VQM) not covered by include validation.

## Tests

Tests are Catch2 (`tests/sources/`). Float/double coverage is shared through templated `*_impl.h` files (e.g. `test_vector4_impl.h`) instantiated from thin `test_*f.cpp` / `test_*d.cpp` files with per-type thresholds (looser under `RTM_NO_INTRINSICS`). Platform-specific runners live in `tests/main_{generic,android,ios,emscripten}`.

## Communication style

Write in ASD-STE100 (Simplified Technical English). This applies to what you say to the user, to
commit messages, to pull requests, and to comments and documents in the tree. Identifiers in code keep their names;
the specification controls prose only.

- **One word, one meaning, one part of speech.** Do not use a noun as a verb. Write "measure the
  function", not "benchmark the function".
- **Use the approved word when there is one.** Write "start", not "kick off"; "remove", not "strip
  out"; "let", not "permit". The technical nouns of this tree -- vector, quaternion, lane, mask,
  register, intrinsic -- are technical names and are approved by that rule.
- **Active voice, and name the agent.** Write "the function sets the [w] component", not "the [w]
  component is set". Use the passive voice only when the agent is not known or is not important.
- **Short sentences.** Maximum 20 words in an instruction, 25 in a description. Write one
  instruction in one sentence.
- **Do not use the -ing form as a noun or as an adjective.** Write "the code that loads", not
  "the loading code"; write "to clear the lane", not "clearing the lane".
- **Keep the articles and the helper words.** Write "the shuffle on the input vector", not "shuffle
  on input vector". Text that drops them is not faster to read.
- **Present tense, no figures of speech, and no words that carry an opinion.** A number is
  "5% lower", not "disappointing".
- **No Unicode characters** in comments, string literals, documents, commit messages, or pull
  requests. Use only ASCII. For example, use `<->` instead of `↔`, `--` instead of `—`, `->`
  instead of `→`. STE does not permit a Unicode dash either.

## Commit messages and pull requests

Commits and pull requests use the
[Angular commit message format](https://github.com/angular/angular/blob/main/CONTRIBUTING.md#commit).
No tool enforces the format, so check each message yourself. The main branch is `develop`.

```
<type>(<scope>): <summary>

<body>

<footer>
```

- **Header** (required): `<type>(<scope>): <summary>`. The scope is optional.
  - `type` is one of: `build`, `ci`, `docs`, `feat`, `fix`, `perf`, `refactor`, `test`. This repo
    also uses `chore` (for example `chore(ci): ...`).
  - `scope` names the area of the change. Scopes in the history include `ci`, `tools`, `tests`,
    `macros`, and a compiler name such as `clang`.
  - `summary` uses the imperative, present tense ("add", not "added" or "adds"). Do not capitalize
    the first letter. Do not put a period at the end.
- **Body**: use the imperative, present tense. Explain the motivation for the change. Compare the
  new behavior with the previous behavior when that helps.
- **Footer**: put breaking changes (`BREAKING CHANGE: <description>`), deprecations
  (`DEPRECATED: <description>`), and issue references (`Fixes #<number>`) here.

A pull request is merged with a merge commit. The pull request title becomes the header of that
merge commit, with the pull request number added (for example `feat: arm64ec support (#247)`).
Thus the pull request title must also use the `<type>(<scope>): <summary>` format, and each commit
in the pull request must use it too. The pull request description follows the
body rules above. Record notable changes in `CHANGELOG.md`.
