[![CLA assistant](https://cla-assistant.io/readme/badge/nfrechette/rtm)](https://cla-assistant.io/nfrechette/rtm)
[![All Contributors](https://img.shields.io/github/all-contributors/nfrechette/rtm)](#contributors-)
[![Build status](https://ci.appveyor.com/api/projects/status/7eh9maq9a721e5on/branch/develop?svg=true)](https://ci.appveyor.com/project/nfrechette/rtm)
[![Build status](https://github.com/nfrechette/rtm/workflows/build/badge.svg)](https://github.com/nfrechette/rtm/actions)
[![Sonar Status](https://sonarcloud.io/api/project_badges/measure?project=nfrechette_rtm&metric=alert_status)](https://sonarcloud.io/dashboard?id=nfrechette_rtm)
[![GitHub release](https://img.shields.io/github/release/nfrechette/rtm.svg)](https://github.com/nfrechette/rtm/releases)
[![GitHub license](https://img.shields.io/badge/license-MIT-blue.svg)](https://raw.githubusercontent.com/nfrechette/rtm/master/LICENSE)
[![Conan](https://img.shields.io/conan/v/rtm)](https://conan.io/center/recipes/rtm)
[![Discord](https://img.shields.io/discord/691048241864769647?label=discord)](https://discord.gg/UERt4bS)

# Realtime Math

This library is geared towards realtime applications that require their math to be as fast as possible. Much care was taken to maximize inlining opportunities and for code generation to be optimal when a function isn't inlined by passing values in registers whenever possible. It contains 3D and 4D arithmetic commonly used in video games and realtime applications.

It offers an alternative to [GLM](https://github.com/g-truc/glm) and [DirectX Math](https://github.com/Microsoft/DirectXMath). See [here](https://nfrechette.github.io/2019/01/19/introducing_realtime_math/) for a comparison with similar libraries.

## Philosophy

Much thought was put into designing the library for it to be as flexible and powerful as possible. To this end, the following decisions were made:

*  The library consists of **100% C++11** header files and is thus easy to integrate in any project
*  The interface follows C-style conventions to ensure optimal code generation
*  Both *float32* and *float64* arithmetic are supported
*  Row vectors are used
*  See [here](./docs/api_conventions.md) for more details

## Supported platforms

*  Windows VS2015 x86 and x64
*  Windows (VS2017 to VS2022) x86, x64, and ARM64
*  Windows (VS2017 to VS2022) with clang x86 and x64
*  Linux (gcc 5 to 13) x86 and x64
*  Linux (clang 4 to 15) x86 and x64
*  OS X (Xcode 12.5, 13.2, 14.2) x64 and ARM64
*  Android (NDK 21) ARMv7-A and ARM64
*  iOS (Xcode 10.3, 11.7, 12.5, 13.2, 14.2) ARM64
*  Emscripten (1.39.11) WASM
*  MSYS2 x64

The above supported platform list is only what is tested every release but if it compiles, it should work just fine.

## Getting started

This library is **100%** headers as such you just need to include them in your own project to start using it. However, if you wish to run the unit tests or to contribute to RTM head on over to the [getting started](./docs/getting_started.md) section in order to setup your environment and make sure to check out the [contributing guidelines](CONTRIBUTING.md).

You can install `rtm` with [Conan](https://conan.io/center/recipes/rtm).

## External dependencies

You don't need anything else to get started: everything is self contained.
See [here](./external) for details.

## License, copyright, and code of conduct

This project uses the [MIT license](LICENSE).

Copyright (c) 2018 Nicholas Frechette & Realtime Math contributors

This project was started from the math code found in the [Animation Compression Library](https://github.com/nfrechette/acl) v1.1.0 and it retains the copyright of the original contributors.

Please note that this project is released with a [Contributor Code of Conduct](CODE_OF_CONDUCT.md). By participating in this project you agree to abide by its terms.

## Contributors ✨

Thanks goes to these wonderful people ([emoji key](https://allcontributors.org/docs/en/emoji-key)):

<!-- ALL-CONTRIBUTORS-LIST:START - Do not remove or modify this section -->
<!-- prettier-ignore-start -->
<!-- markdownlint-disable -->
<table>
  <tbody>
    <tr>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/CodyDWJones"><img src="https://avatars.githubusercontent.com/u/28773740?v=4?s=100" width="100px;" alt="CodyDWJones"/><br /><sub><b>CodyDWJones</b></sub></a><br /><a href="https://github.com/nfrechette/rtm/commits?author=CodyDWJones" title="Code">💻</a> <a href="#platform-CodyDWJones" title="Packaging/porting to new platform">📦</a> <a href="#infra-CodyDWJones" title="Infrastructure (Hosting, Build-Tools, etc)">🚇</a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/mwerschy"><img src="https://avatars.githubusercontent.com/u/6616804?v=4?s=100" width="100px;" alt="Malte Werschy"/><br /><sub><b>Malte Werschy</b></sub></a><br /><a href="https://github.com/nfrechette/rtm/commits?author=mwerschy" title="Code">💻</a> <a href="#platform-mwerschy" title="Packaging/porting to new platform">📦</a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://gitlab.com/intelligide"><img src="https://avatars.githubusercontent.com/u/12529837?v=4?s=100" width="100px;" alt="Yoann Potinet"/><br /><sub><b>Yoann Potinet</b></sub></a><br /><a href="#infra-intelligide" title="Infrastructure (Hosting, Build-Tools, etc)">🚇</a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/operasfantom"><img src="https://avatars.githubusercontent.com/u/31903947?v=4?s=100" width="100px;" alt="Pavel Iatchenii"/><br /><sub><b>Pavel Iatchenii</b></sub></a><br /><a href="https://github.com/nfrechette/rtm/commits?author=operasfantom" title="Code">💻</a> <a href="#maintenance-operasfantom" title="Maintenance">🚧</a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/GermanAizek"><img src="https://avatars.githubusercontent.com/u/21138600?v=4?s=100" width="100px;" alt="Herman Semenov"/><br /><sub><b>Herman Semenov</b></sub></a><br /><a href="https://github.com/nfrechette/rtm/commits?author=GermanAizek" title="Code">💻</a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/valgur"><img src="https://avatars.githubusercontent.com/u/1778160?v=4?s=100" width="100px;" alt="Martin Valgur"/><br /><sub><b>Martin Valgur</b></sub></a><br /><a href="https://github.com/nfrechette/rtm/commits?author=valgur" title="Code">💻</a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/Kethers"><img src="https://avatars.githubusercontent.com/u/49612587?v=4?s=100" width="100px;" alt="Kethers"/><br /><sub><b>Kethers</b></sub></a><br /><a href="https://github.com/nfrechette/rtm/commits?author=Kethers" title="Code">💻</a></td>
    </tr>
  </tbody>
</table>

<!-- markdownlint-restore -->
<!-- prettier-ignore-end -->

<!-- ALL-CONTRIBUTORS-LIST:END -->

This project follows the [all-contributors](https://github.com/all-contributors/all-contributors) specification. Contributions of any kind welcome!
