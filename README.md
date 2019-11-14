[![CLA assistant](https://cla-assistant.io/readme/badge/nfrechette/rtm)](https://cla-assistant.io/nfrechette/rtm)
[![Build status](https://ci.appveyor.com/api/projects/status/7eh9maq9a721e5on/branch/develop?svg=true)](https://ci.appveyor.com/project/nfrechette/rtm)
[![Build Status](https://travis-ci.org/nfrechette/rtm.svg?branch=develop)](https://travis-ci.org/nfrechette/rtm)
[![Sonar Status](https://sonarcloud.io/api/project_badges/measure?project=nfrechette_rtm&metric=alert_status)](https://sonarcloud.io/dashboard?id=nfrechette_rtm)
[![GitHub release](https://img.shields.io/github/release/nfrechette/rtm.svg)](https://github.com/nfrechette/rtm/releases)
[![GitHub license](https://img.shields.io/badge/license-MIT-blue.svg)](https://raw.githubusercontent.com/nfrechette/rtm/master/LICENSE)

# Realtime Math

This library is geared towards realtime applications that require their math to be as fast as possible. Much care was taken to maximize inlining opportunities and for code generation to be optimal when a function isn't inlined by passing values in registers whenever possible.

## Philosophy

Much thought was put into designing the library for it to be as flexible and powerful as possible. To this end, the following decisions were made:

*  The library consists of **100% C++11** header files and is thus easy to integrate in any project
*  The interface follows C-style conventions to ensure optimal code generation
*  Both *float32* and *float64* arithmetic are supported
*  Row vectors are used
*  See [here](./docs/api_conventions.md) for more details

## Supported platforms

*  Windows VS2015 x86 and x64
*  Windows VS2017/2019 x86, x64, and ARM64*
*  Linux (gcc5, gcc6, gcc7, gcc8, clang4, clang5, clang6) x86 and x64
*  OS X (Xcode 8.3, 9.4, 10.3) x86 and x64
*  OS X (Xcode 11.2) x64
*  Android (NVIDIA CodeWorks with clang5) ARMv7-A and ARM64
*  iOS (Xcode 8.3, 9.4, 10.3, 11.2) ARM64

The above supported platform list is only what is tested every release but if it compiles, it should work just fine.

Note: *VS2017* and *VS2019* compile with *ARM64* on *AppVeyor* but I have no device to test them with.

## Getting started

This library is **100%** headers as such you just need to include them in your own project to start using it. However, if you wish to run the unit tests or to contribute to RTM head on over to the [getting started](./docs/getting_started.md) section in order to setup your environment and make sure to check out the [contributing guidelines](CONTRIBUTING.md).

## External dependencies

You don't need anything else to get started: everything is self contained.
See [here](./external) for details.

## License, copyright, and code of conduct

This project uses the [MIT license](LICENSE).

Copyright (c) 2018 Nicholas Frechette & Realtime Math contributors

This project was started from the math code found in the [Animation Compression Library](https://github.com/nfrechette/acl) v1.1.0 and it retains the copyright of the original contributors.

Please note that this project is released with a [Contributor Code of Conduct](CODE_OF_CONDUCT.md). By participating in this project you agree to abide by its terms.
