@echo off

REM Unpack arguments
SET WORKER_IMAGE=%1
SET PLATFORM=%2
SET CONFIG=%3
SET TOOLCHAIN=%4
SET PYTHON_PATH=%5

echo "Worker image: %WORKER_IMAGE%"
echo "Platform: %PLATFORM%"
echo "Config: %CONFIG%"
echo "Toolchain: %TOOLCHAIN%"
echo "Python path: %PYTHON_PATH%"

REM Convert the build image and toolchain into our compiler string
IF /i %TOOLCHAIN%==msvc goto :msvc
IF /i %TOOLCHAIN%==clang goto :clang

echo "Unknown toolchain: %TOOLCHAIN%"
exit /B

:msvc
IF /i %WORKER_IMAGE%=="Visual Studio 2015" SET COMPILER=vs2015
IF /i %WORKER_IMAGE%=="Visual Studio 2017" SET COMPILER=vs2017
IF /i %WORKER_IMAGE%=="Visual Studio 2019" SET COMPILER=vs2019
goto :next

:clang
IF /i %WORKER_IMAGE%=="Visual Studio 2019" SET COMPILER=vs2019-clang
goto :next

:next
REM Set our switch if we need to run unit tests
SET UNIT_TEST_FLAG=-unit_test
IF /i %PLATFORM%==arm64 SET UNIT_TEST_FLAG=""

REM If PYTHON_PATH isn't set, assume it is in PATH
IF NOT DEFINED PYTHON_PATH SET PYTHON_PATH=python.exe

REM Build and run unit tests
%PYTHON_PATH% make.py -build %UNIT_TEST_FLAG% -compiler %COMPILER% -config %CONFIG% -cpu %PLATFORM% -clean
%PYTHON_PATH% make.py -build %UNIT_TEST_FLAG% -compiler %COMPILER% -config %CONFIG% -cpu %PLATFORM% -nosimd
