cmake_minimum_required(VERSION 3.2)
project(rtm_bench CXX)

set(CMAKE_CXX_STANDARD 11)

# Project root is <rtm-dir>\tools\bench\main_android
set(PROJECT_ROOT_DIR "${PROJECT_SOURCE_DIR}/../../../..")

# Google Benchmark
set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "No need to run benchmark's tests" FORCE)
add_subdirectory("${PROJECT_ROOT_DIR}/../../../external/benchmark" google_benchmark)

include_directories("${PROJECT_ROOT_DIR}/../../../includes")
include_directories("${PROJECT_ROOT_DIR}/../../../external/benchmark/include")

# Grab all of our test source files
file(GLOB_RECURSE ALL_BENCH_SOURCE_FILES LIST_DIRECTORIES false
	${PROJECT_ROOT_DIR}/../sources/*.h
	${PROJECT_ROOT_DIR}/../sources/*.cpp)

# Grab all of our main source files
file(GLOB_RECURSE ALL_MAIN_SOURCE_FILES LIST_DIRECTORIES false
	${PROJECT_SOURCE_DIR}/*.cpp)

add_library(${PROJECT_NAME} SHARED ${ALL_BENCH_SOURCE_FILES} ${ALL_MAIN_SOURCE_FILES})

# Enable debug symbols
target_compile_options(${PROJECT_NAME} PRIVATE -g)

# Disable SIMD if not needed
if(NOT USE_SIMD_INSTRUCTIONS)
	add_definitions(-DRTM_NO_INTRINSICS)
endif()

target_include_directories(${PROJECT_NAME} PUBLIC jni)

target_link_libraries(${PROJECT_NAME} m log)
target_link_libraries(${PROJECT_NAME} benchmark)
