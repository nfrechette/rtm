cmake_minimum_required (VERSION 3.2)
project(rtm_bench)

# Google Benchmark
set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "No need to run benchmark's tests" FORCE)
add_subdirectory("${PROJECT_SOURCE_DIR}/../../../external/benchmark" google_benchmark)

# iOS cmake toolchain does not support CMAKE_CXX_STANDARD
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Force enable debug symbols
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

# Enable optimizations in Release
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

set(MACOSX_BUNDLE_EXECUTABLE_NAME ${PROJECT_NAME})
set(MACOSX_BUNDLE_INFO_STRING "com.rtm.rtm-bench")
set(MACOSX_BUNDLE_GUI_IDENTIFIER "com.rtm.rtm-bench")
set(MACOSX_BUNDLE_BUNDLE_NAME "rtm-bench")

include_directories("${PROJECT_SOURCE_DIR}/../../../includes")
include_directories("${PROJECT_SOURCE_DIR}/../../../external/benchmark/include")

# Grab all of our benchmark source files
file(GLOB_RECURSE ALL_BENCH_SOURCE_FILES LIST_DIRECTORIES false
	${PROJECT_SOURCE_DIR}/../sources/*.h
	${PROJECT_SOURCE_DIR}/../sources/*.cpp)

create_source_groups("${ALL_BENCH_SOURCE_FILES}" ${PROJECT_SOURCE_DIR}/..)

# Grab all of our main source files
file(GLOB_RECURSE ALL_MAIN_SOURCE_FILES LIST_DIRECTORIES false
	${PROJECT_SOURCE_DIR}/*.cpp)

create_source_groups("${ALL_MAIN_SOURCE_FILES}" ${PROJECT_SOURCE_DIR})

add_executable(${PROJECT_NAME} MACOSX_BUNDLE ${ALL_BENCH_SOURCE_FILES} ${ALL_MAIN_SOURCE_FILES})

target_link_libraries(${PROJECT_NAME} PRIVATE benchmark)    # Link Google Benchmark

# Disable SIMD if not needed
if(NOT USE_SIMD_INSTRUCTIONS)
	add_definitions(-DRTM_NO_INTRINSICS)
endif()
