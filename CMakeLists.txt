cmake_minimum_required (VERSION 3.2)
project(rtm NONE)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake")

include(CMakeUtils)
include(CMakeCompiler)
include(CMakePlatforms)

set(USE_AVX_INSTRUCTIONS false CACHE BOOL "Use AVX instructions")
set(USE_AVX2_INSTRUCTIONS false CACHE BOOL "Use AVX2 instructions")
set(USE_SIMD_INSTRUCTIONS true CACHE BOOL "Use SIMD instructions")
set(WITH_VECTOR_MIX_TESTS false CACHE BOOL "Enable vector_mix unit tests")
set(CPU_INSTRUCTION_SET false CACHE STRING "CPU instruction set")
set(BUILD_BENCHMARK_EXE false CACHE BOOL "Enable the benchmark projects")

if(CMAKE_CONFIGURATION_TYPES)
	set(CMAKE_CONFIGURATION_TYPES Debug Release)
	set(CMAKE_CONFIGURATION_TYPES "${CMAKE_CONFIGURATION_TYPES}" CACHE STRING "Reset the configurations to what we need" FORCE)
endif()

# Grab all of our include files
file(GLOB_RECURSE RTM_INCLUDE_FILES LIST_DIRECTORIES false
	${PROJECT_SOURCE_DIR}/includes/*.h
	${PROJECT_SOURCE_DIR}/docs/*.md
	${PROJECT_SOURCE_DIR}/cmake/*.cmake
	${PROJECT_SOURCE_DIR}/tools/release_scripts/*.py
	${PROJECT_SOURCE_DIR}/tools/vs_visualizers/*.natvis
	)

create_source_groups("${RTM_INCLUDE_FILES}" ${PROJECT_SOURCE_DIR})

file(GLOB RTM_ROOT_FILES LIST_DIRECTORIES false
	${PROJECT_SOURCE_DIR}/*.md
	${PROJECT_SOURCE_DIR}/*.py)

# Create a dummy target so they show up in the IDE
add_custom_target(${PROJECT_NAME} SOURCES ${RTM_INCLUDE_FILES} ${RTM_ROOT_FILES})

# Enable CTest
enable_testing()

# Add other projects
add_subdirectory("${PROJECT_SOURCE_DIR}/tests")

if(BUILD_BENCHMARK_EXE)
	# Our benchmark executable
	add_subdirectory("${PROJECT_SOURCE_DIR}/tools/bench")
endif()
