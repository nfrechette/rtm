project(rtm_unit_tests CXX)

include_directories("${PROJECT_SOURCE_DIR}/../../includes")
include_directories("${PROJECT_SOURCE_DIR}/../../external/catch2/single_include")
include_directories("${PROJECT_SOURCE_DIR}/../sources")

# Grab all of our test source files
file(GLOB_RECURSE ALL_TEST_SOURCE_FILES LIST_DIRECTORIES false
	${PROJECT_SOURCE_DIR}/../sources/*.h
	${PROJECT_SOURCE_DIR}/../sources/*.cpp)

if(NOT WITH_VECTOR_MIX_TESTS)
	# The vector_mix unit tests are slow to compile, do not compile them when they are disabled
	file(GLOB VECTOR_MIX_SOURCE_FILES LIST_DIRECTORIES false
		${PROJECT_SOURCE_DIR}/../sources/vector_mix/*.cpp)
	list(REMOVE_ITEM ALL_TEST_SOURCE_FILES ${VECTOR_MIX_SOURCE_FILES})
endif()

# Grab all of our main source files
file(GLOB_RECURSE ALL_MAIN_SOURCE_FILES LIST_DIRECTORIES false
	${PROJECT_SOURCE_DIR}/*.cpp)

add_executable(${PROJECT_NAME} ${ALL_TEST_SOURCE_FILES} ${ALL_MAIN_SOURCE_FILES})

# Throw on failure to allow us to catch them and recover
add_definitions(-DRTM_ON_ASSERT_THROW)

if(WITH_VECTOR_MIX_TESTS)
	# Enable vector_mix unit tests
	add_definitions(-DRTM_IMPL_WITH_VECTOR_MIX_TESTS)
endif()

if(USE_SIMD_INSTRUCTIONS)
	target_compile_options(${PROJECT_NAME} PRIVATE -msimd128)		# Enable WASM SIMD
else()
	add_definitions(-DRTM_NO_INTRINSICS)
endif()

target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra)		# Enable all warnings
target_compile_options(${PROJECT_NAME} PRIVATE -Wshadow)			# Enable shadowing warnings
target_compile_options(${PROJECT_NAME} PRIVATE -Werror)				# Treat warnings as errors

# Exceptions are not enabled by default, enable native WebAssembly exception handling
target_compile_options(${PROJECT_NAME} PRIVATE -fwasm-exceptions)
target_link_libraries(${PROJECT_NAME} PRIVATE -fwasm-exceptions)

target_link_libraries(${PROJECT_NAME} PRIVATE "-s NODERAWFS=1")				# Enable the raw node file system

target_link_libraries(${PROJECT_NAME} PRIVATE "-s ENVIRONMENT=node")		# Force the environment to node

target_link_libraries(${PROJECT_NAME} PRIVATE "-s ALLOW_MEMORY_GROWTH=1")	# Allow dynamic memory allocation
target_link_libraries(${PROJECT_NAME} PRIVATE "-s STACK_SIZE=5MB")			# The 64 KB default is too small for the debug tests

# Setup Catch2 so we can find and execute the unit tests with CTest
set(OptionalCatchTestLauncher node)
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/../../external/catch2/contrib")
include(CTest)
include(ParseAndAddCatchTests)
ParseAndAddCatchTests(${PROJECT_NAME})

install(FILES
	${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.js
	${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.wasm
	DESTINATION bin)
