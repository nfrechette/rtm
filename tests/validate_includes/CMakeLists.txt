cmake_minimum_required (VERSION 3.2)
project(rtm_validate_includes CXX)

# The goal of this project is to generate a single cpp file for every public header
# This will allow us to detect if we are missing an include file during development

include_directories("${PROJECT_SOURCE_DIR}/../../includes")

# Grab all of our public header files
file(GLOB ALL_PUBLIC_HEADER_FILES LIST_DIRECTORIES false
	${PROJECT_SOURCE_DIR}/../../includes/rtm/*.h
	${PROJECT_SOURCE_DIR}/../../includes/rtm/packing/*.h)

# Generate the single include cpp files
foreach(HEADER_FILE ${ALL_PUBLIC_HEADER_FILES})
	# Find the root include directory position
	string(FIND ${HEADER_FILE} "rtm" HEADER_FILEN_RTM_POS REVERSE)

	# Strip the root of the include path
	string(SUBSTRING ${HEADER_FILE} ${HEADER_FILEN_RTM_POS} -1 HEADER_INCLUDE_PATH)

	# Configure our cpp file content
	set(RTM_SINGLE_INCLUDE_NAME ${HEADER_INCLUDE_PATH})

	# Sanitize our filename so we can generate a unique cpp file for it
	string(REPLACE "/" "_" HEADER_SANITIZED_FILENAME ${HEADER_INCLUDE_PATH})
	string(REPLACE "\\" "_" HEADER_SANITIZED_FILENAME ${HEADER_SANITIZED_FILENAME})

	# Generate our single include cpp file
	configure_file(${PROJECT_SOURCE_DIR}/single_include.cpp.in single_include_${HEADER_SANITIZED_FILENAME}.cpp @ONLY)
endforeach(HEADER_FILE)

# Grab all of our main source files
file(GLOB_RECURSE ALL_MAIN_SOURCE_FILES LIST_DIRECTORIES false
	${PROJECT_SOURCE_DIR}/*.cpp
	${PROJECT_BINARY_DIR}/*.cpp)

create_source_groups("${ALL_MAIN_SOURCE_FILES}" ${PROJECT_SOURCE_DIR})

add_library(${PROJECT_NAME} STATIC ${ALL_MAIN_SOURCE_FILES})

setup_default_compiler_flags(${PROJECT_NAME})
