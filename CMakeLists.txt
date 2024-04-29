
cmake_minimum_required (VERSION 3.20)

set(VCPKG_ROOT "C:\\c++libs\\vcpkg")
set(VCPKG_DEFAULT_TRIPLET "x64-windows")
set(CMAKE_TOOLCHAIN_FILE "${VCPKG_ROOT}\\scripts\\buildsystems\\vcpkg.cmake")

message("toolchain file: ${CMAKE_TOOLCHAIN_FILE}")

project ("vic")

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_subdirectory ("vic")

# Testing 
if (NOT DEFINED VIC_NO_TESTS)
	add_subdirectory ("vic_gtests")
endif()

# Benchmarking 
if (NOT DEFINED VIC_NO_BENCHMARK)
	add_subdirectory ("vic_benchmark")
endif()
