
cmake_minimum_required (VERSION 3.20)

set(VCPKG_ROOT "C:\\c++libs\\vcpkg")
set(VCPKG_DEFAULT_TRIPLET "x64-windows")
set(CMAKE_TOOLCHAIN_FILE "${VCPKG_ROOT}\\scripts\\buildsystems\\vcpkg.cmake")

message("toolchain file: ${CMAKE_TOOLCHAIN_FILE}")

project ("vic")

enable_testing()
 
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_subdirectory ("vic")


# Testing 
add_subdirectory ("vic_gtests")