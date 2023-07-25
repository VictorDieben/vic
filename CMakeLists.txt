
cmake_minimum_required (VERSION 3.13)

set(VCPKG_ROOT "C:\\c++libs\\vcpkg")
set(VCPKG_DEFAULT_TRIPLET "x64-windows")
set(CMAKE_TOOLCHAIN_FILE "${VCPKG_ROOT}\\scripts\\buildsystems\\vcpkg.cmake")

message("toolchain file: ${CMAKE_TOOLCHAIN_FILE}")

project ("vic")

enable_testing ()
 
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)


file(GLOB_RECURSE  Headers vic/*.h)  


message("${Headers}")
  
add_library(Vic INTERFACE)
target_sources(Vic INTERFACE ${Headers}) 
target_include_directories(Vic INTERFACE
    "${PROJECT_SOURCE_DIR}"
)


# Testing

find_package(GTest CONFIG REQUIRED)

file(GLOB TEST_SOURCES vic_gtests/*.cpp) 
ADD_EXECUTABLE(VicTest ${TEST_SOURCES})
 

TARGET_LINK_LIBRARIES(VicTest 
    GTest::gtest 
    GTest::gtest_main
    Vic
)

add_test(NAME VicTest
         COMMAND VicTest)
