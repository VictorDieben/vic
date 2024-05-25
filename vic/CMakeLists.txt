cmake_minimum_required (VERSION 3.8)

add_library(Vic INTERFACE)

message(STATUS "CMake inatall directory: " ${CMAKE_INSTALL_INCLUDEDIR}) 
        
find_package(range-v3 CONFIG REQUIRED)
add_library(range_v3 INTERFACE IMPORTED)

target_include_directories(Vic INTERFACE ${PROJECT_SOURCE_DIR} range_v3)
 