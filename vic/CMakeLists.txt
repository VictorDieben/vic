cmake_minimum_required (VERSION 3.8)

add_library(Vic INTERFACE)

message(STATUS "CMake inatall directory: " ${CMAKE_INSTALL_INCLUDEDIR}) 
        
target_include_directories(Vic INTERFACE ${PROJECT_SOURCE_DIR})