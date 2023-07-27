cmake_minimum_required (VERSION 3.8)

#file(GLOB_RECURSE projectSRC  
#    ${PROJECT_SOURCE_DIR}/vic/*.h
#)

#list (SORT projectSRC)
#foreach (_src ${projectSRC})
#    message(STATUS "${_src}")
#endforeach()

add_library(Vic INTERFACE)

message(STATUS "CMake inatall directory: " ${CMAKE_INSTALL_INCLUDEDIR})
#target_include_directories(Vic 
#    INTERFACE 
#        $<BUILD_INTERFACE:${PROJECT_INCLUDE_DIR}>
#        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

        
target_include_directories(Vic INTERFACE ${PROJECT_SOURCE_DIR})