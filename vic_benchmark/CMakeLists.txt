

find_package(GTest CONFIG REQUIRED)
find_package(benchmark CONFIG REQUIRED)

# set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
 

include (GoogleTest)


ADD_EXECUTABLE(VicBenchmark 
    "FirstBenchmark.cpp")
 
TARGET_LINK_LIBRARIES(VicBenchmark  
PRIVATE
    benchmark::benchmark
    benchmark::benchmark_main
    GTest::gtest 
    GTest::gtest_main
    Vic
)

 
