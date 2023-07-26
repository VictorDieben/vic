

find_package(GTest CONFIG REQUIRED)

file(GLOB_RECURSE TEST_SOURCES ${PROJECT_SOURCE_DIR}/vic_gtests/*.cpp ${PROJECT_SOURCE_DIR}/vic_gtests/*.h) 
ADD_EXECUTABLE(VicTest ${TEST_SOURCES})
 
TARGET_LINK_LIBRARIES(VicTest  
    GTest::gtest 
    GTest::gtest_main
    Vic
)



add_test(NAME VicTest
         COMMAND VicTest)