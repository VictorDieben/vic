

find_package(GTest CONFIG REQUIRED)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

enable_testing()

include (GoogleTest)


ADD_EXECUTABLE(VicTest 
    "EntitySystem.cpp"
    "Graph.cpp"
    "TestEventQueue.cpp"
    "TestGarbageCollection.cpp"
    "TestGeometry.cpp"
    "TestImages.cpp"
    "TestKinematics.cpp"
    "TestLinprog.cpp"
    "TestMachineLearning.cpp"
    "TestMemory.cpp"
    "TestNetwork.cpp"
    "TestSimulations.cpp"
    "TestSymbolic.cpp"
    "Tree.cpp"
    "TestUnits.cpp"
    "Utils.cpp"
    "TestUtpNetwork.cpp"
    "graph/AdjecencyGraph.cpp"
    "linalg/Add.cpp"
    "linalg/Algorithms.cpp"
    "linalg/Inverse.cpp"
    "linalg/LeastSquares.cpp"
    "linalg/Linalg.cpp"
    "linalg/Matmul.cpp"
    "linalg/Matrices.cpp"
    "linalg/Solve.cpp"
    "linalg/Tools.cpp"
    "linalg/Traits.cpp"
    "linalg/Transpose.cpp"
    "linalg/Views.cpp"
    "test_emulator.cpp" 
 )
 
TARGET_LINK_LIBRARIES(VicTest  
PRIVATE
    GTest::gtest 
    GTest::gtest_main
    Vic
)

gtest_discover_tests(VicTest)
