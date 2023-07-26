

find_package(GTest CONFIG REQUIRED)

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
    "TestTree.cpp"
    "TestUnits.cpp"
    "TestUtils.cpp"
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
    GTest::gtest 
    GTest::gtest_main
    Vic
)

add_test(NAME VicTest
         COMMAND VicTest)