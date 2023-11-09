

find_package(GTest CONFIG REQUIRED)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

enable_testing()

include (GoogleTest)


ADD_EXECUTABLE(VicTest 
    "EntitySystem.cpp"
    "Graph.cpp"
    "EventQueue.cpp"
    "GarbageCollection.cpp"
    "Geometry.cpp"
    "Images.cpp"
    "Kinematics.cpp"
    "Linprog.cpp"
    "MachineLearning.cpp"
    "Memory.cpp"
    "Network.cpp"
    "Simulations.cpp"
    "Symbolic.cpp"
    "Tree.cpp"
    "Units.cpp"
    "Utils.cpp"
    "UtpNetwork.cpp"
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
  "Graph2.cpp" 
  "Math.cpp")
 
TARGET_LINK_LIBRARIES(VicTest  
PRIVATE
    GTest::gtest 
    GTest::gtest_main
    Vic
)


gtest_discover_tests(VicTest)
