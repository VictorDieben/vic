

find_package(GTest CONFIG REQUIRED)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

enable_testing()

include (GoogleTest)

find_package(range-v3 CONFIG REQUIRED)

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
    "Math.cpp" 
    "Serialize.cpp" "SerializeHelpers.h")
 
TARGET_LINK_LIBRARIES(VicTest  
PRIVATE
    GTest::gtest 
    GTest::gtest_main
    Vic
)

add_library(range_v3 INTERFACE IMPORTED)

#set_target_properties(range_v3 PROPERTIES 
#    INTERFACE_INCLUDE_DIRECTORIES <path_to_range_v3>/include)

target_link_libraries(VicTest PUBLIC range_v3)

# target_link_libraries(VicTest PRIVATE range-v3::meta range-v3::concepts range-v3::range-v3)

gtest_discover_tests(VicTest)
