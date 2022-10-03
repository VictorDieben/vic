#include "pch.h"

#include "vic/machine_learning/machine_learning.h"

#include "test_base.h"
#include <algorithm>
#include <random>
#include <vector>

namespace vic
{
namespace ml
{

TEST(TestMachineLearning, Setup)
{
    // test default construction
    vic::ml::System<double> system;

    // set the system to some very small size
    system = vic::ml::System<double>({5, 4, 3, 2});

    // test initializing some system with 100 inputs, two hidden layers (70 and 30), and 10 outputs
    system.Initialize({100, 70, 30, 10});
}

TEST(TestMachineLearning, SimpleModel)
{
    //
}

} // namespace memory
} // namespace vic