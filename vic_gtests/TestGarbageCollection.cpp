#include "pch.h"
#include "test_base.h"

#include "vic/memory/garbage_collector.h"
#include "vic/utils.h"

namespace vic
{
namespace memory
{

TEST(TestGC, HappyFlow)
{
    GarbageCollector gc;

    struct TestObject
    { };

    struct OtherTestObject
    { };

    auto managed = gc.New<TestObject>();
    auto child = gc.New<OtherTestObject>(managed);
}

TEST(TestGC, CircularDependency)
{
    // Test if a circular dependency can be detected, and properly cleaned up
    OnEventObject object{[]() {}, //
                         []() {},
                         []() {},
                         []() {}};
}

} // namespace memory
} // namespace vic