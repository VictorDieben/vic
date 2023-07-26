#include "test_base.h"
#include "gtest/gtest.h"

#include "vic/memory/garbage_collector.h"
#include "vic/utils.h"

namespace vic
{
namespace memory
{

//TEST(TestGC, EmptyManaged)
//{
//    // test assigning a managed ptr after allocating it
//    struct TestObject
//    { };
//
//    Managed<TestObject> empty{};
//    GarbageCollector gc;
//    empty = gc.New<TestObject>();
//}
//
//TEST(TestGC, NestedObject)
//{
//    struct TestChild : public Managed<TestChild>
//    { };
//
//    struct TestParent : public Managed<TestParent>
//    { };
//
//    GarbageCollector gc;
//    auto parent = gc.New<TestParent>();
//}

//TEST(TestGC, HappyFlow)
//{
//    GarbageCollector gc;
//
//    struct TestObject : public Managed<TestObject>
//    {
//        TestObject(GarbageCollector& gc)
//            : Managed<TestObject>(gc)
//        { }
//    };
//
//    struct OtherTestObject : public Managed<OtherTestObject>
//    { };
//
//    auto managed = gc.New<TestObject>();
//    auto child = gc.New<OtherTestObject>(managed);
//
//    auto childCopy = child;
//}

//TEST(TestGC, CircularDependency)
//{
//    // Test if a circular dependency can be detected, and properly cleaned up
//    OnEventObject object{[]() {}, //
//                         []() {},
//                         []() {},
//                         []() {}};
//}

} // namespace memory
} // namespace vic