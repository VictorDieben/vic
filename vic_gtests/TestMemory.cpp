
#include "pch.h"
#include "test_base.h"
#include "vic/memory/refcounter.h"

namespace vic
{
namespace memory
{

TEST(TestMemory, RefCounted)
{

    struct TestStruct
    {
        TestStruct(int* value)
            : mValue(value)
        { }
        ~TestStruct() { *mValue = 10; }
        int* mValue;

        void SetValue(const int val) { *mValue = val; }
        int GetValue() { return *mValue; }
    };

    int value = 1;
    {
        // construct a RefCounted object
        RefCounted<TestStruct> counted{&value};
        EXPECT_EQ(counted.use_count(), 1);

        // test dereferencing object
        counted->SetValue(2);
        EXPECT_EQ(value, 2);

        // check getting a reference to the contained object
        TestStruct& ref = counted.Get();

        // copy the RefCounter, acces data through it,
        // check that refcount is reset after it is destructed.
        {
            memory::RefCounted<TestStruct> copy{counted};
            EXPECT_EQ(counted.use_count(), 2);

            counted->SetValue(3);
            EXPECT_EQ(value, 3);
        }
        EXPECT_EQ(counted.use_count(), 1);
    }

    // destructor sets value to 10
    EXPECT_EQ(value, 10);
}

} // namespace memory
} // namespace vic