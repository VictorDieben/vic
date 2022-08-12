#pragma once

namespace vic
{

// example:
// struct SomeCountedClass : public Counted<SomeCountedClass> {};
// SomeCountedClass::GetCount();
template <typename T>
class Counted
{
public:
    Counted(const Counted<T>& other) { mCount++; }
    Counted(Counted<T>&& other) noexcept { mCount++; }
    ~Counted() { mCount--; }
    static std::size_t GetCount() { return mCount; }

    // todo: fix this
    Counted() { mCount++; }
    //protected:
    //    // make sure only derived class can instantiate this class
    //    Counted() { mCount++; }
    //    friend T;

private:
    static inline std::size_t mCount{0};
};

} // namespace vic