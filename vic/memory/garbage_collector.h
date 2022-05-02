#pragma once

// This file implements a simple garbage collector,
// and a special type of ptr container (Managed<T>)

namespace vic
{
namespace memory
{

class GarbageCollector; // forward declare

template <typename T>
class Managed
{
public:
    Managed(GarbageCollector& gc)
        : mGc(gc)
        , mObject(new T{})
    { }

    ~Managed() { delete mObject; }

    T* Get() const { return mObject; }

private:
    T* mObject{nullptr};
    GarbageCollector& mGc;
};

class GarbageCollector
{
public:
    GarbageCollector() = default;

    template <typename T>
    Managed<T> New()
    {
        return Managed<T>(*this);
    }

    template <typename TChild, typename TParent>
    Managed<TChild> New(const TParent& parent)
    {
        return Managed<TChild>(*this);
    }

private:
};

} // namespace memory
} // namespace vic