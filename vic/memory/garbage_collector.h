#pragma once

// This file implements a simple garbage collector,
// and a special type of ptr container (Managed<T>)

namespace vic
{
namespace memory
{

class GarbageCollector; // forward declare

// CRTP type
template <typename T>
class Managed
{
public:
    Managed() = default;
    Managed(GarbageCollector& gc)
        : mGc(gc)
    { }

    ~Managed() { }

    std::size_t GetCount() const { return 0; }

private:
    GarbageCollector& mGc;
};

class GarbageCollector
{
public:
    GarbageCollector() = default;

    template <typename T>
    Managed<T> New()
    {
        T* newObject = new T{};
        return Managed<T>(*this, newObject);
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