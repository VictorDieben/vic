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
private:
    T* mObject{ nullptr };
};

class GarbageCollector
{
public:
    GarbageCollector() = default;
private:
};


}
}